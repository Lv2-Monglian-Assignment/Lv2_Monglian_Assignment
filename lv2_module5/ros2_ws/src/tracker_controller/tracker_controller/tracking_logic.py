"""추적 상태 판단과 명령 계산 (ROS에 의존하지 않는 순수 로직).

상태
- IDLE       : 추적 꺼짐. 명령 없음
- TRACKING   : 신선한 입력에서 목표 검출. 영상 중심 오차 P 제어
- LOST       : 미검출·입력 타임아웃·탐색 실패. 즉시 0 deg/s (이전 속도 유지 금지)
- SEARCHING  : (심화, search.enabled) 미검출이 search.delay_s 이상 계속되면
               기억한 목표의 예측 방향으로 각도 P 제어. 시간·각도·속도 상한, 실패 시 정지
"""
import math
from dataclasses import dataclass

IDLE, TRACKING, LOST, SEARCHING = 'IDLE', 'TRACKING', 'LOST', 'SEARCHING'


@dataclass
class AxisConfig:
    kp: float               # half_fov_deg > 0: [1/s] 각도 오차 [deg] -> 속도 [deg/s] / 0: [deg/s per 1.0 정규화 오차]
    direction: int          # +1/-1: 오른쪽·아래 목표(ex, ey > 0)를 줄이는 모터 속도 부호
    speed_limit: float      # [deg/s]
    deadband: float         # 정규화 오차. 이보다 작으면 0
    enabled: bool = True
    half_fov_deg: float = 0.0   # 화면 반폭(반높이) 시야각 [deg]. >0이면 정규화 오차를 카메라 각도로 바꿔 Kp를 곱한다


@dataclass
class SearchConfig:
    enabled: bool = False   # 기본 시험(가림 후 정지 확인)에서는 끈다
    delay_s: float = 0.3    # 미검출 후 정지 상태로 기다리는 시간
    timeout_s: float = 3.0  # 탐색 최대 시간. 넘으면 정지하고 다시 탐색하지 않음
    kp: float = 2.0         # [1/s] 각도 오차 -> 속도 (모듈 1의 위치 P와 같은 단위)
    speed_limit: float = 20.0      # [deg/s]
    tolerance_deg: float = 1.0     # 이보다 가까우면 해당 축 0
    pan_max_deg: float = 80.0      # 탐색 목표각 제한 (펌웨어 제한 이내)
    tilt_max_deg: float = 25.0


@dataclass
class Output:
    state: str
    reason: str
    pan_cmd: float = 0.0    # [deg/s]
    tilt_cmd: float = 0.0   # [deg/s]
    search_elapsed: float = math.nan


def error_angle_deg(error, half_fov_deg):
    """정규화 오차 e (-1~+1, 화면 끝 = 1) -> 광축에서 목표까지 각도 [deg] = atan(e x tan(시야각/2)).
    핀홀 카메라에서 화면 끝 픽셀의 각도가 시야각/2이고, 픽셀 거리는 tan(각도)에 비례하기 때문이다."""
    return math.degrees(math.atan(error * math.tan(math.radians(half_fov_deg))))


def p_command(error, axis: AxisConfig):
    """command = clamp(direction x Kp x 오차, -limit, +limit) [deg/s]. 오차는 각도 [deg] (half_fov_deg > 0) 또는 정규화 값"""
    if not axis.enabled or abs(error) < axis.deadband:
        return 0.0
    err = error_angle_deg(error, axis.half_fov_deg) if axis.half_fov_deg > 0 else error
    return max(-axis.speed_limit, min(axis.speed_limit, axis.direction * axis.kp * err))


def angle_command(desired_deg, current_deg, s: SearchConfig, limit_deg):
    """탐색용 각도 P: clamp(Kp x (목표각 - 현재각)) [deg/s]"""
    desired_deg = max(-limit_deg, min(limit_deg, desired_deg))
    err = desired_deg - current_deg
    if abs(err) < s.tolerance_deg:
        return 0.0
    return max(-s.speed_limit, min(s.speed_limit, s.kp * err))


class TrackingLogic:
    def __init__(self, pan: AxisConfig, tilt: AxisConfig, search: SearchConfig = None,
                 input_timeout_s=0.5, recover_frames=3, require_increasing_stamp=True):
        self.pan, self.tilt = pan, tilt
        self.search = search or SearchConfig()
        self.input_timeout_s = input_timeout_s
        self.recover_frames = recover_frames
        self.require_increasing_stamp = require_increasing_stamp
        self.enabled = False
        self.state, self.reason = IDLE, 'disabled'
        self.streak = 0
        self.last = None              # (ex, ey, area, detected)
        self.last_rx = None
        self.last_stamp_ns = None
        self.seq = 0
        self.stale_count = 0
        self.miss_since = None        # 연속 미검출이 시작된 시각 (첫 미검출 프레임 수신 시각)
        self.search_start = None
        self.search_failed = False    # 실패 후에는 새 TRACKING 전까지 다시 탐색하지 않음
        self.stats = {'reacquired_in_view': 0, 'reacquired_search': 0, 'search_failed': 0}

    def set_enabled(self, on: bool):
        self.enabled = on
        self.streak = 0
        self.search_start, self.search_failed, self.miss_since = None, False, None
        self.state, self.reason = (LOST, 'waiting') if on else (IDLE, 'disabled')

    def on_target(self, ex, ey, area, stamp_ns, now):
        """/target 한 건. 시각이 증가하지 않는 입력(같은 영상의 재전송)은 버린다."""
        if (self.require_increasing_stamp and stamp_ns > 0 and
                self.last_stamp_ns is not None and stamp_ns <= self.last_stamp_ns):
            self.stale_count += 1
            return False
        self.last_stamp_ns = stamp_ns
        valid = all(math.isfinite(v) for v in (ex, ey, area))
        detected = valid and area > 0.0
        ex = max(-1.0, min(1.0, ex)) if valid else 0.0
        ey = max(-1.0, min(1.0, ey)) if valid else 0.0
        self.streak = self.streak + 1 if detected else 0
        if detected:
            self.miss_since = None
        elif self.miss_since is None:
            self.miss_since = now
        self.last = (ex, ey, area if valid else 0.0, detected)
        self.last_rx = now
        self.seq += 1
        return True

    def _search_out(self, now, joint_deg, desired_deg, reason):
        if joint_deg is None or desired_deg is None:
            self.search_start = None
            return Output(LOST, 'search_unavailable')
        pan_cmd = angle_command(desired_deg[0], joint_deg[0], self.search, self.search.pan_max_deg)
        tilt_cmd = (angle_command(desired_deg[1], joint_deg[1], self.search, self.search.tilt_max_deg)
                    if self.tilt.enabled else 0.0)
        return Output(SEARCHING, reason, pan_cmd, tilt_cmd, now - self.search_start)

    def step(self, now, joint_deg=None, desired_deg=None):
        """제어 주기마다 호출.

        joint_deg  : 현재 모터 각도 (pan, tilt) [deg] 또는 None
        desired_deg: 기억한 목표를 정중앙에 두는 모터 각도 (pan, tilt) [deg] 또는 None
        """
        out = self._decide(now, joint_deg, desired_deg)
        self.state, self.reason = out.state, out.reason
        return out

    def _decide(self, now, joint_deg, desired_deg):
        if not self.enabled:
            return Output(IDLE, 'disabled')
        age = math.inf if self.last_rx is None else now - self.last_rx

        # 1) 입력 타임아웃: 카메라·인지가 멈춘 것. 탐색도 하지 않고 정지
        if age > self.input_timeout_s:
            self.streak = 0
            self.search_start = None
            self.miss_since = None    # 영상이 다시 들어오면 미검출 시간을 새로 센다
            return Output(LOST, 'input_timeout')

        detected = self.last[3]
        searching = self.search_start is not None

        # 2) 미검출 (영상은 계속 들어옴)
        if not detected:
            if searching:
                if now - self.search_start > self.search.timeout_s:
                    self.search_start, self.search_failed = None, True
                    self.stats['search_failed'] += 1
                    return Output(LOST, 'search_failed')
                return self._search_out(now, joint_deg, desired_deg, 'predicted')
            can_search = (self.search.enabled and not self.search_failed and desired_deg is not None
                          and joint_deg is not None and now - self.miss_since >= self.search.delay_s)
            if can_search:
                self.search_start = now
                return self._search_out(now, joint_deg, desired_deg, 'predicted')
            return Output(LOST, 'search_failed' if self.search_failed else 'no_detection')

        # 3) 검출됐지만 아직 연속 프레임이 모자람
        if self.state != TRACKING and self.streak < self.recover_frames:
            confirm = f'confirming_{self.streak}/{self.recover_frames}'
            if searching:   # 탐색 중 발견: 확인될 때까지 탐색 방향 유지
                return self._search_out(now, joint_deg, desired_deg, confirm)
            return Output(LOST, confirm)

        # 4) 추적: 영상 중심 오차 P 제어
        if self.state != TRACKING:
            key = 'reacquired_search' if searching else 'reacquired_in_view'
            self.stats[key] += 1
            reason = key
        else:
            reason = 'ok'
        self.search_start, self.search_failed = None, False
        ex, ey, _, _ = self.last
        return Output(TRACKING, reason, p_command(ex, self.pan), p_command(ey, self.tilt))

    def target_age(self, now):
        return math.nan if self.last_rx is None else now - self.last_rx
