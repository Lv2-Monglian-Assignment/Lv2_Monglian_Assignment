"""opencr_tracker 펌웨어 시리얼 줄 해석 (ROS에 의존하지 않는 순수 함수)."""
import math

FW_STATES = ('OFF', 'HOLD', 'TRACK', 'HOMING', 'FAULT')


def parse_status(line):
    """'S <ms> <pan_deg> <tilt_deg> <pan_dps> <tilt_dps> <state>' -> (pan, tilt, pan_dps, tilt_dps, state) 또는 None"""
    parts = line.split()
    if len(parts) != 7 or parts[0] != 'S' or parts[6] not in FW_STATES:
        return None
    try:
        vals = [float(v) for v in parts[2:6]]
    except ValueError:
        return None
    if not all(math.isfinite(v) for v in vals):
        return None
    return (*vals, parts[6])


def status_ms(line):
    """상태 줄의 보드 시각 [ms] (보드가 모터 각도를 읽은 시각). 상태 줄이 아니면 None"""
    parts = line.split()
    if len(parts) != 7 or parts[0] != 'S':
        return None
    try:
        return int(parts[1])
    except ValueError:
        return None


class BoardClock:
    """보드 시각 [ms] -> Pi 시각 [s]. 최근 window_s 동안 (받은 시각 - 보드 시각)의 최솟값을 오프셋으로 쓴다.
    브리지는 20 ms 주기로 시리얼을 읽어 받은 시각이 측정보다 0~20 ms 늦다(2026-10-08 실측 90 %: 18 ms).
    가장 빨리 받은 줄이 실제 측정 시각에 가장 가깝다. 창을 두는 이유: 두 시계의 속도 차이(약 0.03 %)와 보드 재시작."""

    def __init__(self, window_s=5.0):
        self.window_s = window_s
        self.buf = []        # (받은 시각, 오프셋)

    def stamp(self, rx_s, board_ms):
        off = rx_s - board_ms / 1000.0
        if self.buf and abs(off - self.buf[-1][1]) > 1.0:   # 보드 재시작·시계 점프: 기록을 버린다
            self.buf = []
        self.buf.append((rx_s, off))
        while self.buf and rx_s - self.buf[0][0] > self.window_s:
            self.buf.pop(0)
        return board_ms / 1000.0 + min(o for _, o in self.buf)


class FaultRetry:
    """FAULT 자동 복구 판단 (2026-10-08 팀 결정): FAULT가 delay_s 이어지면 R을 보낸다. max_tries번 해도 다시 FAULT면
    자동 복구를 멈추고 수동 복구를 요청한다. 정상 상태가 healthy_s 이어지면 횟수를 다시 센다."""

    def __init__(self, max_tries=3, delay_s=2.0, healthy_s=60.0):
        self.max_tries, self.delay_s, self.healthy_s = max_tries, delay_s, healthy_s
        self.tries = 0
        self.fault_since = None
        self.ok_since = None
        self.manual = False      # True: 자동 복구 포기, 사람이 원인을 확인하고 복구해야 함

    def update(self, state, now):
        """상태 줄마다 호출. 'R'을 보내야 하면 True"""
        if state != 'FAULT':
            self.fault_since = None
            if self.ok_since is None:
                self.ok_since = now
            if now - self.ok_since >= self.healthy_s:
                self.tries, self.manual = 0, False
            return False
        self.ok_since = None
        if self.fault_since is None:
            self.fault_since = now
        if self.manual or now - self.fault_since < self.delay_s:
            return False
        if self.tries >= self.max_tries:
            self.manual = True
            return False
        self.tries += 1
        self.fault_since = now          # 다음 시도까지 다시 delay_s 기다린다
        return True


def sim_step(pos_deg, cmd_dps, dt, limit_deg, speed_limit=120.0, limit_gain=3.0):
    """dry_run 관절 시뮬레이션: 펌웨어 applyLimits와 같은 속도 상한·한계 접근 감속. (새 각도, 실제 속도)"""
    v = max(-speed_limit, min(speed_limit, cmd_dps))
    upper = max(0.0, limit_gain * (limit_deg - pos_deg))
    lower = min(0.0, limit_gain * (-limit_deg - pos_deg))
    v = max(lower, min(upper, v))
    return pos_deg + v * dt, v
