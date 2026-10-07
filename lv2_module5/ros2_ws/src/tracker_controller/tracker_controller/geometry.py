"""카메라 좌표 <-> 팬·틸트 기준 좌표 변환과 목표 기억(예측).

좌표계
- 카메라 광학 좌표(camera_color_optical_frame): x 오른쪽, y 아래, z 앞 [m]
- 기준 좌표(pan_tilt_base): 팬 축 위, 팬=0·틸트=0일 때 카메라 정면 방향. x 앞, y 왼쪽, z 위 [m]
- 기하 각도: pan_geo = 왼쪽(반시계) +, tilt_geo = 위 + [rad]
  모터 각도와의 관계는 tracker의 direction(+1/-1)에서 정해진다: geo = -direction x motor
  (direction은 '오른쪽 목표(ex>0)를 줄이는 모터 속도 부호'이므로, 모터 + 방향 = 오른쪽/아래)
- 카메라 광학 중심은 틸트 축에서 앞 cam_forward, 위 cam_up 만큼 떨어져 있다(측정값, #todo).
"""
import math
from collections import deque


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


# ---------- 좌표 변환 ----------
def _optical_to_body(p):   # (오른쪽, 아래, 앞) -> (앞, 왼쪽, 위)
    x, y, z = p
    return (z, -x, -y)


def _body_to_optical(p):
    x, y, z = p
    return (-y, -z, x)


def _pitch_up(p, th):      # 틸트 축(y, 왼쪽) 기준으로 위로 th 회전
    x, y, z = p
    c, s = math.cos(th), math.sin(th)
    return (x * c - z * s, y, x * s + z * c)


def _yaw_left(p, ps):      # 팬 축(z, 위) 기준으로 왼쪽으로 ps 회전
    x, y, z = p
    c, s = math.cos(ps), math.sin(ps)
    return (x * c - y * s, x * s + y * c, z)


def cam_to_base(p_opt, pan_geo, tilt_geo, cam_forward=0.0, cam_up=0.0):
    """카메라 광학 좌표의 점을 기준 좌표로 바꾼다."""
    b = _optical_to_body(p_opt)
    b = (b[0] + cam_forward, b[1], b[2] + cam_up)
    return _yaw_left(_pitch_up(b, tilt_geo), pan_geo)


def base_to_cam(p_base, pan_geo, tilt_geo, cam_forward=0.0, cam_up=0.0):
    """기준 좌표의 점을 현재 자세의 카메라 광학 좌표로 바꾼다."""
    b = _pitch_up(_yaw_left(p_base, -pan_geo), -tilt_geo)
    b = (b[0] - cam_forward, b[1], b[2] - cam_up)
    return _body_to_optical(b)


def look_at(p_base, cam_up=0.0):
    """점이 영상 정중앙에 오는 기하 각도 (pan_geo, tilt_geo) [rad].

    광축이 점을 지나는 조건 r·sin(t) - z·cos(t) + cam_up = 0 의 해. cam_forward는 광축 방향이라 영향이 없다.
    """
    x, y, z = p_base
    r = math.hypot(x, y)
    big_r = math.hypot(r, z)
    pan = math.atan2(y, x)
    if big_r < 1e-6:
        return pan, 0.0
    return pan, math.atan2(z, r) - math.asin(clamp(cam_up / big_r, -1.0, 1.0))


def ray_point(ex, ey, range_m, hfov_rad, vfov_rad):
    """정규화 오차와 거리로 카메라 광학 좌표의 점을 만든다(깊이가 없을 때의 대체값)."""
    return (range_m * ex * math.tan(hfov_rad / 2), range_m * ey * math.tan(vfov_rad / 2), range_m)


def to_normalized(p_opt, hfov_rad, vfov_rad):
    """카메라 광학 좌표의 점이 영상에서 보일 정규화 위치 (ex, ey). 카메라 뒤면 None."""
    x, y, z = p_opt
    if z <= 1e-6:
        return None
    return (x / z) / math.tan(hfov_rad / 2), (y / z) / math.tan(vfov_rad / 2)


# ---------- 시간별 관절 각도 ----------
class JointHistory:
    """OpenCR 상태(또는 dry_run 시뮬레이션)의 모터 각도 [deg] 기록. 영상 촬영 시각의 자세를 보간한다."""

    def __init__(self, keep_s=2.0):
        self.keep_s = keep_s
        self.buf = deque()

    def add(self, t, pan_deg, tilt_deg):
        self.buf.append((t, pan_deg, tilt_deg))
        while self.buf and t - self.buf[0][0] > self.keep_s:
            self.buf.popleft()

    def latest(self):
        return self.buf[-1] if self.buf else None

    def at(self, t):
        if not self.buf:
            return None
        if t <= self.buf[0][0]:
            return self.buf[0][1:]
        prev = self.buf[0]
        for cur in self.buf:
            if cur[0] >= t:
                k = (t - prev[0]) / (cur[0] - prev[0]) if cur[0] > prev[0] else 0.0
                return (prev[1] + k * (cur[1] - prev[1]), prev[2] + k * (cur[2] - prev[2]))
            prev = cur
        return self.buf[-1][1:]


# ---------- 목표 기억과 예측 ----------
class TargetMemory:
    """기준 좌표에서 목표 위치를 기억하고 등속 가정으로 예측한다.

    카메라가 돌아가도 기준 좌표는 고정이므로, 목표가 시야 밖으로 나가거나 가려져도
    '어디쯤 있을지'를 계속 계산할 수 있다(SEARCHING의 근거).
    """

    def __init__(self, window_s=0.5, max_speed_m_s=0.5, horizon_s=1.0, max_age_s=3.0):
        self.window_s = window_s          # 속도 추정에 쓰는 최근 구간
        self.max_speed = max_speed_m_s    # 추정 속도 상한 (잡음 억제)
        self.horizon_s = horizon_s        # 이 시간 이후로는 더 외삽하지 않는다
        self.max_age_s = max_age_s        # 마지막 관측이 이보다 오래되면 기억을 쓰지 않는다
        self.obs = deque()
        self.velocity = (0.0, 0.0, 0.0)
        self.depth_valid = False

    def clear(self):
        self.obs.clear()
        self.velocity = (0.0, 0.0, 0.0)

    def add(self, t, p_base, depth_valid):
        self.obs.append((t, p_base))
        self.depth_valid = depth_valid
        while self.obs and t - self.obs[0][0] > self.window_s:
            self.obs.popleft()
        t0, p0 = self.obs[0]
        dt = t - t0
        if dt >= 0.5 * self.window_s:     # 구간의 절반 이상 쌓였을 때만 (고정 0.1 s면 window 0.1에서 속도가 항상 0)
            v = [(a - b) / dt for a, b in zip(p_base, p0)]
            speed = math.sqrt(sum(c * c for c in v))
            if speed > self.max_speed:
                v = [c * self.max_speed / speed for c in v]
            self.velocity = tuple(v)
        else:
            self.velocity = (0.0, 0.0, 0.0)

    def last_time(self):
        return self.obs[-1][0] if self.obs else None

    def predict(self, now):
        if not self.obs:
            return None
        t_last, p_last = self.obs[-1]
        age = now - t_last
        if age > self.max_age_s:
            return None
        h = clamp(age, 0.0, self.horizon_s)
        return tuple(p + v * h for p, v in zip(p_last, self.velocity))
