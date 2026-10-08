"""같은 색 물체 여러 개에 번호(ID)를 붙여 프레임 사이에서 유지한다 (ROS에 의존하지 않는 순수 로직).

방식: 새로 검출한 물체마다 '회전축 기준 3D 위치 + 실제 면적'이 가장 가까운 기존 물체를 찾아 같은 번호를 붙인다
  (최근접 대응). 가까운 물체가 없으면 새 번호를 준다.

좌표: 모든 위치는 회전축 기준 좌표(pan_tilt_base, tracker_controller/geometry.py의 cam_to_base 결과)로 받는다.
  카메라는 위치가 고정이고 회전만 하므로, 촬영 순간의 모터 각도만큼 되돌리면 가만히 있는 물체의 좌표는
  카메라가 돌아도 변하지 않는다. (카메라 기준 좌표로 기억하면 카메라가 돌 때 물체 좌표도 같이 바뀐다)

동작:
  - 보이는 물체: 예측 위치(마지막 위치 + 속도 x 경과 시간)와 가장 가까운 관측을 같은 번호로 잇는다.
  - 속도: 최근 velocity_window_s(기본 0.1 s = 30 fps에서 약 3 프레임) 동안의 이동 거리 / 경과 시간.
  - 안 보이는 물체: 번호와 마지막 위치·크기를 프로그램이 끝날 때까지 기억한다(재시작하면 초기화).
    비슷한 크기의 물체가 비슷한 위치에서 다시 보이면 같은 번호를 다시 붙인다.
  - 기억은 번호 붙이기에만 쓴다. 안 보이는 물체의 기억 위치를 /target으로 내보내지 않는다
    (발제 문제 1: 미검출 때 이전 좌표를 새 목표로 재사용하지 않는다).
  - 빠른 회전 중(fast_rotation_deg_s 이상)에는 영상 흐림·시각 오차가 커서 허용 반경을 넓히고,
    새 번호 만들기와 크기 기억 갱신을 멈춘다. (#todo 실측 후 유지 여부 결정)
"""
import math
from collections import deque
from dataclasses import dataclass, field


@dataclass
class TrackerConfig:
    gate_m: float = 0.04               # 보이는 물체 매칭 허용 반경 [m] (물체 폭 3 cm + 깊이·중심 흔들림) #todo
    reid_gate_m: float = 0.06          # 안 보이던 물체를 다시 찾을 때 허용 반경 [m] #todo
    size_ratio_max: float = 1.6        # 다시 찾을 때 면적 비 상한 (큰 쪽 / 작은 쪽). 기울기 변화 여유 #todo
    visible_age_s: float = 0.3         # 마지막 관측이 이보다 최근이면 '보이는 물체'로 취급 [s]
    velocity_window_s: float = 0.1     # 속도 추정 구간 [s]. 0.1 또는 0.25 (기존 0.5 s의 약수) #todo
    max_speed_m_s: float = 1.0         # 추정 속도 상한 [m/s] (잡음으로 튀는 속도 억제)
    predict_horizon_s: float = 0.5     # 이 시간 이후로는 속도로 외삽하지 않고 마지막 위치에 멈춘다 [s]
    assumed_range_m: float = 0.6       # 깊이를 모를 때 방향 차이 [rad]를 거리로 환산하는 거리 [m]
    sync_error_s: float = 0.02         # 영상 시각과 모터 각도 시각의 예상 오차 [s] #todo
    fast_rotation_deg_s: float = 30.0  # 이보다 빠르게 돌면 새 번호·크기 갱신 중지 [deg/s]. 0이면 끔 #todo
    max_tracks: int = 50               # 기억할 물체 수 상한 (넘으면 가장 오래 안 보인 것부터 지움)


@dataclass
class Observation:
    """이번 프레임의 후보 하나 (회전축 기준)."""
    bearing: tuple                     # 단위 방향 벡터 (깊이가 없어도 항상 있음)
    pos: tuple = None                  # 3D 위치 [m] (깊이 무효면 None)
    area_cm2: float = math.nan         # 실제 보이는 면적 (모르면 nan)


@dataclass
class Track:
    id: int
    bearing: tuple
    pos: tuple
    area_cm2: float
    first_seen: float
    last_seen: float
    hits: int = 1
    velocity: tuple = (0.0, 0.0, 0.0)
    history: deque = field(default_factory=deque)   # [(t, pos)] 속도 계산용


def _sub(a, b):
    return tuple(x - y for x, y in zip(a, b))


def _norm(a):
    return math.sqrt(sum(x * x for x in a))


def _angle(a, b):
    d = sum(x * y for x, y in zip(a, b)) / max(_norm(a) * _norm(b), 1e-12)
    return math.acos(max(-1.0, min(1.0, d)))


class ObjectTracker:
    def __init__(self, cfg: TrackerConfig = None):
        self.cfg = cfg or TrackerConfig()
        self.tracks = {}
        self.next_id = 1

    # ---------- 예측과 거리 ----------
    def predict(self, tr: Track, t):
        """t 시각의 예상 위치. 안 보인 지 predict_horizon_s가 지나면 마지막 위치에 멈춘다."""
        if tr.pos is None:
            return None
        h = min(max(0.0, t - tr.last_seen), self.cfg.predict_horizon_s)
        return tuple(p + v * h for p, v in zip(tr.pos, tr.velocity))

    def distance(self, tr: Track, ob: Observation, t):
        """기억과 관측 사이 거리 [m]. 둘 다 3D 위치가 있으면 직선거리, 아니면 방향 차이 x 가정 거리."""
        pred = self.predict(tr, t)
        if pred is not None and ob.pos is not None:
            return _norm(_sub(pred, ob.pos))
        return _angle(tr.bearing, ob.bearing) * self.cfg.assumed_range_m

    def size_ok(self, tr: Track, ob: Observation):
        a, b = tr.area_cm2, ob.area_cm2
        if not (math.isfinite(a) and math.isfinite(b)) or a <= 0 or b <= 0:
            return True                                  # 모르면 크기로는 막지 않는다
        return max(a, b) / min(a, b) <= self.cfg.size_ratio_max

    # ---------- 갱신 ----------
    def update(self, t, observations, rot_speed_deg_s=0.0):
        """한 프레임의 관측으로 번호를 갱신한다. 반환: {관측 번호: 물체 ID} (빠른 회전 중 새 물체는 번호 없음)."""
        cfg = self.cfg
        fast = cfg.fast_rotation_deg_s > 0 and abs(rot_speed_deg_s) >= cfg.fast_rotation_deg_s   # 0이면 끔
        rot_extra = math.radians(abs(rot_speed_deg_s)) * cfg.sync_error_s   # [rad] 시각 오차로 생기는 방향 오차

        pairs = []
        for tid, tr in self.tracks.items():
            visible = t - tr.last_seen <= cfg.visible_age_s
            for k, ob in enumerate(observations):
                rng = ob.pos and _norm(ob.pos) or cfg.assumed_range_m
                if visible:
                    speed = _norm(tr.velocity)
                    gate = cfg.gate_m + speed * (t - tr.last_seen) + rot_extra * rng
                else:
                    if not self.size_ok(tr, ob):
                        continue
                    gate = cfg.reid_gate_m + rot_extra * rng
                if fast:
                    gate *= 2.0                          # 빠른 회전: 흐림·롤링 셔터로 중심이 더 흔들림
                d = self.distance(tr, ob, t)
                if d <= gate:
                    # 보이는 물체를 먼저 잇고(0), 그다음 기억 속 물체(1). 같은 그룹 안에서는 가까운 순.
                    pairs.append((0 if visible else 1, d, tid, k))
        pairs.sort()

        assigned, used = {}, set()
        for _, _, tid, k in pairs:                       # 가까운 짝부터 1:1로 확정 (탐욕 매칭)
            if k in assigned or tid in used:
                continue
            assigned[k] = tid
            used.add(tid)
            self._apply(self.tracks[tid], observations[k], t, update_size=not fast)

        for k, ob in enumerate(observations):            # 짝이 없는 관측 = 새 물체
            if k in assigned or fast:
                continue
            tid = self.next_id
            self.next_id += 1
            tr = Track(tid, ob.bearing, ob.pos, ob.area_cm2, t, t)
            if ob.pos is not None:
                tr.history.append((t, ob.pos))
            self.tracks[tid] = tr
            assigned[k] = tid
        self._trim()
        return assigned

    def _apply(self, tr: Track, ob: Observation, t, update_size=True):
        tr.bearing = ob.bearing
        tr.last_seen = t
        tr.hits += 1
        if update_size and math.isfinite(ob.area_cm2):
            tr.area_cm2 = ob.area_cm2
        if ob.pos is None:                               # 깊이 없음: 위치·속도는 그대로 둔다
            return
        tr.pos = ob.pos
        tr.history.append((t, ob.pos))
        while tr.history and t - tr.history[0][0] > self.cfg.velocity_window_s:
            tr.history.popleft()
        t0, p0 = tr.history[0]
        dt = t - t0
        if dt >= 0.5 * self.cfg.velocity_window_s:       # 구간의 절반 이상 쌓였을 때만 속도 계산
            v = tuple(c / dt for c in _sub(ob.pos, p0))
            s = _norm(v)
            if s > self.cfg.max_speed_m_s:
                v = tuple(c * self.cfg.max_speed_m_s / s for c in v)
            tr.velocity = v
        elif len(tr.history) == 1:
            tr.velocity = (0.0, 0.0, 0.0)

    def _trim(self):
        if len(self.tracks) <= self.cfg.max_tracks:
            return
        for tid in sorted(self.tracks, key=lambda i: self.tracks[i].last_seen)[:len(self.tracks) - self.cfg.max_tracks]:
            del self.tracks[tid]

    def visible_ids(self, t):
        return [tid for tid, tr in self.tracks.items() if t - tr.last_seen <= self.cfg.visible_age_s]


class TargetLock:
    """/target으로 내보낼 물체를 정하고, 한 번 잡은 번호를 놓칠 때까지 유지한다.

    - 잡은 번호가 이번 프레임에 보이면 그 물체를 내보낸다.
    - 잡은 번호가 안 보이면 미검출(None)을 돌려준다 -> /target z=0 -> 제어는 정지(LOST).
      다른 파란 물체로 바로 갈아타지 않는다(발제 도전 D: 잘못된 목표 추적 방지, 2초 가림 후 재등장 시험).
    - 잡은 번호를 relock_after_s 이상 못 보면 그때 우선순위 규칙으로 새 물체를 고른다. #todo 팀 합의
    """

    def __init__(self, relock_after_s=3.0):
        self.relock_after_s = relock_after_s
        self.locked_id = None

    def choose(self, t, ids, tracker: ObjectTracker, select_fn):
        """ids: {후보 번호: 물체 ID}, select_fn(): 우선순위로 고른 후보 번호(없으면 None). 반환: 후보 번호 또는 None"""
        for k, tid in ids.items():
            if tid == self.locked_id:
                return k
        tr = tracker.tracks.get(self.locked_id)
        if self.locked_id is not None and tr is not None and t - tr.last_seen <= self.relock_after_s:
            return None                                  # 잡은 물체를 기다리는 중: 미검출로 처리
        k = select_fn()
        self.locked_id = ids.get(k) if k is not None else None
        return k
