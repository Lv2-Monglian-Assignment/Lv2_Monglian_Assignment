"""파란색 목표 검출과 깊이·3D 좌표 계산 (ROS에 의존하지 않는 순수 함수).

흐름(docs/interface.md 2절):
  영상 -> (축소) -> HSV -> inRange 마스크 -> open·close 잡음 제거 -> findContours -> (좌표 원본 복원)
  -> 면적 >= min_area 후보
  -> (깊이가 있으면) 후보마다 깊이 중앙값 Z, 실제 면적 [cm^2], 카메라 좌표 X, Y, Z
  -> (size_check) 실제 면적이 물체 크기 범위 밖이면 제외
  -> 대상 선택(largest | lock | priority) -> moments 중심 -> 정규화 오차·면적비
  -> (기록) 정렬 Depth에서 목표 영역 중앙값 -> 유효성 판정 -> 핀홀 역투영 X, Y, Z

깊이 중앙값·유효 비율·작업 거리 범위·핀홀 역투영은 mouse_test.py(리얼센스 학습 예제)의 방식을 따른다.
mouse_test.py는 커서 주변 10x10 ROI를 썼고, 여기서는 목표 컨투어 내부(경계를 깎은 영역)를 쓴다.

축소 검출(detect_scale < 1)은 insightface SCRFD(Deep-Live-Cam이 쓰는 얼굴 검출기) detect()의 방식을 따른다:
작게 줄인 영상에서 검출하고, 좌표만 원본 크기로 되돌린다(scrfd.py의 `/ det_scale`). 영상 자체는 키우지 않으므로
깊이 계산·저장은 원본 해상도 그대로다.

깊이는 후보 '단위'로만 쓴다. 픽셀 단위로 마스크를 지우거나 한 덩어리를 깊이 차이로 쪼개지 않는다
(단차 있는 물체가 둘로 갈라지는 것 방지). 거리만으로 후보를 버리지도 않는다(먼 물체도 후보로 남김).
깊이를 모르는 후보(유효 픽셀 부족, 깊이 영상 없음)는 크기 검사 없이 남긴다.
"""
import math
from dataclasses import dataclass, field

import cv2
import numpy as np


@dataclass
class DetectorConfig:
    hsv_lower: tuple = (100, 120, 50)     # OpenCV HSV: H 0~179, S·V 0~255. 기본값은 단위 시험용, 실제 사용값은 config/hsv.yaml
    hsv_upper: tuple = (130, 255, 255)    # 파란색 #todo
    morph_kernel: int = 5                 # 잡음 제거 커널 [px] (open -> close)
    min_area_px: float = 150.0            # 후보 최소 면적 [px^2] #todo 거리 범위에 맞게
    selection: str = 'largest'            # largest | lock | priority (#todo 팀 합의)
    lock_gate_px: float = 80.0            # lock: 이전 중심에서 이 거리 안의 후보를 우선
    depth_min_m: float = 0.2              # mouse_test.py의 유효 작업 거리 (target_depth 기록용)
    depth_max_m: float = 3.0
    depth_min_valid_ratio: float = 0.3    # 목표 영역 중 유효 깊이 픽셀 비율 하한 (mouse_test.py: 30 %)
    depth_erode_px: int = 3               # 경계에서 배경 깊이가 섞이지 않게 깎는 폭 [px]
    detect_scale: float = 1.0             # 검출용 축소 배율 (1.0 = 원본, 0.5 = 320x240에서 검출) #todo Pi FPS 실측 후 결정
    # ---- 크기 검증 (깊이로 환산한 실제 보이는 면적) ----
    size_check: bool = False              # True면 실제 면적이 범위 밖인 후보를 제외 (깊이를 모르는 후보는 남김)
    obj_area_min_cm2: float = 5.5         # 이론 최소 7.07 (지름 3 cm 원 = 2.25*pi) x 약 0.78 #todo 실측
    obj_area_max_cm2: float = 30.0        # 이론 최대 27 (3x3x6 직육면체 세 면) x 약 1.1 #todo 실측
    # ---- priority 선택: 1 가장 큰 것 -> 2 가까운 것 -> 3 화면 중앙에 가까운 것 ----
    similar_area_ratio: float = 0.2       # |a1 - a2| / max(a1, a2) <= 0.2 이면 '크기가 비슷' -> 2번 기준으로
    similar_depth_m: float = 0.03         # |z1 - z2| <= max(0.03 m, 5 % x z) 이면 '거리가 비슷' -> 3번 기준으로
    similar_depth_ratio: float = 0.05


@dataclass
class Candidate:
    area_px: float                        # 컨투어 면적 [px^2] (원본 해상도 기준)
    cx: float                             # 중심 [px]
    cy: float
    contour: np.ndarray
    z_m: float = math.nan                 # 후보 깊이 중앙값 [m] (센서가 잰 픽셀만, 거리 범위로 거르지 않음)
    area_cm2: float = math.nan            # 실제 보이는 면적 [cm^2] = area_px x (Z/fx) x (Z/fy) x 1e4
    pos_cam: tuple = None                 # 카메라 광학 좌표 (X 오른쪽, Y 아래, Z 앞) [m], 깊이 무효면 None
    cut: bool = False                     # 화면 가장자리에 닿음 (잘려서 작게 보일 수 있음)


@dataclass
class Detection:
    detected: bool = False
    cx: float = math.nan                  # 목표 중심 [px]
    cy: float = math.nan
    ex: float = 0.0                       # 정규화 오차, 오른쪽 + (미검출이면 0)
    ey: float = 0.0                       # 정규화 오차, 아래 +
    area_px: float = 0.0
    area_ratio: float = 0.0               # contour_area / (W x H), 미검출이면 0
    n_candidates: int = 0
    contour: np.ndarray = None
    index: int = -1                       # 선택된 후보의 candidates 안 번호
    candidates: list = field(default_factory=list)   # [Candidate] 크기 검증을 통과한 후보 (큰 것부터)
    rejected: list = field(default_factory=list)     # [(Candidate, 사유)] 크기 검증에서 제외된 후보


@dataclass
class DepthResult:
    valid: bool = False
    z_m: float = math.nan                 # 광학축 방향 거리 [m]
    x_m: float = math.nan                 # 오른쪽 + [m]
    y_m: float = math.nan                 # 아래 + [m]
    valid_ratio: float = 0.0
    n_pixels: int = 0


def to_hsv(image, encoding):
    """rgb8 / bgr8 영상을 HSV로 바꾼다. 입력 인코딩을 확인하지 않으면 빨강·파랑이 뒤바뀐다."""
    if encoding == 'rgb8':
        return cv2.cvtColor(image, cv2.COLOR_RGB2HSV)
    if encoding == 'bgr8':
        return cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    raise ValueError(f'unsupported color encoding: {encoding}')


def make_mask(hsv, cfg: DetectorConfig, scale=1.0):
    mask = cv2.inRange(hsv, np.array(cfg.hsv_lower, np.uint8), np.array(cfg.hsv_upper, np.uint8))
    if cfg.morph_kernel > 1:
        ks = max(1, int(round(cfg.morph_kernel * scale)))  # 축소 영상에서는 커널도 같은 비율로 줄인다
        ks |= 1                                             # 홀수로 맞춤: 짝수 커널은 중심이 없어 마스크가 한쪽으로 밀린다
        k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (ks, ks))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k)    # 작은 점 잡음 제거
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k)   # 물체 안의 작은 구멍 메우기
    return mask


def restore_contour(c, sx, sy):
    """축소 영상의 컨투어 좌표를 원본 픽셀 좌표로 되돌린다 (scrfd.py detect()의 `/ det_scale`에 해당).

    픽셀 중심 기준으로 변환한다: 원본 x = (축소 x + 0.5) / sx - 0.5
    """
    pts = c.astype(np.float32)
    pts[..., 0] = (pts[..., 0] + 0.5) / sx - 0.5
    pts[..., 1] = (pts[..., 1] + 0.5) / sy - 0.5
    return np.round(pts).astype(np.int32)                # drawContours(깊이 영역·표시)가 정수 좌표를 요구


def find_candidates(mask, cfg: DetectorConfig, sx=1.0, sy=1.0):
    """후보 목록 (큰 것부터). 면적·중심은 항상 원본 해상도 기준이라 min_area_px 값을 바꿀 필요가 없다."""
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cands = []
    for c in contours:
        if sx != 1.0 or sy != 1.0:
            c = restore_contour(c, sx, sy)
        area = cv2.contourArea(c)
        if area < cfg.min_area_px:
            continue
        m = cv2.moments(c)
        if m['m00'] <= 0:
            continue
        cands.append(Candidate(area, m['m10'] / m['m00'], m['m01'] / m['m00'], c))
    cands.sort(key=lambda t: t.area_px, reverse=True)
    return cands


def region_meters(depth, contour, cfg: DetectorConfig, depth_scale_m):
    """컨투어 내부(경계를 depth_erode_px 깎은 영역)의 깊이 [m] 배열. 계산은 외접 사각형 주변만 잘라서 한다."""
    hgt, wid = depth.shape[:2]
    x, y, w, h = cv2.boundingRect(contour)
    pad = cfg.depth_erode_px + 1                          # 깎기가 사각형 경계에서 잘리지 않도록 여유
    x0, y0 = max(0, x - pad), max(0, y - pad)
    x1, y1 = min(wid, x + w + pad), min(hgt, y + h + pad)
    if x1 <= x0 or y1 <= y0:
        return np.empty(0)
    region = np.zeros((y1 - y0, x1 - x0), np.uint8)
    cv2.drawContours(region, [contour], -1, 255, thickness=-1, offset=(-x0, -y0))
    if cfg.depth_erode_px > 0:
        eroded = cv2.erode(region, np.ones((3, 3), np.uint8), iterations=cfg.depth_erode_px)
        if cv2.countNonZero(eroded) > 0:                  # 너무 작으면 깎지 않은 영역 사용
            region = eroded
    return depth[y0:y1, x0:x1][region > 0].astype(np.float64) * depth_scale_m


def candidate_depth(depth, contour, cfg: DetectorConfig, depth_scale_m):
    """후보 하나의 대표 깊이 [m]: 센서가 잰 픽셀(0 아님)의 중앙값. 유효 비율이 부족하면 nan.

    target_depth()와 달리 depth_min_m~depth_max_m 범위로 거르지 않는다(먼 물체도 거리를 알아야 크기를 판단).
    """
    m = region_meters(depth, contour, cfg, depth_scale_m)
    if m.size == 0:
        return math.nan
    ok = np.isfinite(m) & (m > 0)
    if np.count_nonzero(ok) / m.size < cfg.depth_min_valid_ratio:
        return math.nan
    return float(np.median(m[ok]))


def measure_candidates(cands, depth, depth_scale_m, intrinsics, cfg: DetectorConfig, w, h):
    """후보마다 깊이·실제 면적·카메라 좌표를 채운다. intrinsics = (fx, fy, ppx, ppy)"""
    for c in cands:
        bx, by, bw, bh = cv2.boundingRect(c.contour)
        c.cut = bx <= 0 or by <= 0 or bx + bw >= w or by + bh >= h
        if depth is None:
            continue
        c.z_m = candidate_depth(depth, c.contour, cfg, depth_scale_m)
        if not math.isfinite(c.z_m) or intrinsics is None:
            continue
        fx, fy, ppx, ppy = intrinsics
        c.area_cm2 = c.area_px * (c.z_m / fx) * (c.z_m / fy) * 1e4
        c.pos_cam = ((c.cx - ppx) * c.z_m / fx, (c.cy - ppy) * c.z_m / fy, c.z_m)


def size_filter(cands, cfg: DetectorConfig):
    """실제 면적이 물체 크기 범위 밖이면 제외. 반환 (남은 후보, [(후보, 사유)]).

    - 면적은 기울어짐에 강하다: 3x3x6 cm 물체를 어느 방향에서 봐도 보이는 면적은 7.07~27 cm^2 사이.
      (길이는 대각선 7.3 cm까지 길어지지만 길이로는 판정하지 않는다)
    - 화면 가장자리에 잘린 물체는 작게 보일 뿐이므로 하한 검사를 건너뛴다.
    - 깊이를 모르는 후보는 판정할 수 없으므로 남긴다.
    """
    kept, rejected = [], []
    for c in cands:
        a = c.area_cm2
        if not math.isfinite(a):
            kept.append(c)
        elif a > cfg.obj_area_max_cm2:
            rejected.append((c, f'big {a:.1f}cm2'))
        elif a < cfg.obj_area_min_cm2 and not c.cut:
            rejected.append((c, f'small {a:.1f}cm2'))
        else:
            kept.append(c)
    return kept, rejected


def depth_similar(z1, z2, cfg: DetectorConfig):
    return abs(z1 - z2) <= max(cfg.similar_depth_m, cfg.similar_depth_ratio * min(z1, z2))


def select_priority(cands, cfg: DetectorConfig, w, h):
    """1 가장 큰 것 -> 2 가까운 것 -> 3 화면 중앙에 가까운 것. 앞 기준에서 '비슷'하면 다음 기준으로 넘어간다.

    1단계: 가장 큰 후보와 면적이 similar_area_ratio 이내로 비슷한 후보들만 남긴다 (1개면 결정).
    2단계: 그중 가장 가까운 후보와 거리가 비슷한 후보들만 남긴다 (깊이를 모르는 후보는 비교 불가 -> 그대로 남김).
    3단계: 남은 후보 중 화면 중앙(정규화 반지름 ex^2 + ey^2)에 가장 가까운 것.
    반환: cands 안의 번호 (없으면 None)
    """
    if not cands:
        return None
    idx = list(range(len(cands)))
    a_max = max(cands[i].area_px for i in idx)
    idx = [i for i in idx if (a_max - cands[i].area_px) / a_max <= cfg.similar_area_ratio]
    if len(idx) > 1:
        known = [i for i in idx if math.isfinite(cands[i].z_m)]
        if known:
            z_min = min(cands[i].z_m for i in known)
            near = [i for i in known if depth_similar(cands[i].z_m, z_min, cfg)]
            idx = near + [i for i in idx if i not in known]
    if len(idx) == 1:
        return idx[0]

    def r2(i):
        ex = (cands[i].cx - w / 2) / (w / 2)
        ey = (cands[i].cy - h / 2) / (h / 2)
        return ex * ex + ey * ey
    return min(idx, key=r2)


def select_candidate(cands, cfg: DetectorConfig, prev_center=None, w=None, h=None):
    """여러 파란 물체 중 하나를 고른다.

    largest : 가장 큰 후보 (architecture 기본안)
    lock    : 직전에 쫓던 중심과 가까운 후보를 우선(다른 파란 물체로 갈아타기 방지), 없으면 largest
              이전 '좌표를 다시 발행'하는 것이 아니라, 현재 영상의 후보 중에서 고르는 데만 쓴다.
    priority: select_priority() (크기 -> 거리 -> 화면 중앙). 번호(ID) 유지는 object_tracker가 맡는다.
    """
    if not cands:
        return None
    if cfg.selection == 'priority' and w is not None and h is not None:
        return select_priority(cands, cfg, w, h)
    if cfg.selection == 'lock' and prev_center is not None:
        d = [math.hypot(c.cx - prev_center[0], c.cy - prev_center[1]) for c in cands]
        i = int(np.argmin(d))
        if d[i] <= cfg.lock_gate_px:
            return i
    return 0


def find_and_measure(image, encoding, cfg: DetectorConfig, depth=None, depth_scale_m=0.001, intrinsics=None):
    """검출의 앞부분: 마스크 -> 후보 -> (깊이) 측정 -> (크기) 검증. 반환 (후보, 제외 목록, 원본 크기 마스크)."""
    h, w = image.shape[:2]
    s = cfg.detect_scale
    if 0.0 < s < 1.0:
        sw, sh = max(1, int(round(w * s))), max(1, int(round(h * s)))
        small = cv2.resize(image, (sw, sh), interpolation=cv2.INTER_AREA)   # scrfd.py: cv2.resize 후 검출
        sx, sy = sw / w, sh / h                          # 실제 배율 (scrfd.py의 det_scale)
    else:
        small, sx, sy = image, 1.0, 1.0
    mask = make_mask(to_hsv(small, encoding), cfg, sx)
    cands = find_candidates(mask, cfg, sx, sy)
    if sx != 1.0 or sy != 1.0:
        mask = cv2.resize(mask, (w, h), interpolation=cv2.INTER_NEAREST)
    if depth is not None and depth.shape[:2] != (h, w):
        depth = None                                     # 정렬되지 않은 깊이는 쓰지 않는다
    measure_candidates(cands, depth, depth_scale_m, intrinsics, cfg, w, h)
    rejected = []
    if cfg.size_check:
        cands, rejected = size_filter(cands, cfg)
    return cands, rejected, mask


def make_detection(cands, rejected, i, w, h):
    """선택 결과 i(없으면 None)로 Detection을 만든다. 발제 문제 1의 ex·ey·면적비 정의."""
    det = Detection(n_candidates=len(cands), candidates=cands, rejected=rejected)
    if i is None:
        return det                                       # 미검출: ex=ey=area=0
    c = cands[i]
    det.detected, det.index = True, i
    det.cx, det.cy, det.contour, det.area_px = c.cx, c.cy, c.contour, c.area_px
    det.ex = (c.cx - w / 2) / (w / 2)
    det.ey = (c.cy - h / 2) / (h / 2)
    det.area_ratio = c.area_px / (w * h)
    return det


def detect(image, encoding, cfg: DetectorConfig, prev_center=None, depth=None, depth_scale_m=0.001,
           intrinsics=None):
    """한 프레임 검출. (Detection, mask) 반환. W·H는 실제 프레임 크기를 쓴다.

    대상 구분은 색(HSV)·면적과, 깊이가 있으면 실제 크기(size_check)로 한다.
    detect_scale < 1이면 축소 영상에서 마스크·컨투어를 구하고 좌표만 원본으로 되돌린다.
    반환하는 mask는 저장·표시용으로 항상 원본 크기다.
    """
    h, w = image.shape[:2]
    cands, rejected, mask = find_and_measure(image, encoding, cfg, depth, depth_scale_m, intrinsics)
    i = select_candidate(cands, cfg, prev_center, w, h)
    return make_detection(cands, rejected, i, w, h), mask


def target_depth(depth, contour, cfg: DetectorConfig, depth_scale_m, intrinsics=None, center=None):
    """정렬 Depth에서 목표 영역의 깊이 중앙값과 3D 좌표 (기록·/target/position_cam용).

    depth       : 컬러에 정렬된 깊이 영상 (16UC1 mm -> depth_scale_m=0.001, 32FC1 m -> 1.0)
    intrinsics  : (fx, fy, cx, cy)  CameraInfo K. 없으면 Z만 계산
    center      : 역투영할 픽셀 (u, v) = 목표 중심
    """
    res = DepthResult()
    if depth is None or contour is None:
        return res
    meters = region_meters(depth, contour, cfg, depth_scale_m)
    res.n_pixels = int(meters.size)
    if meters.size == 0:
        return res
    ok = np.isfinite(meters) & (meters > 0) & (meters >= cfg.depth_min_m) & (meters <= cfg.depth_max_m)
    res.valid_ratio = float(np.count_nonzero(ok)) / meters.size
    if res.valid_ratio < cfg.depth_min_valid_ratio:
        return res                                       # 무효 깊이: 유효 입력으로 처리하지 않는다
    res.valid = True
    res.z_m = float(np.median(meters[ok]))
    if intrinsics is not None and center is not None:
        fx, fy, ppx, ppy = intrinsics
        res.x_m = (center[0] - ppx) * res.z_m / fx      # mouse_test.py의 핀홀 역투영
        res.y_m = (center[1] - ppy) * res.z_m / fy
    return res


def draw_overlay(image_bgr, det: Detection, dres: DepthResult = None, ids=None):
    """원본 위에 후보·선택 컨투어, 목표 중심, 영상 중심을 그린다 (발제 문제 1 결과물).

    ids: {후보 번호: 물체 ID} (object_tracker 결과, 있으면 후보 옆에 표시)
    """
    out = image_bgr.copy()
    h, w = out.shape[:2]
    for c, reason in det.rejected:                                # 크기 검증 제외: 회색 + 사유
        cv2.drawContours(out, [c.contour], -1, (128, 128, 128), 1)
        cv2.putText(out, reason, (int(c.cx), int(c.cy)), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (128, 128, 128), 1)
    for k, c in enumerate(det.candidates):
        cv2.drawContours(out, [c.contour], -1, (0, 200, 255), 1)          # 후보: 주황
        if ids and k in ids:
            cv2.putText(out, f'#{ids[k]}', (int(c.cx) + 6, int(c.cy) - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                        (0, 200, 255), 1)
    cv2.drawMarker(out, (w // 2, h // 2), (255, 255, 255), cv2.MARKER_CROSS, 24, 1)   # 영상 중심
    if det.detected:
        cv2.drawContours(out, [det.contour], -1, (0, 255, 0), 2)  # 선택: 초록
        p = (int(round(det.cx)), int(round(det.cy)))
        cv2.drawMarker(out, p, (0, 0, 255), cv2.MARKER_TILTED_CROSS, 16, 2)
        cv2.line(out, (w // 2, h // 2), p, (0, 0, 255), 1)
        txt = f'ex {det.ex:+.3f} ey {det.ey:+.3f} area {det.area_ratio:.4f}'
        if dres is not None:
            txt += f'  Z {dres.z_m:.3f} m' if dres.valid else '  Z invalid'
    else:
        txt = 'NO TARGET'
    cv2.rectangle(out, (0, 0), (w, 22), (0, 0, 0), -1)
    cv2.putText(out, f'{txt}  cand {det.n_candidates} rej {len(det.rejected)}', (6, 16),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    return out
