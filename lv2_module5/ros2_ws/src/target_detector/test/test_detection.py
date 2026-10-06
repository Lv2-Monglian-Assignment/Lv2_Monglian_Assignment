"""ROS 없이 검출 규칙을 시험한다: python3 -m pytest test/"""
import math
import numpy as np
from target_detector.detection import DetectorConfig, detect, target_depth

K = (615.0, 615.0, 320.0, 240.0)                            # fx, fy, ppx, ppy (D435 640x480 근사)

BLUE_RGB, RED_RGB = (20, 60, 220), (220, 30, 30)
W, H = 640, 480


def scene(boxes, enc='rgb8'):
    img = np.full((H, W, 3), 170, np.uint8)                 # 회색 배경
    for (x0, y0, x1, y1), rgb in boxes:
        img[y0:y1, x0:x1] = rgb if enc == 'rgb8' else rgb[::-1]
    return img


def test_largest_blue_and_sign():
    img = scene([((400, 200, 460, 320), BLUE_RGB),          # 오른쪽 큰 파랑 (60x120)
                 ((100, 100, 130, 160), BLUE_RGB),          # 왼쪽 작은 파랑
                 ((250, 250, 330, 400), RED_RGB)])          # 빨강은 무시
    det, mask = detect(img, 'rgb8', DetectorConfig())
    assert det.detected and det.n_candidates == 2
    assert abs(det.cx - 429.5) < 1 and abs(det.cy - 259.5) < 1
    assert det.ex > 0 and det.ey > 0                        # 오른쪽·아래 = 양수
    assert abs(det.ex - (429.5 - 320) / 320) < 0.01
    assert abs(det.area_ratio - 59 * 119 / (W * H)) < 0.001  # 컨투어 면적 기준


def test_bgr_input_same_result():
    a, _ = detect(scene([((400, 200, 460, 320), BLUE_RGB)], 'rgb8'), 'rgb8', DetectorConfig())
    b, _ = detect(scene([((400, 200, 460, 320), BLUE_RGB)], 'bgr8'), 'bgr8', DetectorConfig())
    assert a.detected and b.detected and abs(a.cx - b.cx) < 1e-6


def test_no_target_reports_zero():
    det, _ = detect(scene([((250, 250, 330, 400), RED_RGB)]), 'rgb8', DetectorConfig())
    assert not det.detected and (det.ex, det.ey, det.area_ratio) == (0.0, 0.0, 0.0)


def test_small_noise_rejected():
    det, _ = detect(scene([((300, 300, 306, 306), BLUE_RGB)]), 'rgb8', DetectorConfig())
    assert not det.detected


def test_lock_prefers_previous_target():
    img = scene([((400, 200, 460, 320), BLUE_RGB), ((100, 100, 140, 180), BLUE_RGB)])
    cfg = DetectorConfig(selection='lock')
    det, _ = detect(img, 'rgb8', cfg, prev_center=(118, 138))
    assert abs(det.cx - 119.5) < 1                          # 작아도 이전에 쫓던 물체
    det, _ = detect(img, 'rgb8', cfg, prev_center=(300, 400))
    assert abs(det.cx - 429.5) < 1                          # 근처에 없으면 가장 큰 것


def test_half_scale_detection_matches_full():
    img = scene([((400, 200, 460, 320), BLUE_RGB), ((100, 100, 130, 160), BLUE_RGB)])
    full, _ = detect(img, 'rgb8', DetectorConfig())
    half, mask = detect(img, 'rgb8', DetectorConfig(detect_scale=0.5))
    assert half.detected and half.n_candidates == full.n_candidates
    assert abs(half.cx - full.cx) < 1.5 and abs(half.cy - full.cy) < 1.5   # 좌표는 원본 기준으로 복원
    assert abs(half.area_px - full.area_px) / full.area_px < 0.05
    assert mask.shape == (H, W)                                            # 저장용 마스크는 원본 크기


def test_half_scale_keeps_min_area_in_full_pixels():
    # 20x20 = 약 361 px^2 (원본 기준). min_area 300이면 통과, 450이면 제외 — 축소해도 같은 판정
    img = scene([((300, 300, 320, 320), BLUE_RGB)])
    assert detect(img, 'rgb8', DetectorConfig(min_area_px=300, detect_scale=0.5))[0].detected
    assert not detect(img, 'rgb8', DetectorConfig(min_area_px=450, detect_scale=0.5))[0].detected


def px_side(z, cm2):
    """거리 z [m]에서 실제 면적 cm2인 정사각형의 화면상 한 변 [px]"""
    return int(round(math.sqrt(cm2 * 1e-4) * 615 / z))


def test_size_check_rejects_big_blue_keeps_object_sized():
    # 0.5 m의 물체(약 16 cm^2) + 1.5 m 뒤의 큰 파란 배경(약 400 cm^2, 파란 옷·상자)
    s_obj, s_big = px_side(0.5, 16), px_side(1.5, 400)
    img = scene([((100, 200, 100 + s_obj, 200 + s_obj), BLUE_RGB), ((300, 100, 300 + s_big, 100 + s_big), BLUE_RGB)])
    depth = np.full((H, W), 2500, np.uint16)
    depth[200:200 + s_obj, 100:100 + s_obj] = 500
    depth[100:100 + s_big, 300:300 + s_big] = 1500
    off, _ = detect(img, 'rgb8', DetectorConfig(selection='largest'), depth=depth, intrinsics=K)
    assert off.cx > 300                                       # 크기 검증이 없으면 큰 배경을 고른다
    on, _ = detect(img, 'rgb8', DetectorConfig(selection='largest', size_check=True), depth=depth, intrinsics=K)
    assert on.detected and on.cx < 200 and len(on.rejected) == 1
    assert 14 < on.candidates[0].area_cm2 < 18                # 거리로 환산한 실제 면적


def test_size_check_keeps_far_object_and_unknown_depth():
    # 거리만으로는 버리지 않는다: 2.0 m의 물체도 크기가 맞으면 남는다. 깊이가 없으면(0) 판정 없이 남긴다
    s = px_side(2.0, 16)
    img = scene([((100, 100, 100 + s, 100 + s), BLUE_RGB), ((400, 300, 430, 330), BLUE_RGB)])
    depth = np.zeros((H, W), np.uint16)
    depth[100:100 + s, 100:100 + s] = 2000
    det, _ = detect(img, 'rgb8', DetectorConfig(min_area_px=100, size_check=True), depth=depth, intrinsics=K)
    assert det.n_candidates == 2 and not det.rejected
    assert any(math.isnan(c.area_cm2) for c in det.candidates)


def test_size_check_skips_lower_bound_for_cut_objects():
    s = px_side(0.4, 3)                                       # 3 cm^2: 하한(5.5) 미만
    img = scene([((0, 200, s, 200 + s), BLUE_RGB), ((300, 200, 300 + s, 200 + s), BLUE_RGB)])
    depth = np.full((H, W), 400, np.uint16)
    det, _ = detect(img, 'rgb8', DetectorConfig(min_area_px=50, size_check=True), depth=depth, intrinsics=K)
    assert det.n_candidates == 1 and det.candidates[0].cx < 50   # 화면 왼쪽 끝에 잘린 것만 남음
    assert len(det.rejected) == 1


def test_measured_cylinder_areas_fall_in_range():
    # CLAUDE.md 실측 (거리, 면적 px^2) -> 실제 면적이 거리와 상관없이 약 15 cm^2이고 범위 5.5~30 안
    for z, a in [(0.20, 14549), (0.45, 3024), (0.765, 954), (0.83, 920), (1.36, 296)]:
        cm2 = a * (z / 615) ** 2 * 1e4
        assert 13 < cm2 < 17 and 5.5 < cm2 < 30


def test_priority_largest_then_nearest_then_center():
    cfg = DetectorConfig(selection='priority', min_area_px=50)
    # 1) 크기가 확실히 다르면 큰 것
    img = scene([((100, 100, 160, 160), BLUE_RGB), ((400, 300, 430, 330), BLUE_RGB)])
    assert detect(img, 'rgb8', cfg)[0].cx < 200
    # 2) 크기가 비슷(20 % 이내)하면 가까운 것
    img = scene([((100, 100, 160, 160), BLUE_RGB), ((400, 300, 458, 358), BLUE_RGB)])
    depth = np.full((H, W), 2500, np.uint16)
    depth[100:160, 100:160] = 900
    depth[300:358, 400:458] = 500
    assert detect(img, 'rgb8', cfg, depth=depth, intrinsics=K)[0].cx > 400
    # 3) 크기·거리 모두 비슷(3 cm 이내)하면 화면 중앙에 가까운 것
    depth[300:358, 400:458] = 880
    assert detect(img, 'rgb8', cfg, depth=depth, intrinsics=K)[0].cx > 400   # (429,329)가 (129,129)보다 중앙에 가까움
    img2 = scene([((40, 40, 100, 100), BLUE_RGB), ((300, 210, 358, 268), BLUE_RGB)])
    depth2 = np.full((H, W), 600, np.uint16)
    assert abs(detect(img2, 'rgb8', cfg, depth=depth2, intrinsics=K)[0].cx - 328.5) < 1.5


def test_depth_median_and_backproject():
    img = scene([((400, 200, 460, 320), BLUE_RGB)])
    det, _ = detect(img, 'rgb8', DetectorConfig())
    depth = np.full((H, W), 2500, np.uint16)                # 배경 2.5 m
    depth[200:320, 400:460] = 600                           # 물체 0.6 m
    depth[200:320, 400:403] = 0                             # 경계 일부 측정 실패
    r = target_depth(depth, det.contour, DetectorConfig(), 0.001, (615.0, 615.0, 320.0, 240.0), (det.cx, det.cy))
    assert r.valid and abs(r.z_m - 0.6) < 1e-9
    assert abs(r.x_m - (det.cx - 320) * 0.6 / 615) < 1e-9 and r.y_m > 0


def test_depth_invalid_when_mostly_zero():
    img = scene([((400, 200, 460, 320), BLUE_RGB)])
    det, _ = detect(img, 'rgb8', DetectorConfig())
    depth = np.zeros((H, W), np.uint16)
    depth[250:255, 420:425] = 600                           # 유효 픽셀이 30 % 미만
    r = target_depth(depth, det.contour, DetectorConfig(), 0.001)
    assert not r.valid and math.isnan(r.z_m)

