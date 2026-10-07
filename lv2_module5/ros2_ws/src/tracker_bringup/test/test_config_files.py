"""lv2_module5/config/*.yaml 점검 (ROS 없이 실행: python3 -m pytest).

launch는 config/*.yaml을 모든 노드에 함께 넘긴다. 그래서 같은 노드의 같은 키가 두 파일에 있으면
파일 순서에 따라 값이 덮어써진다 → 중복이 없어야 한다. 값은 팀 결정·실측과 맞는지 본다.
"""
import collections
import math
import os

import yaml

CONFIG = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', '..', '..', 'config')
FX, FY = 605.85, 605.68        # 640x480 CameraInfo 실측 (2026-10-05)


def load_all():
    out = {}
    for name in sorted(os.listdir(CONFIG)):
        if name.endswith('.yaml'):
            with open(os.path.join(CONFIG, name)) as f:
                out[name] = yaml.safe_load(f)
    return out


def params(node):
    """노드가 실제로 받는 값: 모든 파일의 /** + 노드 부분"""
    merged = {}
    for doc in load_all().values():
        for key in ('/**', node):
            merged.update((doc.get(key) or {}).get('ros__parameters', {}))
    return merged


def test_files_parse_with_expected_sections():
    docs = load_all()
    assert set(docs) >= {'camera.yaml', 'control.yaml', 'device.yaml', 'hsv.yaml', 'safety.yaml'}
    for name, doc in docs.items():
        for node, body in doc.items():
            assert node in ('/**', 'target_detector', 'tracker_controller', 'opencr_bridge'), (name, node)
            assert 'ros__parameters' in body, (name, node)


def test_no_duplicate_keys_for_same_node():
    seen = collections.defaultdict(list)
    for name, doc in load_all().items():
        for node, body in doc.items():
            for key in body['ros__parameters']:
                seen[(node, key)].append(name)
    dup = {k: v for k, v in seen.items() if len(v) > 1}
    shadow = [k for (n, k) in seen if n != '/**' and ('/**', k) in seen]
    assert not dup, f'같은 노드·같은 키가 여러 파일에 있음: {dup}'
    assert not shadow, f'/**와 노드 부분에 같은 키: {shadow}'


def test_tracking_kp_and_fov():
    """추적 Kp는 report 3-1 각도 Kp(팬 2.0, 틸트 2.5 [1/s]), 시야각은 CameraInfo 실측과 맞아야 한다 (docs/kp_conversion.md)."""
    c = params('tracker_controller')
    assert (c['pan_kp'], c['tilt_kp']) == (2.0, 2.5)
    assert math.isclose(c['hfov_deg'], math.degrees(2 * math.atan(320 / FX)), abs_tol=0.05)
    assert math.isclose(c['vfov_deg'], math.degrees(2 * math.atan(240 / FY)), abs_tol=0.05)
    # 화면 끝 오차(시야각/2)의 명령이 속도 상한 안이어야 Kp 응답이 잘리지 않는다
    assert c['pan_kp'] * c['hfov_deg'] / 2 <= c['pan_speed_limit_deg_s']
    assert c['tilt_kp'] * c['vfov_deg'] / 2 <= c['tilt_speed_limit_deg_s']


def test_detector_and_controller_share_direction():
    """번호 유지의 회전 보정(인지)과 제어가 같은 direction을 써야 한다."""
    d, c = params('target_detector'), params('tracker_controller')
    for key in ('pan_direction', 'tilt_direction'):
        assert d[key] == c[key] and d[key] in (-1, 1), key


def test_home_ticks_valid():
    """브리지가 시작 때 펌웨어에 B로 보내는 기준 자세 tick (scripts/test/pose_tool.py의 h 키로 저장)"""
    ticks = params('opencr_bridge')['home_ticks']
    assert len(ticks) == 2 and all(isinstance(t, int) and 0 <= t <= 4095 for t in ticks)


def test_search_memory_outlives_search():
    """목표 기억이 탐색 시간 상한보다 길어야 탐색이 search_failed(시간 상한)로 끝난다 (2026-10-06 도전 B 시험)."""
    c = params('tracker_controller')
    assert c['memory_max_age_s'] > c['search_delay_s'] + c['search_timeout_s']
