"""ROS 없이 상태·명령·좌표 규칙을 시험한다: python3 -m pytest test/"""
import math
from tracker_controller.tracking_logic import (AxisConfig, SearchConfig, TrackingLogic, error_angle_deg, p_command,
                                             TRACKING, LOST, IDLE, SEARCHING)
from tracker_controller import geometry as g


def make(search=False):
    pan = AxisConfig(kp=20.0, direction=1, speed_limit=30.0, deadband=0.03)
    tilt = AxisConfig(kp=15.0, direction=1, speed_limit=20.0, deadband=0.05)
    lg = TrackingLogic(pan, tilt, SearchConfig(enabled=search, delay_s=0.3, timeout_s=3.0))
    lg.set_enabled(True)
    return lg


def feed(lg, ex, area, t, n=1, dt=1 / 30, ey=0.0):
    for i in range(n):
        lg.on_target(ex, ey, area, int((t + i * dt) * 1e9), t + i * dt)
    return t + n * dt


def test_idle():
    lg = make(); lg.set_enabled(False)
    assert lg.step(0.0).state == IDLE


def test_recover_three_frames_and_signs():
    lg = make()
    t = feed(lg, 0.4, 0.05, 1.0, n=2)
    assert lg.step(t).state == LOST
    t = feed(lg, 0.4, 0.05, t, ey=-0.2)
    o = lg.step(t)
    assert o.state == TRACKING and o.reason == 'reacquired_in_view'
    assert o.pan_cmd == 8.0 and o.tilt_cmd == -3.0
    t = feed(lg, -0.4, 0.05, t); assert lg.step(t).pan_cmd == -8.0
    t = feed(lg, 0.0, 0.05, t); assert lg.step(t).pan_cmd == 0.0


def test_no_detection_stops_first_frame():
    lg = make(search=True)
    t = feed(lg, 0.4, 0.05, 1.0, n=3); lg.step(t)
    t = feed(lg, 0.4, 0.0, t)
    o = lg.step(t, joint_deg=(0, 0), desired_deg=(30, 0))
    assert (o.state, o.pan_cmd, o.tilt_cmd) == (LOST, 0.0, 0.0)   # 탐색 켜져 있어도 첫 프레임은 정지


def test_timeout_no_search():
    lg = make(search=True)
    t = feed(lg, 0.4, 0.05, 1.0, n=3); lg.step(t)
    o = lg.step(t + 0.6, joint_deg=(0, 0), desired_deg=(30, 0))
    assert (o.state, o.reason, o.pan_cmd) == (LOST, 'input_timeout', 0.0)


def test_search_then_reacquire():
    lg = make(search=True)
    t = feed(lg, 0.4, 0.05, 1.0, n=3); lg.step(t)
    t = feed(lg, 0.9, 0.0, t, n=12)                    # 0.4 s 미검출 (영상은 계속)
    o = lg.step(t, joint_deg=(0, 0), desired_deg=(30, 5))
    assert o.state == SEARCHING and o.pan_cmd == 20.0 and o.tilt_cmd == 10.0   # 각도 P: 2x30->상한20, 2x5
    t = feed(lg, 0.1, 0.05, t, n=2)
    assert lg.step(t, (25, 5), (30, 5)).state == SEARCHING                  # 확인 중
    t = feed(lg, 0.1, 0.05, t)
    o = lg.step(t, (28, 5), (30, 5))
    assert o.state == TRACKING and o.reason == 'reacquired_search'


def test_search_timeout_stops_and_latches():
    lg = make(search=True)
    t = feed(lg, 0.4, 0.05, 1.0, n=3); lg.step(t)
    t = feed(lg, 0.0, 0.0, t, n=12)
    assert lg.step(t, (0, 0), (30, 0)).state == SEARCHING
    t = feed(lg, 0.0, 0.0, t + 3.1)
    o = lg.step(t, (30, 0), (30, 0))
    assert (o.state, o.reason, o.pan_cmd) == (LOST, 'search_failed', 0.0)
    t = feed(lg, 0.0, 0.0, t + 0.1)
    assert lg.step(t, (30, 0), (30, 0)).reason == 'search_failed'          # 다시 탐색하지 않음


def test_stale_stamp_ignored():
    lg = make()
    feed(lg, 0.4, 0.05, 1.0, n=3)
    assert lg.on_target(0.4, 0.0, 0.05, int(1.0 * 1e9), 5.0) is False
    assert lg.step(5.0).reason == 'input_timeout'


def test_geometry_roundtrip_and_look_at():
    hf, vf = math.radians(69), math.radians(42)
    for pan, tilt, up in [(0.3, -0.1, 0.0), (-0.8, 0.2, 0.03), (1.2, 0.35, -0.02)]:
        p_opt = g.ray_point(0.3, -0.2, 0.8, hf, vf)
        pb = g.cam_to_base(p_opt, pan, tilt, 0.02, up)
        back = g.base_to_cam(pb, pan, tilt, 0.02, up)
        assert all(abs(a - b) < 1e-9 for a, b in zip(back, p_opt))
        lp, lt = g.look_at(pb, up)
        ex, ey = g.to_normalized(g.base_to_cam(pb, lp, lt, 0.02, up), hf, vf)
        assert abs(ex) < 1e-9 and abs(ey) < 1e-9                             # 정중앙
    # 오른쪽 목표 -> 오른쪽으로 돌아야 함 (pan_geo 감소)
    assert g.look_at(g.cam_to_base((0.1, 0, 1.0), 0, 0))[0] < 0


def test_memory_predicts_motion():
    m = g.TargetMemory(window_s=0.5, max_speed_m_s=1.0, horizon_s=1.0)
    for i in range(10):
        m.add(i * 0.05, (1.0, 0.2 * i * 0.05, 0.0), True)                  # 왼쪽으로 0.2 m/s
    p = m.predict(0.45 + 0.5)
    assert abs(p[1] - (0.2 * 0.45 + 0.2 * 0.5)) < 1e-6
    assert m.predict(10.0) is None                                          # 너무 오래된 기억


def test_memory_short_window_still_estimates_speed():
    m = g.TargetMemory(window_s=0.1, max_speed_m_s=1.0, horizon_s=1.0)     # 30 fps에서 약 3 프레임
    for i in range(6):
        m.add(i / 30, (1.0, 0.3 * i / 30, 0.0), True)                      # 왼쪽으로 0.3 m/s
    assert abs(m.velocity[1] - 0.3) < 1e-6


def test_angle_kp_matches_angle_loop():
    """각도 Kp [1/s]: 화면 오차를 실제 각도로 바꿔 곱한다. 팬 Kp 2.0, 시야각 55.7° (fx 605.85)"""
    pan = AxisConfig(kp=2.0, direction=-1, speed_limit=120.0, deadband=0.03, half_fov_deg=55.7 / 2)
    assert math.isclose(error_angle_deg(1.0, 55.7 / 2), 27.85, abs_tol=1e-6)        # 화면 끝 = 시야각/2
    ex10 = math.tan(math.radians(10)) / math.tan(math.radians(55.7 / 2))           # 목표가 오른쪽 10°에 있을 때
    assert math.isclose(p_command(ex10, pan), -20.0, abs_tol=1e-6)                 # 2.0 x 10° = 20°/s, 오른쪽(−)
    assert math.isclose(p_command(-1.0, pan), 2.0 * 27.85, abs_tol=1e-6)           # 화면 왼쪽 끝 55.7°/s
    assert p_command(0.02, pan) == 0.0                                              # 데드밴드(정규화 0.03) 안


def test_angle_kp_respects_speed_limit():
    tilt = AxisConfig(kp=2.5, direction=1, speed_limit=20.0, deadband=0.05, half_fov_deg=43.2 / 2)
    assert p_command(1.0, tilt) == 20.0 and p_command(-1.0, tilt) == -20.0
