"""opencr_tracker 펌웨어 상태 줄 해석 확인 (ROS 없이 실행)."""
from tracker_bridge.protocol import parse_status


def test_status_line():
    assert parse_status('S 123456 -12.30 4.50 1.37 -2.75 TRACK') == (-12.3, 4.5, 1.37, -2.75, 'TRACK')


def test_rejects_other_lines():
    for line in ('E 1 command timeout; stop', 'READY opencr_tracker: V <pan_dps> <tilt_dps> | I | X | O',
                 'S 1 2 3 4', 'S 1 nan 0 0 0 HOLD', 'S 1 0 0 0 0 RUN', 'S 1 a 0 0 0 OFF'):
        assert parse_status(line) is None, line


def test_status_ms():
    from tracker_bridge.protocol import status_ms
    assert status_ms('S 123456 -12.30 4.50 1.37 -2.75 TRACK') == 123456
    assert status_ms('E 4 DXL read failed') is None


def test_board_clock_uses_earliest_arrival():
    from tracker_bridge.protocol import BoardClock
    c = BoardClock()
    # 보드가 20 ms마다 측정, Pi는 0~18 ms 늦게 읽음 (Pi 시각 = 보드 + 100 s + 지연)
    for k, late in enumerate((0.018, 0.004, 0.0, 0.011)):
        st = c.stamp(100.0 + k * 0.02 + late, k * 20)
    assert abs(st - (100.0 + 3 * 0.02)) < 1e-6          # 가장 빨리 받은 줄 기준: 지연 11 ms가 빠짐
    assert abs(c.stamp(500.0, 100) - 500.0) < 1e-6        # 보드 재시작(시계 점프): 기록을 버리고 새로 맞춤


def test_fault_retry_three_then_manual():
    from tracker_bridge.protocol import FaultRetry
    r = FaultRetry(max_tries=3, delay_s=2.0, healthy_s=60.0)
    t, sent = 0.0, 0
    for _ in range(400):                                   # 20 s 동안 계속 FAULT
        sent += r.update('FAULT', t); t += 0.05
    assert sent == 3 and r.manual
    for _ in range(1300):                                  # 정상 65 s → 다시 자동 복구 가능
        r.update('HOLD', t); t += 0.05
    assert not r.manual and r.tries == 0


def test_sim_step_matches_firmware_limits():
    from tracker_bridge.protocol import sim_step
    assert sim_step(0.0, 200.0, 0.02, 180.0) == (2.4, 120.0)            # 속도 상한 120
    pos, v = sim_step(179.0, 50.0, 0.02, 180.0)
    assert v == 3.0 and abs(pos - 179.06) < 1e-9                         # 한계 1° 앞: 3 x 1 = 3 deg/s
    assert sim_step(181.0, 50.0, 0.02, 180.0)[1] == 0.0                 # 한계 밖: 바깥 방향 0
    assert sim_step(181.0, -50.0, 0.02, 180.0)[1] == -50.0              # 안쪽은 허용
