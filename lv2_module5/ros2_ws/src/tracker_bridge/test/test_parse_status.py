"""opencr_tracker 펌웨어 상태 줄 해석 확인 (ROS 없이 실행)."""
from tracker_bridge.protocol import parse_status


def test_status_line():
    assert parse_status('S 123456 -12.30 4.50 1.37 -2.75 TRACK') == (-12.3, 4.5, 1.37, -2.75, 'TRACK')


def test_rejects_other_lines():
    for line in ('E 1 command timeout; stop', 'READY opencr_tracker: V <pan_dps> <tilt_dps> | I | X | O',
                 'S 1 2 3 4', 'S 1 nan 0 0 0 HOLD', 'S 1 0 0 0 0 RUN', 'S 1 a 0 0 0 OFF'):
        assert parse_status(line) is None, line
