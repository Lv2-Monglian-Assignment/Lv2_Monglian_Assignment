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
