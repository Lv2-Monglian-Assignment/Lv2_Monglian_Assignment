"""analyze_bag.py 지표 계산 검사 — 손으로 만든 시계열, 실제 bag 아님.

실행: python3 -m pytest -q tests/test_analyze_bag.py  (또는 python3 tests/test_analyze_bag.py)
"""
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from analyze_bag import compare_replay, metrics  # noqa: E402


def scene():
    # 0~1 s 추적(오차 0.1), 1~1.5 s 소실, 1.5 s 복귀. 10 Hz 목표, 상태·명령도 10 Hz
    targets, statuses, cmds = [], [], []
    for i in range(20):
        t = i * 0.1
        lost = 1.0 <= t < 1.5
        targets.append((t, i, 0.0 if lost else 0.1, 0.0, 0.0 if lost else 0.02))
        statuses.append((t, 'LOST' if lost else 'TRACKING'))
        cmds.append((t, 0.0 if lost else -6.0, 0.0))
    return targets, statuses, cmds


def test_metrics():
    m = metrics(*scene())
    assert math.isclose(m['target_hz'], 10.0)
    assert math.isclose(m['detect_rate'], 15 / 20)
    assert math.isclose(m['rmse_ex'], 0.1) and m['rmse_ey'] == 0
    assert m['lost_events'] == 1 and m['unrecovered'] == 0
    assert math.isclose(m['recover_mean_s'], 0.5)
    assert m['cmd_when_stopped'] == 0 and m['max_abs_cmd_dps'] == 6.0


def test_cmd_during_lost_is_counted():
    targets, statuses, cmds = scene()
    cmds[12] = (cmds[12][0], 3.0, 0.0)   # LOST 중 0이 아닌 명령
    assert metrics(targets, statuses, cmds)['cmd_when_stopped'] == 1


def test_unrecovered():
    targets, statuses, cmds = scene()
    m = metrics(targets, statuses[:13], cmds)
    assert m['lost_events'] == 1 and m['unrecovered'] == 1 and math.isnan(m['recover_mean_s'])


def test_compare_replay_by_stamp():
    orig = scene()[0]
    replay = [(t + 100, s, ex + 0.01, ey, z) for t, s, ex, ey, z in orig[:10]]   # 다른 시계, 같은 stamp
    replay[0] = (replay[0][0], 0, 0.0, 0.0, 0.0)                                # 1프레임 검출 불일치
    c = compare_replay(orig, replay)
    assert c['matched'] == 10 and math.isclose(c['match_ratio'], 0.5)
    assert math.isclose(c['detect_agree'], 0.9)
    assert math.isclose(c['mean_abs_dex'], 0.01)


if __name__ == '__main__':
    for name, fn in list(globals().items()):
        if name.startswith('test_'):
            fn()
    print('ok')
