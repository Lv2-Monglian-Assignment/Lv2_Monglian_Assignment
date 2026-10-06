"""여러 물체 번호 유지 시험 (ROS 없이): python3 -m pytest test/"""
import math
from target_detector.object_tracker import ObjectTracker, Observation, TargetLock, TrackerConfig

DT = 1 / 30                                                   # 30 fps


def ob(x, y, z=0.0, area=15.0):
    """회전축 기준 위치 (x 앞, y 왼쪽, z 위) [m]의 관측"""
    n = math.sqrt(x * x + y * y + z * z)
    return Observation(bearing=(x / n, y / n, z / n), pos=(x, y, z), area_cm2=area)


def test_ids_persist_for_moving_objects():
    tr = ObjectTracker()
    first = tr.update(0.0, [ob(0.5, 0.10), ob(0.5, -0.10)])
    a, b = first[0], first[1]
    for i in range(1, 30):                                    # 1초 동안 0.3 m/s로 서로 반대 방향 이동
        t = i * DT
        ids = tr.update(t, [ob(0.5, 0.10 + 0.3 * t), ob(0.5, -0.10 - 0.3 * t)])
        assert ids == {0: a, 1: b}


def test_crossing_objects_keep_ids_with_velocity():
    # 앞뒤로 엇갈리는 두 물체: 하나는 다가오고 하나는 멀어진다. 엇갈리는 순간 거리가 2 cm 가까이 줄어든다
    tr = ObjectTracker()
    ids0 = tr.update(0.0, [ob(0.40, 0.01), ob(0.60, -0.01)])
    near, far = ids0[0], ids0[1]
    for i in range(1, 40):
        t = i * DT
        p1 = (0.40 + 0.15 * t, 0.01)                          # 멀어짐
        p2 = (0.60 - 0.15 * t, -0.01)                         # 다가옴
        ids = tr.update(t, [ob(*p2), ob(*p1)])                # 관측 순서를 일부러 뒤집어 넣는다
        assert ids[1] == near and ids[0] == far


def test_hidden_object_keeps_id_and_is_reidentified():
    tr = ObjectTracker()
    ids = tr.update(0.0, [ob(0.5, 0.0), ob(0.5, 0.3)])
    a, b = ids[0], ids[1]
    for i in range(1, 60):                                    # b가 2초 동안 화면 밖 (a만 보임)
        assert tr.update(i * DT, [ob(0.5, 0.0)]) == {0: a}
    assert b in tr.tracks                                     # 번호는 지워지지 않고 기억됨
    ids = tr.update(60 * DT, [ob(0.5, 0.0), ob(0.51, 0.31)])  # 비슷한 위치·크기로 다시 보임
    assert ids == {0: a, 1: b}


def test_hidden_object_not_reidentified_if_size_differs():
    tr = ObjectTracker()
    b = tr.update(0.0, [ob(0.5, 0.3, area=15.0)])[0]
    ids = tr.update(1.0, [ob(0.5, 0.3, area=40.0)])          # 같은 자리, 면적 2.7배 -> 다른 물체
    assert ids[0] != b


def test_fast_rotation_does_not_create_new_ids():
    tr = ObjectTracker(TrackerConfig(fast_rotation_deg_s=30.0))
    a = tr.update(0.0, [ob(0.5, 0.0)])[0]
    ids = tr.update(DT, [ob(0.5, 0.0), ob(0.5, 0.2)], rot_speed_deg_s=45.0)
    assert ids == {0: a} and len(tr.tracks) == 1


def test_target_lock_waits_for_locked_object_then_relocks():
    tr, lock = ObjectTracker(), TargetLock(relock_after_s=3.0)
    ids = tr.update(0.0, [ob(0.5, 0.0), ob(0.5, 0.3)])
    assert lock.choose(0.0, ids, tr, lambda: 0) == 0          # 처음: 선택 규칙으로 0번(=ID a) 잠금
    a = lock.locked_id
    ids = tr.update(1.0, [ob(0.5, 0.3)])                      # a가 가려짐, b만 보임
    assert lock.choose(1.0, ids, tr, lambda: 0) is None       # 갈아타지 않고 미검출(z=0)
    ids = tr.update(2.0, [ob(0.5, 0.3), ob(0.5, 0.0)])        # a가 2초 뒤 재등장
    assert lock.choose(2.0, ids, tr, lambda: 0) == 1 and lock.locked_id == a
    ids = tr.update(6.0, [ob(0.5, 0.3)])                      # a를 3초 넘게 못 봄 -> 새로 고름
    assert lock.choose(6.0, ids, tr, lambda: 0) == 0 and lock.locked_id != a
