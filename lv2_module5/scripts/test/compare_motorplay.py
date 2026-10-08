#!/usr/bin/env python3
"""원본 bag의 실제 모터 각도와, 같은 /pan_tilt/command를 다시 보내 움직인 결과(motorplay bag)를 비교한다.
  python3 scripts/test/compare_motorplay.py recordings/<run_id> recordings/<run_id>_motorplay [--out results/assignment5/<run_id>_motorplay]
시간 기준: 각 bag에서 첫 /pan_tilt/command를 받은 시각을 0으로 맞춘다(bag 기록 시각만 사용).
"""
import argparse
import math
import os

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def read(bag):
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=bag, storage_id=''), rosbag2_py.ConverterOptions('cdr', 'cdr'))
    types = {t.name: t.type for t in r.get_all_topics_and_types()}
    r.set_filter(rosbag2_py.StorageFilter(topics=['/pan_tilt/joint_states', '/pan_tilt/command']))
    js, cm = [], []
    while r.has_next():
        topic, data, t = r.read_next()
        m = deserialize_message(data, get_message(types[topic]))
        if topic == '/pan_tilt/command':
            cm.append((t * 1e-9, m.vector.x, m.vector.y))
        elif list(m.name[:2]) == ['pan', 'tilt']:
            js.append((t * 1e-9, math.degrees(m.position[0]), math.degrees(m.position[1])))
    return np.array(js), np.array(cm)


ap = argparse.ArgumentParser()
ap.add_argument('orig')
ap.add_argument('play')
ap.add_argument('--out')
a = ap.parse_args()
oj, oc = read(a.orig)
pj, pc = read(a.play)
o0, p0 = oc[0, 0], pc[0, 0]
oj[:, 0] -= o0
pj[:, 0] -= p0
dur = min(oc[-1, 0] - o0, pc[-1, 0] - p0)
t = np.arange(0, dur, 0.05)
op, ot = np.interp(t, oj[:, 0], oj[:, 1]), np.interp(t, oj[:, 0], oj[:, 2])
pp, pt = np.interp(t, pj[:, 0], pj[:, 1]), np.interp(t, pj[:, 0], pj[:, 2])
# 출발 자세 차이를 뺀 "움직인 양" 비교 (속도 명령 재생이라 출발점 차이는 그대로 남는다)
dp, dt_ = (pp - pp[0]) - (op - op[0]), (pt - pt[0]) - (ot - ot[0])
rows = [
    ('명령 수 (원본 / 재생)', f'{len(oc)} / {len(pc)}'),
    ('비교 구간', f'{dur:.2f} s (첫 명령 기준)'),
    ('출발 자세 원본', f'pan {op[0]:.2f}°, tilt {ot[0]:.2f}°'),
    ('출발 자세 재생', f'pan {pp[0]:.2f}°, tilt {pt[0]:.2f}°'),
    ('끝 자세 원본', f'pan {op[-1]:.2f}°, tilt {ot[-1]:.2f}°'),
    ('끝 자세 재생', f'pan {pp[-1]:.2f}°, tilt {pt[-1]:.2f}°'),
    ('각도 차이 RMS (pan / tilt)', f'{math.sqrt(np.mean((pp - op) ** 2)):.2f}° / {math.sqrt(np.mean((pt - ot) ** 2)):.2f}°'),
    ('각도 차이 최대 (pan / tilt)', f'{np.max(np.abs(pp - op)):.2f}° / {np.max(np.abs(pt - ot)):.2f}°'),
    ('움직인 양 차이 RMS (출발점 보정, pan / tilt)', f'{math.sqrt(np.mean(dp ** 2)):.2f}° / {math.sqrt(np.mean(dt_ ** 2)):.2f}°'),
    ('pan 범위 원본 / 재생', f'{op.min():.1f}~{op.max():.1f}° / {pp.min():.1f}~{pp.max():.1f}°'),
    ('tilt 범위 원본 / 재생', f'{ot.min():.1f}~{ot.max():.1f}° / {pt.min():.1f}~{pt.max():.1f}°'),
]
md = ['# 실제 모터 재생 비교', '', f'- 원본 bag: `{a.orig}`', f'- 재생 결과 bag: `{a.play}`', '',
      '| 항목 | 값 |', '|---|---|'] + [f'| {k} | {v} |' for k, v in rows]
md += ['', '| t [s] | 원본 pan | 재생 pan | 원본 tilt | 재생 tilt |', '|---|---|---|---|---|']
for s in np.arange(0, dur, 1.0):
    i = int(s / 0.05)
    md.append(f'| {s:.0f} | {op[i]:.1f} | {pp[i]:.1f} | {ot[i]:.1f} | {pt[i]:.1f} |')
print('\n'.join(md))
if a.out:
    os.makedirs(a.out, exist_ok=True)
    open(os.path.join(a.out, 'summary.md'), 'w').write('\n'.join(md) + '\n')
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(2, 1, figsize=(8, 5), sharex=True)
        for k, (o, p, n) in enumerate([(op, pp, 'pan'), (ot, pt, 'tilt')]):
            ax[k].plot(t, o, label='original (bag)')
            ax[k].plot(t, p, '--', label='motor replay')
            ax[k].set_ylabel(f'{n} [deg]')
            ax[k].legend()
            ax[k].grid(alpha=.3)
        ax[1].set_xlabel('time from first command [s]')
        fig.tight_layout()
        fig.savefig(os.path.join(a.out, 'motorplay_joint.png'), dpi=110)
        print(f'\n그래프: {os.path.join(a.out, "motorplay_joint.png")}')
    except Exception as e:
        print(f'그래프 생략: {e}')
