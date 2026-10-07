#!/usr/bin/env python3
"""도전 E (문제 5 심화) — 고정 bag 기반 회귀 비교: 같은 bag을 기준 설정과 변경 설정으로 다시 처리해 지표를 비교한다.

모터 출력 없이 검출기만 재실행한다(replay.launch). 바꾸는 설정은 한 번에 하나(검출기 config 키).
  python3 assignment/assignment_E.py --bag recordings/<run_id> --param min_area_px --value 300
  python3 assignment/assignment_E.py --bag recordings/<run_id> --param hsv_lower --value "[100, 120, 40]"
결과: results/assignment_E/<bag>_<param>/ (compare.csv·summary.md, 재처리 기록), results/plots/assignment_E_*.png
지표 (변경 전·후 같은 코드·산식)
  검출 비율(재처리 /target_replay 중 검출), 원본 /target과 검출 여부 일치율·ex 차이, 처리 속도(1000/평균 proc_ms),
  수평 RMSE(검출 프레임 ex), 미검출 구간 수·최장 길이(복구 관련: 소실 장면에서 목표가 다시 잡히기까지)
bag의 sha256과 기준·변경 커밋을 함께 남긴다. 저장된 /target과 새 결과를 다른 토픽(/target_replay)으로 분리한다.
"""
import argparse
import math
import os
import time

import common as C
from assignment5 import compare_replay, replay_bag, sha256s


def det_metrics(det_csv):
    rows = C.read_csv(det_csv)
    det = [r['detected'] == '1' for r in rows]
    ex = [C.fnum(r['ex']) for r in rows if r['detected'] == '1']
    ms = [C.fnum(r['proc_ms']) for r in rows]
    gaps, run, gaps_s, t_start = [], 0, [], None
    for r, d in zip(rows, det):
        if not d:
            run += 1
            t_start = t_start if t_start is not None else C.fnum(r['stamp_s'])
        elif run:
            gaps.append(run)
            gaps_s.append(C.fnum(r['stamp_s']) - t_start)
            run, t_start = 0, None
    return {'frames': len(rows), 'detect_ratio': sum(det) / len(det) if det else math.nan,
            'rmse_ex': math.sqrt(sum(e * e for e in ex) / len(ex)) if ex else math.nan,
            'mean_proc_ms': sum(ms) / len(ms) if ms else math.nan, 'proc_fps': 1000 / (sum(ms) / len(ms)) if ms else math.nan,
            'miss_segments': len(gaps), 'longest_miss_s': max(gaps_s, default=0.0)}


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('--bag', required=True)
    ap.add_argument('--param', required=True, help='바꿀 검출기 설정 키 (config/hsv.yaml)')
    ap.add_argument('--value', required=True)
    args = ap.parse_args()
    C.check_ros()
    bag = args.bag.rstrip('/')
    name = os.path.basename(bag)
    dest = C.out_dir('assignment_E', f'{name}_{args.param}')
    base_val = C.read_param(args.param)
    if base_val is None:
        raise SystemExit(f'config에 {args.param}가 없습니다')
    variants = [('base', f'{args.param}={base_val}', C.config_variant(os.path.join(dest, 'config_base'), {})),
                ('changed', f'{args.param}={args.value}',
                 C.config_variant(os.path.join(dest, 'config_changed'), {args.param: args.value}))]
    rows, series = [], []
    for key, cond, cfg in variants:
        print(f'=== {key}: {cond}')
        det = replay_bag(bag, f'replay_E_{key}_{C.tag()}', dest, cfg)
        m = det_metrics(det)
        cmp_res, s = compare_replay(bag, det)
        rows.append({'variant': key, 'condition': cond, **m, 'agreement_with_bag_pct': cmp_res['detect_agreement_pct'],
                     'ex_mean_abs_diff': cmp_res['ex_mean_abs_diff'], 'replay_csv': os.path.basename(det)})
        if not series:
            series.append(s[0])
        series.append((f'{key} ex', s[1][1], s[1][2]))
    sums = sha256s(bag)
    plot = C.line_plot(os.path.join(C.out_dir('plots'), f'assignment_E_{name}_{args.param}.png'), series,
                       f'Regression: {args.param}', 'time [s]', 'ex', [(0, '0')])
    C.write_csv(os.path.join(dest, 'compare.csv'), rows)
    C.update_metrics([{'test': 'assignment_E', 'run_id': f'{name}_{r["variant"]}', 'condition': r['condition'],
                       'date': time.strftime('%Y-%m-%d'), **{k: v for k, v in r.items() if k not in ('variant', 'condition')}}
                      for r in rows])
    d = {k: (rows[1][k] - rows[0][k]) for k in ('detect_ratio', 'rmse_ex', 'proc_fps', 'miss_segments', 'longest_miss_s')
         if isinstance(rows[0][k], (int, float))}
    md = [f'# 도전 E 고정 bag 회귀 비교 ({name}, {args.param})', '',
          f'- bag: {bag} (sha256: ' + ', '.join(f'{f} {h[:12]}' for f, _, h in sums) + ')',
          f'- 커밋: {C.git_commit()} (기준·변경은 설정 복사본 config_base/·config_changed/로 구분)', '',
          C.md_table(rows, ['variant', 'condition', 'frames', 'detect_ratio', 'agreement_with_bag_pct', 'ex_mean_abs_diff',
                            'rmse_ex', 'proc_fps', 'miss_segments', 'longest_miss_s']), '',
          '변경 - 기준: ' + ', '.join(f'{k} {v:+.4g}' for k, v in d.items()), '',
          '- 회귀 판단: 변경 후 검출 비율이 줄거나 미검출 구간이 늘면 회귀. 원인은 바꾼 설정 하나로 좁힌다',
          f'- 그래프: {os.path.relpath(plot, C.LV2) if plot else "없음"}']
    open(os.path.join(dest, 'summary.md'), 'w').write('\n'.join(md) + '\n')
    print('\n'.join(md))


if __name__ == '__main__':
    main()
