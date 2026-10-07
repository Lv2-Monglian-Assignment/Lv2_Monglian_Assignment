#!/usr/bin/env python3
"""도전 C (문제 3 심화) — 추적 제어 성능 비교: 기본 P 제어에서 데드밴드 하나만 바꿔 같은 순서로 각 3회 비교한다.

시험 순서·지표는 assignment3.py와 같다(왼쪽 3 s → 중앙 3 s → 오른쪽 3 s → 중앙 3 s, RMSE·유효 추적 비율·응답 시간·흔들림).
바꾸는 값은 한 번에 하나만: 팬 데드밴드(기본 0.03) 또는 틸트 데드밴드(기본 0.05). 필터는 현재 구현에 없어 대상이 아니다.

  python3 assignment/assignment_C.py run --param pan_deadband --value 0.03 --trial 1   # 기본 설정 (3회)
  python3 assignment/assignment_C.py run --param pan_deadband --value 0.08 --trial 1   # 변경 설정 (3회)
  python3 assignment/assignment_C.py analyze --param pan_deadband
결과: results/assignment_C/ (runs/, compare_runs.csv, compare_summary.csv, summary.md), results/plots/assignment_C_*.png
보고서에는 개선 여부와 함께 지연·흔들림·미검출 구간이 늘었는지(부작용)를 같이 적는다. 명령값을 실제 위치로 해석하지 않는다(pan_deg가 실측).
"""
import argparse

from assignment3 import analyze, run_trial

PARAMS = {'pan_deadband': 0.03, 'tilt_deadband': 0.05}


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    sub = ap.add_subparsers(dest='cmd', required=True)
    r = sub.add_parser('run')
    r.add_argument('--param', choices=PARAMS, required=True)
    r.add_argument('--value', type=float, required=True)
    r.add_argument('--trial', type=int, required=True)
    r.add_argument('--dry-run', action='store_true')
    a = sub.add_parser('analyze')
    a.add_argument('--param', choices=PARAMS, required=True)
    args = ap.parse_args()
    if args.cmd == 'run':
        note = f'기본값 {PARAMS[args.param]}' + (' (기본 설정 회차)' if args.value == PARAMS[args.param] else ' (변경 설정 회차)')
        run_trial('assignment_C', f'assignment_C_{args.param}{args.value:g}_t{args.trial}{"_dry" if args.dry_run else ""}',
                  {args.param: args.value}, args.dry_run, note)
    else:
        analyze('assignment_C', args.param, f'Challenge C {args.param} compare')


if __name__ == '__main__':
    main()
