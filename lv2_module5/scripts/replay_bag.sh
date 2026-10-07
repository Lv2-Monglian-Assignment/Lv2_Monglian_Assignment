#!/usr/bin/env bash
# bag 입력 재처리 (Raspberry Pi 또는 PC). 모터 출력 없음: 제어·브리지 노드를 띄우지 않는다.
#   scripts/replay_bag.sh <run_id> [태그, 기본 replay] [config 폴더, 기본 설치된 config] [저장 간격 N, 기본 30]
# bag의 영상·CameraInfo·정렬 Depth·모터 각도만 --clock으로 재생하고 target_detector(use_sim_time)를 다시 돌린다.
# 새 결과는 /target_replay로만 나오고, 그 토픽을 recordings/<run_id>_<태그>/ bag으로 따로 기록한다.
# 회귀 비교(심화): 같은 bag을 태그·config 폴더만 바꿔 두 번 돌린 뒤 analyze_bag.py --replay 로 둘 다 비교한다.
set -e
RUN_ID=${1:?사용: replay_bag.sh <run_id> [태그] [config 폴더] [저장 간격 N]}
TAG=${2:-replay}
LV2=$(cd "$(dirname "$0")/.." && pwd)
BAG="$LV2/recordings/$RUN_ID"
OUT="$LV2/recordings/${RUN_ID}_${TAG}"
CONFIG_ARG=${3:+config_dir:=$(cd "$3" && pwd)}
SAVE_N=${4:-30}
[ -d "$BAG" ] || { echo "bag 없음: $BAG"; exit 1; }
[ -e "$OUT" ] && { echo "이미 있음: $OUT (다른 태그 사용)"; exit 1; }

LOG=~/lv2_module5_logs/${RUN_ID}_${TAG}_bag_record.log
mkdir -p ~/lv2_module5_logs
# set -m: 스크립트 안에서 & 로 띄운 기록기는 SIGINT 무시로 시작해 아래 kill -INT로 끝나지 않는다(Lyrical, 2026-10-07 Pi 시험)
# --disable-keyboard-controls: 키보드 제어가 터미널을 읽다 정지하지 않게
set -m
ros2 bag record -o "$OUT" --use-sim-time --disable-keyboard-controls \
  --topics /target_replay /target_replay/depth /target_replay/position_cam > "$LOG" 2>&1 &
REC=$!
set +m
for _ in $(seq 1 100); do   # 최대 20 s: 기록기가 준비된 뒤 재생 시작 (Pi에서 시작에 수 초 걸림, 앞 프레임 누락 방지)
  grep -q "Listening for topics" "$LOG" 2>/dev/null && break
  kill -0 $REC 2>/dev/null || break
  sleep 0.2
done
# 재생은 launch 안에서 한다. 검출기는 스스로 끝나지 않으므로 bag 길이 + 15 s 뒤 Ctrl+C(INT)로 끝낸다
timeout -s INT "$(( $(ros2 bag info "$BAG" | awk '/^Duration/{print int($2)+15}') ))" \
  ros2 launch tracker_bringup replay.launch.py bag:="$BAG" run_id:="${RUN_ID}_${TAG}" save_every_n:="$SAVE_N" $CONFIG_ARG || true
kill -INT $REC; wait $REC || true
echo "재처리 결과 bag: $OUT"
echo "재처리 기록: ~/lv2_module5_logs/${RUN_ID}_${TAG}_detect.csv · 이미지: ~/lv2_module5_results/images/${RUN_ID}_${TAG}_*"
echo "비교: python3 $LV2/scripts/analyze_bag.py $BAG --replay $OUT"
