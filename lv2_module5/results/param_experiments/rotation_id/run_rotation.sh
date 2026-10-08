#!/usr/bin/env bash
# 실험 4: 물체 2개를 두고 하나(A)를 손으로 좌우로 옮기며 추적시켜, 회전 중 target_id가 다른 물체로 바뀌는지 기록한다.
#   (Raspberry Pi, lv2_module5 폴더, 워크스페이스 source 후) bash results/param_experiments/rotation_id/run_rotation.sh [초, 기본 30]
# full.launch.py(카메라·인지·제어·브리지, 실제 모터)를 config 그대로 띄운다. 1 s마다 검출 스냅샷을 저장한다(~/lv2_module5_results/images).
set -uo pipefail
set -m
SECS=${1:-30}
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-77} RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
TAG=$(date +%Y%m%d_%H%M%S); RUN_ID="exp4_rot_${TAG}"
OUT="results/param_experiments/rotation_id/run_${TAG}"; mkdir -p "$OUT"
ros2 launch tracker_bringup full.launch.py run_id:="$RUN_ID" > "$OUT/launch.log" 2>&1 &
LPID=$!
stop() { kill -INT -- -"$LPID" 2>/dev/null; for _ in $(seq 20); do kill -0 "$LPID" 2>/dev/null || break; sleep 0.5; done; kill -TERM -- -"$LPID" 2>/dev/null; }
trap stop EXIT
CSV=~/lv2_module5_logs/${RUN_ID}_detect.csv
for _ in $(seq 80); do [ -f "$CSV" ] && [ "$(wc -l < "$CSV")" -gt 30 ] && break; sleep 0.5; done
sleep 4
echo; echo "▶ 준비됨 ($RUN_ID). 블록(A)을 카메라 정면 약 0.6 m, 원기둥(B)을 15 cm 옆에 두세요."
read -r -p "  손을 화면 밖으로 빼고 Enter: 추적 시작 " _
ros2 topic pub --once -w 1 /tracking_enable std_msgs/msg/Bool "{data: true}" > /dev/null 2>&1
sleep 3
T0=$(date +%s.%N)
( for i in $(seq "$SECS"); do ros2 topic pub --once -w 1 /target/save_snapshot std_msgs/msg/String "{data: exp4_s$(printf %02d "$i")}" > /dev/null 2>&1; sleep 0.3; done ) &
SNAP=$!
for i in $(seq "$SECS" -1 1); do
  if [ "$i" -gt $((SECS/2)) ]; then M="블록(A)을 들고 좌우로 천천히 옮기세요"; else M="좌우로 빠르게 옮기세요 (B는 그대로)"; fi
  printf '\r  %-40s %2d s ' "$M" "$i"; sleep 1
done
T1=$(date +%s.%N)
printf '\n  끝. 블록을 처음 자리에 두세요.\n'
ros2 topic pub --once -w 1 /tracking_enable std_msgs/msg/Bool "{data: false}" > /dev/null 2>&1
kill "$SNAP" 2>/dev/null; sleep 1
stop; trap - EXIT
cp ~/lv2_module5_logs/${RUN_ID}.csv ~/lv2_module5_logs/${RUN_ID}_detect.csv ~/lv2_module5_logs/${RUN_ID}_serial.log "$OUT/" 2>/dev/null
cp -r config "$OUT/config"
printf 'run_id=%s\nt_start=%s\nt_end=%s\nseconds=%s\ncommit=%s\ndomain=%s\n' "$RUN_ID" "$T0" "$T1" "$SECS" "$(git rev-parse --short HEAD)" "$ROS_DOMAIN_ID" > "$OUT/run.txt"
echo "결과: $OUT"
