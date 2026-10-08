#!/usr/bin/env bash
# 실험 1-2: 카메라 + 검출 노드(perception.launch.py)를 detect_scale 하나만 바꾼 설정 복사본으로 실행하고
# 검출 기록(proc_ms·검출 여부), 검출 프로세스 CPU 사용률(top), 온도를 남긴다. 모터는 쓰지 않는다(브리지 미실행).
#   (Raspberry Pi, lv2_module5 폴더, 워크스페이스 source 후)
#   bash results/param_experiments/detect_scale/run_live.sh <scale> <결과 폴더> [측정 초, 기본 40]
# 원본 config/는 바꾸지 않는다. 미리보기(show_window)는 config 기본값 false 그대로.
set -euo pipefail
set -m                                   # 백그라운드 launch를 별도 프로세스 그룹으로 (SIGINT로 정상 종료되게)
SCALE=$1; OUT=$2; SECS=${3:-40}
TAG=$(date +%Y%m%d_%H%M%S)
RUN_ID="ds_live_${SCALE}_${TAG}"
CFG="$OUT/config_${SCALE}"
mkdir -p "$CFG"
cp config/*.yaml "$CFG/"
sed -i -E "s/^(\s*detect_scale:\s*)[0-9.]+/\1${SCALE}/" "$CFG/hsv.yaml"
grep -E "^\s*detect_scale:" "$CFG/hsv.yaml"
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-77} RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
CSV=~/lv2_module5_logs/${RUN_ID}_detect.csv
temp() { awk '{printf "%.1f", $1/1000}' /sys/class/thermal/thermal_zone0/temp; }

ros2 launch tracker_bringup perception.launch.py run_id:="$RUN_ID" config_dir:="$(realpath "$CFG")" \
  > "$OUT/launch_${SCALE}.log" 2>&1 &
LPID=$!
cleanup() { kill -INT -- -"$LPID" 2>/dev/null || true; for _ in $(seq 20); do kill -0 "$LPID" 2>/dev/null || break; sleep 0.5; done
            kill -TERM -- -"$LPID" 2>/dev/null || true; }
trap cleanup EXIT

for _ in $(seq 60); do [ -f "$CSV" ] && [ "$(wc -l < "$CSV")" -gt 60 ] && break; sleep 0.5; done
[ -f "$CSV" ] || { echo "검출 기록이 생기지 않음: $CSV"; exit 1; }
sleep 5                                   # 시작 직후 구간 제외
DPID=$(pgrep -f "target_detector/target_detector" | head -1)
T_START=$(date +%s.%N); TEMP_START=$(temp)
top -b -d 1 -n "$SECS" -p "$DPID" > "$OUT/top_${SCALE}.txt"
T_END=$(date +%s.%N); TEMP_END=$(temp)
cleanup; trap - EXIT
cp "$CSV" "$OUT/"
printf 'scale=%s\nrun_id=%s\nt_start=%s\nt_end=%s\ntemp_start_c=%s\ntemp_end_c=%s\ndetector_pid=%s\ndomain=%s\ncommit=%s\n' \
  "$SCALE" "$RUN_ID" "$T_START" "$T_END" "$TEMP_START" "$TEMP_END" "$DPID" "$ROS_DOMAIN_ID" "$(git rev-parse --short HEAD)" \
  > "$OUT/run_${SCALE}.txt"
cat "$OUT/run_${SCALE}.txt"
