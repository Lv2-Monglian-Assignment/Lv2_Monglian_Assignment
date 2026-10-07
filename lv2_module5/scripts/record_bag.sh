#!/usr/bin/env bash
# bag 기록 (Raspberry Pi에서 실행, 추적 노드가 이미 같은 run_id로 떠 있어야 한다)
#   ros2 launch tracker_bringup full.launch.py run_id:=<run_id>      # 다른 터미널
#   scripts/record_bag.sh <run_id> [초, 기본 30]
# 결과: recordings/<run_id>/ (bag, git 제외)
#       recordings/<run_id>_info.txt (기준 커밋·설정·ros2 bag info·크기·sha256, git 포함)
#       recordings/<run_id>_config/ (기록 당시 config/*.yaml 사본, git 포함)
# 시리얼 로그는 같은 run_id로 ~/lv2_module5_logs/<run_id>_serial.log, 제어 기록은 <run_id>.csv
set -e
RUN_ID=${1:?사용: record_bag.sh <run_id> [초]}
SECONDS_MAX=${2:-30}
LV2=$(cd "$(dirname "$0")/.." && pwd)
OUT="$LV2/recordings/$RUN_ID"
[ -e "$OUT" ] && { echo "이미 있음: $OUT (다른 run_id 사용)"; exit 1; }

# 계속 발행되는 토픽: 모두 구독한 뒤부터 시간을 센다
WAIT_TOPICS=(/camera/camera/color/image_raw /camera/camera/color/camera_info
             /camera/camera/aligned_depth_to_color/image_raw
             /target /target_depth /target/position_cam
             /tracking_status /pan_tilt/command /pan_tilt/joint_states)
TOPICS=("${WAIT_TOPICS[@]}" /tracking_enable)   # /tracking_enable은 켜고 끌 때만 발행되어 기다리지 않는다

LOG=~/lv2_module5_logs/${RUN_ID}_bag_record.log
mkdir -p ~/lv2_module5_logs
# Lyrical(2026-10-07 Pi 시험):
#  - 키보드 제어가 터미널을 읽어서 timeout 아래(백그라운드 그룹)에서는 정지(State T)한 채 멈춘다 → --disable-keyboard-controls
#  - 스크립트 안에서 & 로 띄운 프로세스는 SIGINT 무시로 시작한다 → set -m (별도 프로세스 그룹, SIGINT로 정상 종료)
#  - 구독까지 수 초 걸린다(10 s 지정에 2.2~4.0 s만 남음) → 구독을 마친 뒤부터 시간을 센다
#    (bag은 첫 구독부터 쓰이므로 지정 시간보다 구독에 걸린 만큼 조금 길다)
set -m
ros2 bag record -o "$OUT" --disable-keyboard-controls --topics "${TOPICS[@]}" > "$LOG" 2>&1 &
REC=$!
set +m
trap 'kill -INT $REC 2>/dev/null' INT          # Ctrl+C로 먼저 끝낼 때도 기록기를 SIGINT로 닫는다(metadata.yaml 정상)
n=0
for _ in $(seq 1 100); do                       # 최대 20 s (노드가 꺼진 토픽이 있으면 20 s 뒤 그대로 시작)
  n=0
  for t in "${WAIT_TOPICS[@]}"; do
    if grep -q "Subscribed to topic '$t'" "$LOG" 2>/dev/null; then n=$((n + 1)); fi
  done
  [ "$n" -ge "${#WAIT_TOPICS[@]}" ] && break
  kill -0 $REC 2>/dev/null || break
  sleep 0.2
done
echo "기록 시작: $OUT (구독 $n/${#WAIT_TOPICS[@]} · ${SECONDS_MAX}s, Ctrl+C로 먼저 끝낼 수 있음)"
[ "$n" -lt "${#WAIT_TOPICS[@]}" ] && echo "  주의: 구독 못 한 토픽이 있습니다(발행 노드 확인). 기록기 로그: $LOG"
sleep "$SECONDS_MAX" &
SLP=$!
wait $SLP || true                               # wait 중에 Ctrl+C가 오면 trap이 바로 실행된다
kill $SLP 2>/dev/null || true
kill -INT $REC 2>/dev/null || true
wait $REC || true
trap - INT

cp -r "$LV2/config" "$LV2/recordings/${RUN_ID}_config"
{
  echo "run_id: $RUN_ID"
  echo "기록 일시: $(date '+%Y-%m-%d %H:%M:%S %z')"
  echo "기록 장비: $(hostname) · ROS_DISTRO=${ROS_DISTRO:-?} · ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-?} · RMW=${RMW_IMPLEMENTATION:-기본값}"
  echo "기준 커밋: $(git -C "$LV2" rev-parse HEAD)$(git -C "$LV2" diff --quiet HEAD -- . || echo ' (+ 커밋 안 된 변경 있음)')"
  echo "설정 사본: recordings/${RUN_ID}_config/"
  echo "연결 기록: ~/lv2_module5_logs/${RUN_ID}.csv (제어) · ${RUN_ID}_detect.csv (인지) · ${RUN_ID}_serial.log (시리얼)"
  echo "카메라 해상도: $(timeout 5 ros2 topic echo --once /camera/camera/color/camera_info 2>/dev/null | grep -E '^(width|height):' | tr '\n' ' ' || echo '기록 후 카메라 꺼짐 — bag의 camera_info 참고')"
  echo "총 크기: $(du -sh "$OUT" | cut -f1)"
  echo "== ros2 bag info"
  ros2 bag info "$OUT"
  echo "== sha256"
  (cd "$OUT" && sha256sum *)
} | tee "$LV2/recordings/${RUN_ID}_info.txt"
echo "메타데이터: recordings/${RUN_ID}_info.txt  (recordings/README.md 표에 한 줄 추가)"
