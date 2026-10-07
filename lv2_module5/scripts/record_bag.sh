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

TOPICS=(/camera/camera/color/image_raw /camera/camera/color/camera_info
        /camera/camera/aligned_depth_to_color/image_raw
        /target /target_depth /target/position_cam
        /tracking_status /tracking_enable /pan_tilt/command /pan_tilt/joint_states)

echo "기록 시작: $OUT (${SECONDS_MAX}s, Ctrl+C로 먼저 끝낼 수 있음)"
# 정해진 시간 뒤 SIGINT로 끝내야 metadata.yaml이 정상으로 닫힌다
timeout -s INT "$SECONDS_MAX" ros2 bag record -o "$OUT" "${TOPICS[@]}" || true

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
