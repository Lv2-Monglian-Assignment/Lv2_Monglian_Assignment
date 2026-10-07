#!/usr/bin/env bash
# OpenCR 펌웨어 빌드·업로드 (Raspberry Pi에서 실행)
#   scripts/upload_fw.sh                 # firmware/opencr_tracker (추적용, 프로토콜 #7)
#   scripts/upload_fw.sh dxl_scan        # 모터 스캔용
# 업로드는 arm64로 빌드한 opencr_ld를 쓴다(README 4절). "CRC OK"와 "[OK] Download"가 모두 있어야 성공으로 본다.
set -o pipefail
SKETCH=${1:-opencr_tracker}
PORT=${PORT:-/dev/ttyACM0}
LV2=$(cd "$(dirname "$0")/.." && pwd)
LOG_DIR="$LV2/results/logs"; mkdir -p "$LOG_DIR/raw"   # 빌드 로그는 raw/(git 제외), 업로드 기록은 logs/
STAMP=$(date +%Y%m%d_%H%M%S)
BUILD="$LV2/build/$SKETCH"

if fuser "$PORT" >/dev/null 2>&1; then
  echo "다른 프로그램이 $PORT 를 쓰고 있습니다(제어 노드·시리얼 모니터를 먼저 끄세요)."; exit 1
fi
echo "== 빌드: firmware/$SKETCH"
arduino-cli compile --fqbn OpenCR:OpenCR:OpenCR --output-dir "$BUILD" "$LV2/firmware/$SKETCH" \
  2>&1 | tee "$LOG_DIR/raw/build_${SKETCH}_${STAMP}.log" | tail -3 || { echo "빌드 실패"; exit 1; }
BIN="$BUILD/$SKETCH.ino.bin"
test -s "$BIN" || { echo "bin 없음: $BIN"; exit 1; }
sha256sum "$BIN" | tee -a "$LOG_DIR/raw/build_${SKETCH}_${STAMP}.log"

echo "== 업로드: $PORT"
opencr_ld "$PORT" 115200 "$BIN" 1 2>&1 | tee "$LOG_DIR/upload_${SKETCH}_${STAMP}.log" | tail -4
if grep -q "CRC OK" "$LOG_DIR/upload_${SKETCH}_${STAMP}.log" && grep -q "\[OK\] Download" "$LOG_DIR/upload_${SKETCH}_${STAMP}.log"; then
  echo "업로드 성공 (기록: results/logs/upload_${SKETCH}_${STAMP}.log)"
else
  echo "업로드 실패: 로그를 확인하세요 (results/logs/upload_${SKETCH}_${STAMP}.log)"; exit 1
fi
