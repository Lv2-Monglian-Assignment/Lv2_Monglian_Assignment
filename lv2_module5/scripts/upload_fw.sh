#!/usr/bin/env bash
# OpenCR 펌웨어 빌드·업로드 (Raspberry Pi에서 실행, 도구 설치는 README 4절)
#   scripts/upload_fw.sh                 # firmware/opencr_tracker (추적용, 프로토콜 #7)
#   scripts/upload_fw.sh dxl_scan        # 모터 스캔용
#   PORT=/dev/ttyACM1 scripts/upload_fw.sh   # 포트가 다를 때 (BASE도 같은 방식으로 바꿀 수 있음)
# "CRC OK"와 "[OK] Download"가 모두 있어야 성공으로 본다. 업로드 기록은 results/logs/에 남긴다(빌드 출력은 화면만).
set -o pipefail
SKETCH=${1:-opencr_tracker}
PORT=${PORT:-/dev/ttyACM0}
BASE=${BASE:-$HOME/pa-opencr-build}
CLI=("$BASE/bin/arduino-cli" --config-file "$BASE/arduino-cli.yaml")
FQBN=ROBOTIS:OpenCR:OpenCR
UPLOADER="$BASE/uploader-src/arduino/opencr_develop/opencr_ld/opencr_ld"
LV2=$(cd "$(dirname "$0")/.." && pwd)
LOG_DIR="$LV2/results/logs"; mkdir -p "$LOG_DIR"
STAMP=$(date +%Y%m%d_%H%M%S)
OUT="$BASE/output/$SKETCH"

test -x "${CLI[0]}" || { echo "arduino-cli 없음: ${CLI[0]} (README 4절)"; exit 1; }
test -x "$UPLOADER" || { echo "opencr_ld 없음: $UPLOADER (README 4절)"; exit 1; }
test -f "$LV2/firmware/$SKETCH/$SKETCH.ino" || { echo "스케치 없음: firmware/$SKETCH"; exit 1; }
if fuser "$PORT" >/dev/null 2>&1; then
  echo "다른 프로그램이 $PORT 를 쓰고 있습니다(제어 노드·시리얼 모니터를 먼저 끄세요)."; exit 1
fi
echo "== 빌드: firmware/$SKETCH"
BIN="$OUT/$SKETCH.ino.bin"
rm -f "$BIN"     # 빌드가 실패했을 때 이전 bin을 올리지 않도록
if ! BUILD_OUT=$("${CLI[@]}" compile --fqbn "$FQBN" --jobs 1 --output-dir "$OUT" "$LV2/firmware/$SKETCH" 2>&1); then
  echo "$BUILD_OUT"; echo "빌드 실패"; exit 1
fi
echo "$BUILD_OUT" | tail -2
test -s "$BIN" || { echo "bin 없음: $BIN"; exit 1; }

echo "== 업로드: $PORT"
LOG="$LOG_DIR/upload_${SKETCH}_${STAMP}.log"
CHANGED=$(git -C "$LV2" status --short "firmware/$SKETCH" | tr '\n' ' ')
{
  echo "# 시각: $(date '+%F %T')  호스트: $(hostname)  포트: $PORT"
  echo "# 커밋: $(git -C "$LV2" log -1 --format='%h %s')"
  echo "# 소스 변경: ${CHANGED:-없음}"
  echo "# ino sha256: $(sha256sum "$LV2/firmware/$SKETCH/$SKETCH.ino" | cut -d' ' -f1)"
  echo "# bin sha256: $(sha256sum "$BIN" | cut -d' ' -f1)  크기: $(stat -c %s "$BIN") B"
} > "$LOG"
"$UPLOADER" "$PORT" 115200 "$BIN" 1 2>&1 | tee -a "$LOG" | tail -4
if grep -q "CRC OK" "$LOG" && grep -q "\[OK\] Download" "$LOG"; then
  echo "업로드 성공 (기록: results/logs/$(basename "$LOG"))"
else
  echo "업로드 실패: 로그를 확인하세요 (results/logs/$(basename "$LOG"))"; exit 1
fi
