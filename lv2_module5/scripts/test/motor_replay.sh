#!/usr/bin/env bash
# bag에 기록된 /pan_tilt/command를 실제 모터로 다시 보내는 시연 (과제의 "모터 출력 끈 재현"과 별개).
#   scripts/test/motor_replay.sh <run_id>
# 브리지(opencr_bridge)만 띄운다. 카메라·검출기·제어 노드는 띄우지 않는다(속도 명령만 그대로 재생, 영상 반응 없음).
# 결과: recordings/<run_id>_motorplay/ (실제 각도·재생 명령 bag), results/assignment5/<run_id>_motorplay/ (비교표·그래프)
RUN=${1:?사용: scripts/test/motor_replay.sh <run_id>}
LV2=$(cd "$(dirname "$0")/../.." && pwd)
T=$LV2/scripts/test
cd "$LV2" || exit 1
source /opt/ros/lyrical/setup.bash
source ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=28 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
LOGS=~/lv2_module5_logs; mkdir -p "$LOGS"
BAG=recordings/$RUN
OUT=recordings/${RUN}_motorplay
RES=results/assignment5/${RUN}_motorplay
B='\033[1m'; Y='\033[1;33m'; G='\033[1;32m'; R='\033[1;31m'; N='\033[0m'
[ -d "$BAG" ] || { echo "bag 없음: $BAG"; exit 1; }
[ -e "$OUT" ] && { echo "이미 있음: $OUT (지우고 다시 실행)"; exit 1; }

clear
echo -e "${B}━━━━━━━━ 실제 모터 재생 시연 · ${RUN} ━━━━━━━━${N}"
echo
if pgrep -f '[c]ontroller_node|[o]pencr_bridge|[r]ealsense2_camera_node|[t]arget_detector' >/dev/null; then
  echo -e "${R}다른 노드가 떠 있어 시작하지 않습니다:${N}"; pgrep -af '[c]ontroller_node|[o]pencr_bridge|[r]ealsense2_camera_node|[t]arget_detector'; exit 1
fi
fuser /dev/ttyACM0 >/dev/null 2>&1 && { echo -e "${R}/dev/ttyACM0을 다른 프로그램이 쓰고 있습니다${N}"; exit 1; }
read -r P0 T0 < <(python3 - "$BAG" <<'EOF' 2>/dev/null
import sys, math, rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import JointState
r = rosbag2_py.SequentialReader()
r.open(rosbag2_py.StorageOptions(uri=sys.argv[1], storage_id=''), rosbag2_py.ConverterOptions('cdr', 'cdr'))
r.set_filter(rosbag2_py.StorageFilter(topics=['/pan_tilt/joint_states']))
while r.has_next():
    m = deserialize_message(r.read_next()[1], JointState)
    if list(m.name[:2]) == ['pan', 'tilt']:
        print(f'{math.degrees(m.position[0]):.2f} {math.degrees(m.position[1]):.2f}'); break
EOF
)
[ -n "$P0" ] || { echo "bag에서 시작 자세를 읽지 못했습니다"; exit 1; }
DUR=$(ros2 bag info "$BAG" | awk '/^Duration/{print $2}')

echo -e "${Y}이 시연은${N}"
echo "   - bag에 기록된 팬·틸트 속도 명령(/pan_tilt/command)을 ${DUR} 동안 기록 때와 같은 시간 간격으로 실제 모터에 보냅니다."
echo "   - 카메라를 보지 않습니다. 원통 위치와 상관없이 기록 때 명령 그대로 움직입니다."
echo "   - 안전: 펌웨어 각도 제한, 명령이 끊기면 0.2 s(브리지)·0.3 s(보드) 안에 정지."
echo -e "   - ${R}비상 정지: 이 창에서 Ctrl+C (재생 중지 → 정지 명령 X) 또는 12V 전원 차단${N}"
echo
echo -e "${Y}① 지금 준비${N}"
echo "   - 로봇 주변(팬 −50°~0°, 틸트 −10°~30° 범위)에 걸리는 물건과 손이 없는지 봅니다."
echo "   - 12V 전원 스위치에 손이 닿는 곳에 있습니다."
echo
echo "──────── 브리지 실행 (모터 연결) ────────"
BLOG=$LOGS/${RUN}_motorplay_launch.log
setsid ros2 run tracker_bridge opencr_bridge --ros-args --params-file config/device.yaml -p run_id:=${RUN}_motorplay > "$BLOG" 2>&1 &
BP=$!
REC=
cleanup() {
  trap - EXIT INT TERM HUP
  echo -e "\n${Y}정리: 재생 중지 → 정지(X) → 브리지 종료${N}"
  pkill -INT -f "[r]os2 bag play $BAG" 2>/dev/null
  [ -n "$REC" ] && { kill -INT $REC 2>/dev/null; wait $REC 2>/dev/null; }
  kill -INT -$BP 2>/dev/null
  for _ in $(seq 1 15); do kill -0 $BP 2>/dev/null || break; sleep 1; done
  pgrep -f '[o]pencr_bridge' >/dev/null && echo -e "${R}주의: 브리지가 남아 있습니다${N}" || echo "브리지 종료 (모터 정지·토크 유지)"
}
trap cleanup EXIT
trap 'exit 130' INT TERM HUP
for _ in $(seq 1 30); do grep -q "OpenCR state" "$BLOG" && break; kill -0 $BP 2>/dev/null || break; sleep 0.5; done
grep -q "OpenCR state" "$BLOG" || { echo -e "${R}OpenCR 상태를 받지 못했습니다: $BLOG${N}"; exit 1; }
python3 "$T/pose_go.py" --show || exit 1
echo
echo -e "${Y}② 시작 자세로 이동${N}  bag 시작 자세: pan ${P0}°  tilt ${T0}°  (최대 15°/s로 천천히)"
read -r -p "▶ Enter: 시작 자세로 이동 "
python3 "$T/pose_go.py" --pan "$P0" --tilt "$T0" || { echo -e "${R}시작 자세로 가지 못했습니다${N}"; exit 1; }
echo
echo -e "${Y}③ 재생${N}  Enter 를 누르면 기록기를 켠 뒤 1초 뒤부터 ${DUR} 동안 모터가 기록 때처럼 움직입니다."
echo "   웹뷰(http://<pi>.local:8080/)에서는 관절 각도·명령만 바뀝니다 (카메라는 꺼져 있음)."
read -r -p "▶ Enter: 재생 시작 "
set -m
ros2 bag record -o "$OUT" --disable-keyboard-controls --topics /pan_tilt/joint_states /pan_tilt/command > "$LOGS/${RUN}_motorplay_bag_record.log" 2>&1 &
REC=$!
set +m
for _ in $(seq 1 100); do
  [ "$(grep -c "Subscribed to topic" "$LOGS/${RUN}_motorplay_bag_record.log" 2>/dev/null)" -ge 2 ] && break
  sleep 0.2
done
echo -e "${G}재생 중...${N}"
# 재생은 앞에서(포그라운드) 돌린다: Ctrl+C가 재생에도 바로 가서 멈추고, 그다음 cleanup이 정지(X)를 보낸다
ros2 bag play "$BAG" --topics /pan_tilt/command --disable-keyboard-controls -d 1 > "$LOGS/${RUN}_motorplay_play.log" 2>&1
sleep 1                                   # 마지막 명령 뒤 정지까지 기록
kill -INT $REC 2>/dev/null; wait $REC 2>/dev/null; REC=
echo -e "${G}재생 끝 — 모터 정지${N}"
python3 "$T/pose_go.py" --show
echo
echo -e "${B}━━━━ 원본 각도와 비교 ━━━━${N}"
python3 "$T/compare_motorplay.py" "$BAG" "$OUT" --out "$RES" 2>&1 | grep -v -i deprecat | grep -v read_next
mkdir -p "$RES"; cp "$BLOG" "$LOGS/${RUN}_motorplay_serial.log" "$RES/" 2>/dev/null
(cd "$OUT" && sha256sum *) > "$RES/sha256.txt" 2>/dev/null
echo
echo -e "${G}시연 끝${N}"
