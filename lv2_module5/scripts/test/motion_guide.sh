#!/usr/bin/env bash
# 모터가 움직이는 추적 장면 bag 기록과 재생(모터 출력 없음) 안내.
#   scripts/test/motion_guide.sh record            # 노드 실행 → 추적 켬 → 15 s 기록 → 추적 끔 → 노드 정지
#   scripts/test/motion_guide.sh replay <run_id>   # 모터 노드 없이 bag 재생 → 검출기 재처리(/target_replay) → 원본과 비교 → 결과 재분석
MODE=${1:?사용: scripts/test/motion_guide.sh record | replay <run_id>}
SEC=15
LV2=$(cd "$(dirname "$0")/../.." && pwd)
T=$LV2/scripts/test
cd "$LV2" || exit 1
source /opt/ros/lyrical/setup.bash
source ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=28 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
LOGS=~/lv2_module5_logs; mkdir -p "$LOGS"
B='\033[1m'; Y='\033[1;33m'; G='\033[1;32m'; R='\033[1;31m'; N='\033[0m'

# 웹뷰는 다른 노드와 같은 DDS 탐색 범위(SUBNET)로 돌린다. --local-dds(LOCALHOST)로 켜면 다른 노드와 연결이 안 되는 경우가 있었다
# tmux 세션 bag에 web 창이 있을 때만 웹뷰를 다시 켠다 (없으면 아무것도 하지 않음)
web_restart() {
  tmux has-session -t bag 2>/dev/null || return 0
  tmux send-keys -t bag:web C-c 2>/dev/null; sleep 1
  tmux send-keys -t bag:web "clear; python3 scripts/web_view.py --width 320 --hz 3" Enter 2>/dev/null
}
web_running() { pgrep -f '[w]eb_view.py --width' >/dev/null; }
motor_nodes() { pgrep -f '[c]ontroller_node|[o]pencr_bridge'; }

if [ "$MODE" = record ]; then
  RUN=motion_$(date +%Y%m%d_%H%M%S)
  clear
  echo -e "${B}━━━━━━━━ 모터 추적 장면 기록 · ${SEC}초 · ${RUN} ━━━━━━━━${N}"
  echo
  echo -e "${Y}① 지금 준비${N}"
  echo "   - 파란 원통을 책상 위, 화면 가운데에 세워 둡니다. 손은 화면 밖에 둡니다."
  echo "   - 로봇 주변(팬·틸트가 도는 범위)에 걸리는 물건이 없는지 봅니다."
  echo "   - 비상 정지: 이 창에서 Ctrl+C (추적을 끄고 노드를 멈춤) 또는 12V 전원 차단"
  echo
  echo -e "${Y}② 노드 실행 (자동, 10~20초)${N} — '준비 완료'가 나올 때까지 기다립니다."
  echo
  LOG=$LOGS/${RUN}_launch.log
  setsid ros2 launch tracker_bringup full.launch.py run_id:=$RUN > "$LOG" 2>&1 &
  LP=$!
  cleanup() {
    trap - EXIT INT TERM HUP
    echo -e "\n${Y}정리: 추적 끄기 → 노드 정지${N}"
    python3 "$T/tracking_set.py" off --timeout 8
    kill -INT -$LP 2>/dev/null
    for _ in $(seq 1 25); do kill -0 $LP 2>/dev/null || break; sleep 1; done
    motor_nodes >/dev/null && echo -e "${R}주의: 모터 노드가 남아 있습니다${N}" || echo "노드 정지 완료"
  }
  trap cleanup EXIT
  trap 'exit 130' INT TERM HUP
  for _ in $(seq 1 60); do
    grep -q "processing FPS" "$LOG" && grep -q "state None -> IDLE" "$LOG" && break
    kill -0 $LP 2>/dev/null || { echo -e "${R}노드 실행 실패: $LOG${N}"; exit 1; }
    sleep 1
  done
  grep -q "processing FPS" "$LOG" || { echo -e "${R}카메라·검출기가 준비되지 않았습니다(재시도 하세요): $LOG${N}"; exit 1; }
  web_restart
  echo -e "${G}준비 완료${N} (추적 꺼짐 · 모터 정지 상태). 웹뷰 http://<pi>.local:8080/ 에서 초록 박스를 확인하세요."
  echo
  echo -e "${Y}③ 시작${N}  Enter 를 누르면 추적을 켜고 기록기를 준비합니다 (약 7초)."
  echo -e "${Y}④ '기록 시작' 줄이 나오면${N} (${SEC}초 동안)"
  echo "   - 원통을 손끝으로 잡고 책상 위에서 천천히 옆으로 밀어 옮깁니다: 가운데 → 왼쪽 → 오른쪽 → 가운데."
  echo "   - 한 번에 한 뼘 정도, 2~3초에 걸쳐 천천히. 로봇이 따라 도는지 웹뷰로 봅니다."
  echo "   - 원통을 손으로 감싸 가리지 말고, 손가락 끝만 닿게 합니다."
  echo -e "${Y}⑤ 끝${N}  '== sha256'과 '메타데이터:' 줄이 나오면 끝입니다. 원통에서 손을 뗍니다."
  echo
  read -r -p "▶ 준비되면 Enter: "
  if ! python3 "$T/tracking_set.py" on; then
    echo -e "${R}추적이 켜지지 않아 기록하지 않습니다. 로그: $LOG${N}"; exit 1
  fi
  echo -e "${G}추적 켬 — 지금 로봇이 원통을 화면 가운데로 맞춥니다.${N} 기록기 준비 중(약 7초, 아직 원통을 움직이지 마세요)..."
  trap 'echo -e "\n${R}중단 요청 — 기록을 닫은 뒤 정리합니다${N}"' INT   # 기록 중 Ctrl+C: record_bag.sh가 bag을 닫고, 그다음 정리
  scripts/record_bag.sh "$RUN" "$SEC"
  trap 'exit 130' INT
  echo
  echo -e "${G}기록 끝: recordings/$RUN/${N}"
  exit 0
fi

if [ "$MODE" = replay ]; then
  RUN=${2:?사용: scripts/test/motion_guide.sh replay <run_id>}
  [ -d "recordings/$RUN" ] || { echo "bag 없음: recordings/$RUN"; exit 1; }
  clear
  echo -e "${B}━━━━━━━━ bag 재생 (모터 출력 없음) · ${RUN} ━━━━━━━━${N}"
  echo
  if motor_nodes >/dev/null; then
    echo -e "${R}제어·브리지 노드가 떠 있어 재생하지 않습니다. 먼저 멈추세요:${N}"; motor_nodes; exit 1
  fi
  echo -e "${G}확인: tracker_controller · opencr_bridge 없음 → 모터는 움직이지 않습니다.${N}"
  echo
  web_running || web_restart
  echo -e "${Y}① 재생하면 (약 30초)${N}"
  echo "   - bag의 카메라 영상 · CameraInfo · 정렬 Depth · 모터 각도(/pan_tilt/joint_states)만 bag 시계로 다시 냅니다."
  echo "   - 검출기가 영상을 다시 처리해 /target_replay 로 냅니다. 저장된 /target 은 재생하지 않습니다(섞이지 않게)."
  echo "   - 재처리 결과: results/assignment5/${RUN}_replay/ , 재분석: results/assignment5/${RUN}/"
  echo -e "${Y}② 보는 곳${N}  웹뷰 http://<pi>.local:8080/ (지금 열어 두세요)"
  echo "   - 기록 당시 카메라 영상과 '관절 pan·tilt 각도'가 기록 때처럼 바뀝니다 (실제 모터는 그대로)."
  echo "   - /target·상태 칸은 재생하지 않는 토픽이라 비어 있거나 '수신 끊김'인 것이 정상입니다."
  echo -e "${Y}③ 끝${N}  '입력 재처리' 표와 '결과 재분석' 표가 나오면 끝입니다."
  echo
  read -r -p "▶ 웹뷰를 열어 둔 뒤 Enter: "
  mkdir -p "results/assignment5/$RUN"     # 재분석이 실행 중 제어 CSV와 비교하도록 같은 run_id 기록을 옮겨 둔다
  cp -n "$LOGS/$RUN.csv" "$LOGS/${RUN}_detect.csv" "$LOGS/${RUN}_serial.log" "results/assignment5/$RUN/" 2>/dev/null
  echo -e "${B}━━━━ 입력 재처리 ━━━━${N}"
  python3 assignment/assignment5.py replay "recordings/$RUN" 2>&1 | grep -v -i deprecat | grep -v read_next
  echo
  echo -e "${B}━━━━ 결과 재분석 ━━━━${N}"
  python3 assignment/assignment5.py reanalyze "recordings/$RUN" 2>&1 | grep -v -i deprecat | grep -v read_next
  echo
  echo -e "${G}재생 끝${N}"
fi
