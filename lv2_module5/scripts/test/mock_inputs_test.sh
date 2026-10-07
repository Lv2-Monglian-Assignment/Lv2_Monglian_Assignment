#!/usr/bin/env bash
# 문제 2: 모터 출력을 끈 상태의 모의 입력 시험 (Pi의 SSH·tmux 창에서 실행)
#   tracker_controller만 실행 (opencr_bridge를 띄우지 않음 → OpenCR·모터에 명령이 가지 않음)
#   + scripts/mock_target_pub.py (mode:=fixed) 로 발제의 다섯 입력과 틸트 확인용 입력 1개를 넣는다.
# 경우마다 tracker를 새로 띄워 run_id별 CSV를 남기고, 끝에 summarize_mock.py로 판정표를 만든다.
source /opt/ros/lyrical/setup.bash
LV2=$(cd "$(dirname "$0")/../.." && pwd)
source "$LV2/ros2_ws/install/setup.bash"
set -u   # ROS setup 파일은 정의 안 된 변수를 쓰므로 source 뒤에 켠다
CFG="$LV2/config"
LOG=~/lv2_module5_logs
TAG=mock_$(date +%Y%m%d_%H%M%S)
mkdir -p "$LOG"

run_case() {  # run_case <이름> <mock 실행 초> <mock 파라미터...>
  local name=$1 secs=$2
  shift 2
  echo "=== ${name} (${secs}s): $*"
  local files=()
  for f in "$CFG"/*.yaml; do files+=(--params-file "$f"); done   # launch와 같게 config 전부
  # -p는 /** 항목이라 yaml의 tracker_controller 항목(auto_enable: false)에 진다 → 노드 이름 yaml을 마지막에 넘김
  local ovr="$LOG/${TAG}_${name}_override.yaml"
  printf 'tracker_controller:\n  ros__parameters:\n    auto_enable: true\n    run_id: %s\n' "${TAG}_${name}" > "$ovr"
  ros2 run tracker_controller controller_node --ros-args "${files[@]}" --params-file "$ovr" \
    > "$LOG/${TAG}_${name}_tracker.out" 2>&1 &
  local tracker=$!
  sleep 2   # tracker 준비 (이 동안은 입력 없음 → LOST:input_timeout 이 정상)
  timeout -s INT "$secs" python3 "$LV2/scripts/mock_target_pub.py" --ros-args \
    -p hfov_deg:=55.7 -p vfov_deg:=43.2 "$@" > "$LOG/${TAG}_${name}_mock.out" 2>&1
  sleep 1   # 발행이 끝난 뒤의 정지까지 기록
  kill -INT "$tracker"
  wait "$tracker" 2>/dev/null
}

run_case c1_center   4 -p ex:=0.0  -p area:=0.05                    # x=0, z>0 → 회전 없음
run_case c2_right    4 -p ex:=0.4  -p area:=0.05                    # x=+0.4 → 오른쪽 오차를 줄이는 명령
run_case c3_left     4 -p ex:=-0.4 -p area:=0.05                    # x=-0.4 → 반대 방향
run_case c4_nodetect 4 -p ex:=0.4  -p area:=0.0                     # z=0 → 이전 목표를 쫓지 않고 정지
run_case c5_silence  4 -p ex:=0.4  -p area:=0.05 -p duration_s:=2.0 # 2초 후 발행 중단 → 타임아웃 정지
run_case c6_down     4 -p ey:=0.4  -p area:=0.05                    # (2축 확인) y=+0.4 → 아래쪽 오차를 줄이는 틸트 명령

python3 "$LV2/scripts/test/summarize_mock.py" "$LOG" "$TAG"
echo "MOCK_TEST_END $TAG"
