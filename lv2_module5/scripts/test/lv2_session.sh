#!/usr/bin/env bash
# 통합 메뉴(assignment/main.py)를 tmux 세션 lv2에서 실행한다 (Raspberry Pi).
#   부팅 때: cron @reboot가 --boot로 부른다(install_autostart.sh가 등록). 네트워크·장치가 잡히도록 잠시 기다린 뒤 시작
#   손으로:  scripts/test/lv2_session.sh   (이미 있으면 그대로 둠)
# 붙기: tmux attach -t lv2 (또는 lv2). 나오기: 메뉴에서 z 또는 Ctrl+b d. 메뉴를 x로 끝내도 세션에는 셸이 남는다.
LV2=$(cd "$(dirname "$0")/../.." && pwd)
SESSION=lv2
TMUX_BIN=$(command -v tmux || echo /usr/bin/tmux)
[ "$1" = "--boot" ] && sleep "${LV2_BOOT_DELAY:-15}"
"$TMUX_BIN" has-session -t "$SESSION" 2>/dev/null && { echo "세션 $SESSION 이미 실행 중"; exit 0; }
# cron 환경의 SHELL은 /bin/sh라 tmux 서버의 기본 셸이 sh가 된다(같은 서버의 다른 세션도 sh로 열림, 2026-10-07).
# 로그인 셸(bash)로 맞춘다.
export SHELL=$(getent passwd "$(id -un)" | cut -d: -f7)
"$TMUX_BIN" new-session -d -s "$SESSION" -x 200 -y 50 \
  "bash -c 'source /opt/ros/lyrical/setup.bash; source \"$LV2/ros2_ws/install/setup.bash\"; \
export ROS_DOMAIN_ID=28 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp; cd \"$LV2\"; python3 assignment/main.py; exec bash'"
"$TMUX_BIN" set-option -g default-shell "$SHELL" >/dev/null
echo "세션 $SESSION 시작 ($(date '+%F %T'), 기본 셸 $SHELL)"
