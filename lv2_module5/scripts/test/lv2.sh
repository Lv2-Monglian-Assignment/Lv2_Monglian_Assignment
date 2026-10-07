#!/bin/sh
# 통합 메뉴 tmux 세션 lv2에 붙는다 (sh·bash 모두 동작, install_autostart.sh가 ~/bin/lv2로 연결).
#   tmux 밖(SSH 셸): 붙기   /   tmux 안(예: pi 세션): lv2 세션으로 전환 (메뉴에서 z로 원래 세션으로 돌아감)
#   세션이 없으면 먼저 시작한다.
DIR=$(dirname "$(readlink -f "$0")")
tmux has-session -t lv2 2>/dev/null || "$DIR/lv2_session.sh"
# ssh -X 화면(DISPLAY)을 메뉴 세션에 넘긴다: 메뉴가 실행하는 프로그램(1번 이미지 창 등)이 이 화면에 뜨게.
# tmux 안이면 지금 세션의 값(재접속 때 tmux가 갱신)을, 밖이면 셸의 값을 쓴다.
if [ -n "$TMUX" ]; then D=$(tmux show-environment DISPLAY 2>/dev/null | sed -n 's/^DISPLAY=//p'); else D=$DISPLAY; fi
[ -n "$D" ] && tmux set-environment -t lv2 DISPLAY "$D"
if [ -n "$TMUX" ]; then exec tmux switch-client -t lv2; else exec tmux attach -t lv2; fi
