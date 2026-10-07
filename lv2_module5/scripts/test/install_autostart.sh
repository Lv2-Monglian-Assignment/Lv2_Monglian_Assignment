#!/usr/bin/env bash
# 부팅 자동 실행 설정 (Raspberry Pi, sudo 필요 없음): 전원이 들어오면 통합 메뉴가 tmux 세션 lv2에서 실행된다.
#   scripts/test/install_autostart.sh            # 사용자 crontab에 @reboot 등록 + ~/bin/lv2 명령(세션에 붙기, sh·bash 모두)
#   scripts/test/install_autostart.sh --remove   # 둘 다 지움
# 확인: crontab -l, 재부팅 후 SSH 접속 → lv2
LV2=$(cd "$(dirname "$0")/../.." && pwd)
START="$LV2/scripts/test/lv2_session.sh"
LOG="$HOME/lv2_module5_logs/lv2_session.log"
MARK="# lv2_module5 autostart"
mkdir -p "$(dirname "$LOG")"
command -v crontab >/dev/null || { echo "crontab이 없습니다: sudo apt install cron"; exit 1; }
current=$(crontab -l 2>/dev/null | grep -v "$MARK")
if [ "$1" = "--remove" ]; then
  printf '%s\n' "$current" | sed '/^$/d' | crontab -
  sed -i "/$MARK/d" ~/.bashrc
  [ -L ~/bin/lv2 ] && rm ~/bin/lv2
  echo "자동 실행과 lv2 명령을 지웠습니다"; exit 0
fi
{ printf '%s\n' "$current" | sed '/^$/d'; echo "@reboot $START --boot >> $LOG 2>&1 $MARK"; } | crontab -
sed -i "/$MARK/d" ~/.bashrc                      # 예전 bash 전용 alias 정리 (sh에서는 안 보였음)
mkdir -p ~/bin && ln -sf "$LV2/scripts/test/lv2.sh" ~/bin/lv2   # ~/bin은 로그인 셸 PATH에 들어 있다(.profile)
systemctl is-active --quiet cron 2>/dev/null || echo "주의: cron 서비스가 꺼져 있습니다 (sudo systemctl enable --now cron)"
echo "등록: $(crontab -l | grep "$MARK")"
echo "lv2 명령: $(readlink -f ~/bin/lv2) (SSH 셸에서는 붙기, tmux 안에서는 전환)"
echo "지금 바로 시작: $START"
