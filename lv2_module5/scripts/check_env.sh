#!/usr/bin/env bash
# 실행 환경 확인 (Raspberry Pi에서 실행). README 1절 표와 같은지 본다. 결과는 화면과 results/logs/env_<시각>.txt
LV2=$(cd "$(dirname "$0")/.." && pwd)
OUT="$LV2/results/logs/env_$(date +%Y%m%d_%H%M%S).txt"
mkdir -p "$(dirname "$OUT")"
{
  echo "== OS";            . /etc/os-release; echo "$PRETTY_NAME $(uname -m)"
  echo "== ROS";           echo "ROS_DISTRO=${ROS_DISTRO:-없음} ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-없음} RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-기본값}"
  echo "   (팀 기준: lyrical · 28 · rmw_cyclonedds_cpp)"
  echo "== Python·OpenCV"; python3 -c "import sys, cv2; print('Python', sys.version.split()[0], '/ OpenCV', cv2.__version__)"
  echo "== apt 패키지"
  dpkg-query -W -f='${Package} ${Version}\n' ros-lyrical-realsense2-camera ros-lyrical-librealsense2 \
    ros-lyrical-cv-bridge ros-lyrical-rmw-cyclonedds-cpp python3-opencv python3-serial 2>&1
  echo "== 장치";          lsusb | grep -E "8086:0b07|0483:5740" || echo "D435(8086:0b07) 또는 OpenCR(0483:5740) 없음"
  ls -l /dev/ttyACM* 2>/dev/null || echo "/dev/ttyACM* 없음"
  id -nG | grep -qw dialout && echo "dialout 그룹: 있음" || echo "dialout 그룹: 없음 (sudo usermod -aG dialout \$USER)"
  ls /etc/udev/rules.d/ | grep -i realsense || echo "RealSense udev 규칙 없음"
  echo "== OpenCR 도구"
  command -v arduino-cli >/dev/null && arduino-cli version || echo "arduino-cli 없음"
  arduino-cli core list 2>/dev/null | grep -i opencr || true
  command -v opencr_ld >/dev/null && file -L "$(command -v opencr_ld)" | cut -d, -f1-2 || echo "opencr_ld 없음 (README 4절)"
  git -C ~/Arduino/libraries/Dynamixel2Arduino log -1 --format='Dynamixel2Arduino %h %cs' 2>/dev/null || echo "Dynamixel2Arduino 없음"
} 2>&1 | tee "$OUT"
echo "저장: $OUT"
