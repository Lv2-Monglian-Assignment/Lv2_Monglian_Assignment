# 프로젝트 1 — 비전 기반 객체 추적 시스템

카메라로 단일 색상 목표 1개를 HSV·Contour로 검출하고, 정규화 중심 오차(ex, ey)를 ROS2 `/target` 토픽으로 전달하여 OpenCR·DYNAMIXEL 팬(필수)·틸트(선택) P 제어로 추적합니다.
목표 소실·통신 중단 시 안전 정지하고, 시야 내 재등장 시 추적을 재개합니다. 카메라·인지·제어·OpenCR은 **모두 Raspberry Pi에서 실행**합니다.

```
카메라 영상 → HSV·Contour 검출(+깊이 크기 검증) → 중심 오차 → 추적 제어 → OpenCR → DYNAMIXEL → 카메라 방향 변화 → 새 영상의 오차 확인
```

> 이 README만 보고 작성자가 아닌 팀원이 실행·재현할 수 있어야 합니다. `TODO`는 실제 값으로 채운 뒤 삭제합니다.
> 표의 **(결정)** 은 팀이 정했지만 아직 장비에 적용·확인하지 않은 값, **(실측)** 은 장비에서 확인한 값입니다.

---

## 1. 실행 환경 및 장비

| 구분 | 항목 | 상세 사양 및 설정값 |
|---|---|---|
| 소프트웨어 (Raspberry Pi) | 역할 | 카메라·인지·제어 노드, OpenCR 빌드·업로드·시리얼 (모두 Pi에서 실행) |
| | OS · 아키텍처 | Ubuntu Server 26.04.1 LTS · aarch64 |
| | ROS2 | Lyrical (공식 apt 패키지, resolute 빌드) |
| | RMW 구현체 | rmw_cyclonedds_cpp (팀 결정, `~/.bashrc`의 `RMW_IMPLEMENTATION`. 2026-10-08 bag 기록·재현에 사용) |
| | ROS_DOMAIN_ID | 28 (Raspberry Pi·PC 모두 `~/.bashrc` 맨 위에 `export ROS_DOMAIN_ID=28`) |
| | Python | 3.14.4 |
| | OpenCV | 4.10.0 (python3-opencv 4.10.0+dfsg-7ubuntu5) |
| | rosbag2 | 0.33.3 (저장 형식: mcap, 2026-10-08 기록 bag의 `ros2 bag info` Storage id) |
| | arduino-cli · OpenCR 코어 | arduino-cli 1.5.1 · OpenCR 코어 1.5.3 (릴리스 파일 수동 설치, `core list`에는 1.0.0으로 표시) · FQBN `ROBOTIS:OpenCR:OpenCR` |
| | OpenCR 코어 파일 | `https://github.com/ROBOTIS-GIT/OpenCR/releases/download/1.5.3/opencr.tar.bz2` (sha256 `418656e5…`) |
| | 컴파일러 | `arm-none-eabi-g++ 14.2.1` (Ubuntu apt `gcc-arm-none-eabi`) |
| | DYNAMIXEL 라이브러리 | Dynamixel2Arduino 0.8.1 (`~/pa-opencr-build/user/libraries`) |
| | OpenCR 업로더 | `opencr_ld` arm64 소스 빌드 (보드 패키지의 업로더는 x86용이라 Pi에서 실행 불가) |
| 소프트웨어 (PC) | 용도 | Raspberry Pi SSH 접속(`ssh -X`로 화면 확인) · Isaac Sim 실행 |
| | OS · 아키텍처 | Ubuntu 24.04.5 LTS · x86_64 |
| | ROS2 | Lyrical ([NVIDIA Isaac ROS release-5.0](https://nvidia-isaac-ros.github.io/v/release-5.0/getting_started/index.html) 문서의 터미널 설치 절차) |
| | RMW · ROS_DOMAIN_ID | Pi와 같게 rmw_cyclonedds_cpp · 28 (결정, PC에서 Pi 토픽을 볼 때만 필요) |
| | Python | 3.12.3 |
| 하드웨어 | SBC | Raspberry Pi 4 Model B Rev 1.5 · 메모리 4GB (`free -h` 3.7Gi) |
| | Camera | Intel RealSense D435 (D435i 아님) · 펌웨어 5.15.1.55 · USB 3.2 (5 Gbps) 포트 연결 |
| | Camera 설정 | Color **640x480 @ 30 Hz 고정** (실측 30.06 Hz) · rgb8 · frame_id `camera_color_optical_frame` · 왜곡 모델 plumb_bob (계수 0) · 정렬 Depth 640x480 @ 30 Hz (`aligned_depth_to_color`, 실측 30.0 Hz) |
| | Camera 내부 파라미터 (CameraInfo K) | fx 605.85 · fy 605.68 · cx 324.37 · cy 245.14 |
| | Camera ROS 패키지 | realsense2_camera 4.58.4 · librealsense2 2.58.4 · realsense2_camera_msgs 4.58.4 (apt `ros-lyrical-*`) · udev 규칙 `99-realsense-libusb.rules` (librealsense v2.58.4) |
| | Control Board | OpenCR 1.0 · Raspberry Pi와 USB 시리얼 115200 bps (`/dev/ttyACM0`, 사용자 `dialout` 그룹 필요) |
| | Actuator | ROBOTIS DYNAMIXEL XM430-W350-T × 2 · 팬 ID 11 · 틸트 ID 12 · 1,000,000 bps · Protocol 2.0 · 펌웨어 50 (실측 2026-10-03 Serial3 스캔) |
| | Power | 12V 외부 전원 공급 장치 |
| | 기구 | 팬·틸트 2축 (필수 추적은 팬 1축, 틸트는 선택) · ROBOTIS FR12-H101K · 팬 마운트 · 카메라 마운트(틸트 모터) |
| | 회전 범위 · 속도 상한 | 기구 범위 팬 ±180° (틸트 모터 케이블 때문에 연속 회전 금지) · 틸트 ±40°, 기준 자세(IDLE) 팬 0°·틸트 180° (모터 원시값 = tick 0·2048) · 속도 상한 120°/s (Kp 계단 응답 시험과 같은 값) |
| 목표물 | 대상 | 파란색 단일 색 직육면체·원기둥 (밑면 3 × 3 cm, 높이 6 cm) · TODO 사진 `results/images/target.jpg` |

- 카메라 Color 토픽 이름: `/camera/camera/color/image_raw` (640×480 rgb8, 2026-10-08 Pi에서 기록한 bag으로 확인) · 정렬 Depth 토픽은 `/camera/camera/aligned_depth_to_color/image_raw` · 30 Hz
- 모터 ID·baud·프로토콜은 실제 장비에서 확인한 값입니다. 확인 방법과 날짜: TODO
- PC는 Raspberry Pi SSH 접속과 Isaac Sim 실행에 사용합니다. OpenCR 빌드·업로드·시리얼 확인은 Raspberry Pi에서 수행합니다.

## 2. 폴더 구조

```
lv2_module5/
├── README.md            # 실행·재현 가이드 (이 문서)
├── report.md            # 문제 1~5 구현·검증·해석·한계
├── team.md              # 4인 역할·Issue·PR·리뷰, 보호 설정, 통합 확인
├── presentation.md      # 5분 시연 순서와 핵심 결과
├── ros2_ws/src/
│   ├── target_detector/     # [인지] HSV·Contour·크기 검증 → /target
│   ├── tracker_controller/  # [제어] P 제어·제한·IDLE/TRACKING/LOST
│   ├── tracker_bridge/      # [제어·통합] 모터 명령 ↔ OpenCR 시리얼 (#7 합의 중)
│   └── tracker_bringup/     # [통합] perception·control·full·replay launch
├── firmware/            # OpenCR 펌웨어 소스 (통신 타임아웃 정지 포함)
├── config/              # hsv·camera·control·device·safety .yaml
├── scripts/             # check_env.sh, upload_fw.sh, record_bag.sh, replay_bag.sh, mock_target_pub.py
├── results/
│   ├── images/          # 정상·대상 없음·가림 장면 원본/마스크/검출 이미지
│   ├── logs/            # 원본 CSV·상태 로그·시리얼 로그
│   ├── plots/           # Kp 비교 등 그래프
│   └── metrics.csv      # 회차별 성능표
└── recordings/README.md # bag·영상 위치, 메타데이터, 크기·해시, 재생 방법
```

> TODO: `ros2_ws/src`의 노드 코드·`config/*.yaml`·`scripts/`는 아직 빈 파일입니다. 실험용 Pi에서 검증한 인지·제어 코드를 이 구조로 옮기는 PR에서 채웁니다.

## 3. 설치 및 빌드

```bash
# (PC, 한 번만, 선택) SSH 키를 만들어 Pi에 등록하면 접속할 때 비밀번호를 묻지 않음
ssh-keygen -t ed25519            # 이미 키가 있으면 생략
ssh-copy-id <user>@<pi-host>     # 본인 Pi의 사용자·호스트 (예: monglian@monglian.local)

# Raspberry Pi에 SSH 접속 (PC 터미널, 화면 확인이 필요하면 -X)
ssh -X <user>@<pi-host>

# 의존성 설치 (Pi)
sudo apt update
sudo apt install -y ros-lyrical-realsense2-camera ros-lyrical-cv-bridge python3-opencv python3-serial \
                    ros-lyrical-rmw-cyclonedds-cpp ros-dev-tools
sudo usermod -aG dialout $USER   # OpenCR 시리얼 권한 (다시 로그인)

# RealSense udev 규칙 (librealsense 버전과 같은 태그)
curl -fsSL -o /tmp/99-realsense-libusb.rules \
  https://raw.githubusercontent.com/IntelRealSense/librealsense/v2.58.4/config/99-realsense-libusb.rules
sudo install -m 644 /tmp/99-realsense-libusb.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger

# ROS 환경 (~/.bashrc에 한 번 추가, PC도 같은 DOMAIN·RMW)
echo 'source /opt/ros/lyrical/setup.bash' >> ~/.bashrc
echo 'export ROS_DOMAIN_ID=28' >> ~/.bashrc
echo 'export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp' >> ~/.bashrc
source ~/.bashrc

# 저장소 clone (개인별 폴더 사용, 같은 폴더에서 동시에 브랜치 변경 금지)
git clone https://github.com/Lv2-Monglian-Assignment/Lv2_Monglian_Assignment.git
cd Lv2_Monglian_Assignment/lv2_module5/ros2_ws

# 빌드·테스트
colcon build --symlink-install
source install/setup.bash
python3 -m pytest -q src
```

- 비대화형 셸(`ssh pi '명령'`, 스크립트)은 `~/.bashrc`를 읽지 않아 DOMAIN·RMW가 비어 토픽이 안 보입니다. 이때는 명령 앞에서 `export ROS_DOMAIN_ID=28 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`를 함께 설정합니다.
- ROS 노드는 가상환경을 끄고(`deactivate`) 시스템 Python으로 실행합니다.

## 4. OpenCR 펌웨어 빌드·업로드

3절처럼 Pi에 SSH로 접속한 상태에서 진행합니다. 펌웨어 빌드·업로드는 모두 Pi에서 실행합니다.

### OS·아키텍처 확인
```bash
hostname
cat /etc/os-release
uname -m
df -h "$HOME"
```
`uname -m`이 `aarch64`인지 확인합니다 (예: 제출 장비는 Ubuntu 26.04.1 LTS, `aarch64`). 아래 arduino-cli는 ARM64용이므로 다른 아키텍처라면 그에 맞는 파일을 받아야 합니다. 다운로드·설치를 위해 5GB 이상의 여유 공간을 권장합니다.

###  필수도구 준비
```bash
export BASE="$HOME/pa-opencr-build"
set -o pipefail
mkdir -p "$BASE"/{bin,downloads,data,user,sketches,output}
sudo apt update
sudo apt install -y build-essential curl git python3-serial \
  gcc-arm-none-eabi libnewlib-arm-none-eabi \
  libstdc++-arm-none-eabi-newlib usbutils file
arm-none-eabi-g++ --version
sudo usermod -aG dialout "$USER"
```
SSH를 종료하고 다시 접속하여 `id -nG` 출력에 `dialout`이 포함되는지 확인하세요. 새 세션에서는 아래 설정도 다시 적용합니다.

```bash
export BASE="$HOME/pa-opencr-build"
set -o pipefail
```

### Arduino CLI 1.5.1 설치

```bash
cd "$BASE/downloads"
VER=1.5.1
ASSET="arduino-cli_${VER}_Linux_ARM64.tar.gz"
RELEASE="https://github.com/arduino/arduino-cli/releases/download"
curl -fL -o "$ASSET" "$RELEASE/v$VER/$ASSET"
curl -fL -o checksums.txt \
  "$RELEASE/v$VER/${VER}-checksums.txt"
grep "  $ASSET\$" checksums.txt | sha256sum -c -
```

체크섬 검증이 `OK`일 때만 압축을 풀어 실행합니다.

```bash
tar -xzf "$ASSET" -C "$BASE/bin" arduino-cli
file "$BASE/bin/arduino-cli"
"$BASE/bin/arduino-cli" version
```

ARM aarch64 실행 파일과 버전 1.5.1을 확인하세요. `Exec format error`이면 다운로드한 파일과 호스트 아키텍처를 확인합니다.

### OpenCR 코어 1.5.3 설치

```bash
cd "$BASE/downloads"
CORE_URL="https://github.com/ROBOTIS-GIT/OpenCR/releases/download"
curl -fL -o opencr.tar.bz2 "$CORE_URL/1.5.3/opencr.tar.bz2"
CORE_SHA=418656e5e6d99d45d187ffdb28dece0f450c6707da3f6db56769f3ecafdc413c
printf '%s  opencr.tar.bz2\n' "$CORE_SHA" | sha256sum -c -
```

`OK`를 확인한 뒤 다음을 실행합니다. 이 파일은 확장자와 달리 실제로는 gzip 형식이므로 `tar -xf`로 자동 감지합니다.

```bash
mkdir -p "$BASE/user/hardware/ROBOTIS/OpenCR"
tar -xf opencr.tar.bz2 \
  -C "$BASE/user/hardware/ROBOTIS/OpenCR" --strip-components=1
```

설치 후 `core list`에는 `ROBOTIS:OpenCR 1.0.0`으로 표시됩니다. 릴리스 태그는 1.5.3이지만 파일 안 `platform.txt`의 버전 값이 1.0.0이기 때문이며 정상입니다.

### CLI 설정과 라이브러리 설치

```bash
cat > "$BASE/arduino-cli.yaml" <<EOF
directories:
  data: $BASE/data
  downloads: $BASE/downloads
  user: $BASE/user
EOF
"$BASE/bin/arduino-cli" --config-file "$BASE/arduino-cli.yaml" \
  core update-index
"$BASE/bin/arduino-cli" --config-file "$BASE/arduino-cli.yaml" \
  board listall
```

보드 목록에 `OpenCR Board`와 `ROBOTIS:OpenCR:OpenCR`이 있어야 합니다. 이번 수동 설치의 vendor 폴더가 `ROBOTIS`이므로 보드 매니저 설치에서 쓰는 `OpenCR:OpenCR:OpenCR`과 다릅니다.

```bash
mkdir -p "$BASE/user/libraries"
git clone https://github.com/ROBOTIS-GIT/Dynamixel2Arduino.git \
  "$BASE/user/libraries/Dynamixel2Arduino"
git -C "$BASE/user/libraries/Dynamixel2Arduino" checkout \
  cfbbaf79581ecfcdec952a87916572885453f4ab
"$BASE/bin/arduino-cli" --config-file "$BASE/arduino-cli.yaml" lib list   # Dynamixel2Arduino 0.8.1
```

### Ubuntu 컴파일러 연결

원본 `platform.txt`는 유지하고 로컬 설정으로 컴파일러 경로를 지정합니다.

```bash
cat > "$BASE/user/hardware/ROBOTIS/OpenCR/platform.local.txt" <<'EOF'
compiler.path=/usr/bin/
EOF
arm-none-eabi-g++ --version
```

검증한 컴파일러는 Ubuntu 26.04.1(aarch64)의 `arm-none-eabi-g++ 14.2.1`(apt 패키지 `15:14.2.rel1-1`)입니다. 이 조합에서 `opencr_tracker`가 추가 호환 옵션 없이 빌드됐습니다. 다른 버전에서 오류가 발생하면 버전과 첫 오류를 확인하고, 임의의 옵션으로 오류를 숨기지 마세요.

### 저장소 소스 준비와 빌드

빌드할 펌웨어는 `firmware/opencr_tracker`이며 대상은 **XM430-W350 2개(팬 ID 11, 틸트 ID 12), 1 Mbps, Protocol 2.0**입니다. 시작할 때 모델 번호를 확인해 다르면 FAULT로 멈춥니다. 본인 장비의 ID·baud가 다르면 소스의 `IDS`·`DXL_BAUD`를 맞추고, 모델 검사는 제거하지 마세요.

```bash
# 3절에서 clone했다면 그 폴더를 사용 (없으면 clone)
cd ~
test -d Lv2_Monglian_Assignment || git clone https://github.com/Lv2-Monglian-Assignment/Lv2_Monglian_Assignment.git
cd ~/Lv2_Monglian_Assignment/lv2_module5
git switch main && git pull     # 다른 브랜치를 빌드할 때: git fetch && git switch <브랜치>
git log -1 --oneline            # 빌드한 커밋을 기록
git status --short firmware/    # 출력이 없으면 저장소와 같은 소스
SKETCH="$PWD/firmware/opencr_tracker"
sha256sum "$SKETCH/opencr_tracker.ino"
```

PC에서 수정한 파일을 Pi에서 빌드해 보려면 같은 위치로 복사합니다. 복사 후 Pi의 `git status`에 `M`으로 표시되며, 스케치 폴더 이름과 `.ino` 이름은 같아야 합니다(`opencr_tracker/opencr_tracker.ino`).

```bash
# (PC)
scp opencr_tracker.ino <user>@<pi-host>:~/Lv2_Monglian_Assignment/lv2_module5/firmware/opencr_tracker/
```

```bash
set -o pipefail
"$BASE/bin/arduino-cli" --config-file "$BASE/arduino-cli.yaml" \
  compile --fqbn ROBOTIS:OpenCR:OpenCR --jobs 1 \
  --output-dir "$BASE/output/opencr_tracker" \
  "$SKETCH" 2>&1 | tee "$BASE/build_opencr_tracker.log"
```

빌드가 성공했을 때만 아래 결과를 확인합니다. 실패했다면 이전에 생성된 파일을 새 빌드 결과로 사용하지 마세요.

```bash
OUT="$BASE/output/opencr_tracker"
test -s "$OUT/opencr_tracker.ino.bin"
file "$OUT/opencr_tracker.ino.elf"
arm-none-eabi-size "$OUT/opencr_tracker.ino.elf"
sha256sum "$OUT/opencr_tracker.ino.bin"
```

`.elf`는 OpenCR 타깃의 섹션·주소·심볼 정보를 포함하고, `.bin`은 업로드할 원시 펌웨어입니다. 빌드 성공은 파일 생성 완료를 의미하며 장치 업로드 성공과는 별개입니다.

같은 커밋이라도 컴파일러 버전이 다르면 `.bin`의 크기·해시가 달라질 수 있습니다. 빌드 기록에는 커밋(`git log -1`), 소스 sha256, `.bin` sha256을 함께 남깁니다.

### 라즈베리파이용 업로더 빌드

보드 패키지에 들어 있는 업로더(`opencr_ld`)는 x86용이라 Pi에서 실행되지 않으므로(`arduino-cli upload`도 동작하지 않음) 소스에서 한 번 빌드합니다. 펌웨어는 `arm-none-eabi-g++`로 OpenCR용 파일을 만듭니다. 아래 업로더는 일반 `gcc`로 라즈베리파이에서 실행할 파일을 만듭니다.

```bash
mkdir "$BASE/uploader-src"
cd "$BASE/uploader-src"
git init
git remote add origin https://github.com/ROBOTIS-GIT/OpenCR.git
git sparse-checkout init --cone
git sparse-checkout set arduino/opencr_develop/opencr_ld
git fetch --depth 1 --filter=blob:none origin \
  68ec75d8a400949580ecf263e0105ea9743b878e
git checkout --detach FETCH_HEAD
make -C arduino/opencr_develop/opencr_ld
file arduino/opencr_develop/opencr_ld/opencr_ld
```

ARM64 라즈베리파이라면 업로더도 ARM aarch64 실행 파일인지 확인합니다 (예: `ELF 64-bit LSB pie executable, ARM aarch64`).

설치가 끝나면 환경 확인 스크립트로 도구 버전을 한 번에 확인할 수 있습니다(결과는 `results/logs/env_<시각>.txt`).

```bash
cd ~/Lv2_Monglian_Assignment/lv2_module5
scripts/check_env.sh
```

### OpenCR 포트 확인과 업로드

OpenCR을 라즈베리파이에 USB로 연결하고 연결 전후의 목록을 비교해 포트를 확인하세요. 다른 시리얼 프로그램이 열려 있으면 종료합니다.

```bash
lsusb
ls -l /dev/ttyACM*
```

아래 `/dev/ttyACM0`은 예시입니다. 실제로 확인한 OpenCR 포트로 지정하세요.

```bash
PORT=/dev/ttyACM0
udevadm info --query=property --name="$PORT"
test -r "$PORT" && test -w "$PORT" && echo 'Port access OK'
fuser -v "$PORT"    # 출력이 없어야 다른 프로그램이 포트를 쓰지 않는 상태 (브리지·시리얼 모니터 종료)
```

이 업로드는 OpenCR의 기존 응용 펌웨어를 교체합니다. 장비 설정과 모터 고정·이동 범위·전원 차단 방법을 확인한 상태에서 수행하세요.

```bash
UPLOADER="$BASE/uploader-src/arduino/opencr_develop/opencr_ld/opencr_ld"
set -o pipefail
"$UPLOADER" "$PORT" 115200 \
  "$OUT/opencr_tracker.ino.bin" 1 \
  2>&1 | tee "$BASE/upload_opencr_tracker.log"
```

위에서 빌드한 `$OUT/opencr_tracker.ino.bin`을 업로드합니다.

출력에 `CRC OK`와 `[OK] Download`가 **모두** 있는지 확인합니다. 이 업로더는 내부 오류가 있어도 종료 코드가 성공으로 보일 수 있으므로 종료 코드만으로 성공을 판단하지 않습니다.

#### 스크립트로 한 번에 빌드·업로드

도구를 설치한 뒤에는 위 빌드·업로드를 명령 하나로 할 수 있습니다. 빌드 출력은 화면에만 나오고, 업로드 기록(커밋·소스 변경 여부·sha256·업로더 출력)은 `results/logs/upload_<스케치>_<시각>.log`에 남습니다. 성공 판정 기준(`CRC OK`와 `[OK] Download`)은 위와 같습니다.

```bash
cd ~/Lv2_Monglian_Assignment/lv2_module5
scripts/upload_fw.sh                      # firmware/opencr_tracker
PORT=/dev/ttyACM1 scripts/upload_fw.sh    # 포트가 다를 때
```

### 업로드 이후 동작 확인

포트 번호가 달라질 수 있으므로 다시 확인합니다. 저장소의 시험 스크립트로 펌웨어 동작을 확인합니다(ROS 없이 시리얼만 사용).

```bash
ls -l /dev/ttyACM*
PORT=/dev/ttyACM0
cd ~/Lv2_Monglian_Assignment/lv2_module5
python3 scripts/test/fw_test.py --port "$PORT"
```

- 처음에 `상태: OFF  각도: …`가 나오면 펌웨어가 동작하고 모터 ID·baud·전원·모델 확인을 통과한 것입니다. 상태 줄이 없거나 `FAULT`이면 안내 문구에 따라 확인합니다. 여기까지는 모터가 움직이지 않으므로, 움직이지 않고 끝내려면 이때 `Ctrl+C`를 누릅니다.
- Enter를 누르면 보드 타임아웃 시험을 합니다. 토크가 켜지고 팬이 약 1.5° 움직였다가 약 300 ms 뒤 멈추면 `PASS`입니다(기대: 약 300 ms, HOLD).
- 기록은 `~/lv2_module5_logs/fw_test_<날짜시간>.log`에 남습니다.
- 보드 측 통신 타임아웃: 속도 명령이 300 ms 없으면 속도 0, 토크는 유지(`CMD_TIMEOUT_MS`, `TORQUE_OFF_AFTER_MS = 0`). 모터 Bus Watchdog 200 ms.
- 주의: OpenCR 전원이 켜진 상태에서 모터 케이블을 빼거나 꽂지 마세요(보드·모터에 무리가 가고 FAULT의 원인이 됩니다). 연결을 바꿀 때는 전원을 끄고 바꾼 뒤 다시 켭니다.

## 5. 실행

> 처음에는 모터 출력을 끈 상태로 데이터 전달을 확인하고, 실제 회전은 낮은 속도·제한 범위에서 수행합니다. FPS·지연 측정과 bag 기록 중에는 화면 미리보기를 끕니다(켜면 처리 FPS 30 → 약 14).

```bash
# 터미널 1: RealSense 카메라 (Color·정렬 Depth 640x480 @ 30 Hz)
ros2 launch realsense2_camera rs_launch.py \
  rgb_camera.color_profile:=640x480x30 depth_module.depth_profile:=640x480x30 align_depth.enable:=true

# 터미널 2: 인지 노드  (TODO: 이식 PR 후 동작)
ros2 launch tracker_bringup perception.launch.py

# 터미널 3: 제어 노드
ros2 launch tracker_bringup control.launch.py

# 또는 전체 실행
ros2 launch tracker_bringup full.launch.py
```

| 설정 파일 | 내용 (현재 실험 값) |
|---|---|
| `config/hsv.yaml` | HSV `[102,120,40]`~`[110,255,255]` (TODO 팀 확정) · 커널 5 px · 최소 면적 100 px² · 크기 검증 2.0~30 cm² (깊이로 환산, 가림 장면 포함) · 선택 규칙 priority(큼 → 가까움 → 화면 중앙) |
| `config/camera.yaml` | Color·정렬 Depth 640x480 @ 30 Hz |
| `config/control.yaml` | 각도 루프 Kp 팬 2.0·틸트 2.5 [1/s] (report 3-1) → 추적 Kp 팬 60.5·틸트 56.8 [°/s per 1.0 정규화 오차] (= 각도 Kp × 30.26°·22.70°) · direction 팬 −1·틸트 +1 · 데드밴드 0.03·0.05 · 제어 주기 50 Hz |
| `config/safety.yaml` | 입력 타임아웃 0.5 s · 복귀 조건 연속 3프레임 · 보드 타임아웃 300 ms · 속도 상한 120°/s |
| `config/device.yaml` | 팬 ID 11·틸트 ID 12 · 1 Mbps · Protocol 2.0 · 팬 ±180°·틸트 ±40° · 기준 tick 0·2048 · `/dev/ttyACM0` 115200 bps |

- Kp 단위 주의: report 3-1의 Kp는 **각도 오차 [°] → 각속도 [°/s]** 기준입니다. 추적 노드는 **정규화 오차 ex**를 쓰므로 화면 중심 근처 기울기(ex 1.0 = 320/fx rad = 30.26°, ey 1.0 = 240/fy rad = 22.70°)로 환산합니다. 영상 지연이 있어 추적 Kp 2종 × 3회 시험으로 다시 확인합니다.

### 동작 확인

```bash
ros2 topic hz /camera/camera/color/image_raw   # 카메라 영상 약 30 Hz
ros2 topic echo /target            # 정규화 오차 ex/ey, z=면적비(0=미검출)
ros2 topic echo /tracking_status   # IDLE / TRACKING / LOST (+사유)
ros2 topic hz /target              # 처리 주기 (미리보기 끔: 28.7~30 Hz 실측)
```

### 모의 입력 시험 (모터 출력 OFF)

```bash
# x=+0.4, z>0 → 오른쪽 오차를 줄이는 명령(팬 음수)이 나와야 함
ros2 topic pub /target geometry_msgs/msg/PointStamped "{point: {x: 0.4, y: 0.0, z: 0.05}}" -r 30
```

- 실험용 구현에서 다섯 입력(0, +0.4, −0.4, z=0, 발행 중단)과 틸트(y=+0.4) 모두 기대대로 동작했습니다(2026-10-05, 입력 중단 후 0.515 s에 LOST:input_timeout).

## 6. 중지

```bash
# 정상 중지: 각 터미널에서 Ctrl+C (제어 노드는 종료 시 속도 0을 보냄)
# 추적만 멈춤(토크 유지, 카메라 처짐 없음)  (현재 구현 기준, #7 합의 후 갱신)
ros2 topic pub --once /tracking_enable std_msgs/msg/Bool "{data: false}"
```

- 제어 프로그램이 종료되거나 통신이 끊기면 OpenCR 측 타임아웃(300 ms)으로 모터가 정지합니다. 확인 기록: `results/logs/TODO`
- 비상 시: 12V 전원 차단 (토크가 꺼지므로 틸트·카메라를 손으로 받침)
- 통신 중단 시험은 제어 노드를 Ctrl+C가 아니라 `kill -9`로 끊습니다(Ctrl+C는 정지 명령을 보내고 끝나 시험이 안 됨).

## 7. bag 기록 및 재현

bag 목록·접근 위치·메타데이터·재현 확인 기록은 [recordings/README.md](recordings/README.md)에 있습니다.

### 순서대로 따라 하기: 기록 → 확인 → 공유 → 재생

아래 명령은 2026-10-08 Pi(pa23)에서 bag 3개를 기록·재생하고, PC에서 같은 재생을 다시 실행해 확인한 순서입니다. `<pi>`는 Pi 호스트 이름(예: `pa23.local`, 이름이 안 잡히면 IP)입니다.

**0단계. Pi 접속과 준비**

```bash
ssh <user>@<pi>
cd ~/git/Lv2_Monglian_Assignment/lv2_module5
source ros2_ws/install/setup.bash        # ROS·ROS_DOMAIN_ID=28·rmw_cyclonedds_cpp는 ~/.bashrc가 설정
tmux ls                                  # lv2 세션(자동 메뉴)이 있으면 대기 추적이 카메라·포트를 잡고 있음
tmux kill-session -t lv2                 # (lv2가 있을 때만) 메뉴와 대기 추적 종료
fuser /dev/ttyACM0                       # 아무것도 안 나오면 OpenCR 포트가 비어 있음
df -h ~                                  # 여유 공간 확인: 컬러+깊이 640×480 30 fps는 약 30 MB/s (20 s ≈ 600 MB)
```

**1단계. 기록 (Pi, 모터 동작)**

`assignment5.py record`가 노드를 직접 띄우고 추적을 켠 뒤 기록하고, 끝나면 추적을 끄고 노드를 정지합니다. 목표(파란 원통)를 화면 가운데에 두고 실행합니다.

```bash
# 대표 성공: 원통을 책상 위에 두고 기록이 끝날 때까지 손대지 않음
python3 assignment/assignment5.py record --name success --seconds 20
# 소실·복귀: 화면에 "가리세요!"가 나오면 손바닥으로 원통을 덮고, "치우세요!"가 나오면 손을 화면 밖으로 뺌
python3 assignment/assignment5.py record --name lost --seconds 15
# 모터가 따라 움직이는 장면(추가): 안내에 따라 Enter 후 원통을 천천히 좌우로 옮김
scripts/test/motion_guide.sh record
```

- `Enter: 진행`이 나오면 Enter를 한 번 누릅니다. 노드 준비(10~20 s)와 기록기 준비(약 7 s) 뒤에 기록이 시작됩니다.
- **기록 중에는 Ctrl+C를 누르지 않습니다.** `assignment5.py record`를 Ctrl+C로 끊으면 추적이 켜진 채 노드가 남습니다. 그때는 `ros2 topic pub --once -w 1 /tracking_enable std_msgs/msg/Bool "{data: false}"`로 추적을 끄고 남은 launch를 Ctrl+C로 멈춥니다.
- 같은 명령은 통합 메뉴(`tmux attach -t lv2`)의 `5` → `s`(성공)·`l`(소실)로도 실행할 수 있습니다.

**2단계. 기록 확인 (Pi)**

```bash
ls recordings/                           # <run_id>/ (metadata.yaml + .mcap), <run_id>_info.txt
cat recordings/<run_id>_info.txt         # 길이·토픽별 메시지 수·크기·sha256·기준 커밋
```

- `/target`·`/pan_tilt/joint_states` 메시지가 기록 길이 전체에 고르게 있는지 봅니다. Pi 부하로 일부 구간이 빠질 수 있습니다(report.md 5-5).
- `assignment5.py record`는 `recordings/README.md` 끝에 자동으로 표 한 줄을 붙입니다. 커밋 전에 1절 표 형식에 맞게 옮깁니다.

**3단계. PC로 복사와 공유 (PC)**

```bash
cd ~/Lv2_Monglian_Assignment/lv2_module5           # PC의 저장소 위치
rsync -a <user>@<pi>:git/Lv2_Monglian_Assignment/lv2_module5/recordings/<run_id> \
         <user>@<pi>:git/Lv2_Monglian_Assignment/lv2_module5/recordings/<run_id>_info.txt recordings/
(cd recordings/<run_id> && sha256sum *)             # <run_id>_info.txt의 sha256과 같아야 함
```

- bag 원본은 Git에서 제외됩니다(`.gitignore`의 `recordings/*/`). [공유 드라이브](recordings/README.md)에 bag 폴더와 `SHA256SUMS.txt`를 올리고, 저장소에는 `<run_id>_info.txt`와 `recordings/README.md` 표만 커밋합니다.
- 다른 사람은 공유 드라이브에서 bag 폴더를 받아 `recordings/` 아래에 두고 `sha256sum -c SHA256SUMS.txt`로 확인합니다.

**4단계. 재생 — 모터 출력 없음 (Pi 또는 PC)**

PC에서 처음 재생한다면 먼저 워크스페이스를 빌드합니다(`colcon build --symlink-install`, 3절).

```bash
cd ~/git/Lv2_Monglian_Assignment/lv2_module5        # PC는 PC의 저장소 위치
source /opt/ros/lyrical/setup.bash && source ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=28 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
pgrep -af "controller_node|opencr_bridge"           # 아무것도 안 나와야 함 (모터 노드가 있으면 먼저 정지)

python3 assignment/assignment5.py replay    recordings/<run_id>   # 입력 재처리: bag 영상 → 검출기 → /target_replay
python3 assignment/assignment5.py reanalyze recordings/<run_id>   # 결과 재분석: 저장된 /target·상태·명령으로 지표 재계산
```

- 입력 재처리는 bag의 컬러·CameraInfo·정렬 Depth·`/pan_tilt/joint_states`만 `--clock`으로 재생하고, 저장된 `/target`은 재생하지 않습니다. 결과는 `results/assignment5/<run_id>_replay/summary.md`(검출 여부 일치율·ex 차이)에 남습니다.
- 결과 재분석은 검출기를 실행하지 않습니다. 결과는 `results/assignment5/<run_id>/reanalysis.md`와 `results/metrics.csv`에 남습니다.
- motion bag은 `scripts/test/motion_guide.sh replay <run_id>`로 두 단계를 한 번에 실행할 수 있습니다(모터 노드가 있으면 시작하지 않음).
- 같은 PC·Pi에서 로봇이 DOMAIN 28로 실행 중이면, 재생은 `ROS_DOMAIN_ID`를 다른 값(예: 77)으로 바꿔 섞이지 않게 합니다.

**5단계. 재생 화면 보기 (선택)**

```bash
python3 scripts/web_view.py --width 320 --hz 3     # 브라우저: http://<pi>.local:8080/ (PC에서 실행하면 http://localhost:8080/)
```

- 웹뷰를 먼저 켠 뒤 4단계를 실행합니다. bag 영상과 관절 각도가 기록 때처럼 바뀝니다(실제 모터는 움직이지 않음). `/target`·상태는 재생하지 않는 토픽이라 비어 있거나 "수신 끊김"인 것이 정상입니다.
- `--local-dds`는 쓰지 않습니다. 다른 노드와 DDS 탐색 범위가 달라 토픽을 받지 못한 경우가 있었습니다.

**6단계. 별도 시연: 기록된 명령으로 실제 모터 재생 (선택, Pi)**

과제의 재현(4단계, 모터 출력 없음)과 구분합니다. bag의 `/pan_tilt/command`만 브리지로 다시 보내 실제 모터를 움직입니다. 카메라·검출·제어 노드는 띄우지 않습니다.

```bash
scripts/test/motor_replay.sh <run_id>
# Enter: bag 시작 자세로 이동 → Enter: 재생 → 원본 각도와 비교표 (results/assignment5/<run_id>_motorplay/)
# 비상 정지: Ctrl+C (재생 중지 → 정지 명령 X) 또는 12V 차단
```

**7단계. 정리 (Pi)**

```bash
pgrep -af "realsense2_camera_node|target_detector|controller_node|opencr_bridge|web_view"   # 남은 노드 없음 확인
rm -rf recordings/<run_id>               # PC·공유 드라이브에 같은 sha256 사본이 있는 것을 확인한 뒤에만
df -h ~
```

### 스크립트별 상세

```bash
# 기록 (Pi): full.launch.py run_id:=<run_id>로 실행·추적 중인 상태에서
scripts/record_bag.sh <run_id> 30          # recordings/<run_id>/ + <run_id>_info.txt(커밋·설정·bag info·sha256)
```

**재현할 때는 반드시 모터 출력을 비활성화합니다.** 재현 스크립트는 제어·브리지 노드를 띄우지 않습니다.

| 재현 | 방법 |
|---|---|
| 입력 재처리 | `scripts/replay_bag.sh <run_id>`: bag의 영상·CameraInfo·정렬 Depth·모터 각도만 `--clock`으로 재생 → `target_detector`(`use_sim_time`)가 **`/target_replay`** 로 출력 → `recordings/<run_id>_replay/`로 기록 |
| 결과 재분석 | `python3 scripts/analyze_bag.py recordings/<run_id> --csv ~/lv2_module5_logs/<run_id>.csv --replay recordings/<run_id>_replay --save`: 저장된 `/target`·상태·명령으로 지표를 다시 계산해 실행 중 CSV와 대조하고, 재처리 결과와 같은 영상 stamp끼리 비교 |

토픽 선택·remap은 [replay.launch.py](ros2_ws/src/tracker_bringup/launch/replay.launch.py)에 있습니다. 직접 실행하려면 아래 명령을 씁니다.

```bash
ros2 launch tracker_bringup replay.launch.py run_id:=<run_id>_replay      # 검출기만, 출력 /target_replay*
ros2 bag play recordings/<run_id> --clock --topics \
  /camera/camera/color/image_raw /camera/camera/color/camera_info \
  /camera/camera/aligned_depth_to_color/image_raw /pan_tilt/joint_states
```

- 저장된 `/target`은 재생하지 않아, 새 검출 결과와 섞이지 않습니다.
- 분석은 bag의 시각만 씁니다. 과거 bag 시각과 현재 벽시계를 섞어 타임아웃·지연을 계산하지 않습니다.
- 검출의 번호 유지(같은 색 물체 여러 개 구분)는 촬영 순간의 모터 각도를 쓰므로 재처리에도 `/pan_tilt/joint_states`가 필요합니다.
- 오프라인 재현은 실제 하드웨어 폐루프 시연과 별개입니다.

## 8. 결과 위치

| 결과 | 위치 |
|---|---|
| 3종 장면 이미지 (정상·대상 없음·가림) | `results/images/` |
| 검출률 평가 (튜닝에 쓰지 않은 대상 30·없음 10프레임, 프레임별 정답) | `results/` TODO |
| Kp 계단 응답(각도 루프) 시험 | `results/logs/kp*`, `results/plots/kp_step_*.png`, [report.md](report.md) 3-1 |
| 추적 Kp 2종 × 3회 CSV | `results/logs/` TODO |
| 성능표 (FPS·검출률·RMSE·복구) | `results/metrics.csv` |
| bag·시연 영상 | `recordings/README.md` |
| 해석 및 한계 | [report.md](report.md) |

## 9. 재현 확인 기록

| 확인자 | 날짜 | 기준 커밋 | 수행 내용 | 결과 · 수정 사항 |
|---|---|---|---|---|
| 권형중 (JuneKunst) | 2026-10-08 | 8f897bb (Pi) | 실행(`full.launch.py`)·정지·bag 기록 3개·입력 재처리·결과 재분석, motion bag 실제 모터 재생 시연 | 재처리 검출 여부 일치 99.1 % / 83.1 % / 89.1 % ([report.md 문제 5](report.md#문제-5--ros2-bag-및-재현-기록)). 수정: 웹뷰 `--local-dds` 사용 시 토픽 미수신 → 옵션 없이 실행, 추적 켜기 확인 도구 `scripts/test/tracking_set.py` 추가 |

## 인지 구현과 재현

[인지 실행 안내](docs/perception/README.md)에 두 패키지 빌드, 선택적 카메라 launch, 촬영·검출·평가 명령과 산출물 경로를 정리했다. 설정 원본은 `config/hsv.yaml`이다. 실제 인지 실험에서는 Cyclone DDS·DOMAIN 30으로 Pi에서 검출하고 PC에서 `/target`을 받았다. 위 환경 표의 기본 Fast DDS와 구별하며 최종 제출 장비의 공통 RMW·DOMAIN 값은 통합 단계에서 확정한다.

[인지 보고서](docs/perception/report.md)와 [수행 계획](../vision_todo/vision_todo.md)은 기존 검증 5/7(약 71%)과 미완료 평가를 구분한다. 새 패키지는 PC에서 빌드·합성 검사·카메라 없는 launch 기동/종료를 확인했다. 새 패키지로 Pi 카메라 실측을 재수행한 결과는 아직 없다. 제어·통합의 빈 launch·설정은 그대로 유지한다.
