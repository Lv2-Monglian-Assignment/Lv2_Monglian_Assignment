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
| | RMW 구현체 | rmw_fastrtps_cpp (기본값, `ros2 doctor --report`로 확인) |
| | ROS_DOMAIN_ID | 28 (Raspberry Pi·PC 모두 `~/.bashrc` 맨 위에 `export ROS_DOMAIN_ID=28`) |
| | Python | 3.14.4 |
| | OpenCV | 4.10.0 (python3-opencv 4.10.0+dfsg-7ubuntu5) |
| | rosbag2 | 0.33.3 (저장 형식: TODO mcap / sqlite3) |
| | arduino-cli · OpenCR 보드 패키지 | arduino-cli 1.5.1 · OpenCR 보드 패키지 1.5.3 (FQBN `ROBOTIS:OpenCR:OpenCR`) |
| | OpenCR 보드 패키지 주소 | `https://raw.githubusercontent.com/ROBOTIS-GIT/OpenCR/master/arduino/opencr_release/package_opencr_index.json` |
| | DYNAMIXEL 라이브러리 | Dynamixel2Arduino (커밋 `cfbbaf7`, `~/Arduino/libraries`) |
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

- 카메라 Color 토픽 이름: TODO (카메라 노드를 실행한 상태에서 `ros2 topic list | grep color`로 확인) · 정렬 Depth 토픽은 `/camera/camera/aligned_depth_to_color/image_raw` · 30 Hz
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
# Raspberry Pi에 SSH 접속 (PC 터미널, 화면 확인이 필요하면 -X)
ssh -X <user>@<raspberrypi-ip>

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

보드 패키지의 업로더(`opencr_ld`)는 x86용이라 `arduino-cli upload`가 Pi에서 동작하지 않습니다. 업로더를 소스에서 한 번 빌드합니다.

```bash
# (한 번만) Dynamixel2Arduino 설치
git clone https://github.com/ROBOTIS-GIT/Dynamixel2Arduino.git ~/Arduino/libraries/Dynamixel2Arduino
git -C ~/Arduino/libraries/Dynamixel2Arduino checkout cfbbaf79581ecfcdec952a87916572885453f4ab

# (한 번만) arm64 opencr_ld 빌드 → ~/bin/opencr_ld
mkdir -p ~/opencr_tools && cd ~/opencr_tools && git init -q uploader-src && cd uploader-src
git remote add origin https://github.com/ROBOTIS-GIT/OpenCR.git
git sparse-checkout init --cone && git sparse-checkout set arduino/opencr_develop/opencr_ld
git fetch -q --depth 1 --filter=blob:none origin 68ec75d8a400949580ecf263e0105ea9743b878e && git checkout -q --detach FETCH_HEAD
make -C arduino/opencr_develop/opencr_ld
mkdir -p ~/bin && ln -sf ~/opencr_tools/uploader-src/arduino/opencr_develop/opencr_ld/opencr_ld ~/bin/opencr_ld

# 빌드 (TODO: 펌웨어 이름은 이식 PR에서 확정)
cd ~/Lv2_Monglian_Assignment/lv2_module5
arduino-cli compile --fqbn OpenCR:OpenCR:OpenCR --output-dir build/<sketch> firmware/<sketch>

# 업로드: 출력에 "CRC OK"와 "[OK] Download"가 있어야 성공 (종료 코드만으로 판단하지 않음)
opencr_ld /dev/ttyACM0 115200 build/<sketch>/<sketch>.ino.bin 1

# 시리얼 확인 (다른 프로그램이 포트를 열고 있지 않을 때)
python3 -m serial.tools.miniterm /dev/ttyACM0 115200 --eol LF
```

- 보드 측 통신 타임아웃: 속도 명령이 **300 ms** 없으면 속도 0 (실측 310 ms에 `TIMEOUT`, 팬 1.6°만 움직이고 정지). 모터 Bus Watchdog 100 ms.
- 업로드·시리얼 확인 기록: `results/logs/TODO`
- 주의: OpenCR이 켜진 뒤 모터를 빼거나 꽂으면 활성화 때 응답 없음으로 FAULT가 날 수 있습니다. 모터 연결을 바꾼 뒤에는 OpenCR을 리셋(또는 재업로드)합니다.

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
| TODO (작성자가 아닌 팀원) | | | 빌드·실행·정지·bag 재현 | |

## 인지 구현과 재현

[인지 실행 안내](docs/perception/README.md)에 두 패키지 빌드, 선택적 카메라 launch, 촬영·검출·평가 명령과 산출물 경로를 정리했다. 설정 원본은 `config/hsv.yaml`이다. 실제 인지 실험에서는 Cyclone DDS·DOMAIN 30으로 Pi에서 검출하고 PC에서 `/target`을 받았다. 위 환경 표의 기본 Fast DDS와 구별하며 최종 제출 장비의 공통 RMW·DOMAIN 값은 통합 단계에서 확정한다.

[인지 보고서](docs/perception/report.md)와 [수행 계획](../vision_todo/vision_todo.md)은 기존 검증 5/7(약 71%)과 미완료 평가를 구분한다. 새 패키지는 PC에서 빌드·합성 검사·카메라 없는 launch 기동/종료를 확인했다. 새 패키지로 Pi 카메라 실측을 재수행한 결과는 아직 없다. 제어·통합의 빈 launch·설정은 그대로 유지한다.
