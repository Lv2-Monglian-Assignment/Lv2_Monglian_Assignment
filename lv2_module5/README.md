# 프로젝트 1 — 비전 기반 객체 추적 시스템

카메라로 단일 색상 목표 1개를 HSV·Contour로 검출하고, 정규화 중심 오차(ex)를 ROS2 `/target` 토픽으로 전달하여 OpenCR·DYNAMIXEL 수평 1축 P 제어로 추적합니다.
목표 소실·통신 중단 시 안전 정지하고, 시야 내 재등장 시 추적을 재개합니다.

```
카메라 영상 → HSV·Contour 검출 → 중심 오차 → 추적 제어 → OpenCR → DYNAMIXEL → 카메라 방향 변화 → 새 영상의 오차 확인
```

> 이 README만 보고 작성자가 아닌 팀원이 실행·재현할 수 있어야 합니다. `TODO`는 실제 값으로 채운 뒤 삭제합니다.

---

## 1. 실행 환경 및 장비

| 구분 | 항목 | 상세 사양 및 설정값 |
|---|---|---|
| 소프트웨어 | OS | Ubuntu Server 22.04 LTS (Raspberry Pi) |
| | ROS2 | Humble |
| | OpenCV | 4.x (TODO: `python3 -c "import cv2; print(cv2.__version__)"` 결과로 정확한 버전 기입) |
| | Python | TODO |
| | Arduino IDE / arduino-cli · OpenCR 보드 패키지 | TODO: 버전 |
| 하드웨어 | SBC | Raspberry Pi 4 |
| | Camera | USB Camera (해상도: 640x480 / 설정 FPS: 30) |
| | Control Board | OpenCR 1.0 |
| | Actuator | DYNAMIXEL (모델: TODO / ID: 1 / Baudrate: 57600 / Protocol 2.0) |
| | Power | 12V 외부 전원 공급 장치 |
| | 기구 | 고정 브래킷 · 수평 1축 (회전 범위: TODO) |
| 목표물 | 대상 | TODO: 카드·공·블록 중 선택, 색상 (사진: `results/images/target.jpg`) |

- 모터 ID·baud·프로토콜은 실제 장비에서 확인한 값입니다. 확인 방법과 날짜: TODO
- PC는 SSH 접속과 결과 확인에만 사용합니다. OpenCR 빌드·업로드·시리얼 확인은 Raspberry Pi에서 수행합니다.

## 2. 폴더 구조

```
lv2_module5/
├── README.md            # 실행·재현 가이드 (이 문서)
├── report.md            # 문제 1~5 구현·검증·해석·한계
├── team.md              # 4인 역할·Issue·PR·리뷰, 보호 설정, 통합 확인
├── presentation.md      # 5분 시연 순서와 핵심 결과
├── ros2_ws/src/         # 인지·제어 ROS2 패키지
├── firmware/            # OpenCR 펌웨어 소스
├── config/              # HSV·카메라·제어·장치·정지·시험 설정
├── results/
│   ├── images/          # 정상·대상 없음·가림 장면 원본/마스크/검출 이미지
│   ├── logs/            # 원본 CSV·상태 로그·시리얼 로그
│   ├── plots/           # Kp 비교 등 그래프
│   └── metrics.csv      # 회차별 성능표
└── recordings/README.md # bag·영상 위치, 메타데이터, 크기·해시, 재생 방법
```

## 3. 설치 및 빌드

```bash
# Raspberry Pi에 SSH 접속
ssh <user>@<raspberrypi-ip>

# 의존성 설치 (TODO: 실제 사용한 패키지로 수정)
sudo apt update
sudo apt install -y ros-humble-cv-bridge ros-humble-usb-cam python3-opencv

# 저장소 clone (개인별 폴더 사용, 같은 폴더에서 동시에 브랜치 변경 금지)
git clone <팀 저장소 URL>
cd Lv2_Monglian_Assignment/lv2_module5/ros2_ws

# 빌드
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## 4. OpenCR 펌웨어 빌드·업로드

```bash
# TODO: 실제 사용한 방법(arduino-cli 또는 Arduino IDE)과 명령으로 수정
arduino-cli compile --fqbn OpenCR:OpenCR:OpenCR firmware/<sketch>
arduino-cli upload -p /dev/ttyACM0 --fqbn OpenCR:OpenCR:OpenCR firmware/<sketch>

# 시리얼 출력 확인
sudo apt install -y minicom   # 또는 screen
minicom -D /dev/ttyACM0 -b 115200   # TODO: 실제 시리얼 속도
```

- 업로드·시리얼 확인 기록: `results/logs/TODO`
- 보드 측 통신 타임아웃 정지 설정값: TODO (ms)

## 5. 실행

> 처음에는 모터 출력을 끈 상태로 데이터 전달을 확인하고, 실제 회전은 낮은 속도·제한 범위에서 수행합니다.

```bash
# 터미널 1: 카메라 + 인지 노드  (TODO: 실제 패키지·실행 파일명)
ros2 launch <pkg> perception.launch.py

# 터미널 2: 제어 노드
ros2 launch <pkg> control.launch.py

# 또는 전체 실행
ros2 launch <pkg> tracking.launch.py
```

| 설정 파일 | 내용 |
|---|---|
| `config/TODO.yaml` | HSV 범위·최소 면적·해상도 |
| `config/TODO.yaml` | Kp·direction·속도 상한·회전 범위·데드밴드·제어 주기 |
| `config/TODO.yaml` | 입력 타임아웃(기본 0.5초)·복귀 조건(연속 3프레임) |

### 동작 확인

```bash
ros2 topic echo /target            # 정규화 오차 ex/ey, z=면적비(0=미검출)
ros2 topic echo /tracking_status   # IDLE / TRACKING / LOST
ros2 topic hz /target              # 처리 주기
```

### 모의 입력 시험 (모터 출력 OFF)

```bash
# x=+0.4, z>0 → 오른쪽 오차를 줄이는 명령이 나와야 함 (TODO: 실제 명령으로 수정)
ros2 topic pub /target geometry_msgs/msg/PointStamped "{point: {x: 0.4, y: 0.0, z: 0.05}}" -r 30
```

## 6. 중지

```bash
# 정상 중지: 각 터미널에서 Ctrl+C
# 정지 명령 (TODO: IDLE 전환 또는 정지 명령 방법)
```

- 제어 프로그램이 종료되거나 통신이 끊기면 OpenCR 측 타임아웃으로 모터가 정지합니다. 확인 기록: `results/logs/TODO`
- 비상 시: 12V 전원 차단

## 7. bag 기록 및 재현

자세한 파일 위치·메타데이터·해시는 [recordings/README.md](recordings/README.md)에 있습니다.

```bash
# 기록 (영상·목표·상태·명령 토픽)
ros2 bag record -o recordings/<run_id> /image_raw /target /tracking_status /<motor_cmd_topic>

# 정보 확인
ros2 bag info recordings/<run_id>
```

**재현할 때는 반드시 모터 출력을 비활성화합니다.**

| 재현 | 방법 |
|---|---|
| 입력 재처리 | bag의 영상만 검출기로 전달하고 결과는 `/target_replay`로 분리 출력 |
| 결과 재분석 | 저장된 `/target`·상태·명령으로 지표 재계산 |

```bash
# 입력 재처리 예시 (TODO: 실제 명령으로 수정)
ros2 bag play recordings/<run_id> --clock --topics /image_raw
ros2 run <pkg> <detector> --ros-args -p use_sim_time:=true -r /target:=/target_replay

# 결과 재분석
python3 <analysis_script>.py recordings/<run_id>
```

- 저장된 `/target`과 새 검출 결과를 같은 토픽에 섞지 않습니다.
- 과거 bag 시각과 현재 시각을 섞어 타임아웃·지연을 계산하지 않습니다.

## 8. 결과 위치

| 결과 | 위치 |
|---|---|
| 3종 장면 이미지 (정상·대상 없음·가림) | `results/images/` |
| Kp 2종 × 3회 추적 CSV | `results/logs/` |
| 비교 그래프 | `results/plots/` |
| 성능표 (FPS·검출률·RMSE·복구) | `results/metrics.csv` |
| bag·시연 영상 | `recordings/README.md` |
| 해석 및 한계 | [report.md](report.md) |

## 9. 재현 확인 기록

| 확인자 | 날짜 | 기준 커밋 | 수행 내용 | 결과 · 수정 사항 |
|---|---|---|---|---|
| TODO (작성자가 아닌 팀원) | | | 빌드·실행·정지·bag 재현 | |
