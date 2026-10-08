# bag 기록과 재현 (문제 5)

bag 원본(`recordings/<run_id>/`)은 용량 때문에 Git에서 제외합니다. 저장소에는 메타데이터(`<run_id>_info.txt`)만 올리고, 원본은 팀 공유 드라이브에 올립니다.

**다운로드: [팀 공유 드라이브 (Notion)](https://app.notion.com/p/ROS2-3f37bcf74d93802cb3f4c6022eb4a160?source=copy_link)**

공유 드라이브의 폴더 구성(PC에서 올린 `lv2_module5_공유드라이브_업로드/`):

| 경로 | 내용 |
|---|---|
| `01_필수_문제5_bag/<run_id>/` | 문제 5 필수 bag 2개 (`metadata.yaml` + `.mcap`), 같은 폴더에 `<run_id>_info.txt` |
| `02_추가_모터추적_bag/<run_id>/` | 추가: 모터가 따라 움직인 장면 bag (실제 모터 재생 시연에 사용) |
| `SHA256SUMS.txt` | 위 모든 파일의 sha256 (`sha256sum -c SHA256SUMS.txt`) |
| `README.txt` | 폴더 구성·확인·재생 명령 |

내려받은 bag 폴더는 폴더째 이 `recordings/` 아래에 둡니다(예: `recordings/assignment5_success_20261008_121732/metadata.yaml`).

## 1. 기록한 bag

| run_id | 장면 | 길이 [s] | 크기 | 저장 형식 | 기준 커밋 | 메타데이터 | 접근 위치 | 기록자 · 일자 |
|---|---|---|---|---|---|---|---|---|
| assignment5_success_20261008_121732 | 대표 성공 (책상 위 정지 목표, 손대지 않음) | 20.7 | 590.3 MiB | mcap | 8f897bb (+ 커밋 안 된 결과 파일) | [info](assignment5_success_20261008_121732_info.txt) | [공유 드라이브](https://app.notion.com/p/ROS2-3f37bcf74d93802cb3f4c6022eb4a160?source=copy_link) `01_필수_문제5_bag/` | JuneKunst · 2026-10-08 |
| assignment5_lost_20261008_122845 | 소실·복귀 (손바닥으로 가림 → 치움) | 15.2 | 445.7 MiB | mcap | 8f897bb (+ 커밋 안 된 결과 파일) | [info](assignment5_lost_20261008_122845_info.txt) | [공유 드라이브](https://app.notion.com/p/ROS2-3f37bcf74d93802cb3f4c6022eb4a160?source=copy_link) `01_필수_문제5_bag/` | JuneKunst · 2026-10-08 |
| motion_20261008_125139 (추가) | 원통을 들고 옮기며 팬·틸트가 따라 움직임 | 15.5 | 452.7 MiB | mcap | 8f897bb (+ 커밋 안 된 결과 파일) | [info](motion_20261008_125139_info.txt) | [공유 드라이브](https://app.notion.com/p/ROS2-3f37bcf74d93802cb3f4c6022eb4a160?source=copy_link) `02_추가_모터추적_bag/` | JuneKunst · 2026-10-08 |

- 기록 방법: 성공·소실은 `python3 assignment/assignment5.py record --name success|lost --seconds 20|15` (Pi, 통합 메뉴 5 → s/l과 같음). 노드를 직접 띄우고 추적을 켠 뒤 기록하며, 기록 당시 config·노드 기록은 `results/assignment5/<run_id>/`에 복사됩니다. 추가 장면(motion)은 `scripts/test/motion_guide.sh record`(노드 실행 → 추적 켜짐 확인 → `scripts/record_bag.sh <run_id> 15`)로 기록했습니다.
- 길이 10~30초. 토픽·메시지 수·기간·sha256은 `<run_id>_info.txt`에 있습니다(`assignment5.py record`가 자동으로 작성).
- 기록 토픽과 메시지 형식 (메시지 수: 성공 / 소실 / 추가 motion):

  | 토픽 | 형식 | 성공 | 소실 | motion |
  |---|---|---|---|---|
  | `/camera/camera/color/image_raw` | sensor_msgs/msg/Image (640×480 rgb8) | 432 | 345 | 325 |
  | `/camera/camera/color/camera_info` | sensor_msgs/msg/CameraInfo | 465 | 375 | 329 |
  | `/camera/camera/aligned_depth_to_color/image_raw` | sensor_msgs/msg/Image (16UC1) | 358 | 242 | 284 |
  | `/target` | geometry_msgs/msg/PointStamped | 290 | 256 | 271 |
  | `/target_depth` | geometry_msgs/msg/PointStamped | 289 | 256 | 271 |
  | `/target/position_cam` | geometry_msgs/msg/PointStamped | 289 | 255 | 270 |
  | `/tracking_status` | std_msgs/msg/String | 679 | 668 | 683 |
  | `/pan_tilt/command` | geometry_msgs/msg/Vector3Stamped | 676 | 667 | 681 |
  | `/pan_tilt/joint_states` | sensor_msgs/msg/JointState | 880 | 759 | 772 |

  `assignment5.py record`는 `/tracking_enable`을 기록하지 않습니다. `record_bag.sh`(motion)는 구독하지만, 추적을 기록 시작 전에 켰으므로 메시지가 없어 bag에 이 토픽이 없습니다.
- sha256 (파일 전체 해시는 info 파일에 있음):

  | 파일 | 크기 [B] | sha256 |
  |---|---|---|
  | `assignment5_success_20261008_121732/0_assignment5_success_20261008_121732_2026_10_08-12_20_33.mcap` | 618942046 | `f0a553be6c9ba25dd0f73832c2f6c80f8cd8234099d74892bb23279ce69a1976` |
  | `assignment5_success_20261008_121732/metadata.yaml` | 7489 | `3378a0f9f86357166251590119dbcde7169b03b8675f23586499c5f3a491343d` |
  | `assignment5_lost_20261008_122845/0_assignment5_lost_20261008_122845_2026_10_08-12_30_01.mcap` | 467371169 | `faf0fb4d0b403b8508f19660599584f29553374b0cdf3c09f552c27e8d6be738` |
  | `assignment5_lost_20261008_122845/metadata.yaml` | 7483 | `849cd77b3d094927fa8c58c47cbe5f2bb0ef24569484dd0998dae86954ac2dd4` |
  | `motion_20261008_125139/0_motion_20261008_125139_2026_10_08-12_56_36.mcap` | 474729902 | `a2d150242c4906bf8663cb91d99a9a21bd52af1d14d7f29a385dbd49f2002651` |
  | `motion_20261008_125139/metadata.yaml` | 7463 | `18f4a49e29b25ac444ad0c7851e881db3ddf6f1dfa48567f0ae2b60bd532d81a` |

  내려받은 뒤 확인: 공유 드라이브 폴더에서 `sha256sum -c SHA256SUMS.txt`, 또는 `cd recordings/<run_id> && sha256sum *`를 위 표와 비교
- 같은 `run_id`로 연결되는 기록: `results/assignment5/<run_id>/` (`<run_id>.csv` 제어, `<run_id>_detect.csv` 인지, `<run_id>_serial.log` OpenCR 시리얼, `config/` 기록 당시 설정)
- 재현 결과(원본과의 일치·차이)는 [report.md 문제 5](../report.md#문제-5--ros2-bag-및-재현-기록)에 있습니다.

## 2. 기록 (Raspberry Pi)

```bash
cd ~/Lv2_Monglian_Assignment/lv2_module5
# 터미널 1: 전체 실행. run_id가 인지·제어·시리얼 기록 이름이 된다
ros2 launch tracker_bringup full.launch.py run_id:=success_001
# 터미널 2: 추적 시작
ros2 topic pub --once /tracking_enable std_msgs/msg/Bool "{data: true}"
# 터미널 3: 30초 기록 (화면 미리보기는 끈 상태로)
scripts/record_bag.sh success_001 30
```

소실·복귀 장면도 같은 방법으로 `run_id`만 바꿔(예: `lost_001`) 기록합니다. 기록 중에 목표를 손으로 가렸다가 다시 보이게 합니다.

## 3. 재현 — 실제 모터 출력 없음

재현 중에는 `tracker_controller`·`opencr_bridge`를 띄우지 않습니다. OpenCR USB를 빼 두면 더 확실합니다. 오프라인 재현은 실제 하드웨어 폐루프 시연과 별개입니다.

| 재현 | 하는 일 | 확인 |
|---|---|---|
| 입력 재처리 | bag의 영상·CameraInfo·정렬 Depth·모터 각도만 `--clock`으로 재생하고, `target_detector`(`use_sim_time`)가 다시 검출해 **`/target_replay`** 로 냅니다. 저장된 `/target`은 재생하지 않습니다. | 같은 영상 stamp끼리 저장된 `/target`과 비교했을 때 검출 일치율·ex/ey 차이가 작아야 합니다. |
| 결과 재분석 | 저장된 `/target`·`/tracking_status`·`/pan_tilt/command`로 지표를 다시 계산합니다. 검출기는 실행하지 않습니다. | 실행 중 남긴 제어 CSV로 계산한 값과 일치해야 합니다. |

```bash
cd ~/Lv2_Monglian_Assignment/lv2_module5
# 입력 재처리: 결과는 recordings/success_001_replay/ (bag), 30프레임마다 재처리 이미지 저장
scripts/replay_bag.sh success_001
# 결과 재분석 + 재처리 비교 + 실행 중 CSV 대조, --save로 results/에 저장
python3 scripts/analyze_bag.py recordings/success_001 \
  --csv ~/lv2_module5_logs/success_001.csv \
  --replay recordings/success_001_replay --save
```

2026-10-08 bag 두 개는 아래 명령으로 재현했습니다(Pi, 모터 노드 없음 확인 후). 내려받은 bag을 `recordings/`에 두면 같은 명령으로 다시 재현할 수 있습니다.

```bash
cd ~/git/Lv2_Monglian_Assignment/lv2_module5
source /opt/ros/lyrical/setup.bash && source ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=28 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
for b in recordings/assignment5_success_20261008_121732 recordings/assignment5_lost_20261008_122845; do
  python3 assignment/assignment5.py replay $b       # 입력 재처리 → results/assignment5/<run_id>_replay/summary.md
  python3 assignment/assignment5.py reanalyze $b    # 결과 재분석 → results/assignment5/<run_id>/reanalysis.md, results/metrics.csv
done
```

추가 motion bag은 안내 스크립트로 같은 두 단계를 실행했습니다. 모터 노드가 떠 있으면 시작하지 않습니다.

```bash
scripts/test/motion_guide.sh replay motion_20261008_125139   # 입력 재처리 + 결과 재분석 (모터 출력 없음)
```

- 재생 중에 웹뷰(`python3 scripts/web_view.py`)로 bag 영상과 모터 각도를 볼 수 있습니다. 웹뷰를 `--local-dds`로 켜면 다른 노드와 DDS 탐색 범위가 달라져 토픽을 받지 못한 경우가 있었으므로 이 옵션 없이 켭니다.

### 별도 시연: 기록된 명령으로 실제 모터 재생 (과제의 재현과 구분)

위 재현은 모터 출력을 끈 오프라인 재현입니다. 아래는 bag의 `/pan_tilt/command`만 브리지로 다시 보내 실제 모터를 움직이는 **별도 시연**입니다. 카메라·검출·제어 노드는 띄우지 않으므로 영상에 반응하지 않고, 기록된 속도 명령을 같은 시간 간격으로 그대로 따라 합니다.

```bash
scripts/test/motor_replay.sh motion_20261008_125139
#  1) 다른 노드·포트 사용 확인 → opencr_bridge만 실행
#  2) Enter: bag 시작 자세(pan −10.02°, tilt 26.28°)로 최대 15°/s 이동 (scripts/test/pose_go.py)
#  3) Enter: ros2 bag play --topics /pan_tilt/command (15.5 s), 실제 각도를 recordings/<run_id>_motorplay/ 에 기록
#  4) scripts/test/compare_motorplay.py로 원본 각도와 비교 → results/assignment5/<run_id>_motorplay/
#  비상 정지: Ctrl+C (재생 중지 → X) 또는 12V 차단
```

결과는 [results/assignment5/motion_20261008_125139_motorplay/summary.md](../results/assignment5/motion_20261008_125139_motorplay/summary.md)에 있습니다(0~11 s 각도 차이 RMS pan 0.56° · tilt 0.74°).

- 토픽 분리: 재처리 출력은 `/target_replay`, `/target_replay/depth`, `/target_replay/position_cam`입니다(`replay.launch.py`가 출력 토픽 3개를 모두 바꿈).
- 시간 기준: `ros2 bag play --clock` + 검출기 `use_sim_time:=true` + 재처리 기록기 `--use-sim-time`을 씁니다. 분석은 bag의 시각만 쓰고 현재 벽시계는 쓰지 않습니다.
- 재처리 이미지: `~/lv2_module5_results/images/<run_id>_replay_*`에 저장됩니다. 제출할 장면은 `results/images/replay/`로 복사합니다.
- 분석 결과: `results/logs/replay/<run_id>_analysis.txt`, 성능표 `results/metrics.csv`(source = bag / csv / 재처리 태그)

### 심화: 고정 bag 회귀 비교

같은 bag을 설정만 바꿔 다시 처리하고, 두 결과를 한 번에 비교합니다.

```bash
cp -r config /tmp/config_new && nano /tmp/config_new/hsv.yaml        # 바꿀 설정만 수정
scripts/replay_bag.sh success_001 before                              # 현재 설정
scripts/replay_bag.sh success_001 after /tmp/config_new               # 변경한 설정
python3 scripts/analyze_bag.py recordings/success_001 \
  --replay recordings/success_001_before --replay recordings/success_001_after --save
```

## 4. 재현 확인 기록

재현 확인 결과는 아래 표와 [README 9절](../README.md#9-재현-확인-기록)에 있습니다.

| 확인자 | 날짜 | 기준 커밋 | bag (sha256 확인) | 수행 | 결과 |
|---|---|---|---|---|---|
| 권형중 (JuneKunst) | 2026-10-08 | 8f897bb (Pi) | success · lost · motion (PC 사본 = Pi 원본) | 입력 재처리 · 결과 재분석 (모터 출력 없음), motion은 실제 모터 재생 시연까지 | 검출 여부 일치 success 99.1 % · lost 83.1 % · motion 89.1 %. 상세: [report.md 5-2·5-3·5-5](../report.md#문제-5--ros2-bag-및-재현-기록) |

