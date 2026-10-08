# bag 기록과 재현 (문제 5)

bag 원본(`recordings/<run_id>/`)은 용량 때문에 Git에서 제외합니다. 저장소에는 메타데이터(`<run_id>_info.txt`)와 기록 당시 설정 사본(`<run_id>_config/`)만 올리고, 원본은 아래 **접근 위치**에 올립니다.

## 1. 기록한 bag

| run_id | 장면 | 길이 [s] | 크기 | 저장 형식 | 기준 커밋 | 메타데이터 | 접근 위치 | 기록자 · 일자 |
|---|---|---|---|---|---|---|---|---|
| assignment5_success_20261008_121732 | 대표 성공 (책상 위 정지 목표, 손대지 않음) | 20.7 | 590.3 MiB | mcap | 8f897bb (+ 커밋 안 된 결과 파일) | [info](assignment5_success_20261008_121732_info.txt) | TODO (공유 드라이브 링크) · 현재 PC `recordings/` | JuneKunst · 2026-10-08 |
| assignment5_lost_20261008_122845 | 소실·복귀 (손바닥으로 가림 → 치움) | 15.2 | 445.7 MiB | mcap | 8f897bb (+ 커밋 안 된 결과 파일) | [info](assignment5_lost_20261008_122845_info.txt) | TODO (공유 드라이브 링크) · 현재 PC `recordings/` | JuneKunst · 2026-10-08 |

- 기록 방법: `python3 assignment/assignment5.py record --name success|lost --seconds 20|15` (Pi, 통합 메뉴 5 → s/l과 같음). 노드를 직접 띄우고 추적을 켠 뒤 기록하며, 기록 당시 config·노드 기록은 `results/assignment5/<run_id>/`에 복사됩니다.
- 길이 10~30초. 토픽·메시지 수·기간·sha256은 `<run_id>_info.txt`에 있습니다(`assignment5.py record`가 자동으로 작성).
- 기록 토픽과 메시지 형식 (메시지 수: 성공 / 소실):

  | 토픽 | 형식 | 성공 | 소실 |
  |---|---|---|---|
  | `/camera/camera/color/image_raw` | sensor_msgs/msg/Image (640×480 rgb8) | 432 | 345 |
  | `/camera/camera/color/camera_info` | sensor_msgs/msg/CameraInfo | 465 | 375 |
  | `/camera/camera/aligned_depth_to_color/image_raw` | sensor_msgs/msg/Image (16UC1) | 358 | 242 |
  | `/target` | geometry_msgs/msg/PointStamped | 290 | 256 |
  | `/target_depth` | geometry_msgs/msg/PointStamped | 289 | 256 |
  | `/target/position_cam` | geometry_msgs/msg/PointStamped | 289 | 255 |
  | `/tracking_status` | std_msgs/msg/String | 679 | 668 |
  | `/pan_tilt/command` | geometry_msgs/msg/Vector3Stamped | 676 | 667 |
  | `/pan_tilt/joint_states` | sensor_msgs/msg/JointState | 880 | 759 |

  `assignment5.py record`는 `/tracking_enable`을 기록하지 않습니다(`record_bag.sh`만 기록). 추적은 기록 시작 전에 켭니다.
- sha256 (파일 전체 해시는 info 파일에 있음):

  | 파일 | 크기 [B] | sha256 |
  |---|---|---|
  | `assignment5_success_20261008_121732/0_assignment5_success_20261008_121732_2026_10_08-12_20_33.mcap` | 618942046 | `f0a553be6c9ba25dd0f73832c2f6c80f8cd8234099d74892bb23279ce69a1976` |
  | `assignment5_success_20261008_121732/metadata.yaml` | 7489 | `3378a0f9f86357166251590119dbcde7169b03b8675f23586499c5f3a491343d` |
  | `assignment5_lost_20261008_122845/0_assignment5_lost_20261008_122845_2026_10_08-12_30_01.mcap` | 467371169 | `faf0fb4d0b403b8508f19660599584f29553374b0cdf3c09f552c27e8d6be738` |
  | `assignment5_lost_20261008_122845/metadata.yaml` | 7483 | `849cd77b3d094927fa8c58c47cbe5f2bb0ef24569484dd0998dae86954ac2dd4` |

  내려받은 뒤 확인: `cd recordings/<run_id> && sha256sum *`
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

작성자가 아닌 팀원이 README만 보고 기록·재처리·재분석을 실행한 결과는 [README 9절](../README.md#9-재현-확인-기록)에 남깁니다.
