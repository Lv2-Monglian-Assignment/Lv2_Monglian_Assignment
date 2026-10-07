# 인지 코드 배치와 실행 안내

## 실행 환경과 목적

ROS 2 Lyrical·시스템 Python·OpenCV·cv_bridge를 사용한다. D435가 연결된 Pi에서 카메라와 검출 노드를 실행하고 PC에는 `/target` 결과를 전달하는 구성을 짧게 검증했다. 실제 장면 평가는 저장 사진을 같은 설정으로 처리한다. 모터 구동은 이 안내의 실행 범위 밖이다.

## 폴더별 역할

| 위치 | 내용 |
|---|---|
| `ros2_ws/src/target_detector/` | ament_python 인지 패키지·검출 함수·ROS 노드·실행 파일 등록 |
| `ros2_ws/src/tracker_bringup/launch/perception.launch.py` | 인지 실행과 선택적 D435 컬러 실행 |
| `config/hsv.yaml` | 기존 perception.yaml을 이 구조에 배치한 단일 인지 설정 |
| `config/perception-evaluation-used.yaml` | 평가를 위해 고정한 설정 사본 |
| `scripts/` | 저장 사진 검출·실제 프레임 수집·정답 라벨 사본·평가·진단 도구 |
| `tests/` | 검출 계산과 평가 출처 보호 합성 검사 |
| `results/images/perception/` | 세 장면 원본·마스크·검출과 정답 미확정 후보 사진 |
| `results/logs/perception/` | 실제 진단 JSON·로그·검증 코드 사본·실험 설정 |
| `docs/perception/` | 인지 보고서·실험 요약·작성 기여·파일 대응표·제출 초안 |
| [vision_todo/vision_todo.md](../../../vision_todo/vision_todo.md) | 인지 담당 작업 목록과 완료 범위 |

원래 작업 폴더의 파일은 유지하고 필요한 사본을 이 구조로 분류했다. [파일 대응표](FILE_MAP.json)에 원본 위치·해시와 새 위치를 기록한다. 외부 제공 예제와 제어·통합 코드의 기존 파일은 바꾸지 않았다.

## 빌드와 기동 확인

팀 저장소 루트에서 다음 명령으로 인지 패키지만 빌드한다. ROS Python 의존성은 시스템 패키지를 사용한다. 새 장비에는 ROS 2 Lyrical setup, colcon, rosdep가 필요하며 rosdep 초기화·캐시 갱신을 마친 뒤 다음 설치 명령을 실행한다. 기존 설치가 확인된 장비에서는 의존성 설치를 반복하지 않는다.

```bash
cd lv2_module5/ros2_ws
source /opt/ros/lyrical/setup.bash
rosdep install --from-paths src --ignore-src --rosdistro lyrical -r -y
```

빌드는 팀 저장소 루트에서 시작한다.

```bash
cd lv2_module5/ros2_ws
source /opt/ros/lyrical/setup.bash
colcon build --symlink-install --packages-select target_detector tracker_bringup
source install/setup.bash
```

Pi에서 카메라와 인지를 동시에 시작한다. DOMAIN·RMW·발견 대상은 PC와 맞춘다. 다음 PC 주소는 현재 실험 장비의 예이며 다른 장비에서는 실제 주소로 바꾼다. 이미 실행 중인 카메라·인지 노드가 있으면 중복 실행하지 않는다.

```bash
export ROS_DOMAIN_ID=30 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET ROS_STATIC_PEERS="10.2.12.132"
ros2 launch tracker_bringup perception.launch.py start_camera:=true
```

카메라가 이미 실행 중이면 인지만 실행한다. 이 launch의 기본 start_camera 값은 false다.

```bash
ros2 launch tracker_bringup perception.launch.py start_camera:=false
```

설정 파일을 명시해 실행할 수도 있다. 앞의 빌드·source를 끝낸 `lv2_module5/ros2_ws` 위치에서 다음 명령을 실행한다.

```bash
cd ..
ros2 run target_detector target_detector --ros-args --params-file config/hsv.yaml
```

PC 확인창에서는 같은 DOMAIN·RMW를 설정하고 작은 결과를 구독한다. RMW와 DOMAIN을 제출 장비 공통값으로 확정한 것으로 해석하지 않는다.

```bash
source /opt/ros/lyrical/setup.bash
export ROS_DOMAIN_ID=30 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET ROS_STATIC_PEERS="10.2.12.61"
ros2 topic echo /target geometry_msgs/msg/PointStamped \
  --qos-reliability best_effort --qos-depth 1 --once --timeout 15
```

정상 중지는 실제 인지 실행창에서 Ctrl+C 키를 누른다. 카메라를 같이 실행한 launch를 중지하면 카메라도 종료된다. 카메라 별도 실행창은 별도로 관리한다.

## 저장 사진 처리와 평가

다음 명령은 `lv2_module5` 루트에서 실행한다. 결과 폴더는 아직 없는 새 순번을 사용한다.

```bash
/usr/bin/python3 scripts/detect_image.py \
  results/images/perception/normal/color_original.png \
  --config config/hsv.yaml --out results/images/perception/normal-recheck-001
```

[capture_ros_dataset.py](../../scripts/capture_ros_dataset.py)는 실행 중인 Pi Image에서 실제 프레임을 저장한다. 목표가 맞는지·물체를 제거했는지 확인한 근거로 라벨을 지정한다. 후보는 unlabelled로 저장하고, 확인 뒤 [label_perception_dataset.py](../../scripts/label_perception_dataset.py)로 원본 JSON을 유지한 별도 라벨 파일을 만든다.

```bash
/usr/bin/python3 scripts/capture_ros_dataset.py \
  --topic /camera/camera/color/image_raw \
  --output results/images/perception/evaluation-002/present \
  --scene present --label-basis "실제 목표 배치 및 원본 확인" \
  --count 30 --interval 0.5 --timeout 60
```

목표만 화면 밖으로 치운 사실과 원본을 확인한 뒤 같은 설정의 카메라에서 없음 10장을 수집한다.

```bash
/usr/bin/python3 scripts/capture_ros_dataset.py \
  --topic /camera/camera/color/image_raw \
  --output results/images/perception/evaluation-002/absent \
  --scene absent --label-basis "실제 목표 제거 및 원본 확인" \
  --count 10 --interval 0.5 --timeout 60
```

[evaluate_perception_dataset.py](../../scripts/evaluate_perception_dataset.py)는 30·10개·원본/소스 hash·장면 라벨·source stamp·영상 형식을 확인하고 고정 설정으로 처리한다. 두 장면을 같은 머신의 지정 경로에 모은 뒤 평가한다. 아직 없음 10장·최종 정답 평가가 없어 다음 명령은 재현 방법이며 수행 완료 기록이 아니다.

```bash
/usr/bin/python3 scripts/evaluate_perception_dataset.py \
  --present results/images/perception/evaluation-002/present/dataset.json \
  --absent results/images/perception/evaluation-002/absent/dataset.json \
  --config config/perception-evaluation-used.yaml \
  --out results/images/perception/evaluation-002-output \
  --exclude-images results/images/perception/normal/color_original.png \
    results/images/perception/absent/color_original.png \
    results/images/perception/occluded/color_original.png
```

촬영 메타데이터와 capture_source.py는 사진과 같은 폴더에 보관해야 hash 대조와 재현이 가능하다. 정답 미확정 후보 30장의 found 개수를 정확도로 표기하지 않는다.

## 검증 결과와 한계

```bash
/usr/bin/python3 tests/test_detector.py
/usr/bin/python3 tests/test_evaluation_provenance.py
```

PC에서 두 패키지 빌드와 합성 검사 7+8개를 확인했다. 분리된 로컬 DOMAIN 187에서 카메라 없이 패키지 launch 기동과 정상 종료를 확인했다. [검증 결과](../../results/logs/perception/repository-layout-check/checks.json)에 새 구조의 검증 범위를 기록했다. 새 폴더의 패키지로 실제 카메라 스트림을 재실행한 결과와는 구별한다. 기존 실제 Pi 모듈 실행의 약 30Hz 입력·결과 전달 근거는 [보고서](report.md)에 있다.

정리 단계의 중복 SIGINT를 처리하도록 팀 패키지의 종료 부분을 보완했다. HSV 계산과 메시지 계약은 유지했다. 패키지 연락처·라이선스의 TODO 메타데이터는 팀에서 확정할 항목이다.

[실험 요약](practice_summary.md), [진단 요약](troubleshooting.md), [인지 기여와 제출 상태](team_perception.md), [작성·실행 도구 기여](AI_CONTRIBUTIONS.md)를 연결한다. 기존 제어·통합 launch는 빈 틀이며 그 실행이나 시스템 전체 성공을 검증한 것으로 표시하지 않는다.
