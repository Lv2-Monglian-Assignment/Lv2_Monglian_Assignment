# 인지 담당 수행 계획

## 실험 환경

D435를 Raspberry Pi USB에 연결한다. 기존 PC 영상 처리 조건과 구별해 현재는 같은 HSV·Contour 검출 코드를 Pi에서 실행하고 PC에는 /target을 전달한다. PC와 Pi는 ROS 2 Lyrical·Cyclone DDS·DOMAIN 30을 사용한다. PC 검출은 시스템 Python과 OpenCV 4.6.0으로 실행한다.

기존 사진·ROS 인터페이스 검증은 RGB8 640×480 입력으로 수행했다. 이전 통신 진단은 RGB8 320×240·요청 6Hz였다. 현재 Pi 로컬 검출 구성에서 RGB8 640×480 입력 약 29.99Hz와 PC 결과 전달 약 29.88Hz를 짧게 관측했다. 진단용 조건을 최종 평가 조건이나 640×480·30Hz 검증 완료로 바꾸어 기록하지 않는다.

## 수행 목적과 담당 범위

**컬러 영상 → HSV 마스크 → 잡음 제거 → 목표 윤곽 선택 → 중심·면적 계산 → `/target` 발행 → 실제 영상 평가·보고서**를 수행한다.

| 역할 | 수행 범위와 전달할 결과 |
|---|---|
| 인지 | 영상·검출 설정·목표 중심 오차·면적비·`/target`·검출 평가 자료 |
| 제어 | 인지 출력을 받아 모터 명령·제어 로직을 구현하고 구동 동작을 검증 |
| 통합 | 카메라·인지·제어의 실행 구성과 연결을 조정하고 시스템 전체 동작을 검증 |

인지에서 전달할 인터페이스는 `geometry_msgs/msg/PointStamped`의 `/target`이다. 인지 결과 전달에 필요한 영상 입력·통신 확인은 수행하지만, 모터·OpenCR 구동과 시스템 전체 추적 성공은 이 계획의 완료 항목에 포함하지 않는다.

발제 위치는 [인지 실습 노트의 대응표](../lv2_module5/docs/perception/practice_summary.md)에 연결돼 있다. 아래 작업은 그 대응표와 기존 요구사항·구현·실측 증거를 정리한 계획이다. 설계 문서의 환경 기재와 발제 원문 요구는 구별한다.

## 계획한 방법과 수행 목록

기존 7단계 중 증거를 확인한 단계는 **5개, 약 71%**다. 아래 완료 표시는 확인한 범위에 한정한다. 세부 체크박스 수로 진행률을 다시 계산하지 않는다.

### 1. 실제 컬러 사진 확보 — 확인 완료

- [x] D435 USB 연결과 컬러 영상 입력을 확인한다.
- [x] 실제 영상 크기·인코딩·frame_id와 촬영 정보를 기록한다.
- [x] 파란 목표 물체의 컬러 사진을 저장한다.
- [x] 원본 사진·촬영 메타데이터·소스 사본을 보존한다.

발제 연결: [시작 전 준비](https://trapezoidal-slip-4ac.notion.site/1-1-3eb8a651517c8043815ed7c0be0746ac#1da8a651517c830bba1201f680dc3375), 문제 1의 목표 영상 준비.

코드: [ROS 사진 저장](../lv2_module5/scripts/capture_ros_image.py). 저장 위치: `data/captures/capture-001/`처럼 순번별 폴더. 기존 근거: [실제 사진과 촬영 정보](../lv2_module5/results/logs/perception/capture-001/capture.json).

### 2. 컬러 배열과 색 순서 확인 — 확인 완료

- [x] RGB와 BGR 배열의 차이를 확인한다.
- [x] 입력 배열 순서와 OpenCV HSV 변환 상수를 맞춘다.
- [x] ROS의 `rgb8` 입력과 저장 사진의 BGR 처리를 구별한다.

발제 연결: [문제 1 구현 내용](https://trapezoidal-slip-4ac.notion.site/1-1-3eb8a651517c824999eb81fd49294ebc)의 컬러 영상·HSV 처리.

코드: [검출 함수](../lv2_module5/ros2_ws/src/target_detector/target_detector/detector.py), [저장 사진 처리](../lv2_module5/scripts/detect_image.py), [ROS 연결](../lv2_module5/ros2_ws/src/target_detector/target_detector/target_detector.py). 근거: [색 순서 검토](../lv2_module5/docs/perception/practice_summary.md).

### 3. HSV·윤곽 검출과 설정 관리 — 확인 완료

- [x] 파란 목표의 HSV 하한·상한을 선택한다.
- [x] HSV 마스크에 morphology open/close를 적용한다.
- [x] 최소 면적 이상 후보 중 가장 큰 윤곽을 선택한다.
- [x] 윤곽의 중심과 면적을 계산한다.
- [x] 실제 프레임 크기 `W,H`로 정규화한다.
- [x] HSV·최소 면적·커널·요청 해상도를 YAML에 관리한다.

계산은 `ex=(cx-W/2)/(W/2)`, `ey=(cy-H/2)/(H/2)`, `area_ratio=contour_area/(W*H)`다. 오른쪽·아래가 양수다.

현재 선택값은 HSV `[92,80,26]`~`[120,255,255]`, 타원 커널 5×5, 최소 면적 400px²다. YAML의 요청 크기는 640×480이며 실제 320×240 입력도 실제 크기로 계산한다. 해상도가 달라지면 같은 최소 면적의 상대적인 효과도 달라지므로 평가 조건을 기록한다.

발제 연결: 문제 1의 색상 검출·후보 선택·중심 계산. 코드·설정: [detector.py](../lv2_module5/ros2_ws/src/target_detector/target_detector/detector.py), [perception.yaml](../lv2_module5/config/hsv.yaml). 근거: [선택 설정의 정상 검출](../lv2_module5/results/images/perception/normal/CHECK_KO.md).

### 4. 동일 설정의 정상·대상 없음·가림 검증 — 확인 완료

- [x] 정상 장면에서 목표 윤곽·중심·면적비를 확인한다.
- [x] 대상 없음 장면에서 미검출과 0 값을 확인한다.
- [x] 부분 가림 장면에서 보이는 목표 영역의 검출을 확인한다.
- [x] 세 장면에 같은 HSV·커널·최소 면적을 적용한다.
- [x] 원본·마스크·검출 이미지·수치 결과·설정 사본을 보관한다.

발제 연결: 문제 1, [역할별 체크리스트](https://trapezoidal-slip-4ac.notion.site/1-1-3eb8a651517c826bb60c01eb389fe021)의 세 장면 검증. 코드: [detect_image.py](../lv2_module5/scripts/detect_image.py). 근거: [세 장면 보고서](../lv2_module5/results/logs/perception/three-scenes-001/REPORT_KO.md).

각 장면 한 장의 탐색 검증이다. 조명 차이가 있으며 정상 사진은 튜닝에도 사용했다. 독립 정확도 평가와 구별한다.

### 5. ROS 인지 출력 인터페이스 — 기존 검증 범위 확인 완료

- [x] `/camera/camera/color/image_raw`의 새 Image를 처리한다.
- [x] `/target`을 `geometry_msgs/msg/PointStamped`로 발행한다.
- [x] `point.x=ex`, `point.y=ey`, `point.z=area_ratio`를 적용한다.
- [x] 입력 영상의 `header.stamp`와 `frame_id`를 유지한다.
- [x] 정상 영상에서 미검출이면 `x=y=z=0`을 발행한다.
- [x] 입력 영상이 끊기면 이전 결과를 재발행하지 않는다.
- [x] 실제 발행 QoS의 Best-effort·Keep-last·depth 1을 확인한다.

발제 연결: 문제 2의 `/target` 인터페이스, 역할별 체크리스트의 인지 출력. 코드: [target_detector.py](../lv2_module5/ros2_ws/src/target_detector/target_detector/target_detector.py). 근거: [ROS 인터페이스 검증 보고서](../lv2_module5/results/logs/perception/ros-interface-check-001/REPORT_KO.md).

`point.z`는 깊이(m)가 아니라 면적비다. 기존 확인은 실제 입력·header 10쌍·미검출 0·QoS·카메라 정상 종료 후 무발행 관측에 한정한다.

현재 조건의 추가 확인은 별도로 관리한다.

- [x] 관측 004는 카메라가 없는 조건의 0장임을 확인해 속도 비교에서 제외한다.
- [x] 같은 코드의 Pi 로컬 검출 구성을 검증하고 실행 위치 변경과 한계를 기록한다.
- [x] 현재 Pi 구성의 실제 640×480 rgb8·요청 30fps·입력과 출력의 짧은 수신 빈도를 기록한다.

이 항목은 현재 입력·전달 진단이며 제어 구동이나 통합 전체 시험이 아니다. 10초에 30개·최대 간격 0.5초 같은 기존 관측 기준은 진단용으로 정한 값이며 발제 공식 합격 기준으로 표기하지 않는다.

### 6. 대상 30프레임·없음 10프레임 독립 평가 — 미완료

- [ ] 튜닝에 사용하지 않은 대상 있는 30프레임과 대상 없는 10프레임을 확보한다.
- [ ] 최종 HSV·커널·최소 면적·영상 조건을 고정하고 설정 사본을 보관한다.
- [ ] 원본을 확인해 프레임별 대상 유무 정답을 기록한다.
- [ ] 같은 설정으로 검출하고 원본·마스크·검출 이미지·계산값을 저장한다.
- [ ] 프레임별 정답과 검출을 대조해 정상 검출·미검출·오검출·정상 미검출을 기록한다.
- [ ] 오검출·미검출 사진과 조명·배경색·가림·작은 면적 등 원인을 분석한다.
- [ ] 평가 보고서에 사용 조건·실제 결과·해석·한계를 작성한다.

발제 연결: 역할별 체크리스트의 검출 평가. 사용할 검출 코드: [detect_image.py](../lv2_module5/scripts/detect_image.py). 실제 프레임 수집·라벨 사본·출처 검증과 평가 도구를 준비하고 실제 후보 30장을 unlabelled로 보관했다. 목표 확인·없음 10장 수집과 40개 정답 평가는 아직 남아 있다. 연속 프레임의 유사성도 한계로 기록한다.

Pi 영상 관측의 32개·58개는 이미지·정답 라벨을 저장한 평가 데이터가 아니므로 이 40프레임에 합산하지 않는다. 합성 검사 자료도 실제 카메라 평가로 제출하지 않는다.

### 7. 인지 결과 보고서와 팀 제출 증빙 — 미완료

- [ ] 인지 구현·설정·실험 환경·결과·한계를 최종 보고서에 정리한다.
- [ ] 실행 명령·폴더 구조·입력과 산출물을 README에서 재현 가능하게 정리한다.
- [x] 팀 저장소 주소를 확인하고 인지 코드·설정·문서를 지정 구조에 배치한다.
- [ ] 인지 작업의 Issue·PR·리뷰 링크와 확인 근거를 보관한다.
- [ ] 인지 담당 작업·기여·검증 결과를 `team.md`에 정리한다.
- [ ] 필요한 AI 기여 내역은 별도 제출 항목에 실제 작성·실행 도구와 범위를 기록한다.
- [ ] 제어·통합 담당에게 `/target`의 타입·필드 의미·부호·QoS·미검출·입력 중단 동작을 전달할 문서를 정리한다.

발제 연결: 역할별 체크리스트의 팀 문서·Issue·PR·리뷰 증빙. 팀 저장소는 Lv2-Monglian-Assignment/Lv2_Monglian_Assignment이며 인지 파일을 이 저장소에 분류했다. 실제 Issue·PR·병합·리뷰 증빙은 남아 있다. [새 구조 실행 안내](../lv2_module5/docs/perception/README.md)를 따른다.

## 실제 수행 방법과 현재 결과

촬영·검출·ROS 관측 명령을 터미널에 직접 입력하고 나온 결과를 기록했다. 코드·기록 작성 도구와 원본 증거의 출처는 각 기록에 남긴다. 상세 수행 이력은 [인지 실습 노트](../lv2_module5/docs/perception/practice_summary.md)에서 확인한다.

| 현재 진단 조건 | 확인 결과 | 근거 |
|---|---|---|
| PC 검출 실행 중 Pi 10초 관측 | 32개·약 3.33Hz | [Pi 관측 검토](../lv2_module5/results/logs/perception/pi-image-stream-check-002/CHECK_KO.md) |
| PC 검출 중단 후 같은 Pi 관측 | 58개·약 5.99Hz | [중단 조건 비교](../lv2_module5/results/logs/perception/pi-image-stream-check-003/CHECK_KO.md) |
| PC 검출 재시작 | 실제 320×240 rgb8 수신·found=True | [재시작 근거](../lv2_module5/results/logs/perception/practice_records/target-restart-comparison-001.json) |
| 이전 재관측 004 | 카메라 부재로 0장; 속도 평가 제외 | [진단 이력](../lv2_module5/docs/perception/troubleshooting.md) |

현재 Pi 로컬 검출·PC PointStamped 수신과 실제 후보 30장 확보를 확인했다. 다음 순서는 후보 물체의 목표 정답 확인 → 실제 목표 제거 후 없음 10장 수집 → 독립 30·10프레임 평가 → 팀 저장소·PR·리뷰 증빙 연결이다. 같은 설정의 재검증은 비교 목적이 있을 때 진행하고, 통신 진단 횟수를 인지 진행률에 추가하지 않는다.

## 해석 및 한계

현재 증거는 사진 검출과 제한된 ROS 인터페이스 동작을 뒷받침한다. PC 검출 중단 조건에서 Pi 관측 빈도가 개선됐지만 Wi-Fi·DDS·프로세스 부하 중 근본 원인은 확정하지 않았다. 320×240·6Hz 진단 결과는 640×480·30Hz 복구, 장기 안정성, 검출 정확도 완료를 뜻하지 않는다.

깊이 영상이나 `/target_depth`는 설계의 추가 검토 항목이다. 필수 인지 작업으로 확정하지 않으며 현재 `/target`의 면적비 필드를 깊이로 바꾸지 않는다. 필요하면 발제 원문·역할 합의를 확인한 뒤 유효성 기록용 별도 항목으로 계획한다.
