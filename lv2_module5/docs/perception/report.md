# 인지 구현과 검증 보고서

## 실험 환경

D435를 Raspberry Pi USB 3에 연결한다. Pi는 Ubuntu 26.04.1·ROS 2 Lyrical·OpenCV 4.10.0·RealSense ROS 4.58.4·SDK 2.58.4다. PC는 Ubuntu 24.04.5·ROS 2 Lyrical·OpenCV 4.6.0이다. 실제 카메라 펌웨어는 5.15.1.55다.

일반 통신은 DOMAIN 30·Cyclone DDS·SUBNET이다. 현재 검증 구성은 **Pi 컬러 영상 → Pi의 동일 HSV 검출 코드 → `/target` → PC 수신**이다. 앞선 PC 검출 구성과 실행 위치가 달라졌음을 구별한다. 현재 요청 프로파일은 RGB8 640×480·30fps이며 깊이 영상은 비활성이다.

## 실험 목적

파란 목표의 중심 오차와 면적비를 계산해 `geometry_msgs/msg/PointStamped`로 전달한다. 동일 설정의 정상·없음·가림 검증, ROS 인터페이스 검증, 독립 대상 30프레임·없음 10프레임 평가와 인지 제출 증빙을 정리한다. 모터 제어와 시스템 전체 통합 성공은 이 보고서의 평가 범위에 포함하지 않는다.

## 계획한 방법

1. 실제 컬러 입력과 RGB/BGR 처리 순서를 확인한다.
2. HSV 마스크·morphology open/close·최대 유효 윤곽 선택을 수행한다.
3. 같은 설정으로 세 장면 결과를 저장한다.
4. 실제 입력 크기로 계산하고 `/target`의 header·QoS·미검출·입력 중단 동작을 확인한다.
5. 튜닝 사진과 분리한 실제 30·10프레임을 수집하고 정답과 검출 결과를 대조한다.
6. 코드·설정·환경·실험·Issue·PR·리뷰·팀 문서를 정리한다.

## 실제 수행 방법

[검출 함수](../../ros2_ws/src/target_detector/target_detector/detector.py)와 [ROS 노드](../../ros2_ws/src/target_detector/target_detector/target_detector.py), [설정](../../config/hsv.yaml)을 사용했다. HSV 범위는 `[92,80,26]`~`[120,255,255]`, 타원 커널은 5×5, 최소 면적은 400px²다. 최소 면적 이상 후보 중 가장 큰 윤곽의 모멘트로 중심을 구한다.

계산은 `ex=(cx-W/2)/(W/2)`, `ey=(cy-H/2)/(H/2)`, `area_ratio=contour_area/(W*H)`다. `W,H`는 실제 배열 크기를 사용하며 오른쪽·아래가 양수다. `point.x=ex`, `point.y=ey`, `point.z=area_ratio`다. `z`는 깊이가 아니다.

영상 콜백에서만 새 출력을 발행하고 입력 header를 보존한다. 정상 미검출은 0 값을 발행하며 입력이 끊긴 경우 이전 결과를 반복하지 않는다. `/target` QoS는 Best-effort·Keep-last depth 1·Volatile이다.

큰 컬러 영상의 PC 전송 조건에서 수신 저하가 관측됐다. 동일 검출 소스와 YAML의 사본을 Pi에 배치해 검출 위치만 바꾸고 작은 PointStamped를 PC로 전달했다. SDK·펌웨어·검출 임계값은 변경하지 않았다. 실행·작성 도구와 기여는 [별도 기록](AI_CONTRIBUTIONS.md)에 구분한다.

## 확인 결과

| 검증 | 결과와 범위 | 근거 |
|---|---|---|
| 같은 설정의 세 장면 | 정상·가림 검출, 없음 0; 장면별 한 장 | [세 장면 보고서](../../results/logs/perception/three-scenes-001/REPORT_KO.md) |
| 기존 ROS 인터페이스 | 실제 rgb8·header 10쌍·미검출 0·발행 QoS·입력 중단 무발행 | [기존 ROS 보고서](../../results/logs/perception/ros-interface-check-001/REPORT_KO.md) |
| 현재 PC 영상 처리 경로 | 10초 `/target` 25개·약 2.50Hz, 진단 수·간격·stamp 증가 기준 미달 | [원본 관측](../../results/logs/perception/target-stream-baseline-001/result.json) |
| Pi 로컬 검출 → PC | 10초 `/target` 297개·약 29.88Hz, 최대 도착 간격 약 0.086초 | [원본 관측](../../results/logs/perception/target-stream-pi-local-001/result.json) |
| Pi 검출 중 실제 컬러 입력 | 10초 로컬 관측 293개·약 29.99Hz | [입력 관측](../../results/logs/perception/pi-local-input-001/result.json) |
| Pi 입력·출력 header | 같은 stamp 10쌍, frame_id 불일치 0·발행 QoS 일치 | [header 대조](../../results/logs/perception/pi-local-header-001/result.json) |
| Pi 카메라 입력 중단 | 카메라 정상 종료·검출 유지 후 PC 10초 무발행 | [기대 동작 대조](../../results/logs/perception/target-input-stop-pi-local-001/expected-stop-result.json) |
| PC·Pi OpenCV 대조 | 같은 실제 사진의 마스크 해시·중심 오차·면적비 일치 | [버전 대조](../../results/logs/perception/target-stream-pi-local-001/cross-opencv-check.json) |
| 독립 대상 30·없음 10 평가 | 미완료: 실제 후보 30장 확보·목표 정답 확인 및 없음 10장 대기 | [인지 수행 목록](../../../vision_todo/vision_todo.md) |
| Issue·PR·팀 리뷰 | 미완료: 로컬 파일 배치 완료, 실제 Issue·PR·외부 리뷰 필요 | [팀 기록](team_perception.md) |

### 실제 프레임 평가 준비

[capture_ros_dataset.py](../../scripts/capture_ros_dataset.py)는 단조 증가하는 서로 다른 source stamp의 실제 RGB/BGR 프레임을 순번 PNG로 저장한다. 원본 hash·header·크기·인코딩·장면 정답 근거·실행 소스를 메타데이터에 남긴다. 기존 폴더를 덮어쓰지 않는다.

[evaluate_perception_dataset.py](../../scripts/evaluate_perception_dataset.py)는 대상 30·없음 10개, 원본 hash, 촬영 소스 hash, source stamp 중복, 장면 라벨 일치와 영상 조건을 검증하고 고정된 설정으로 처리한다. 프레임별 원본·마스크·검출·CSV·JSON과 설정·검출 소스 사본을 저장한다. 튜닝 사진의 해시와 겹치는 입력은 제외한다.

기존 검출 합성 검사 7개와 평가 출처 보호 검사 8개를 통과했다. 이 합성 검사 결과를 실제 카메라 정확도 결과로 사용하지 않는다. 현재 실제 후보 30장을 unlabelled로 저장했고 원본 해시와 사진 모음을 확인했다. 검출 후보 통계는 처리 상태이며 정답 정확도가 아니다. 목표 확인 후 label_perception_dataset.py로 원본 메타데이터를 유지한 별도 라벨 사본을 작성한다. 대상 없음 10장은 실제 물체 제거 확인 뒤 수집한다.

## 해석 및 한계

Pi 검출 구성의 짧은 관측에서 PC 결과 전달이 개선됐다. 큰 영상 전송을 줄인 효과와 관련된 근거이며 Wi-Fi·DDS의 근본 원인을 확정한 것은 아니다. 같은 사진 한 장의 버전 대조는 모든 사진의 계산 일치를 보장하지 않는다. 수신 관측은 장기 안정성·무손실·전체 시스템 추적 성능을 증명하지 않는다.

세 장면은 장면별 한 장이며 조명 차이와 튜닝 사진 재사용이 있다. 독립 40프레임은 별도로 평가한다. 대상 유무 판정 정확도와 목표 중심의 정량 위치 정확도도 구별한다. 연속 프레임의 유사성과 단일 장소·물체 조건은 평가 보고서에 한계로 남긴다.

진행률은 기존 증거 기준 5/7, 약 71%다. 독립 평가와 팀 제출 증빙이 모두 확인되기 전에는 7단계 완료로 표시하지 않는다.

## 팀 폴더 배치 검증

기존 `tracker_perception` 모듈을 팀 구조의 `target_detector` ROS 패키지로 배치했다. 알고리즘 파일은 동일하며 import·실행 등록·문서 경로와 중복 SIGINT 종료 처리만 조정했다. PC에서 두 ROS 패키지 빌드·합성 검사 15개·분리된 DOMAIN 187의 카메라 없는 launch 기동과 정상 종료를 확인했다. [배치 검증](../../results/logs/perception/repository-layout-check/checks.json)을 남겼다. 위 실제 Pi 실측은 기존 모듈에서 수행한 증거이며 새 ROS 패키지의 실제 Pi 검증을 대신하지 않는다.
