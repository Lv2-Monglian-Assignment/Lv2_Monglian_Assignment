# 인지 담당 작업 및 제출 상태

## 담당 범위

D435 컬러 입력·HSV와 윤곽 검출·목표 중심 오차와 면적비·ROS `/target` 발행·실제 영상 평가와 인지 보고서를 담당한다. 제어에는 출력 인터페이스를 전달하고, 통합에는 실행 환경·구성·검증 범위를 전달한다.

## 구현과 설정

- [검출 함수](../../ros2_ws/src/target_detector/target_detector/detector.py): HSV → open/close → 최대 유효 윤곽 → 실제 크기 정규화.
- [ROS 노드](../../ros2_ws/src/target_detector/target_detector/target_detector.py): Image 구독·입력 header 보존·PointStamped 발행·정상 미검출 0·입력 중단 무발행.
- [고정 설정](../../config/hsv.yaml): HSV 범위·타원 커널·최소 면적·요청 해상도.
- [실제 프레임 수집](../../scripts/capture_ros_dataset.py), [프레임 평가](../../scripts/evaluate_perception_dataset.py): 실제 정답 장면·출처 검증·원본과 결과 저장.

## 검증과 산출물

| 항목 | 상태 | 근거 |
|---|---|---|
| 실제 사진·색 순서·HSV | 확인 완료 | [인지 실습 노트](practice_summary.md) |
| 같은 설정의 정상·없음·가림 | 확인 완료; 장면별 한 장 | [세 장면 보고서](../../results/logs/perception/three-scenes-001/REPORT_KO.md) |
| 기존 ROS 인터페이스 | 제한된 실제 구간 확인 완료 | [ROS 보고서](../../results/logs/perception/ros-interface-check-001/REPORT_KO.md) |
| Pi 로컬 검출 → PC 전달 | 10초 297개·약 29.88Hz 관측 | [수신 근거](../../results/logs/perception/target-stream-pi-local-001/result.json) |
| 독립 대상 30·없음 10 평가 | 실제 후보 30장 확보; 정답·없음 10장 대기 | [수행 목록](../../../vision_todo/vision_todo.md) |
| 인지 보고서 | 현재 결과·한계 정리, 독립 평가 결과 대기 | [보고서](report.md) |
| 작성·실행 도구 기여 | 별도 기록 | [기여 내역](AI_CONTRIBUTIONS.md) |

## Issue·PR·리뷰 증빙

- [x] 팀 저장소 Lv2-Monglian-Assignment/Lv2_Monglian_Assignment와 인지 패키지·설정·문서 위치를 확인하고 로컬 사본을 배치했다.
- [ ] [Issue 초안](submission/ISSUE_DRAFT.md)을 실제 작업 Issue와 연결한다.
- [ ] [PR 초안](submission/PR_DRAFT.md)을 실제 PR·커밋과 연결한다.
- [ ] 팀원의 실제 리뷰·피드백·수정 증거를 연결한다.
- [ ] 최종 코드·설정·평가·보고서 링크를 제출 상태와 대조한다.

로컬 초안은 게시된 Issue·PR·리뷰 증빙이 아니다. 자체 코드 검토와 합성 검사도 팀원의 승인과 구별한다. 팀 저장소 대신 개인 키트 저장소에 임의로 올리지 않는다.

## 전달할 인터페이스

`/target`: `geometry_msgs/msg/PointStamped`, `x=ex`, `y=ey`, `z=area_ratio`이며 오른쪽·아래가 양수다. 입력 header를 유지하고 Best-effort·Keep-last depth 1을 사용한다. 정상 미검출은 0 메시지, 영상 입력 중단은 무발행이다. `z`를 깊이로 해석하지 않는다.

현재 검증 구성은 Pi에서 카메라와 동일 검출 코드를 실행하고 PC에는 작은 결과 메시지를 전달한다. 이전 PC 영상 처리 구성과 실행 위치가 달라졌음을 통합 시 공유한다. 모터·OpenCR 구동과 시스템 전체 추적 성공을 인지 결과로 기록하지 않는다.
