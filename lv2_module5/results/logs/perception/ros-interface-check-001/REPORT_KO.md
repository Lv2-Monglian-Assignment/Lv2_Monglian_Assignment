# ROS 인지 인터페이스 검증 보고서

## 실험 환경

PC Ubuntu 24.04.5 / ROS Lyrical / 시스템 OpenCV 4.6.0, Pi Ubuntu 26.04.1 / ROS Lyrical / RealSense ROS·SDK 4.58.4·2.58.4를 사용했다. D435 USB 5000M, FW 5.15.1.55를 확인했다. 컬러 프로파일 요청은 RGB8 640×480·30fps이며 깊이는 비활성이다. DOMAIN 30 / Cyclone DDS / SUBNET을 사용했다. PC Wi-Fi 명시와 Pi IP 추가 발견 조건에서 전달을 확인했다.

## 실험 목적

발제 문제 2의 /target PointStamped, 입력 header 유지, 정상 미검출 0, Best-effort / Keep-last depth 1, 입력 중단 시 재발행 없음 동작을 실제 카메라 입력으로 확인한다. 정확도 평가와 장기 통신 성능 측정은 별도다.

## 계획한 방법

실제 영상·출력을 확인하고 같은 stamp의 입력·출력을 대조한다. 실제 발행 QoS를 조회한다. 정상 미검출을 확인한 뒤 카메라를 정상 종료하고 검출 노드를 유지해 무발행을 관측한다.

## 실제 수행 방법과 결과

| 확인 항목 | 결과 | 근거 |
|---|---|---|
| 실제 컬러 Image 입력 | PC 한 개 수신, 새 노드 rgb8 640×480 처리 | [Image 수신](../practice_records/pc-direct-image-003-static-peer-received.json), [노드 로그](../practice_records/target-node-start-004-static-peer-received.log) |
| PointStamped 전달 | /target의 header·point 수신 | [실시간 미검출 0](../practice_records/target-message-003-zero-received.json) |
| stamp·frame_id 유지 | 같은 stamp 10쌍에서 frame_id까지 일치 | [header 재시험](../target-header-check-002/CHECK_KO.md) |
| 정상 미검출 | 대조된 10쌍 모두 x=y=z=0 | [header 재시험 결과](../target-header-check-002/result.json) |
| 실제 발행 QoS | 단일 target_detector, Best-effort / Keep-last 1 / Volatile | [header 재시험 결과](../target-header-check-002/result.json) |
| 입력 중단 | 카메라 정상 종료·검출 유지 후 15초 상한 관측에 재발행 없음 | [입력 중단 검토](../target-input-stop-check-001/CHECK_KO.md) |

명령은 터미널에 직접 입력했다. 저장 결과·프로세스·로그를 확인해 정리했다. 검출기의 HSV [92,80,26]~[120,255,255], 타원 커널 5, 최소 면적 400은 유지했다. 입력 실제 크기 로그 및 저장 사진 계산은 확인했지만 비영 ROS 출력과 동일 프레임의 중심 좌표를 정량 대조하는 정확도 평가는 아직 수행하지 않았다.

## 해석 및 한계

ROS 인터페이스 검증 단계의 근거를 확보했다. 확인 구간에 한정한 완료이며 640×480·30fps의 PC 장기 수신 안정성·전체 프레임 손실률·실제 물체 검출 정확도 완료가 아니다. 20초 첫 header 시험은 8쌍만 받아 미통과했고 대기 상한 45초 재시험은 10쌍을 대조해 통과했다. 추가 관찰용 Image 구독의 트래픽 영향을 기록했다. Pi IP 명시 뒤 발견·전달 성공을 관측했지만 멀티캐스트 차단을 근본 원인으로 확정하지 않는다.

진행률은 기존 7단계 중 5단계, 약 71%다. 다음은 새 평가용 대상 30프레임·없음 10프레임을 확보하고 정답 확인·검출 결과·오검출 및 미검출 원인을 기록한다. 최종 보고서·Issue·PR·리뷰·팀 문서 증빙은 별도로 남아 있다.
