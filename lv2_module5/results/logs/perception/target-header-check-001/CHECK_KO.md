# 입력·출력 header 대조의 부분 확인

## 실험 환경과 목적

Pi D435 RGB8 640×480·30fps 요청 프로파일, PC 검출 노드, DOMAIN 30/Cyclone/SUBNET, Pi IP 추가 지정과 PC Wi-Fi XML을 사용했다. 입력 Image와 출력 PointStamped의 header 유지·미검출 0·실제 발행 QoS를 확인한다.

## 계획한 방법과 실제 수행 방법

카메라·검출 실행창을 유지하고 PC 확인창에서 tools/check_target_headers.py를 --matches 10 --timeout 20 --expect-zero로 직접 실행했다. 입력·출력 토픽을 동시에 구독했다. 검출기·설정은 변경하지 않았다.

## 결과

- [결과 JSON](result.json): matched_pairs=8/10, header_mismatches=0, nonzero_or_invalid_pairs=0, target_qos_ok=true, passed=false.
- 관측 건수는 Image 8개, PointStamped 11개다. 같은 stamp의 8쌍은 frame_id까지 모두 같고 point는 모두 0이다.
- 대조된 입력은 모두 rgb8 640×480, frame_id=camera_color_optical_frame이다.
- 실제 target_detector 발행자 1개, Best-effort / Keep-last 1 / Volatile을 확인했다.
- 모든 관측 Image는 출력과 대조됐고 미대조 출력 header는 3개다. pending cache evictions는 0이다.
- [실행 소스 사본](check_source.py)의 SHA-256은 메타데이터와 일치하며 현재 진단 소스와 동일하다.

## 해석 및 한계

확인한 8쌍의 header 유지·0 값과 현재 발행 QoS는 통과했다. 계획한 10쌍을 20초 내 받지 못해 전체 시험은 미통과다. 이것을 header 불일치나 QoS 오류로 해석하지 않는다. 추가 Image 구독이 영상 트래픽을 늘렸으며 드라이버 요청 30fps를 PC 수신 실측률로 대신하지 않는다. 미대조 출력 3개만으로 어느 경로의 손실인지 확정하지 않는다. 독립 30/10프레임 정확도 평가가 아니다.

다음 계획: 동일 코드·HSV·통신 설정·10쌍 기준을 유지하고 대기시간만 45초로 늘려 results/target-header-check-002에 새 결과를 저장한다. 기존 결과는 보존한다. 재시험은 아직 수행 전이며 성공하더라도 장기 안정성 검증으로 일반화하지 않는다. 진행률은 4/7, 약 57%다.

후속 결과: 동일 도구·10쌍 기준에서 대기시간 상한을 45초로 늘린 [재시험](../target-header-check-002/CHECK_KO.md)은 10쌍의 header·0 값·실제 발행 QoS 대조를 통과했다. 첫 시험의 원본 판정은 보존한다.
