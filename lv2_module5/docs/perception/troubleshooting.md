# 인지 영상 전달 진단 요약

## 실험 환경과 목적

D435 컬러 영상을 Pi에서 만들고 PC에서 HSV 검출하는 구성의 수신 저하를 확인했다. 인지의 입력·결과 전달 경로를 구분하며 제어 구동은 수행하지 않았다.

## 계획한 방법과 실제 수행

발견·영상 수신·소규모 메시지·입력/출력 header·프로파일·PC 구독 유무를 구분했다. 새 관측은 새 폴더에 보관하고 기준·설정·출처를 기록했다. 같은 검출 코드와 설정을 Pi에서 실행해 PC에는 PointStamped만 전송했다.

## 확인 결과

| 조건 | 짧은 관측 | 근거 |
|---|---|---|
| PC 검출 중 Pi 320×240·6Hz 입력 | 32개·약 3.33Hz | [Pi 입력](../../results/logs/perception/pi-image-stream-check-002/CHECK_KO.md) |
| PC 검출 중단 후 같은 Pi 입력 | 58개·약 5.99Hz | [중단 조건](../../results/logs/perception/pi-image-stream-check-003/CHECK_KO.md) |
| 현재 PC 640×480 영상 처리 경로 | PointStamped 25개·약 2.50Hz | [원본](../../results/logs/perception/target-stream-baseline-001/result.json) |
| 동일 코드의 Pi 검출 → PC | PointStamped 297개·약 29.88Hz | [현재 구성](../../results/logs/perception/target-stream-pi-local-001/CHECK_KO.md) |

## 해석 및 한계

큰 영상 전송을 제거한 실행 위치 변경에서 짧은 전달이 개선됐다. Wi-Fi·DDS 근본 원인이나 장기 안정성을 확정하지 않는다. 이전 관측 004는 카메라가 없는 상태의 0장이라 속도 비교에 사용하지 않는다. 원래 상세 진단 이력은 원본 작업 폴더에 보존했다. 현재 패키지 재배치의 빌드·기동 확인과 원본 모듈의 실제 카메라 실측을 구별한다.
