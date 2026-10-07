# 인지 실험 요약

## 실험 환경

Pi D435 RGB8 640×480, Lyrical·Cyclone DDS·DOMAIN 30을 사용했다. PC OpenCV 4.6, Pi OpenCV 4.10과 실제 장치·카메라 버전을 [보고서](report.md)에 기록했다.

## 실험 목적과 계획한 방법

실제 영상·색 순서·HSV·동일 설정의 세 장면·ROS 인터페이스·독립 30·10프레임 평가·제출 자료를 순서대로 확인한다. 발제 문제 1은 HSV 검출과 장면 검증, 문제 2는 `/target` 인터페이스에 대응한다. 모터 제어·전체 통합 성공은 별도 역할이다.

## 실제 수행 방법

이전 터미널 직접 입력 실습의 원본·로그를 보존했다. 이후 남은 인지 실행을 맡긴 범위에서 Codex 터미널 도구로 같은 검출 코드의 Pi 실행·관측·출처 검사·사진 수집을 수행했다. 실제 수행 도구를 직접 입력한 과거 실습으로 바꾸어 기록하지 않는다. 원래 상세 진행 노트는 원본 작업 폴더에 보존하고 이 저장소에는 필요한 요약·근거를 분류했다.

## 확인 결과

- 정상·없음·가림 세 장면의 같은 설정 검증: [장면 보고서](../../results/logs/perception/three-scenes-001/REPORT_KO.md).
- 기존 실제 입력·header·QoS·미검출·입력 중단 검증: [ROS 보고서](../../results/logs/perception/ros-interface-check-001/REPORT_KO.md).
- Pi 검출 → PC PointStamped 10초 297개·약 29.88Hz, Pi 로컬 입력 약 29.99Hz: [현재 구성](../../results/logs/perception/target-stream-pi-local-001/CHECK_KO.md).
- 같은 실제 사진의 PC·Pi 마스크·좌표 일치: [대조 근거](../../results/logs/perception/target-stream-pi-local-001/cross-opencv-check.json).
- 실제 후보 30장 수집·hash 검토·검출 처리, 목표 정답 미확인: [후보 원본](../../results/images/perception/evaluation-001/candidate-present/dataset.json).

## 해석 및 한계

완료 증거는 기존 7단계 중 5개, 약 71%다. 실제 목표 확인·목표 제거 후 없음 10장·40프레임 정답 평가는 남아 있다. Issue·PR·의미 있는 타인 리뷰의 실제 협업 근거도 남아 있다. 후보 30장과 합성 검사·진단 메시지를 정확도 평가로 합산하지 않는다.
