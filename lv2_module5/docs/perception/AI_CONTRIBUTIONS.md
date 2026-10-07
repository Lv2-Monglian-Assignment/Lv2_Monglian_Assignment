# 인지 작업 작성·실행 도구 기록

## 기여 구분

| 항목 | 실제 기여와 실행 범위 |
|---|---|
| 제공 예제 | `examples/`는 외부 제공 자료이며 원본을 보존했다. 작성 기여로 계산하지 않는다. |
| 기존 실습 | 터미널에 직접 입력한 촬영·검출·ROS 출력과 보관 파일을 근거로 기록했다. 각 원본 기록의 출처를 유지한다. |
| 코드·기록 정리 | Codex로 ROS 연결 코드·진단 도구·문서 준비와 읽기 전용 확인을 수행했다. 기존 파일의 작성 기여는 보존한다. |
| 남은 작업 실행 | 인지 7단계까지 터미널 실행을 맡기는 명시적 요청에 따라 Codex 터미널 도구로 카메라·검출·진단·합성 검사를 실행했다. 이 실행을 직접 입력한 과거 실습으로 바꾸어 쓰지 않는다. |
| 새 평가 도구 | Codex로 실제 프레임 수집·출처 검증·고정 설정 평가 코드와 출처 보호 검사를 작성했다. 실제 후보 30장을 정답 라벨 없이 수집·처리했다. 독립 40프레임 정답 평가는 아직 대기 상태다. |
| 입력 장면 정답 | 실제 목표 지정·물체 배치와 원본 사진 검토를 근거로 정한다. 검출 결과 자체를 정답으로 사용하지 않는다. |
| 외부 리뷰 | 아직 확보하지 않았다. 코드 검사와 자체 검토를 팀원의 PR 승인으로 표기하지 않는다. |

## 증거

- [직접 실행 요청과 시작 상태](../../results/logs/perception/practice_records/autonomous-perception-001.json)
- [동일 Pi 실행 소스·설정 사본의 해시](../../results/logs/perception/autonomous-runtime-001/pi-bundle-manifest.json)
- [PC 영상 처리 조건](../../results/logs/perception/target-stream-baseline-001/result.json)
- [Pi 검출 결과의 PC 수신](../../results/logs/perception/target-stream-pi-local-001/result.json)
- [같은 실제 사진의 OpenCV 대조](../../results/logs/perception/target-stream-pi-local-001/cross-opencv-check.json)

합성 검사, 실제 컬러 설치 확인, 진단 메시지 관측, 실제 정답 라벨 평가를 각각 구별한다. 비밀번호·개인키·토큰은 기록하지 않는다.

## 팀 구조 배치

Codex 터미널 도구로 원본 사본을 팀 폴더에 분류하고 ROS 패키지 등록·launch·import 경로·중복 SIGINT 종료 처리를 작성했다. PC 빌드·합성 검사·카메라 없는 launch 기동/종료를 실행했다. 기존 원본 코드와 실제 Pi 실행 자료를 보존했다.
