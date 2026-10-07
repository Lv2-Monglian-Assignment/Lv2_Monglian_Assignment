# 모의 입력 (assignment2_mock_20261007_123707)

- Kp 팬 2.0·틸트 2.5 [1/s], direction 팬 -1·틸트 1, 입력 타임아웃 0.5 s, 모터 출력 없음(브리지 미실행)

| case | expected | status | pan_cmd | tilt_cmd | timeout_s | cmd_zero_s | inputs_received | verdict |
|---|---|---|---|---|---|---|---|---|
| center | x=0, z>0: 회전 없음 | TRACKING:ok | 0 | 0 |  |  | 90/90 | PASS |
| right | x=+0.4: 팬 음수(오른쪽으로 회전) | TRACKING:ok | -23.87 | 0 |  |  | 90/90 | PASS |
| left | x=-0.4: 팬 양수 | TRACKING:ok | 23.87 | 0 |  |  | 90/90 | PASS |
| nodetect | z=0: LOST:no_detection, 명령 0 | LOST:no_detection | 0 | 0 |  |  | 90/90 | PASS |
| silence | 발행 중단: 0.5 s 뒤 LOST:input_timeout, 명령 0 | LOST:input_timeout | 0 | 0 | 0.5216 | 0.5216 | 60/60 | PASS |
| stale | 같은 stamp 재전송: 신선한 입력 아님 → input_timeout, 명령 0 | LOST:input_timeout | 0 | 0 |  |  | 1/1 | PASS |
| down | y=+0.4: 틸트 양수(아래로 회전) | TRACKING:ok | 0 | 22.5 |  |  | 90/90 | PASS |

- inputs_received: 제어 노드가 받은 신선한 입력 / 보낸 입력(같은 stamp 재전송은 첫 1개). 90 % 미만이면 RETEST
- timeout_s·cmd_zero_s: 실제로 마지막 /target을 보낸 시각부터 LOST:input_timeout·명령 0이 관측될 때까지 (상태·명령은 50 Hz로 받음)

- 미검출(z=0)은 정상 영상에서 목표가 없다는 뜻이라 즉시 정지(LOST:no_detection), 토픽 침묵은 인지가 멈췄다는 뜻이라 0.5 s 뒤 정지(LOST:input_timeout)로 구분한다.
- 제어 기록: ~/lv2_module5_logs/assignment2_mock_20261007_123707.csv, 구조도: structure.md
