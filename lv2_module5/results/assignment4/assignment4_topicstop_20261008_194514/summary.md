# 인지 입력 중단 (assignment4_topicstop_20261008_194514)

| status_before | kill_to_timeout_s | input_timeout_cfg_s | nonzero_cmd_rows_after_timeout | verdict |
|---|---|---|---|---|
| TRACKING:ok | 0.4819 | 0.5 | 0 | PASS |

- kill 시각 1791456361.452 (시스템 시계), 마지막 입력 stamp 1791456361.270. 타임아웃은 마지막 신선한 입력 수신 기준이라 kill 시각과 최대 1프레임(33 ms) 차이
- 판정: LOST:input_timeout 진입, 그 뒤 명령 0만 발행
