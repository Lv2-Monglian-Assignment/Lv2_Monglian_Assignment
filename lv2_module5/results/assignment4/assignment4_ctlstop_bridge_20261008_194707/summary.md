# 제어 통신 중단: bridge kill -9 (assignment4_ctlstop_bridge_20261008_194707)

| condition | serial_read_from_s | timeout_event | final_state | final_speed_dps | verdict |
|---|---|---|---|---|---|
| bridge kill -9 | 0.08089 | 0.298s E 1 command timeout; stop | HOLD | 0.00 0.00 | PASS |

- 측정: 브리지 kill 직후 시리얼을 직접 열어 OpenCR 상태 줄(S ...)을 1.5 s 읽음. 기대: 300 ms 안에 속도 0, 상태 HOLD(토크 유지)
