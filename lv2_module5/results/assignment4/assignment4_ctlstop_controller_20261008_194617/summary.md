# 제어 통신 중단: controller kill -9 (assignment4_ctlstop_controller_20261008_194617)

| condition | pan_speed_before_dps | last_motion_after_kill_s | bridge_cmd_timeout_cfg_s | verdict |
|---|---|---|---|---|
| controller kill -9 | 8.24 | 0.3215 | 0.2 | PASS |

- 측정: 브리지가 회신한 /pan_tilt/joint_states 속도(실제 모터). 1 deg/s 넘는 마지막 시각 = 정지까지 걸린 시간
- 기대: 브리지 cmd_timeout 0.2 s 뒤 V 0 0 + 감속 시간
