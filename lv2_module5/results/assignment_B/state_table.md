# 상태 전이표 (tracker_controller, search_enabled=true)

| 상태 | 들어가는 조건 | 출력 명령 | 나가는 조건 |
|---|---|---|---|
| IDLE | 시작, /tracking_enable false | 0 | /tracking_enable true → LOST(confirming) |
| TRACKING | 신선한 검출 연속 recover_frames(3)프레임 | clamp(direction × Kp × 각도 오차) | 미검출 → LOST:no_detection, 입력 0.5 s 없음 → LOST:input_timeout |
| LOST | 미검출·입력 타임아웃·탐색 실패 | 0 (첫 프레임부터) | 미검출이 search_delay_s 이상 + 기억한 목표 있음 → SEARCHING, 검출 3프레임 → TRACKING |
| SEARCHING | (심화) 기억한 목표의 예측 방향으로 | 각도 P: clamp(search_kp × (예측 각 − 현재 각), ±search_speed_limit), 각도는 ±search_*_max_deg 안 | 검출 3프레임 → TRACKING(reacquired_search), search_timeout_s 초과 → LOST:search_failed(정지, 다음 TRACKING 전까지 재탐색 없음), /tracking_enable false → IDLE |
