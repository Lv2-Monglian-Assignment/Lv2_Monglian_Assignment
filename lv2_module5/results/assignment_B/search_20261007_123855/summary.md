# 도전 B SEARCHING 시험 (모터 출력 없음: 브리지 dry_run + 가상 물체)

- 탐색 설정: search_delay_s 0.3, search_timeout_s 3.0, search_kp 2.0, search_speed_limit_deg_s 20.0, search_pan_max_deg 80.0, search_tilt_max_deg 25.0

| scenario | verdict | final_state | final_max_cmd | longest_search_s | transitions |
|---|---|---|---|---|---|
| found | PASS | TRACKING:ok | 3.991 | 1.334 | LOST:input_timeout@0.00 → LOST:confirming_1/3@2.27 → LOST:confirming_2/3@2.29 → TRACKING:reacquired_in_view@2.33 → TRACKING:ok@2.35 → LOST:no_detection@6.23 → SEARCHING:predicted@6.53 → SEARCHING:confirming_1/3@7.83 → SEARCHING:confirming_2/3@7.87 → TRACKING:reacquired_search@7.89 → TRACKING:ok@7.91 |
| cancel | PASS | IDLE:disabled | 0 | 0.4975 | LOST:input_timeout@0.84 → LOST:confirming_1/3@2.25 → LOST:confirming_2/3@2.29 → TRACKING:reacquired_in_view@2.32 → TRACKING:ok@2.33 → LOST:no_detection@6.24 → SEARCHING:predicted@6.53 → IDLE:disabled@7.05 |
| notfound | PASS | LOST:search_failed | 0 | 2.997 | LOST:input_timeout@1.11 → LOST:confirming_1/3@2.32 → LOST:confirming_2/3@2.36 → TRACKING:reacquired_in_view@2.38 → TRACKING:ok@2.39 → LOST:no_detection@6.28 → SEARCHING:predicted@6.58 → LOST:search_failed@9.60 |

- 상태 전이표: state_table.md. 시야 밖 탐색 성공(found)은 문제 4의 시야 내 재등장 복구와 별도 통계다
- 미발견·취소 모두 명령 0으로 끝나야 한다(무제한 탐색 없음)
