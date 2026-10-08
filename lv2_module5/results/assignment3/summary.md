# Problem 3 Kp compare

회차별 (발제 문제 3·4 산식, 제어 기록 50 Hz 행 기준)

| run_id | pan_kp | rmse_ex | tracking_ratio | response_mean_s | response_missing | cmd_flips_per_s | rmse_rows | excluded_rows |
|---|---|---|---|---|---|---|---|---|
| assignment3_kp2_t1 | 2.0 | 0.1735 | 1 | 0.447 | 0 | 0.1658 | 604 | 0 |
| assignment3_kp2_t2 | 2.0 | 0.2688 | 0.9289 | 0.05977 | 0 | 0.5348 | 562 | 43 |
| assignment3_kp2_t3 | 2.0 | 0.259 | 0.9489 | 0.5337 | 0 | 0.3478 | 576 | 31 |

설정별 평균

| pan_kp | trials | rmse_ex_mean | rmse_ex_sd | tracking_ratio_mean | response_mean_s | cmd_flips_per_s_mean |
|---|---|---|---|---|---|---|
| 2.0 | 3 | 0.2338 | 0.05245 | 0.9593 | 0.3468 | 0.3495 |

- RMSE = sqrt(mean(ex²)), 검출·TRACKING 행만 (제외 행 수 병기). 응답 시간 = 구간 시작 → |ex| <= 0.1 첫 도달
- 흔들림 = TRACKING 중 팬 명령 부호가 바뀐 횟수 / TRACKING 시간
- pan_deg는 모터가 회신한 실제 각도(명령 적분 아님). dry_run 회차는 시뮬레이션 각도

그래프: results/plots/assignment3_ex.png, results/plots/assignment3_pan_deg.png, results/plots/assignment3_pan_cmd.png
