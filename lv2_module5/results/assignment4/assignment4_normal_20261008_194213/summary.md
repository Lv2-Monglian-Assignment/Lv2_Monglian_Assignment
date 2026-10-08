# 문제 4 정상 추적 (assignment4_normal_20261008_194213)

| processing_fps | camera_fps | frames | duration_s | node_detect_ratio | tracking_ratio | rmse_ex | rmse_rows | excluded_rows | max_abs_ex | cmd_flips_per_s |
|---|---|---|---|---|---|---|---|---|---|---|
| 27.35 | 27.3 | 958 | 34.98 | 0.8069 | 0.7811 | 0.3307 | 1367 | 383 | 0.9652 | 0.9516 |

- 처리 FPS = 처리 완료 프레임 수 / 실제 경과 초(인지 노드 발행 시각), 카메라 FPS = stamp 기준 (서로 다름)
- node_detect_ratio는 노드의 detected 비율이다. 사람 대조 검출률은 eval·score로 따로 낸다
- RMSE = sqrt(mean(ex²)), 검출·TRACKING 행만 사용(제외 행 수 병기). 유효 추적 비율 = TRACKING 행 / 전체 행
