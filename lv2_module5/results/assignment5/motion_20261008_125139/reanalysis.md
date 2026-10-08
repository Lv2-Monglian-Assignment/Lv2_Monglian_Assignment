# 결과 재분석 (motion_20261008_125139)

| source | frames | duration_s | target_rate_hz | node_detect_ratio | tracking_ratio | rmse_ex | rmse_rows | to_tracking_transitions | max_abs_pan_cmd |
|---|---|---|---|---|---|---|---|---|---|
| bag 재분석 (/target 30 Hz 기준) | 271 | 15.38 | 17.56 | 0.9446 | 0.9188 | 0.2259 | 247 | 3 | 27.07 |
| 원본 제어 기록 (50 Hz 행 기준) |  | 52.38 |  | 0.8793 | 0.8403 | 0.2223 | 1747 |  |  |

- 재분석은 저장된 결과 토픽만 읽어 지표를 다시 계산한다(검출기 재실행 아님). 입력 재처리와 구분한다
- 원본 제어 기록은 추적을 켠 전체 구간, bag은 기록 구간이라 길이가 다를 수 있다. 표본 단위(30 Hz 영상 / 50 Hz 제어)도 다르다
