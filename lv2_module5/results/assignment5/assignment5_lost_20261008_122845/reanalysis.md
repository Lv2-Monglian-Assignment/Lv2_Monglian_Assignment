# 결과 재분석 (assignment5_lost_20261008_122845)

| source | frames | duration_s | target_rate_hz | node_detect_ratio | tracking_ratio | rmse_ex | rmse_rows | to_tracking_transitions | max_abs_pan_cmd |
|---|---|---|---|---|---|---|---|---|---|
| bag 재분석 (/target 30 Hz 기준) | 256 | 14.98 | 17.02 | 0.7812 | 0.6914 | 0.2126 | 171 | 6 | 45.22 |
| 원본 제어 기록 (50 Hz 행 기준) |  | 30.53 |  | 0.8215 | 0.7514 | 0.2373 | 804 |  |  |

- 재분석은 저장된 결과 토픽만 읽어 지표를 다시 계산한다(검출기 재실행 아님). 입력 재처리와 구분한다
- 원본 제어 기록은 추적을 켠 전체 구간, bag은 기록 구간이라 길이가 다를 수 있다. 표본 단위(30 Hz 영상 / 50 Hz 제어)도 다르다
