# 보드 FAULT 자동 복구·기준 자세 이동 장비 시험 (2026-10-08)

- 장비: Raspberry Pi(pa23) + OpenCR + XM430 ×2, 펌웨어 `opencr_tracker`(FAULT 연속 3회·상태 보고·`R` 복구·`B` 바퀴 수 유지)
- 코드: 병합 전 수정 브랜치 4개(인지·제어·시험 프로그램·브리지/펌웨어)를 합친 Pi 시험 폴더. 이후 [#48](https://github.com/Lv2-Monglian-Assignment/Lv2_Monglian_Assignment/pull/48)·[#52](https://github.com/Lv2-Monglian-Assignment/Lv2_Monglian_Assignment/pull/52)·[#53](https://github.com/Lv2-Monglian-Assignment/Lv2_Monglian_Assignment/pull/53)·[#54](https://github.com/Lv2-Monglian-Assignment/Lv2_Monglian_Assignment/pull/54)로 병합
- 실행: `full.launch.py`(카메라·인지·제어·브리지), ROS_DOMAIN_ID 77(다른 실행과 분리), 추적 켬
- 시각: 각 기록 파일의 시작 기준 [s]. 시리얼 기록(브리지)과 제어 CSV(제어 노드)는 시작 시점이 달라 같은 사건도 시각이 조금 다르다

## 파일

| 파일 | 내용 |
|---|---|
| `hw_t2_serial.log` | 시험 2: 브리지 재시작 시 팬 바퀴 수 유지. 브리지 ↔ OpenCR 송수신(`>>>` = Pi → OpenCR) |
| `hw_t4_serial.log` | 시험 4: 시작 시 기준 자세 이동 + 추적 중 모터 케이블 약 1 s 분리 → FAULT → 자동 복구 |
| `hw_t4.csv` | 시험 4의 제어 노드 기록(상태·사유·명령·관절 각도) |

## 시험 2 — 브리지 재시작 (`B`가 바퀴 수를 유지하는지)

| 시각 [s] | 기록 | 팬 [°] | 틸트 [°] |
|---|---|---|---|
| 0.008 | `>>> B 1654 2007` (기준 tick 전송) | | |
| 0.105 | 상태 `HOLD` (재시작 직전 자세) | −51.68 | 28.39 |
| 0.171 | 회신 `B 1654 2007` | | |
| 0.173 | 상태 `HOMING` | −51.59 | 28.39 |
| 2.925 | 상태 `HOLD` (기준 자세 도착) | −0.44 | 0.18 |

`B` 전후 팬 각도가 −51.68° → −51.59°로 이어져, 기준 자세를 다시 설정해도 바퀴 수가 바뀌지 않았다.

## 시험 4 — 시작 기준 자세 이동과 FAULT 자동 복구

시리얼 기록 (`hw_t4_serial.log`)

| 시각 [s] | 기록 | 팬 [°] | 틸트 [°] |
|---|---|---|---|
| 0.023 · 0.024 | `>>> B 1654 2007` · `>>> I` | | |
| 0.165 | 상태 `HOMING` | −4.57 | 13.01 |
| 1.520 | 상태 `HOLD` | −0.26 | 0.44 |
| 2.798 | 상태 `TRACK` | | |
| 74.876 | `E 4 DXL read failed` → `E 4 FAULT: send R to recover` (오류 보고 1회) | | |
| 74.887 | 상태 `FAULT` (토크 OFF, 상태 줄은 계속 보냄) | −15.56 | 29.18 |
| 76.902 | `>>> R` (FAULT 2 s 뒤 자동 복구 1회차) | | |
| 76.918 | `R OK` | | |
| 76.919 | `>>> I` (복구 후 기준 자세 이동) | | |
| 76.956 | 상태 `HOMING` (토크 OFF 동안 처진 틸트에서 출발) | −15.47 | 56.87 |
| 79.940 | 상태 `HOLD` | −0.09 | 0.44 |
| 83.717 | 상태 `TRACK` | | |

제어 기록 (`hw_t4.csv`, 확인 중 상태 `LOST:confirming_*` 생략)

| 시각 [s] | 상태:사유 | 명령 [°/s] |
|---|---|---|
| 0.177 | `LOST:board_homing` | 0 |
| 2.517 | `TRACKING:reacquired_in_view` | 추적 |
| 74.273 | `LOST:board_silent` (브리지가 OpenCR 상태 줄 0.5 s 없음을 알림, `NO_STATUS`) | 0 |
| 74.694 | `LOST:board_fault` | 0 |
| 76.755 | `LOST:board_homing` | 0 |
| 79.752 | `LOST:no_detection` | 0 |
| 83.474 | `TRACKING:reacquired_in_view` | 추적 |

- FAULT 보고(74.876 s)부터 `R OK`까지 2.04 s, `R OK`부터 기준 자세 도착(`HOLD`)까지 3.02 s, `R OK`부터 추적(`TRACK`) 재개까지 6.80 s (시리얼 기록 기준).
- `LOST:board_silent`부터 다시 TRACKING이 될 때까지(74.273~83.474 s) 제어 노드의 팬·틸트 명령은 모두 0이었다.
- 케이블을 1 s만 뺐으므로 자동 복구 1회차에 성공했다. 3회 모두 실패해 `FAULT_MANUAL`이 되는 경우는 이 시험 전 같은 날 시험(틸트 토크 켜기 제한 45°였던 펌웨어)에서 `LOST:board_fault_manual`까지 확인했으나, 그 기록은 이 폴더에 넣지 않았다.
