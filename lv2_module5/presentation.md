# 5분 시연 — 비전 기반 객체 추적 시스템 (몽글리안)

발제 권장 순서(구조 → 정상 추적 → 소실·복귀 → 정량 결과 → 재현·협업 → 한계)를 따른다. 현장 시연이 실패하면 아래 "대체 자료"의 녹화·로그로 마지막 확인 상태와 원인을 설명하고, 녹화는 녹화라고 밝힌다.

## 준비 (시연 전, Pi)

```bash
cd ~/git/Lv2_Monglian_Assignment/lv2_module5 && source ros2_ws/install/setup.bash
python3 assignment/main.py        # 통합 메뉴: t = 바로 최종 추적, 숫자 키 = 시험 프로그램
python3 scripts/web_view.py       # (선택) PC 브라우저 http://<pi-host>:8080/ 에 카메라·상태 화면
```

- 화면에 파란 물체는 목표 1개만 둔다(같은 색 물체가 있으면 가림 때 다른 물체로 넘어감 — report 문제 1 설정값 근거 실험 3).
- 카메라 주변을 비우고, 12 V 전원 차단 위치를 확인한다.

## 순서

| 순서 | 시간 | 보여 줄 것 | 말할 핵심 | 근거 |
|---|---|---|---|---|
| 1. 목표와 구성 | 0:00–0:30 | 노드 연결도 (report 문제 2 구조도) | D435 → `target_detector` → `/target`(ex·ey·면적비, 미검출 0) → `tracker_controller`(각도 Kp P 제어) → `opencr_bridge` → OpenCR → XM430 팬·틸트. 모두 Pi에서 실행 | report 문제 2 |
| 2. 정상 추적 | 0:30–1:30 | 목표를 좌우로 옮기며 추적 (웹 화면에 `TRACKING`·명령) | 속도형 P 제어 `clamp(dir × Kp × atan(e·tan(시야각/2)))`, 팬 Kp 2.0·틸트 2.5 [1/s], 상한 120°/s, 각도 한계 175°·38° | report 3-1·3-2 |
| 3. 소실·복귀 | 1:30–2:30 | 목표를 손으로 2 s 가림 → 정지 → 치우면 복귀. (선택) 검출 노드를 끊어 0.5 s 뒤 정지 | 미검출 첫 프레임부터 명령 0, 입력 0.5 s 없으면 `LOST:input_timeout`, 연속 3프레임 검출로 복귀. 층별 정지(제어 0.5 s → 브리지 0.2 s → OpenCR 300 ms → 모터 200 ms), 모터 통신 연속 3회 실패 시 FAULT → 자동 복구 | report 문제 2 정지 표, 문제 4, 발표 이후 변경 사항 |
| 4. 정량 결과 | 2:30–3:30 | 결과 표 | 검출률 30/30·오검출 0/10, 처리 FPS 30(카메라 입력과 같음), Kp 계단 응답(팬 2.0·틸트 2.5 선택 근거), 정상 추적 35 s 처리 FPS 27.4·유효 추적 비율 0.78·RMSE 0.33, 선택한 Kp 실제 추적 3회 RMSE 평균 0.23, 가림 복구 5/5·평균 0.23 s, 중단 시험 4종 PASS | report 문제 1·3·4 |
| 5. 재현·협업 | 3:30–4:30 | bag 재처리 그래프, team.md | 성공·소실 bag을 모터 없이 재처리(검출 일치 99.1 %·83.1 %), 다른 팀원 재현 확인, 4인 역할·PR·리뷰, 팀장 외 병합은 ruleset 확인용 #13 1건 | report 문제 5, team.md |
| 6. 한계 | 4:30–5:00 | 한계 목록 | 같은 색 물체가 함께 있으면 가림 중 다른 물체로 넘어감, 작은 먼 목표는 깊이 오측정으로 크기 상한에 걸릴 수 있음(1.37 m), 약 2.4 m부터 검출 한계, 손에 든 목표를 움직이면 번호 유지가 끊겨 0.43 s씩 정지(35 s 중 9회), bag에 원본 프레임 일부 누락 | report 각 문제 한계 |

## 대체 자료 (현장 시연 실패 시)

| 장면 | 자료 |
|---|---|
| 방향 확인 | `results/media/direction_test_20261008_142638.mp4` (녹화) |
| 정상·소실 추적 | 공유 드라이브 bag `assignment5_success_*`, `assignment5_lost_*`와 재처리 그래프 `results/plots/assignment5_*_replay_ex.png` |
| 보드 FAULT 자동 복구 | `results/logs/fault_recovery_20261008/summary.md` (시리얼·제어 기록) |
| 추적 시연 영상 | [최종 추적 자유 추적 15 s (녹화, 2026-10-08)](https://app.notion.com/p/teamsparta/D-_-3eb2dc3ef514800e9f49c3eba87f8c0d#3f32dc3ef51480ed940fd8dd482fb775) |
| bag 실험 영상 | [팀 Notion 영상 (녹화)](https://app.notion.com/p/teamsparta/D-_-3eb2dc3ef514800e9f49c3eba87f8c0d#3f32dc3ef5148004bc9cc7f97275749c) |

## 발표 이후 바뀐 점 (2026-10-07 발표 대비)

- FAULT: 모터 통신 1회 실패 → **연속 3회 실패**, FAULT 뒤 **2 s 간격 자동 복구 3회 → 실패 시 수동 복구 요청**. 근거와 장비 확인은 report "발표(2026-10-07) 이후 변경 사항".
