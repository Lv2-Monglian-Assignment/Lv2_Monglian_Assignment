# 인지 → 제어 인터페이스 (`/target`)

인지 노드가 보내고 제어 노드가 받는 목표 정보의 규약이다. 인지·제어·통합은 이 문서를 기준으로 구현·리뷰하고, 값을 바꾸면 같은 PR에서 이 문서를 함께 고친다(리뷰어: 인지·제어 담당 모두).
기준: 2026-10-06 실험용 Raspberry Pi 실측. 제어 → 브리지 → OpenCR 명령은 4절과 #7(펌웨어 `firmware/opencr_tracker`)을 따른다.

## 1. `/target` 메시지

| 항목 | 값 |
|---|---|
| 토픽 · 형식 | `/target` · `geometry_msgs/msg/PointStamped` |
| 발행 노드 | 인지 (`target_detector`) |
| 구독 노드 | 제어 (`tracker_controller`) |
| QoS | best-effort · keep-last · depth 1 (구독 측도 best-effort로 맞춤) |
| 주기 | 처리한 영상마다 1번 (카메라 30 Hz, 실측 28.7~30 Hz). 고정 타이머로 보내지 않음 |

| 필드 | 의미 | 단위 · 범위 | 부호·규칙 |
|---|---|---|---|
| `header.stamp` | 원본 Color 영상의 stamp를 그대로 복사 | ROS 시간 | 새 시각을 붙이지 않음. 같은/과거 stamp 영상은 발행하지 않음 |
| `header.frame_id` | 원본 영상의 frame_id | `camera_color_optical_frame` | |
| `point.x` | ex = (cx − W/2) / (W/2) | 정규화, −1 ~ +1 | **오른쪽 +** |
| `point.y` | ey = (cy − H/2) / (H/2) | 정규화, −1 ~ +1 | **아래 +** |
| `point.z` | 면적비 = contour_area / (W × H) | 0 ~ 1 | **0 = 미검출** (깊이·거리가 아님) |

- W, H는 실제 수신한 영상 크기다(640 × 480 고정).
- 미검출이면 `x = y = z = 0`을 보낸다. 이때 x, y는 의미가 없으므로 제어에 쓰지 않는다.
- 안 보이는 목표의 이전 위치·예측 위치는 `/target`으로 보내지 않는다(발제 문제 1).

## 2. 인지 쪽 발행 규칙

| 상황 | 인지 노드 동작 | 제어 노드가 보는 것 |
|---|---|---|
| 목표 검출 | ex, ey, 면적비 발행 | z > 0 |
| 정상 영상인데 목표 없음 (가림·시야 밖) | z = 0 발행 (침묵 아님) | 미검출 → `LOST:no_detection` |
| 카메라·인지가 멈춤 | 아무것도 발행하지 않음 | 입력 없음 → 0.5 s 뒤 `LOST:input_timeout` |
| 같은/과거 stamp 영상 | 버림 (재발행 금지) | 입력 없음과 같음 |

- 대상 선택: HSV `[102,120,40]`~`[110,255,255]` → 잡음 제거(5 px) → 면적 ≥ 100 px² → 깊이 크기 검증 2.0~30 cm² → priority(가장 큼 → 가까움 → 화면 중앙). 같은 색 물체가 여럿이면 한 번 고른 물체를 번호로 고정한다(그 물체가 안 보이면 z = 0, 3 s 뒤 다시 고름).

## 3. 제어 쪽 수신 규칙

| 규칙 | 값 |
|---|---|
| 미검출(z = 0) | 첫 프레임부터 속도 0 (이전 속도 유지 금지) |
| 입력 타임아웃 | 마지막 신선한 입력 후 0.5 s → 속도 0, `LOST:input_timeout` |
| 복귀 | 신선한 검출 연속 3프레임 → TRACKING |
| 명령 | `clamp(direction × Kp × atan(ex × tan(시야각/2)), ±속도 상한)` [°/s]. Kp 팬 2.0·틸트 2.5 [1/s], 시야각 55.7°·43.2°, 상한 120°/s. 팬 direction −1 (ex > 0이면 오른쪽으로 회전), 틸트 +1 |
| 데드밴드 | 팬 |ex| < 0.03, 틸트 |ey| < 0.05 → 0 |

## 4. 함께 쓰는 토픽

| 토픽 | 형식 | 발행 → 구독 | 내용 |
|---|---|---|---|
| `/target_depth` | PointStamped | 인지 → (기록) | x = 깊이 유효 1/0, y = 유효 픽셀 비율, z = 거리 [m]. `/target`과 같은 stamp. 평가표 4번(depth_valid·z 구분) 증거 |
| `/target/position_cam` | PointStamped | 인지 → 제어 | 목표의 카메라 광학 좌표 [m] (X 오른쪽, Y 아래, Z 앞), 무효면 NaN. 목표 기억·탐색(선택)용 |
| `/tracking_status` | String | 제어 → (기록) | `"상태:사유"` 예: `TRACKING:ok`, `LOST:no_detection`, `LOST:input_timeout`, `LOST:confirming_1/3` |
| `/pan_tilt/command` | Vector3Stamped | 제어 → 브리지 | x = 팬, y = 틸트 [°/s], 50 Hz (IDLE·LOST에서도 0 발행). 브리지는 0.2 s 끊기면 `V 0 0` |
| `/pan_tilt/joint_states` | JointState | 브리지 → 제어·인지 | 모터 각도 [rad]·속도 [rad/s], 이름 `pan`·`tilt`. 인지의 번호 유지 회전 보정에 사용. 모터 출력 없는 시험(dry_run)은 이름 `pan_sim`·`tilt_sim`이며 인지는 무시 |

## 5. 확인 방법 (모터 출력 끔)

| 입력 (`/target`) | 기대 결과 |
|---|---|
| x = 0, z > 0 | 회전 없음 |
| x = +0.4, z > 0 | 오른쪽 오차를 줄이는 명령 (팬 음수) |
| x = −0.4, z > 0 | 반대 방향 (팬 양수) |
| z = 0 | `LOST:no_detection`, 명령 0 |
| 발행 중단 | 0.5 s 뒤 `LOST:input_timeout`, 명령 0 |

```bash
# 예: x=+0.4 입력 (Pi, ROS_DOMAIN_ID=28·Cyclone DDS 환경)
ros2 topic pub /target geometry_msgs/msg/PointStamped "{point: {x: 0.4, y: 0.0, z: 0.05}}" -r 30
ros2 topic echo /tracking_status
```

- 2026-10-05 실험용 구현에서 다섯 입력과 틸트(y = +0.4) 모두 기대대로 동작했다(입력 중단 후 0.515 s에 정지).
