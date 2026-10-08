# 입력 재처리 (motion_20261008_125139)

- 명령: `ros2 launch tracker_bringup replay.launch.py bag:=... run_id:=...` (bag의 영상·CameraInfo·정렬 Depth·모터 각도만 재생, 검출 결과는 /target_replay로 분리, 모터 노드 없음)
- 기준 커밋 8f897bb-dirty, 설정 config/

| orig_frames | replay_frames | matched_frames | orig_detect_ratio | replay_detect_ratio | detect_agreement_pct | ex_mean_abs_diff | ex_max_abs_diff |
|---|---|---|---|---|---|---|---|
| 271 | 308 | 229 | 0.9446 | 0.961 | 89.08 | 2.306e-05 | 4.984e-05 |

- 같은 영상 stamp끼리 비교: 검출 여부 일치율, 둘 다 검출한 프레임의 ex 차이
- 번호 유지(object_tracker)는 모터 각도(/pan_tilt/joint_states)를 쓰므로 bag에 각도가 있어야 같은 선택이 재현된다
- 그래프: results/plots/motion_20261008_125139_replay_ex.png, 재처리 기록: replay_motion_20261008_125139_20261008_141530_detect.csv
