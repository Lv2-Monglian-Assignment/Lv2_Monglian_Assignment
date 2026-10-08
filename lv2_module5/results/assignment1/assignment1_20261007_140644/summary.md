# 문제 1 세 장면 (assignment1_20261007_140644)

- 설정: HSV [102, 120, 40]~[110, 255, 255], 최소 면적 100.0 px², 크기 검증 2.0~30.0 cm², 선택 priority (config/ 복사본)
- 미검출 전달: 정상 영상에서 목표가 없으면 /target z=0 (x=y=0) 발행, 이전 좌표 재사용 없음

| scene | detected | ex | ey | area_ratio | n_candidates | n_rejected | z_m | images |
|---|---|---|---|---|---|---|---|---|
| normal | 1 | 0.0406 | 0.0635 | 0.00338 | 1 | 0 | 0.7730 | assignment1_20261007_140644_normal_20261007_140654_f000133_detect.png assignment1_20261007_140644_normal_20261007_140654_f000133_mask.png assignment1_20261007_140644_normal_20261007_140654_f000133_raw.png |
| empty | 0 | 0.0000 | 0.0000 | 0.00000 | 0 | 0 |  | assignment1_20261007_140644_empty_20261007_140704_f000409_detect.png assignment1_20261007_140644_empty_20261007_140704_f000409_mask.png assignment1_20261007_140644_empty_20261007_140704_f000409_raw.png |
| occluded | 1 | 0.0517 | 0.1314 | 0.00161 | 1 | 0 | 0.7500 | assignment1_20261007_140644_occluded_20261007_140721_f000910_detect.png assignment1_20261007_140644_occluded_20261007_140721_f000910_mask.png assignment1_20261007_140644_occluded_20261007_140721_f000910_raw.png |

카메라 기록: camera.txt
