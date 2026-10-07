# assignment — 문제·도전 실습별 실행 프로그램

발제의 문제 1~5와 도전 실습 A~E마다 시험을 실행하고 결과(이미지·CSV·표·그래프)를 `results/`에 남기는 프로그램이다.
ROS 노드 자체는 `ros2_ws/src`에 있고, 이 폴더의 프로그램은 노드를 띄우고·안내하고·기록을 분석한다.
모두 Raspberry Pi에서 워크스페이스를 source한 셸에서 실행한다.

```bash
cd ~/git/Lv2_Monglian_Assignment/lv2_module5 && source ros2_ws/install/setup.bash
python3 assignment/main.py          # 통합 메뉴 (숫자 키로 아래 프로그램 실행)
```

## 통합 메뉴 (`main.py`)와 부팅 자동 실행

**입력이 없으면(IDLE) 10 s 뒤 최종 추적**(`full.launch.py`, 추적 켬)이 백그라운드로 실행된다. 화면에 상태(TRACKING·LOST)가 보인다.
다른 프로그램 키를 누르면 추적을 먼저 멈추고(카메라·시리얼 공유) 실행하며, 프로그램이 끝나면 바로 다시 추적한다. 스페이스로 대기 추적을 끄고 켠다.

| 키 | 실행 | 키 | 실행 |
|---|---|---|---|
| 1 | 문제 1 `assignment1.py` | 6 | 도전 A `assignment_A.py` |
| 2 | 문제 2 `assignment2.py` | 7 | 도전 B `assignment_B.py` |
| 3 | 문제 3 `assignment3.py` | 8 | 도전 C `assignment_C.py` |
| 4 | 문제 4 `assignment4.py` | 9 | 도전 D `assignment_D.py` |
| 5 | 문제 5 `assignment5.py` | 0 | 도전 E `assignment_E.py` |
| (없음) | 10 s 뒤 최종 추적 | 스페이스 | 대기 추적 끄기·켜기 |
| t | 대기 없이 바로 최종 추적 | p · d · f · e | 기준 자세 설정 · 방향 시험 · 정지 시험 · 환경 확인 |
| z | 화면 나가기 (tmux detach, 메뉴·추적 계속) | x | 메뉴 끝내기 (추적 정지) |

- 하위 선택(시험 종류, Kp, 회차 등)은 기본값을 보여 주고 Enter로 받는다. 실행 중인 프로그램은 Ctrl+C로 멈추고 메뉴로 돌아온다.
- 부팅 자동 실행(한 번만 설정): `scripts/test/install_autostart.sh` → 전원이 들어오면 tmux 세션 `lv2`에서 메뉴가 실행된다. PC에서 `ssh pa23@<Pi>` 접속 후 `lv2`(또는 `tmux attach -t lv2`)로 붙는다. 해제: `--remove`.
- 확인용: `LV2_MENU_ECHO=1 python3 assignment/main.py` → 실행하지 않고 명령만 보여 준다.

## 발제 항목 → 프로그램 → 결과

| 항목 | 프로그램 · 명령 | 모터 | 결과 위치 |
|---|---|---|---|
| 문제 1 세 장면(정상·없음·가림) 원본·마스크·검출, 카메라 프로파일·K·encoding·버전 | `assignment1.py` | 안 씀 | `results/assignment1/<run>/`, `results/images/assignment1_*` |
| 문제 2 모의 입력 0·+0.4·−0.4·z=0·발행 중단(+같은 stamp 재전송, 틸트), 구조도 | `assignment2.py` | 안 씀(브리지 미실행) | `results/assignment2/<run>/` |
| 문제 3 Kp 2종 × 3회, 오차·명령·실측 각도 그래프 | `assignment3.py run --pan-kp K --trial N` → `analyze` | 씀 (`--dry-run` 가능) | `results/assignment3/`, `results/plots/assignment3_*` |
| 문제 4 정상 30 s(FPS·RMSE·유효 추적 비율) | `assignment4.py normal` | 씀 | `results/assignment4/` |
| 문제 4 검출률 30장·배경 오검출 10장 (사람 대조) | `assignment4.py eval` → labels.csv 채우기 → `score` | 안 씀 | `results/assignment4/eval_*/` |
| 문제 4 2 s 가림 후 재등장 5회 (복구 성공률·시간, 정지 확인) | `assignment4.py occlusion` | 씀 | `results/assignment4/` |
| 문제 4 인지 입력 중단 / 제어 통신 중단(제어 노드·브리지) | `assignment4.py topic-stop` / `control-stop --node controller\|bridge` | 씀 | `results/assignment4/` |
| 문제 5 성공·소실 bag 10~30 s, 정보·sha256 | `assignment5.py record --name success\|lost` | 씀 | `recordings/`, `results/assignment5/` |
| 문제 5 입력 재처리 / 결과 재분석 / 다른 팀원 재현 기록 | `assignment5.py replay\|reanalyze <bag>` / `reproduce --who 이름` | 안 씀 | `results/assignment5/` |
| 도전 A 조명·거리 조건 변경 검출률·오검출·처리 속도 | `assignment_A.py capture` → `score` | 안 씀 | `results/assignment_A/` |
| 도전 B 인터페이스 재확인, SEARCHING 성공·취소·미발견, 상태 전이표 | `assignment_B.py interface\|search` | 안 씀(dry_run + 가상 물체) | `results/assignment_B/` |
| 도전 C 데드밴드 하나 변경, 각 3회 | `assignment_C.py run --param pan_deadband --value V --trial N` → `analyze` | 씀 | `results/assignment_C/` |
| 도전 D 가림 반복·실패 원인 분류, 조건 하나 변경 비교 | `assignment_D.py run [--param P --value V]` → `analyze` | 씀 | `results/assignment_D/` |
| 도전 E 고정 bag 기준·변경 설정 재처리 비교 | `assignment_E.py --bag <bag> --param P --value V` | 안 씀 | `results/assignment_E/` |

모든 회차별 지표는 `results/metrics.csv`에 (test, run_id) 기준으로 모인다. 노드 원본 기록은 `~/lv2_module5_logs/`에 남고 시험 폴더로 복사된다.

## 공용 (`common.py`)
- 지표 산식(발제 문제 4 표): 처리 FPS(처리 완료 프레임 / 실제 경과 초), 수평 RMSE(검출·TRACKING 행의 sqrt(mean(ex²)), 제외 행 수 병기), 유효 추적 비율, 복구 성공률(재등장 후 3 s 이내 TRACKING), 복구 시간(TRACKING 복귀 − 재등장, 실패는 실패로 표시).
- 재등장 시각은 인지 기록에서 후보가 없던 영상 다음 첫 후보 영상의 stamp로 정한다(사람이 본 시각과 다를 수 있음을 보고서에 적는다).
- 설정을 바꾸는 시험은 `config/` 복사본만 바꿔 결과 폴더에 함께 남긴다(원본 config는 그대로).
- 그래프는 matplotlib 없이 OpenCV로 그린다(Pi에 추가 설치 없음).

## 주의
- 실제 모터를 쓰는 시험은 시리얼 모니터·pose_tool·다른 launch를 끈 상태에서 실행한다(`/dev/ttyACM0` 점유 확인).
- bag 원본은 약 46 MB/s(Color·Depth 원본)라 20 s에 약 0.9 GB다. 기록 전에 `df -h`로 남은 공간을 본다(2026-10-06 Pi 남은 공간 2.7 GB).
- 실험·설정 보조 프로그램(방향·정지 시험, 기준 자세, 미리보기, 범위 측정, 요약)은 `scripts/test/`에 있다.
