# 이슈 #34 고정 설정 40장 평가 (시험 데이터)

문제 1(색 기반 객체 인식)의 고정 설정 평가 자료다. 목표 있음 30장과 목표 없음 10장을 같은 설정으로 검출한 결과를 담는다. 이 폴더와 [목표 있음 30장 판정 폴더](../issue34-present-color-review-001/REPORT_KO.md)를 함께 본다.

## 실험 환경

- 카메라: Raspberry Pi + RealSense D435 컬러 640×480 rgb8 30 fps. 모터는 사용하지 않았다.
- 설정: HSV [102,120,40]~[110,255,255], 최소 면적 100 px², 커널 5, 후보 선택 priority. 설정 파일 SHA-256은 `d69f5526e24856b83fb26e757e3f6c8f9b13e5be371f37dacab5e6f18f4f1ea1`이다([source/hsv.yaml](source/hsv.yaml)).
- 검출 코드: 커밋 `76e77bd`(docs/perception-issue34, main에 포함)의 `detection.py`(SHA-256 `5300ef23…`).
  - 이후 main에서 이 파일은 주석 2줄만 바뀌었다.
  - `hsv.yaml`도 주석 1줄만 바뀌었다.
  - ROS 노드 `target_detector.py`는 이후 바뀌었지만 이 평가에서는 실행하지 않았다.
- 컬러 사진만 사용했다. 깊이·CameraInfo·관절 자료는 없다.

## 실험 목적

같은 고정 설정에서 목표가 있을 때 올바른 목표를 고르는지(검출률)와 목표가 없을 때 엉뚱한 것을 고르지 않는지(오검출률)를 함께 계산한다.

## 계획한 방법

1. 목표 있음 30장: 사람 판정이 끝난 `issue34-present-color-review-001` 결과를 쓴다. 쓰기 전에 원본 사진 해시·순서·설정 SHA를 대조한다.
2. 목표 없음 10장: 파란 원통을 치운 뒤 새로 촬영한다. 사진마다 파란 물체가 없는지 눈으로 확인한 장만 '없음' 정답으로 쓴다.
3. 같은 코드로 10장을 검출하고 TP/FN/FP/TN을 계산한다.

## 실제 수행 방법

- 첫 번째 목표 없음 촬영(capture-008)은 원통을 치우기 전에 찍혀 10장 모두 원통이 보였다. 그래서 정답으로 쓰지 않았고, 이 저장소에도 올리지 않았다.
- 원통을 치운 뒤 미리보기 1장으로 원통이 없는 것을 확인했다. 이어서 0.5 s 간격으로 10장을 새로 촬영했다(capture-009, 수신 140장 중 10장 저장).
- 10장을 하나씩 눈으로 확인했다. 상자·원판·배경만 보였다. 배경의 연한 파란 세로선(창틀)은 목표가 아니다.
  - 사진별 확인 내용은 [absent-visual-review.json](absent-visual-review.json)에 있다.
- 평가 코드는 다음 조건이 모두 맞아야만 계산한다.
  - 목표 있음 30장, 목표 없음 10장.
  - 확인 항목의 해시가 사진 해시와 일치.
  - 사진 이름·해시 중복 없음.
  - 판정 누락 없음.
- 확인 시험: 해시 불일치·9장 입력·이름 중복을 각각 넣었고, 세 경우 모두 해당 오류로 중단됐다.

## 결과

| 지표 | 값 |
|---|---|
| 목표 있음 30장 | TP 30, FN 0, 다른 물체 선택 0 |
| 목표 없음 10장 | FP 0, TN 10(10장 모두 마스크 픽셀 0) |
| 목표 있음 검출률 | 100% (30/30) |
| 목표 없음 오검출률 | 0% (0/10) |
| 정확도 | 100% (40/40) |
| 목표 없음 RGB·BGR 입력 결과 일치 | 10/10 |

TP·FN·FP·TN의 정의는 [evaluation.json](evaluation.json)의 `definitions`에 있다.

## 해석 및 한계

- 표본은 한 장소·한 조명·같은 배경이다. 목표 없음 10장은 거의 같은 장면이라, 다른 배경에서의 오검출률을 대표하지 않는다.
- 깊이·CameraInfo가 없는 컬러 사진이다. 실제 크기 판정(2~30 cm²)과 번호 유지(0.5 s)는 검증하지 않았다(`size_check_verified`·`identity_reselection_verified` = false).
- 목표 없음 정답은 현장 확인과 사진 확인에 근거했다. 별도 사람 판정 화면은 거치지 않았다.
- 현재 main 코드로 다시 돌린 결과가 아니라, 위 커밋의 코드로 계산한 결과다.

## 폴더 내용

| 경로 | 내용 |
|---|---|
| [evaluation.json](evaluation.json) | 40장 집계, 지표 정의, 입력 파일 해시 |
| [frames.csv](frames.csv) | 사진별 판정(해시, 정답, 검출 여부, 후보 수, 오차) |
| `absent-001`~`absent-010/` | 목표 없음 사진별 원본·마스크·검출 이미지와 result.json |
| [absent-visual-review.json](absent-visual-review.json) | 목표 없음 10장 시각 확인 기록 |
| [source/hsv.yaml](source/hsv.yaml) | 평가에 쓴 설정 사본 |

목표 있음 30장의 사진·사람 판정 결과는 `../issue34-present-color-review-001/`에 있다.

## 올리지 않은 것

| 항목 | 이유 |
|---|---|
| `source/*.py`(평가 당시 코드 사본) | 코드는 저장소 본문에 이미 있다(커밋 `76e77bd`). 해시는 아래 표에 남긴다 |
| 첫 번째 평가 결과(-001) | 평가 코드 입력 검증을 보강하기 전 실행이다. 숫자는 이 폴더와 같다 |
| capture-008 | 원통을 치우기 전 촬영이라 정답으로 쓸 수 없다 |
| ROS bag(156~188 MB) | GitHub 파일 크기 제한(100 MB)을 넘는다 |
| 평가 스크립트(`scripts/evaluate_issue34_color_frames.py`, `scripts/evaluate_issue34_forty_frames.py`) | 시험 데이터만 올린다. 그래서 목표 있음 폴더 보고서의 평가 코드 링크는 이 저장소에서 열리지 않는다 |

평가 당시 코드 사본의 SHA-256:

| 파일 | SHA-256 |
|---|---|
| detection.py | `5300ef23cb560c9c333e157a240830166abd6dfdb57c628271a7a988114ac839` |
| target_detector.py | `c3a4cbc7838d1bf03652888824fcc8dadb3b1821ab5a8f229e437c1fc6fa9e41` |
| object_tracker.py | `dcc86ea668bfbf99c7a62faa09e921cccc9b1de81f0fa05dae02ba0bfaf3f485` |
| evaluation_source.py(40장 평가 코드) | `031a8da373dcf3d221521baf140ee19028b62f8001158868d5d7f18e74c64ed7` |
