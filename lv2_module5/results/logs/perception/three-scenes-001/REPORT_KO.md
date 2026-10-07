# 같은 설정의 세 장면 검증

## 실험 환경과 목적

Pi D435에서 저장한 640×480 컬러 사진 3장을 PC OpenCV로 처리했다. 정상·대상 없음·부분 가림에서 검출 여부, 중심, 면적 변화와 산출물을 확인한다. 발제 문제 1의 세 장면 검증에 해당한다.

## 수행 방법

촬영과 detect_image.py 명령을 터미널에 직접 입력했다. 정상 사진으로 선택한 HSV [92,80,26]~[120,255,255], 타원 5×5 open/close, 최소 면적 400 이상 최대 후보를 세 장면에 동일하게 적용했다. [설정 사본](perception-used.yaml)과 [수치 요약](summary.json)에 설정·입력·결과를 기록했다.

## 결과

| 장면 | 검출 | ex | ey | 면적비 |
|---|---|---|---|---|
| 정상 | 있음 | +0.049376 | +0.374081 | 0.013389 |
| 대상 없음 | 없음 | +0.000000 | +0.000000 | 0.000000 |
| 가림 | 있음 | +0.052657 | +0.431839 | 0.008151 |

| 장면 | 원본 | 마스크 | 검출 |
|---|---|---|---|
| 정상 | [원본](../../../images/perception/normal/color_original.png) | [마스크](../../../images/perception/normal/color_mask.png) | [검출](../../../images/perception/normal/color_detection.png) |
| 대상 없음 | [원본](../../../images/perception/absent/color_original.png) | [마스크](../../../images/perception/absent/color_mask.png) | [검출](../../../images/perception/absent/color_detection.png) |
| 가림 | [원본](../../../images/perception/occluded/color_original.png) | [마스크](../../../images/perception/occluded/color_mask.png) | [검출](../../../images/perception/occluded/color_detection.png) |

## 해석 및 한계

정상·가림 사진의 윤곽과 중심은 보이는 파란 영역에 대응한다. 대상 없음 사진은 검은 마스크, found=false, x=y=z=0이다. 세 결과의 HSV·커널·최소 면적이 동일하며 저장 원본이 각 촬영 사진과 바이트 단위로 일치한다.

가림에서 면적비는 약 1.34%에서 0.82%로 감소했고 중심은 아래로 이동했다. 전체 물체의 숨겨진 중심을 추정한 결과가 아니라 보이는 영역의 중심이다. 물리적 가림 비율은 측정하지 않았다.

주요 배치는 유지했으나 조명·블라인드 그림자 차이가 있어 완전히 통제된 비교는 아니다. 각 장면 한 장의 탐색 검증이며 정상 사진은 튜닝에도 사용했다. 전체 검출률·독립 대상 30/없음 10 평가·ROS /target 동작 성공을 뜻하지 않는다.

## 진행 상태

기존 실습 7단계 중 4단계 완료, 약 57%다. 남은 단계는 ROS /target 구현·검증, 독립 평가, 팀 제출 증빙이다.
