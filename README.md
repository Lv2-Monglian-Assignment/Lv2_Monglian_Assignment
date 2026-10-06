Lv2_몽글리안_과제

# Lv2_몽글리안_Assignment
physicalAI_lv2_project1 제출용 레퍼지토리입니다.

- 팀명: 몽글리안
- 팀장: 온창범 ([Onbeom](https://github.com/Onbeom))
- 프로젝트: 프로젝트 1 — 비전 기반 객체 추적 시스템

## 팀원

|직급|이름|GitHub ID|담당 역할|
|---|---|---|---|
|팀장|온창범|[Onbeom](https://github.com/Onbeom)|테크리드 + 검증|
|팀원|권형중|[JuneKunst](https://github.com/JuneKunst)|통합|
|팀원|최성진|[Choi-sungjin](https://github.com/Choi-sungjin)|인지|
|팀원|천경호|[pizzaafterhangover](https://github.com/pizzaafterhangover)|제어|

## 프로젝트 폴더

|구분|경로|설명|
|---|---|---|
|프로젝트 메인|[lv2_module5/](lv2_module5/)|과제 수행 소스코드, 실행 가이드, 보고서 및 결과물|
|실행 가이드|[lv2_module5/README.md](lv2_module5/README.md)|환경·설치·빌드·실행·중지·재현 방법|
|보고서|[lv2_module5/report.md](lv2_module5/report.md)|문제 1~5 구현·검증 결과|
|팀 협업|[lv2_module5/team.md](lv2_module5/team.md)|역할·Issue·PR·리뷰 기록|
|발표|[lv2_module5/presentation.md](lv2_module5/presentation.md)|5분 시연 자료|

## 인지 구현

인지 코드는 `lv2_module5/ros2_ws/src/target_detector/`, 설정은 `lv2_module5/config/hsv.yaml`, 촬영·평가 도구는 `lv2_module5/scripts/`에 배치했다.

- [인지 빌드·실행·폴더 안내](lv2_module5/docs/perception/README.md)
- [인지 수행 계획](vision_todo/vision_todo.md)
- [인지 보고서](lv2_module5/docs/perception/report.md)
- [분류 대응표](lv2_module5/docs/perception/FILE_MAP.json)

ROS 2 Lyrical 환경에서 `lv2_module5/ros2_ws`로 이동해 `colcon build --symlink-install --packages-select target_detector tracker_bringup`으로 두 패키지를 빌드하고 `source install/setup.bash`를 실행한다. 설치 의존성과 실행 명령은 인지 안내에 있다. 빌드 산출물은 Git에 포함하지 않는다. 이미지·마스크는 `results/images/perception/`, 설정·원본 로그는 `results/logs/perception/`에 구분한다. 독립 40프레임 평가와 실제 PR·리뷰는 미완료다.
