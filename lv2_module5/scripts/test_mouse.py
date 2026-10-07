# 문법 복구본: 원래의 ROI 중앙값·핀홀 수식·화면 표시 흐름을 유지한다.
# ROI 대표 깊이를 커서 광선에 대입하는 근사이며, 물체 경계에서는 다른 표면의 값이 섞일 수 있다.
# 단순 핀홀 역투영은 왜곡 보정된 픽셀과 그 영상에 맞는 내부 파라미터를 전제로 한다.
# [추가] 's' 키를 누르면 그 순간의 컬러 원본(PNG)과 정렬된 깊이 원본(NPY)을 samples 폴더에 저장한다.
import os  # [추가] 저장 폴더 생성 및 파일 존재 여부 확인용
import cv2  # 영상 처리 라이브러리인 OpenCV를 가져옵니다. (화면 출력 및 그래픽 드로잉용)
import numpy as np  # 수치 연산 라이브러리인 NumPy를 가져옵니다. (배열 처리 및 중앙값 필터링용)
import pyrealsense2 as rs  # Intel RealSense 카메라 제어를 위한 공식 SDK를 가져옵니다.

# 전역 변수: 마우스의 현재 실시간 가로(u) 및 세로(v) 픽셀 좌표를 저장 (초기값 -1은 화면 밖을 의미)
mouse_u, mouse_v = -1, -1


def mouse_move_callback(event, x, y, flags, param):
    """마우스가 움직일 때마다 현재 픽셀 좌표를 실시간으로 업데이트하는 콜백 함수"""
    global mouse_u, mouse_v  # 함수 내부에서 전역 변수인 mouse_u와 mouse_v를 수정할 수 있도록 지정합니다.
    if event == cv2.EVENT_MOUSEMOVE:  # 발생한 마우스 이벤트가 '마우스 포인터 이동'인지 확인합니다.
        mouse_u, mouse_v = x, y  # 움직인 위치의 현재 x, y 픽셀 좌표를 전역 변수에 실시간 저장합니다.


def main():
    global mouse_u, mouse_v  # 메인 함수 내에서도 전역 변수인 마우스 좌표를 사용하기 위해 전역 선언합니다.

    # 1. 리얼센스 설정 및 스트림 선언
    pipeline = rs.pipeline()  # 카메라 데이터 흐름 및 하드웨어 통신을 총괄하는 파이프라인 객체를 생성합니다.
    config = rs.config()  # 카메라 해상도, 프레임 레이트 등의 설정을 지정하기 위한 구성 객체를 생성합니다.

    width, height = 640, 480  # 입력받을 영상의 가로 해상도를 640, 세로 해상도를 480픽셀로 정의합니다.
    config.enable_stream(
        rs.stream.color, width, height, rs.format.bgr8, 30
    )  # 컬러 스트림(해상도 640x480, BGR 8비트 포맷, 30fps)을 활성화합니다.
    config.enable_stream(
        rs.stream.depth, width, height, rs.format.z16, 30
    )  # 깊이 스트림(해상도 640x480, 16비트 정수 깊이 값, 30fps)을 활성화합니다.

    # 2. 파이프라인 시작
    profile = (
        pipeline.start(config)
    )  # 앞서 설정한 config 구성을 바탕으로 파이프라인을 가동하여 스트리밍을 시작합니다.

    # 3. [공간 정렬] Depth를 Color 영상 격자에 정렬
    align_to = (
        rs.stream.color
    )  # 시점 정렬의 기준 축을 컬러 카메라(Color Stream) 중심 좌표계로 지정합니다.
    align = rs.align(
        align_to
    )  # 보정 정보를 이용해 깊이를 컬러 영상 격자에 재투영하는 정렬(Align) 객체를 생성합니다.

    # 4. [단위/스케일] 센서 고유 depth_scale 조회
    depth_sensor = (
        profile.get_device().first_depth_sensor()
    )  # 현재 연결된 리얼센스 장치로부터 물리적인 깊이 센서 객체를 구합니다.
    depth_scale = (
        depth_sensor.get_depth_scale()
    )  # 픽셀 정수 값을 실제 미터(m) 단위로 변환해 주는 센서 고유 스케일 계수를 조회합니다.

    # 5. [내부 파라미터] 역투영 관계식용 Intrinsics 조회
    color_stream = profile.get_stream(
        rs.stream.color
    )  # 파이프라인 profile 정보에서 컬러 스트림의 세부 하드웨어 정보를 추출합니다.
    intrinsics = (
        color_stream.as_video_stream_profile().get_intrinsics()
    )  # 초점거리(fx, fy)와 주점(ppx, ppy) 등 3D 투영 계산에 필요한 카메라 내부 매개변수를 구합니다.

    # [추가] 샘플 저장 준비
    save_dir = os.path.join(
        os.path.dirname(os.path.abspath(__file__)), "samples"
    )  # 이 스크립트가 있는 폴더 아래 samples 폴더에 저장합니다.
    os.makedirs(save_dir, exist_ok=True)  # 폴더가 없으면 만들고, 이미 있으면 그대로 사용합니다.
    save_count = 0  # 저장 번호. 아래에서 기존 파일과 겹치지 않는 번호를 찾아 사용합니다.

    # 2개의 개별 윈도우 창 생성
    cv2.namedWindow(
        "1. Color Image"
    )  # 컬러 영상을 표시할 "1. Color Image"라는 이름의 윈도우 창을 생성합니다.
    cv2.namedWindow(
        "2. Depth Image"
    )  # 깊이 영상을 표시할 "2. Depth Image"라는 이름의 윈도우 창을 생성합니다.

    # 컬러 이미지 창에 마우스 이동 콜백 등록 (마우스를 올리기만 해도 작동)
    cv2.setMouseCallback(
        "1. Color Image", mouse_move_callback
    )  # 컬러 창 위에서 마우스가 움직일 때마다 콜백 함수가 실행되도록 연결합니다.

    print(
        "💡 [사용법] '1. Color Image' 창 위에 마우스를 올려두면 실시간 3D 좌표가 화면에 표시됩니다."
    )  # 콘솔에 조작법 안내 문구를 출력합니다.
    print(
        "💡 [저장] 's'를 누르면 컬러 원본과 깊이 원본을 samples 폴더에 저장합니다."
    )  # [추가] 저장 방법 안내 문구를 출력합니다.
    print(
        "💡 [종료] 아무 창이나 선택한 상태에서 키보드 'q'를 누르세요."
    )  # 콘솔에 종료 방법 안내 문구를 출력합니다.
    print(
        f"[정보] depth_scale = {depth_scale} (깊이 원본 값 x depth_scale = 미터)"
    )  # [추가] 저장된 깊이 값을 미터로 바꿀 때 필요한 계수를 출력합니다. D435 기본값은 0.001(=mm 단위)입니다.

    try:  # 예외 상황이 발생하더라도 하드웨어 자원을 안전하게 해제하기 위해 try 블록을 시작합니다.
        while (
            True
        ):  # 사용자가 프로그램을 종료하기 전까지 실시간 영상 처리를 무한히 반복합니다.
            # 6. 실시간 프레임 수신 및 공간 정렬
            frames = (
                pipeline.wait_for_frames()
            )  # 카메라 센서로부터 컬러 및 깊이 프레임 세트가 완벽히 도착할 때까지 대기 후 수신합니다.
            aligned_frames = align.process(
                frames
            )  # 컬러와 깊이 이미지의 픽셀 좌표가 1:1로 매칭되도록 시점 정렬 가공을 처리합니다.

            color_frame = (
                aligned_frames.get_color_frame()
            )  # 정렬 처리된 프레임 묶음에서 컬러 영상 프레임을 추출합니다.
            depth_frame = (
                aligned_frames.get_depth_frame()
            )  # 정렬 처리된 프레임 묶음에서 깊이 영상 프레임을 추출합니다.

            if (
                not color_frame or not depth_frame
            ):  # 일시적인 하드웨어 오류 등으로 프레임이 정상 수신되지 않았다면,
                continue  # 아래 연산 과정을 건너뛰고 다음 프레임을 받기 위해 루프의 처음으로 돌아갑니다.

            # NumPy 이미지 데이터 배열화
            color_image = np.asanyarray(
                color_frame.get_data()
            )  # 리얼센스 컬러 데이터를 연산하기 편리한 NumPy 행렬(BGR 이미지) 구조로 변환합니다.
            depth_image = np.asanyarray(
                depth_frame.get_data()
            )  # 리얼센스 깊이 데이터를 연산하기 편리한 NumPy 행렬(Z16 깊이 값) 구조로 변환합니다.

            # [추가] 아래에서 color_image 위에 십자선과 글자를 직접 그리므로, 그리기 전 원본을 따로 복사해 둡니다.
            raw_color = color_image.copy()  # 저장용 컬러 원본 (표시용 그림이 섞이지 않은 상태)
            raw_depth = depth_image.copy()  # 저장용 깊이 원본 (컬러에 정렬된 Z16 정수 값)

            # 시각화를 위해 원시 깊이(Z16) 영상에 컬러 맵(JET 효과)을 적용 (작은 표시값은 파랑 계열, 큰 표시값은 빨강 계열)
            depth_colormap = cv2.applyColorMap(  # 사람이 입체적으로 식별할 수 있도록 원시 픽셀 값에 컬러 레이아웃을 입힙니다.
                cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET
                # convertScaleAbs로 데이터 범위를 8비트(0~255)로 밝기 압축한 뒤, JET 컬러맵(무지개 색상)을 매핑합니다.
            )

            # 마우스 커서가 컬러 창 내부 유효 영역에 있을 때 기하 계산 처리
            if (
                0 <= mouse_u < width and 0 <= mouse_v < height
            ):  # 현재 마우스 좌표가 실제 이미지 가로/세로 해상도 범위 안에 존재할 때만 실행합니다.
                u, v = (
                    mouse_u,
                    mouse_v,
                )  # 연산 도중 마우스 좌표가 변경되어 튀는 현상을 막기 위해 현재 좌표를 지역 변수에 고정 복사합니다.

                # [필터링 및 중앙값 채택] 강인성을 위한 주변 10x10 ROI 분석
                roi_size = 10  # 단일 픽셀 노이즈 오차를 방지하기 위해 분석할 주변 영역의 사각형 크기를 10픽셀로 정의합니다.
                u_start = max(
                    0, u - roi_size // 2
                )  # ROI 사각형의 왼쪽 가로 시작점이 영상 경계(0) 밖으로 나가지 않도록 보정합니다.
                v_start = max(
                    0, v - roi_size // 2
                )  # ROI 사각형의 위쪽 세로 시작점이 영상 경계(0) 밖으로 나가지 않도록 보정합니다.
                u_end = min(
                    width, u + roi_size // 2
                )  # ROI 사각형의 오른쪽 가로 끝점이 최대 해상도 폭을 벗어나지 않도록 보정합니다.
                v_end = min(
                    height, v + roi_size // 2
                )  # ROI 사각형의 아래쪽 세로 끝점이 최대 해상도 높이를 벗어나지 않도록 보정합니다.

                roi_depths = depth_image[
                    v_start:v_end, u_start:u_end
                ]  # 깊이 이미지 행렬 슬라이싱을 통해 마우스 영역 주변 10x10 크기의 깊이 값들만 잘라냅니다.

                # 유효 작업 거리 마스킹 (0.2m ~ 3.0m 범위)
                min_w = int(
                    0.2 / depth_scale
                )  # 최소 작업 거리인 0.2m를 센서 고유 정수 값(단위 스케일 반영 정수)으로 변환합니다.
                max_w = int(
                    3.0 / depth_scale
                )  # 최대 작업 거리인 3.0m를 센서 고유 정수 값(단위 스케일 반영 정수)으로 변환합니다.
                valid_mask = (
                    (roi_depths > 0)
                    & (roi_depths >= min_w)
                    & (roi_depths <= max_w)
                )  # 0(측정 불능)이 아니면서 20cm~3m 유효 범위 내에 정상 부합하는 픽셀만 True로 체크하는 마스크를 만듭니다.

                total_samples = (
                    roi_depths.size
                )  # 잘라낸 사각형 영역(ROI) 내부에 들어있는 전체 샘플 픽셀의 총 개수를 구합니다.
                valid_samples = np.sum(
                    valid_mask
                )  # 전체 샘플 중 노이즈나 측정 실패를 제외하고 정상 유효 판정을 받은 픽셀의 개수를 구합니다.

                # 유효 샘플 비율이 30% 이상일 때만 화면에 좌표 드로잉 수행 (노이즈 거부 조건)
                if valid_samples >= (
                    total_samples * 0.3
                ):  # 마우스 주변 픽셀 중 정상 측정 데이터가 최소 30% 이상일 때만 연산 신뢰성이 있다고 간주합니다.
                    raw_median_depth = np.median(
                        roi_depths[valid_mask]
                    )  # 유효 픽셀 값들 중 일부 이상값의 영향을 줄이는 중앙값(Median)을 대표 원시 값으로 채택합니다.

                    # 단위 복원: 미터(m) 단위 깊이 값
                    Z = (
                        raw_median_depth * depth_scale
                    )  # 센서 전용 정수 값에 변환 계수를 곱해 깊이의 광학 축 방향 성분 Z(m)를 복원합니다.

                    # 역투영 관계식 연산 (2D -> 3D 물리 거리 산출)
                    X = (
                        (u - intrinsics.ppx) * Z / intrinsics.fx
                    )  # 핀홀 카메라 모델 공식을 적용해 카메라 가로 렌즈 중심 기준 좌우 실제 물리 수평 거리 X(m)를 연산합니다.
                    Y = (
                        (v - intrinsics.ppy) * Z / intrinsics.fy
                    )  # 핀홀 카메라 모델 공식을 적용해 카메라 세로 렌즈 중심 기준 상하 실제 물리 수직 거리 Y(m)를 연산합니다.

                    # 실제 유클리드 직선거리 계산 (슬라이드 10 내용)
                    Distance = np.sqrt(
                        X**2 + Y**2 + Z**2
                    )  # 피타고라스 정리를 활용해 카메라 원점(0,0,0)에서 최종 3D 좌표 표면까지의 직선 도해 거리를 구합니다.

                    # 화면에 그려줄 텍스트 가공
                    text_u_v = f"Pixel: ({u}, {v})"  # 2D 이미지 평면 상의 픽셀 위치를 문자열 포맷으로 가공합니다.
                    text_x = f"X (Horiz): {X:+.3f} m"  # 소수점 3자리 미터 단위의 수평 좌우 물리 거리 문자열을 가공합니다. (+/- 부호 포함)
                    text_y = f"Y (Vert) : {Y:+.3f} m"  # 소수점 3자리 미터 단위의 수직 상하 물리 거리 문자열을 가공합니다. (+/- 부호 포함)
                    text_z = f"Z (Depth): {Z:.3f} m"  # 소수점 3자리 미터 단위의 카메라 정면 깊이 거리 문자열을 가공합니다.
                    text_d = f"Dist     : {Distance:.3f} m"  # 소수점 3자리 미터 단위의 최종 3차원 물리 공간 직선거리 문자열을 가공합니다.

                    # 시각적 가독성을 위해 마우스 위치에 조준선(크로스헤어) 그리기
                    cv2.drawMarker(
                        color_image,
                        (u, v),
                        (0, 255, 0),
                        cv2.MARKER_CROSS,
                        20,
                        2,
                    )  # 컬러 화면 마우스 위치에 초록색 십자가 마커(크기 20, 두께 2)를 그려 시인성을 높입니다.
                    cv2.drawMarker(
                        depth_colormap,
                        (u, v),
                        (255, 255, 255),
                        cv2.MARKER_CROSS,
                        20,
                        2,
                    )  # 깊이 화면 마우스 위치에도 동일한 하얀색 십자가 마커를 추가합니다.

                    # 텍스트가 화면 경계를 벗어나지 않도록 좌표 계산 후 화면 우측/하단 배정
                    text_x_pos = (
                        u + 15 if u < width - 200 else u - 180
                    )  # 마우스가 화면 오른쪽에 너무 가까우면 문자 팝업 박스를 왼쪽으로 반전시켜 출력 위치를 조정합니다.
                    text_y_pos = (
                        v + 20 if v < height - 120 else v - 100
                    )  # 마우스가 화면 아래쪽에 너무 가까우면 문자 팝업 박스를 위쪽으로 반전시켜 출력 위치를 조정합니다.

                    # 불투명 텍스트 박스 배경 그리기 (가독성 확보)
                    cv2.rectangle(
                        color_image,
                        (text_x_pos - 5, text_y_pos - 20),
                        (text_x_pos + 185, text_y_pos + 85),
                        (0, 0, 0),
                        -1,
                    )  # 정보가 표기될 영역에 검은색 배경 사각형 박스(두께 -1은 내부 가득 채움)를 먼저 그려 글씨 가독성을 보장합니다.

                    # 컬러 화면 위에 문자열 드로잉 (OpenCV 폰트 적용)
                    font = cv2.FONT_HERSHEY_SIMPLEX  # OpenCV 기본 글꼴이다.
                    scale, thickness = 0.45, 1  # 글자 크기 배율과 선 두께다.
                    cv2.putText(
                        color_image, text_u_v, (text_x_pos, text_y_pos),
                        font, scale, (255, 255, 0), thickness,
                    )  # 픽셀 좌표를 표시한다.
                    cv2.putText(
                        color_image, text_x, (text_x_pos, text_y_pos + 20),
                        font, scale, (0, 255, 255), thickness,
                    )  # X 좌표를 표시한다.
                    cv2.putText(
                        color_image, text_y, (text_x_pos, text_y_pos + 40),
                        font, scale, (0, 255, 255), thickness,
                    )  # Y 좌표를 표시한다.
                    cv2.putText(
                        color_image, text_z, (text_x_pos, text_y_pos + 60),
                        font, scale, (100, 255, 100), thickness,
                    )  # 광학 축 방향 깊이 Z를 표시한다.
                    cv2.putText(
                        color_image, text_d, (text_x_pos, text_y_pos + 80),
                        font, scale, (255, 150, 150), thickness,
                    )  # 계산한 직선거리를 표시한다.
                else:  # 유효 깊이 샘플이 부족하면 좌표를 계산하지 않는다.
                    cv2.putText(
                        color_image, "Invalid / Out of Range",
                        (mouse_u + 15, mouse_v), cv2.FONT_HERSHEY_SIMPLEX,
                        0.5, (0, 0, 255), 1,
                    )

            # while 내부에서 매 프레임 두 창을 갱신한다.
            cv2.imshow("1. Color Image", color_image)
            cv2.imshow("2. Depth Image", depth_colormap)

            # waitKey는 창 이벤트를 처리한다. 눌린 키를 한 번만 읽어 q(종료)와 s(저장)에 함께 사용한다.
            key = cv2.waitKey(1) & 0xFF  # [수정] 키 값을 변수에 저장해 두 번 비교할 수 있게 합니다.
            if key == ord("q"):
                break
            if key == ord("s"):  # [추가] s를 누른 순간의 원본을 저장합니다.
                save_count += 1  # 다음 저장 번호로 넘어갑니다.
                while os.path.exists(
                    os.path.join(save_dir, f"color_{save_count:02d}.png")
                ):  # 프로그램을 다시 켰을 때 이전 샘플을 덮어쓰지 않도록 비어 있는 번호까지 건너뜁니다.
                    save_count += 1
                cv2.imwrite(
                    os.path.join(save_dir, f"color_{save_count:02d}.png"), raw_color
                )  # 컬러 원본을 무손실 PNG로 저장합니다.
                np.save(
                    os.path.join(save_dir, f"depth_{save_count:02d}.npy"), raw_depth
                )  # 깊이 원본(Z16 정수 행렬)을 그대로 NPY로 저장합니다.
                print(f"[저장] {save_dir} 에 color_{save_count:02d}.png / depth_{save_count:02d}.npy")
    finally:  # try와 같은 들여쓰기 수준에 배치한다.
        pipeline.stop()  # 이 try 블록을 빠져나올 때 스트림을 정리한다.
        cv2.destroyAllWindows()  # OpenCV 창을 닫는다.
        print("[시스템] 리얼센스 카메라 스트림 및 모든 창이 정상 종료되었습니다.")


# 양쪽에 밑줄 두 개가 들어간 __name__과 문자열 __main__을 사용한다.
if __name__ == "__main__":
    main()
