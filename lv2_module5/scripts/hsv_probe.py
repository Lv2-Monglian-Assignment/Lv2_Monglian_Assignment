# HSV 범위 측정 도구 — mouse_test.py(리얼센스 학습 예제)를 바탕으로 만든 버전.
# mouse_test.py와 같은 방식(정렬 Depth, 10x10 ROI 중앙값, 0.2~3.0 m 유효 범위)에
#   (1) 커서 주변의 RGB·HSV 중앙값·최소·최대 표시
#   (2) 현재 HSV 범위로 만든 마스크 창(어디까지 파란색으로 잡히는지 확인)
#   (3) 's' 키로 측정값을 CSV에 저장, 종료 시 저장한 값들의 H·S·V 최소~최대 출력
# 을 더했다. ROS 없이 pyrealsense2로 카메라를 직접 연다(모니터가 있는 PC에서 실행).
#
# 실행: python3 hsv_probe.py --lower 100 120 50 --upper 130 255 255 --out hsv_samples.csv
# 조작: 커서를 물체에 올림 -> 's' 저장, 'q' 종료.  realsense2_camera 노드가 켜져 있으면 먼저 끈다(카메라 동시 사용 불가).
import argparse
import csv
import os
from datetime import datetime

import cv2
import numpy as np
import pyrealsense2 as rs

ap = argparse.ArgumentParser()
ap.add_argument('--lower', type=int, nargs=3, default=[100, 120, 50], help='H S V 하한 (OpenCV: H 0~179)')
ap.add_argument('--upper', type=int, nargs=3, default=[130, 255, 255], help='H S V 상한')
ap.add_argument('--out', default='hsv_samples.csv')
ap.add_argument('--roi', type=int, default=10, help='커서 주변 측정 영역 크기 [px]')
args = ap.parse_args()

mouse_u, mouse_v = -1, -1


def on_mouse(event, x, y, flags, param):
    global mouse_u, mouse_v
    if event == cv2.EVENT_MOUSEMOVE:
        mouse_u, mouse_v = x, y


def main():
    width, height = 640, 480
    pipeline, config = rs.pipeline(), rs.config()
    config.enable_stream(rs.stream.color, width, height, rs.format.bgr8, 30)
    config.enable_stream(rs.stream.depth, width, height, rs.format.z16, 30)
    profile = pipeline.start(config)
    align = rs.align(rs.stream.color)
    depth_scale = profile.get_device().first_depth_sensor().get_depth_scale()
    intr = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
    print(f'fx={intr.fx:.1f} fy={intr.fy:.1f} ppx={intr.ppx:.1f} ppy={intr.ppy:.1f}  depth_scale={depth_scale}')

    new_file = not os.path.exists(args.out)
    f = open(args.out, 'a', newline='')
    w = csv.writer(f)
    if new_file:
        w.writerow(['time', 'u', 'v', 'z_m', 'h_med', 's_med', 'v_med', 'h_min', 's_min', 'v_min',
                    'h_max', 's_max', 'v_max', 'r', 'g', 'b', 'in_mask_ratio'])
    saved = []
    lower, upper = np.array(args.lower, np.uint8), np.array(args.upper, np.uint8)
    cv2.namedWindow('1. Color')
    cv2.setMouseCallback('1. Color', on_mouse)
    print("커서를 물체에 올리고 's' = 저장, 'q' = 종료")
    try:
        while True:
            frames = align.process(pipeline.wait_for_frames())
            cf, df = frames.get_color_frame(), frames.get_depth_frame()
            if not cf or not df:
                continue
            color = np.asanyarray(cf.get_data())          # BGR
            depth = np.asanyarray(df.get_data())
            hsv = cv2.cvtColor(color, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv, lower, upper)
            view = color.copy()
            row = None
            if 0 <= mouse_u < width and 0 <= mouse_v < height:
                u, v, r = mouse_u, mouse_v, args.roi // 2
                us, ue, vs, ve = max(0, u - r), min(width, u + r), max(0, v - r), min(height, v + r)
                roi_hsv = hsv[vs:ve, us:ue].reshape(-1, 3)
                roi_bgr = color[vs:ve, us:ue].reshape(-1, 3)
                med, mn, mx = np.median(roi_hsv, 0), roi_hsv.min(0), roi_hsv.max(0)
                b, g, rr = np.median(roi_bgr, 0)
                d = depth[vs:ve, us:ue]
                ok = (d > 0) & (d >= 0.2 / depth_scale) & (d <= 3.0 / depth_scale)
                z = float(np.median(d[ok]) * depth_scale) if ok.sum() >= d.size * 0.3 else float('nan')
                in_mask = float(np.count_nonzero(mask[vs:ve, us:ue])) / mask[vs:ve, us:ue].size
                row = [datetime.now().isoformat(timespec='seconds'), u, v, f'{z:.3f}',
                       *[int(x) for x in med], *[int(x) for x in mn], *[int(x) for x in mx],
                       int(rr), int(g), int(b), f'{in_mask:.2f}']
                cv2.rectangle(view, (us, vs), (ue, ve), (0, 255, 0), 1)
                lines = [f'({u},{v})  Z {z:.3f} m' if z == z else f'({u},{v})  Z invalid',
                         f'HSV med {int(med[0])},{int(med[1])},{int(med[2])}',
                         f'HSV min {int(mn[0])},{int(mn[1])},{int(mn[2])}  max {int(mx[0])},{int(mx[1])},{int(mx[2])}',
                         f'RGB {int(rr)},{int(g)},{int(b)}   in-mask {in_mask:.0%}']
                tx = u + 15 if u < width - 300 else u - 300
                ty = v + 20 if v < height - 100 else v - 90
                cv2.rectangle(view, (tx - 5, ty - 16), (tx + 290, ty + 66), (0, 0, 0), -1)
                for i, t in enumerate(lines):
                    cv2.putText(view, t, (tx, ty + i * 18), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 0), 1)
            cv2.putText(view, f'range {args.lower} ~ {args.upper}  saved {len(saved)}', (6, 16),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
            cv2.imshow('1. Color', view)
            cv2.imshow('2. Mask (current HSV range)', mask)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('s') and row is not None:
                w.writerow(row); f.flush(); saved.append(row)
                print('saved', row)
            elif key == ord('q'):
                break
    finally:
        pipeline.stop()
        cv2.destroyAllWindows()
        f.close()
        if saved:
            a = np.array([[r[4], r[5], r[6]] for r in saved], float)
            print(f'저장 {len(saved)}개  H {a[:,0].min():.0f}~{a[:,0].max():.0f}  '
                  f'S {a[:,1].min():.0f}~{a[:,1].max():.0f}  V {a[:,2].min():.0f}~{a[:,2].max():.0f} (중앙값 기준)')


if __name__ == '__main__':
    main()
