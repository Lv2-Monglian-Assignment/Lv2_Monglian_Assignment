#!/usr/bin/env python3
"""실험 2: 장면 하나를 정해진 시간 동안 관측해, 검출 코드(find_and_measure)가 계산한 후보별 실제 면적 [cm^2]·거리·제외 사유를 기록한다.

노드 코드는 바꾸지 않고 같은 함수와 같은 설정(config/camera.yaml + config/hsv.yaml의 target_detector 값)을 쓴다.
카메라(perception.launch.py)가 떠 있는 상태에서 Raspberry Pi, lv2_module5 폴더에서 실행:
  python3 results/param_experiments/size_range/probe_sizes.py --label tilt30_0.6m --out <결과 폴더> [--seconds 3]
같은 라벨로 /target/save_snapshot도 보내 검출 노드가 원본·마스크·검출 이미지를 저장하게 한다.
출력: <결과 폴더>/candidates.csv (프레임·후보별 행, 누적), <결과 폴더>/scenes.csv (장면별 요약, 누적)
"""
import argparse
import csv
import math
import os
import statistics
import sys
import time
from dataclasses import fields
from pathlib import Path

import rclpy
import yaml
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / 'ros2_ws/src/target_detector'))
from target_detector.detection import DetectorConfig, find_and_measure, select_candidate  # noqa: E402

COLOR = '/camera/camera/color/image_raw'
DEPTH = '/camera/camera/aligned_depth_to_color/image_raw'
INFO = '/camera/camera/color/camera_info'


def load_cfg(cfg_dir):
    params = {}
    for name in ('camera.yaml', 'hsv.yaml'):
        data = yaml.safe_load(open(Path(cfg_dir) / name)) or {}
        for key in ('/**', 'target_detector'):
            params.update((data.get(key) or {}).get('ros__parameters') or {})
    names = {f.name for f in fields(DetectorConfig)}
    return DetectorConfig(**{k: tuple(v) if k in ('hsv_lower', 'hsv_upper') else v for k, v in params.items() if k in names}), params


class Probe(Node):
    def __init__(self, cfg, tol_s):
        super().__init__('size_probe')
        self.cfg, self.tol_ns = cfg, int(tol_s * 1e9)
        self.bridge = CvBridge()
        self.depths, self.intr, self.frames = [], None, []
        self.create_subscription(Image, COLOR, self.on_color, qos_profile_sensor_data)
        self.create_subscription(Image, DEPTH, self.on_depth, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, INFO, self.on_info, qos_profile_sensor_data)
        self.snap = self.create_publisher(String, '/target/save_snapshot', 10)

    def on_info(self, m):
        self.intr = (m.k[0], m.k[4], m.k[2], m.k[5])

    def on_depth(self, m):
        self.depths = (self.depths + [m])[-10:]

    def on_color(self, m):
        ns = m.header.stamp.sec * 10**9 + m.header.stamp.nanosec
        if self.intr is None or not self.depths:
            return
        d = min(self.depths, key=lambda x: abs(x.header.stamp.sec * 10**9 + x.header.stamp.nanosec - ns))
        if abs(d.header.stamp.sec * 10**9 + d.header.stamp.nanosec - ns) > self.tol_ns:
            return
        img = self.bridge.imgmsg_to_cv2(m, desired_encoding='passthrough')
        dep = self.bridge.imgmsg_to_cv2(d, desired_encoding='passthrough')
        scale = 0.001 if d.encoding in ('16UC1', 'mono16') else 1.0
        cands, rejected, _ = find_and_measure(img, m.encoding, self.cfg, dep, scale, self.intr)
        h, w = img.shape[:2]
        sel = select_candidate(cands, self.cfg, None, w, h)
        self.frames.append((ns * 1e-9, cands, rejected, sel))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--label', required=True)
    ap.add_argument('--out', required=True)
    ap.add_argument('--seconds', type=float, default=3.0)
    ap.add_argument('--config', default=str(ROOT / 'config'))
    args = ap.parse_args()
    cfg, params = load_cfg(args.config)
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    node = Probe(cfg, params.get('depth_sync_tol_s', 0.02))
    t_end = time.monotonic() + 2.0
    while time.monotonic() < t_end:            # 구독 연결 대기 (이 동안의 프레임은 버림)
        rclpy.spin_once(node, timeout_sec=0.05)
    node.frames.clear()
    node.snap.publish(String(data=args.label))
    t_end = time.monotonic() + args.seconds
    while time.monotonic() < t_end:
        rclpy.spin_once(node, timeout_sec=0.05)
    frames = node.frames
    node.destroy_node()
    rclpy.shutdown()

    f = lambda v, n=2: '' if v is None or (isinstance(v, float) and not math.isfinite(v)) else round(v, n)  # noqa: E731
    cpath = out / 'candidates.csv'
    new = not cpath.exists()
    with open(cpath, 'a', newline='') as fp:
        wr = csv.writer(fp)
        if new:
            wr.writerow(['label', 'stamp_s', 'status', 'reason', 'selected', 'area_px', 'z_m', 'area_cm2', 'cut', 'cx', 'cy'])
        for t, cands, rejected, sel in frames:
            for k, c in enumerate(cands):
                wr.writerow([args.label, f(t, 6), 'kept', '', int(k == sel), f(c.area_px, 1), f(c.z_m, 3), f(c.area_cm2),
                             int(c.cut), f(c.cx, 1), f(c.cy, 1)])
            for c, reason in rejected:
                wr.writerow([args.label, f(t, 6), 'rejected', reason, 0, f(c.area_px, 1), f(c.z_m, 3), f(c.area_cm2),
                             int(c.cut), f(c.cx, 1), f(c.cy, 1)])

    sel_c = [cands[sel] for _, cands, _, sel in frames if sel is not None]
    a = [c.area_cm2 for c in sel_c if math.isfinite(c.area_cm2)]
    z = [c.z_m for c in sel_c if math.isfinite(c.z_m)]
    rej = [(c, r) for _, _, rejected, _ in frames for c, r in rejected]
    other = [c for _, cands, _, sel in frames for k, c in enumerate(cands) if k != sel]
    row = {'label': args.label, 'date': time.strftime('%Y-%m-%d %H:%M:%S'), 'frames': len(frames),
           'detect_ratio': f(len(sel_c) / len(frames), 3) if frames else '',
           'target_z_m_mean': f(statistics.mean(z), 3) if z else '',
           'target_area_cm2_mean': f(statistics.mean(a)) if a else '', 'target_area_cm2_min': f(min(a)) if a else '',
           'target_area_cm2_max': f(max(a)) if a else '', 'target_area_px_mean': f(statistics.mean(c.area_px for c in sel_c), 0) if sel_c else '',
           'target_cut_frames': sum(c.cut for c in sel_c), 'target_no_depth_frames': len(sel_c) - len(a),
           'rejected_count': len(rej), 'rejected_reasons': '; '.join(sorted({r.split()[0] for _, r in rej})),
           'rejected_area_cm2_range': f'{min(c.area_cm2 for c, _ in rej):.2f}~{max(c.area_cm2 for c, _ in rej):.2f}' if rej else '',
           'other_kept_count': len(other),
           'obj_area_min_cm2': cfg.obj_area_min_cm2, 'obj_area_max_cm2': cfg.obj_area_max_cm2, 'detect_scale': cfg.detect_scale}
    spath = out / 'scenes.csv'
    new = not spath.exists()
    with open(spath, 'a', newline='') as fp:
        wr = csv.DictWriter(fp, fieldnames=list(row))
        if new:
            wr.writeheader()
        wr.writerow(row)
    for k, v in row.items():
        print(f'{k}: {v}')


if __name__ == '__main__':
    main()
