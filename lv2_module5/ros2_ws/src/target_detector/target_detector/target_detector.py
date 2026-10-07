#!/usr/bin/env python3
"""파란색 목표 검출 노드 (인지).

입력 (realsense2_camera)
  /camera/camera/color/image_raw                   sensor_msgs/Image rgb8      30 Hz
  /camera/camera/aligned_depth_to_color/image_raw  sensor_msgs/Image 16UC1 mm  30 Hz
  /camera/camera/color/camera_info                 sensor_msgs/CameraInfo      30 Hz
출력 (새 컬러 영상마다 1번, header = 입력 컬러 영상 header 그대로)
  /target               PointStamped  x=ex, y=ey, z=면적비 (미검출: 0, 0, 0)
  /target_depth         PointStamped  x=depth_valid(0/1), y=유효 픽셀 비율, z=z_m (무효면 0)
  /target/position_cam  PointStamped  카메라 광학 좌표 X, Y, Z [m] (무효·미검출이면 NaN)
규칙 (architecture.md 1. 인지)
  - 카메라가 멈추면 발행하지 않는다(이전 영상에 새 시각을 붙이지 않음). stamp가 증가하지 않는 영상은 버린다.
  - 정상 영상에서 미검출이면 z=0을 발행한다(침묵 아님). 이전 좌표를 다시 쓰지 않는다.
  - 깊이는 후보 판단에 쓸 수 있다: size_check(실제 면적 검증)와 object_tracker(번호 유지).
    깊이 영상이 없거나 stamp가 안 맞으면 색·면적만으로 판단한다(깊이 때문에 검출이 멈추지 않음).
  - object_tracker를 쓰면 /target은 '잡은 번호'의 물체다. 그 물체가 안 보이면 다른 물체가 보여도 z=0
    (relock_after_s가 지나면 새 물체를 고른다). 기억 위치를 /target으로 내보내지 않는다.
입력 (object_tracker용, 선택)
  /pan_tilt/joint_states  sensor_msgs/JointState  모터 각도 [rad]·속도 [rad/s] (tracker_controller가 발행)
"""
import csv
import math
import os
import time
from collections import deque
from datetime import datetime

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import CameraInfo, Image, JointState
from std_msgs.msg import String

from target_detector.detection import (DetectorConfig, find_and_measure, make_detection, select_candidate,
                                          target_depth, draw_overlay, to_hsv)
from target_detector.object_tracker import ObjectTracker, Observation, TargetLock, TrackerConfig

try:                                     # 회전축 기준 좌표 변환은 제어 패키지의 것을 그대로 쓴다(값·부호 일치)
    from tracker_controller import geometry as geo
except ImportError:                      # 제어 패키지가 없는 컴퓨터: 카메라 좌표를 그대로 사용(회전 보정 없음)
    geo = None

NAN = float('nan')


def stamp_ns(header):
    return header.stamp.sec * 1_000_000_000 + header.stamp.nanosec


class TargetDetector(Node):
    def __init__(self):
        super().__init__('target_detector')
        p = lambda name, default: self.declare_parameter(name, default).value  # noqa: E731

        # ---- 토픽 (#todo 팀 합의: architecture.md 토픽 총괄) ----
        color_topic = p('color_topic', '/camera/camera/color/image_raw')
        depth_topic = p('depth_topic', '/camera/camera/aligned_depth_to_color/image_raw')
        info_topic = p('info_topic', '/camera/camera/color/camera_info')
        target_topic = p('target_topic', '/target')                      # 재처리 시 /target_replay로 remap
        depth_out_topic = p('depth_out_topic', '/target_depth')          # 📝 architecture 초안
        position_topic = p('position_topic', '/target/position_cam')     # #todo 심화(목표 기억)용, 합의 필요
        snapshot_topic = p('snapshot_topic', '/target/save_snapshot')    # #todo 합의
        self.publish_position = p('publish_position', True)

        # ---- 검출 설정 (config/perception.yaml) ----
        self.cfg = DetectorConfig(
            hsv_lower=tuple(p('hsv_lower', [100, 120, 50])),
            hsv_upper=tuple(p('hsv_upper', [130, 255, 255])),
            morph_kernel=int(p('morph_kernel', 5)),
            min_area_px=float(p('min_area_px', 150.0)),
            selection=p('selection', 'largest'),
            lock_gate_px=float(p('lock_gate_px', 80.0)),
            depth_min_m=float(p('depth_min_m', 0.2)),
            depth_max_m=float(p('depth_max_m', 3.0)),
            depth_min_valid_ratio=float(p('depth_min_valid_ratio', 0.3)),
            depth_erode_px=int(p('depth_erode_px', 3)),
            detect_scale=float(p('detect_scale', 1.0)),
            size_check=p('size_check', False),
            obj_area_min_cm2=float(p('obj_area_min_cm2', 5.5)),
            obj_area_max_cm2=float(p('obj_area_max_cm2', 30.0)),
            similar_area_ratio=float(p('similar_area_ratio', 0.2)),
            similar_depth_m=float(p('similar_depth_m', 0.03)),
            similar_depth_ratio=float(p('similar_depth_ratio', 0.05)))
        self.lock_timeout_s = p('lock_timeout_s', 0.5)
        self.use_depth = p('use_depth', True)
        self.depth_sync_tol_s = p('depth_sync_tol_s', 0.02)   # 컬러·깊이 stamp 차이 허용 [s]

        # ---- 여러 물체 번호 유지 (object_tracker.py) ----
        self.use_tracker = p('use_object_tracker', False)
        self.tracker = ObjectTracker(TrackerConfig(
            gate_m=float(p('track_gate_m', 0.04)),
            reid_gate_m=float(p('track_reid_gate_m', 0.06)),
            size_ratio_max=float(p('track_size_ratio_max', 1.6)),
            velocity_window_s=float(p('track_velocity_window_s', 0.1)),
            predict_horizon_s=float(p('track_predict_horizon_s', 0.5)),
            assumed_range_m=float(p('track_assumed_range_m', 0.6)),
            sync_error_s=float(p('track_sync_error_s', 0.02)),
            fast_rotation_deg_s=float(p('track_fast_rotation_deg_s', 30.0))))
        self.lock = TargetLock(relock_after_s=float(p('relock_after_s', 3.0)))
        # 모터 각도 -> 기하 각도 변환값. tracker_controller(config/tracker.yaml)와 반드시 같은 값 #todo
        self.pan_direction = int(p('pan_direction', 1))
        self.tilt_direction = int(p('tilt_direction', 1))
        self.cam_forward = float(p('cam_forward_m', 0.0))
        self.cam_up = float(p('cam_up_m', 0.0))
        joint_topic = p('joint_state_topic', '/pan_tilt/joint_states')
        self.joints = geo.JointHistory(keep_s=2.0) if geo else None
        self.rot_speed_deg_s = 0.0
        self.last_ids = {}
        self.sim_joint_warned = False

        # ---- 저장·표시 ----
        self.save_dir = os.path.expanduser(p('save_dir', '~/lv2_module5_results/images'))
        self.save_every_n = int(p('save_every_n', 0))          # 0: 주기 저장 안 함
        self.show_window = p('show_window', False)             # 모니터가 있는 PC에서만 true
        self.run_id = p('run_id', '') or datetime.now().strftime('det_%Y%m%d_%H%M%S')
        log_dir = os.path.expanduser(p('log_dir', '~/lv2_module5_logs'))
        os.makedirs(log_dir, exist_ok=True)
        os.makedirs(self.save_dir, exist_ok=True)
        self.csv_file = open(os.path.join(log_dir, f'{self.run_id}_detect.csv'), 'w', newline='')
        self.csv = csv.writer(self.csv_file)
        self.csv.writerow(['run_id', 'stamp_s', 'frame_id', 'frame_seq', 'width', 'height', 'detected',
                           'cx_px', 'cy_px', 'ex', 'ey', 'area_ratio', 'n_candidates',
                           'depth_valid', 'depth_valid_ratio', 'z_m', 'x_m', 'y_m',
                           'proc_ms', 'depth_dt_ms', 'pub_time_s',
                           'target_id', 'n_rejected', 'rot_speed_deg_s'])   # 뒤에 추가한 열 (기존 열 순서 유지)

        # ---- 상태 ----
        self.bridge = CvBridge()
        self.intrinsics = None             # (fx, fy, cx, cy)
        self.depth_cache = deque(maxlen=10)
        self.last_stamp = None
        self.prev_center, self.prev_center_t = None, None
        self.frame_seq = 0
        self.snapshot_label = None
        self.fps_count, self.fps_t0, self.proc_sum = 0, time.monotonic(), 0.0
        self.mouse = (-1, -1)
        if self.show_window:
            cv2.namedWindow('target_detector')
            cv2.setMouseCallback('target_detector', self.on_mouse)   # mouse_test.py와 같은 픽셀 조회

        # ---- ROS ----
        sensor_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                                history=HistoryPolicy.KEEP_LAST, depth=1)   # RELIABLE 발행자와 호환
        depth_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                               history=HistoryPolicy.KEEP_LAST, depth=5)
        out_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                             history=HistoryPolicy.KEEP_LAST, depth=1)     # 발제 기본 QoS
        self.create_subscription(CameraInfo, info_topic, self.on_info, sensor_qos)
        if self.use_depth:
            self.create_subscription(Image, depth_topic, self.on_depth, depth_qos)
        self.create_subscription(Image, color_topic, self.on_color, sensor_qos)
        self.create_subscription(String, snapshot_topic, self.on_snapshot, 10)
        self.create_timer(1.0, self.csv_file.flush)       # 기록을 1 s마다 디스크로 (강제 종료 때도 직전까지 남김)
        if self.use_tracker:
            self.create_subscription(JointState, joint_topic, self.on_joint, 10)
            if geo is None:
                self.get_logger().warn('tracker_controller 없음: 회전 보정 없이 카메라 좌표로 번호를 유지합니다')
        self.target_pub = self.create_publisher(PointStamped, target_topic, out_qos)
        self.depth_pub = self.create_publisher(PointStamped, depth_out_topic, out_qos)
        self.pos_pub = self.create_publisher(PointStamped, position_topic, out_qos)
        self.get_logger().info(
            f'run_id={self.run_id} hsv={self.cfg.hsv_lower}~{self.cfg.hsv_upper} '
            f'min_area={self.cfg.min_area_px} selection={self.cfg.selection} use_depth={self.use_depth} '
            f'detect_scale={self.cfg.detect_scale} size_check={self.cfg.size_check} '
            f'object_tracker={self.use_tracker}')

    # ================= 콜백 =================
    def on_info(self, msg):
        self.intrinsics = (msg.k[0], msg.k[4], msg.k[2], msg.k[5])   # fx, fy, cx, cy

    def on_depth(self, msg):
        self.depth_cache.append((stamp_ns(msg.header), msg))

    def on_joint(self, msg):
        """모터 각도 기록 (영상 촬영 시각의 자세를 보간하기 위해). position [rad] -> [deg]"""
        if len(msg.position) < 2:
            return
        if list(msg.name[:2]) != ['pan', 'tilt']:      # dry_run의 시뮬레이션 각도(pan_sim)는 실제 카메라 자세가 아니다
            if not self.sim_joint_warned:
                self.get_logger().warn(f'joint_states {list(msg.name)} 무시: 실제 모터 각도가 아님 (회전 보정 없이 동작)')
                self.sim_joint_warned = True
            return
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.joints is not None:
            self.joints.add(t, math.degrees(msg.position[0]), math.degrees(msg.position[1]))
        if len(msg.velocity) >= 2 and all(math.isfinite(v) for v in msg.velocity[:2]):
            self.rot_speed_deg_s = max(abs(math.degrees(v)) for v in msg.velocity[:2])

    def to_base(self, p_opt, t):
        """카메라 광학 좌표의 점 -> 회전축 기준 좌표. 각도를 모르면 카메라 좌표를 그대로 쓴다."""
        joint = self.joints.at(t) if self.joints is not None else None
        if geo is None or joint is None:
            return p_opt
        pan_g = -self.pan_direction * math.radians(joint[0])     # controller_node.motor_to_geo와 같은 식
        tilt_g = -self.tilt_direction * math.radians(joint[1])
        return geo.cam_to_base(p_opt, pan_g, tilt_g, self.cam_forward, self.cam_up)

    def observations(self, cands, t):
        """후보 -> object_tracker 관측 (방향은 항상, 3D 위치는 깊이가 있을 때만)."""
        fx, fy, ppx, ppy = self.intrinsics
        obs = []
        for c in cands:
            ray = ((c.cx - ppx) / fx, (c.cy - ppy) / fy, 1.0)
            o, r = self.to_base((0.0, 0.0, 0.0), t), self.to_base(ray, t)
            d = tuple(a - b for a, b in zip(r, o))                     # 회전만 적용한 방향
            n = math.sqrt(sum(x * x for x in d))
            obs.append(Observation(bearing=tuple(x / n for x in d),
                                   pos=self.to_base(c.pos_cam, t) if c.pos_cam else None,
                                   area_cm2=c.area_cm2))
        return obs

    def on_snapshot(self, msg):
        self.snapshot_label = msg.data or 'snapshot'      # 다음 영상의 원본·마스크·검출 이미지를 저장

    def on_mouse(self, event, x, y, flags, param):
        if event == cv2.EVENT_MOUSEMOVE:
            self.mouse = (x, y)

    def find_depth(self, ns):
        """컬러 영상과 stamp가 가장 가까운 정렬 깊이 영상. 허용 차이를 넘으면 None."""
        if not self.depth_cache:
            return None, math.nan
        best = min(self.depth_cache, key=lambda d: abs(d[0] - ns))
        dt = abs(best[0] - ns) * 1e-9
        if dt > self.depth_sync_tol_s:
            return None, dt * 1e3
        msg = best[1]
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        scale = 0.001 if msg.encoding in ('16UC1', 'mono16') else 1.0   # 16UC1: mm, 32FC1: m
        return (img, scale), dt * 1e3

    def on_color(self, msg):
        ns = stamp_ns(msg.header)
        if self.last_stamp is not None and ns <= self.last_stamp:
            return                                        # 같은/과거 영상: 발행하지 않음
        self.last_stamp = ns
        t_start = time.perf_counter()
        self.frame_seq += 1

        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        encoding = msg.encoding
        now = time.monotonic()
        prev = self.prev_center if (self.prev_center_t and now - self.prev_center_t < self.lock_timeout_s) else None

        # 깊이를 먼저 찾는다: 후보마다 실제 크기를 재고(size_check) 번호를 붙이는 데(object_tracker) 쓴다
        dres, depth_dt_ms, depth_pack = None, math.nan, None
        if self.use_depth:
            depth_pack, depth_dt_ms = self.find_depth(ns)
        depth, scale = depth_pack if depth_pack is not None else (None, 0.001)
        h, w = image.shape[:2]
        cands, rejected, mask = find_and_measure(image, encoding, self.cfg, depth, scale, self.intrinsics)

        def pick():
            return select_candidate(cands, self.cfg, prev, w, h)

        self.last_ids = {}
        if self.use_tracker and self.intrinsics is not None:
            t_img = ns * 1e-9                              # 같은 Pi라 영상 stamp와 모터 상태 시계가 같다
            self.last_ids = self.tracker.update(t_img, self.observations(cands, t_img), self.rot_speed_deg_s)
            i = self.lock.choose(t_img, self.last_ids, self.tracker, pick)
        else:
            i = pick()
        det = make_detection(cands, rejected, i, w, h)
        if det.detected:
            self.prev_center, self.prev_center_t = (det.cx, det.cy), now
            if depth is not None:
                dres = target_depth(depth, det.contour, self.cfg, scale, self.intrinsics, (det.cx, det.cy))

        self.publish(msg.header, det, dres)
        pub_time_s = self.get_clock().now().nanoseconds * 1e-9   # 발행 시각 - stamp = 촬영(또는 드라이버) -> 발행 지연
        proc_ms = (time.perf_counter() - t_start) * 1e3
        self.log(msg, image, det, dres, proc_ms, depth_dt_ms, pub_time_s)
        if self.save_every_n > 0 and self.frame_seq % self.save_every_n == 0:
            self.save_images(image, encoding, mask, det, dres, f'f{self.frame_seq:06d}')
        if self.snapshot_label:
            # 프레임 번호를 붙여 같은 라벨로 여러 번 찍어도 덮어쓰지 않는다 (2026-10-06: 'normal' 4번이 1장만 남음)
            self.save_images(image, encoding, mask, det, dres, f'{self.snapshot_label}_f{self.frame_seq:06d}')
            self.get_logger().info(f'snapshot saved: {self.snapshot_label}')
            self.snapshot_label = None
        if self.show_window:
            self.show(image, encoding, det, dres, depth_pack)

    # ================= 출력 =================
    def publish(self, header, det, dres):
        t = PointStamped()
        t.header = header                                  # 원본 영상 stamp·frame_id 유지
        if det.detected:
            t.point.x, t.point.y, t.point.z = det.ex, det.ey, det.area_ratio
        self.target_pub.publish(t)                         # 미검출이면 (0, 0, 0)

        d = PointStamped()
        d.header = header
        valid = dres is not None and dres.valid
        d.point.x = 1.0 if valid else 0.0
        d.point.y = dres.valid_ratio if dres is not None else 0.0
        d.point.z = dres.z_m if valid else 0.0
        self.depth_pub.publish(d)

        if self.publish_position:
            pos = PointStamped()
            pos.header = header
            pos.point.x, pos.point.y, pos.point.z = ((dres.x_m, dres.y_m, dres.z_m)
                                                     if valid and math.isfinite(dres.x_m) else (NAN, NAN, NAN))
            self.pos_pub.publish(pos)

    def log(self, msg, image, det, dres, proc_ms, depth_dt_ms, pub_time_s):
        f = lambda v, n=4: '' if v is None or not math.isfinite(v) else f'{v:.{n}f}'  # noqa: E731
        h, w = image.shape[:2]
        dv = dres is not None and dres.valid
        self.csv.writerow([self.run_id, f(stamp_ns(msg.header) * 1e-9, 6), msg.header.frame_id, self.frame_seq,
                           w, h, int(det.detected), f(det.cx, 1), f(det.cy, 1), f(det.ex), f(det.ey),
                           f(det.area_ratio, 5), det.n_candidates, int(dv),
                           f(dres.valid_ratio if dres else None, 3), f(dres.z_m if dv else None),
                           f(dres.x_m if dv else None), f(dres.y_m if dv else None),
                           f(proc_ms, 2), f(depth_dt_ms, 2), f(pub_time_s, 6),
                           self.last_ids.get(det.index, '') if det.detected else '', len(det.rejected),
                           f(self.rot_speed_deg_s, 1)])
        # 처리 FPS: 처리 완료 프레임 수 / 실제 경과 초 (발제 문제 4 정의). 5초마다 로그
        self.fps_count += 1
        self.proc_sum += proc_ms
        elapsed = time.monotonic() - self.fps_t0
        if elapsed >= 5.0:
            self.get_logger().info(f'processing FPS {self.fps_count / elapsed:.1f}, '
                                   f'mean proc {self.proc_sum / self.fps_count:.1f} ms')
            self.fps_count, self.fps_t0, self.proc_sum = 0, time.monotonic(), 0.0

    def save_images(self, image, encoding, mask, det, dres, label):
        bgr = cv2.cvtColor(image, cv2.COLOR_RGB2BGR) if encoding == 'rgb8' else image
        base = os.path.join(self.save_dir, f'{self.run_id}_{label}')
        cv2.imwrite(base + '_raw.png', bgr)                 # 원본
        cv2.imwrite(base + '_mask.png', mask)               # 마스크
        cv2.imwrite(base + '_detect.png', draw_overlay(bgr, det, dres, self.last_ids))   # 검출

    def show(self, image, encoding, det, dres, depth_pack):
        """mouse_test.py처럼 커서 픽셀의 RGB·HSV·깊이를 표시한다 (HSV 범위 조정용)."""
        bgr = cv2.cvtColor(image, cv2.COLOR_RGB2BGR) if encoding == 'rgb8' else image
        out = draw_overlay(bgr, det, dres, self.last_ids)
        u, v = self.mouse
        h, w = out.shape[:2]
        if 0 <= u < w and 0 <= v < h:
            b, g, r = (int(c) for c in bgr[v, u])
            hh, ss, vv = (int(c) for c in to_hsv(image[v:v + 1, u:u + 1], encoding)[0, 0])
            txt = f'({u},{v}) RGB {r},{g},{b}  HSV {hh},{ss},{vv}'
            if depth_pack is not None:
                txt += f'  D {float(depth_pack[0][v, u]) * depth_pack[1]:.3f} m'
            cv2.drawMarker(out, (u, v), (0, 255, 0), cv2.MARKER_CROSS, 20, 1)
            cv2.rectangle(out, (0, h - 22), (w, h), (0, 0, 0), -1)
            cv2.putText(out, txt, (6, h - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
        cv2.imshow('target_detector', out)
        cv2.waitKey(1)

    def close(self):
        self.csv_file.close()
        if self.show_window:
            cv2.destroyAllWindows()


def main():
    rclpy.init()
    node = TargetDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
