#!/usr/bin/env python3
"""카메라 화면 미리보기 + 세 장면 스냅샷 키 (Pi에서 ssh -X로 접속한 창에서 실행).

인지 노드(perception.launch.py)가 돌고 있는 상태에서 실행한다. 카메라를 직접 열지 않고 ROS 토픽을 본다.
화면은 target_detector와 같은 검출 함수·같은 config/*.yaml 값(크기 검증·선택 규칙 포함)으로 다시 계산해 그린다(보기 전용).
깊이는 가장 최근 정렬 깊이 영상을 쓴다(노드처럼 stamp를 맞추지 않음). 번호(#ID) 유지는 노드에서만 한다.
스냅샷은 target_detector가 직접 저장한다(/target/save_snapshot) → ~/lv2_module5_results/images/
  n : 정상(normal) 장면 저장     e : 대상 없음(empty) 저장     o : 가림(occluded) 저장
  m : 검출 화면 ↔ 마스크 화면 전환     q : 종료
X 전달은 느리므로 화면은 기본 5 Hz로만 갱신한다(검출 노드 성능에는 영향 없음).
"""
import glob
import math
import os
import sys
import time

import cv2
import rclpy
import yaml
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String

from target_detector.detection import DetectorConfig, detect, draw_overlay

CFG = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', 'config')
HZ = float(sys.argv[1]) if len(sys.argv) > 1 else 5.0
KEYS = {ord('n'): 'normal', ord('e'): 'empty', ord('o'): 'occluded'}


class Preview(Node):
    def __init__(self):
        super().__init__('preview')
        params = {}   # launch와 같게 config/*.yaml의 /** 공통 값 + target_detector 항목 (토픽은 camera.yaml·control.yaml)
        for f in sorted(glob.glob(os.path.join(CFG, '*.yaml'))):
            data = yaml.safe_load(open(f)) or {}
            for key in ('/**', 'target_detector'):
                params.update((data.get(key) or {}).get('ros__parameters') or {})
        keys = ('morph_kernel', 'min_area_px', 'selection', 'depth_min_valid_ratio', 'depth_erode_px', 'size_check',
                'obj_area_min_cm2', 'obj_area_max_cm2', 'similar_area_ratio', 'similar_depth_m', 'similar_depth_ratio')
        self.cfg = DetectorConfig(hsv_lower=tuple(params['hsv_lower']), hsv_upper=tuple(params['hsv_upper']),
                                  **{k: params[k] for k in keys if k in params})
        self.depth, self.intrinsics = None, None
        self.bridge = CvBridge()
        self.frame, self.encoding, self.target = None, None, None
        self.show_mask = False
        self.notice, self.notice_until = '', 0.0
        self.create_subscription(Image, params['color_topic'], self.on_image, qos_profile_sensor_data)
        self.create_subscription(PointStamped, params['target_topic'], self.on_target, qos_profile_sensor_data)
        if params.get('use_depth', True):
            self.create_subscription(Image, params['depth_topic'], self.on_depth, qos_profile_sensor_data)
            self.create_subscription(CameraInfo, params['info_topic'], self.on_info, qos_profile_sensor_data)
        self.snap_pub = self.create_publisher(String, params['snapshot_topic'], 10)
        self.create_timer(1.0 / HZ, self.draw)
        self.get_logger().info(f'HSV {self.cfg.hsv_lower}~{self.cfg.hsv_upper}, keys: n e o (snapshot), m, q')

    def on_image(self, msg):
        self.frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')  # 표시용으로 BGR 변환

    def on_depth(self, msg):
        self.depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')   # 16UC1 [mm]

    def on_info(self, msg):
        self.intrinsics = (msg.k[0], msg.k[4], msg.k[2], msg.k[5])

    def on_target(self, msg):
        self.target = msg

    def draw(self):
        if self.frame is None:
            return
        det, mask = detect(self.frame, 'bgr8', self.cfg, None, self.depth, 0.001, self.intrinsics)
        view = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR) if self.show_mask else draw_overlay(self.frame.copy(), det)
        h, w = view.shape[:2]
        cv2.drawMarker(view, (w // 2, h // 2), (255, 255, 255), cv2.MARKER_CROSS, 20, 1)
        t = self.target
        if t is not None and t.point.z > 0:   # 검출 노드가 실제로 발행한 값
            line = f'/target ex={t.point.x:+.3f} ey={t.point.y:+.3f} area={t.point.z:.4f}'
        else:
            line = '/target NO TARGET (z=0)'
        cv2.putText(view, line, (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        cv2.putText(view, 'MASK' if self.show_mask else 'DETECT', (8, h - 12),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        if time.time() < self.notice_until:
            cv2.putText(view, self.notice, (8, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        cv2.imshow('lv2_module5 preview', view)
        key = cv2.waitKey(1) & 0xFF
        if key in KEYS:
            self.snap_pub.publish(String(data=KEYS[key]))
            self.notice, self.notice_until = f'snapshot: {KEYS[key]}', time.time() + 2.0
            self.get_logger().info(f'snapshot requested: {KEYS[key]}')
        elif key == ord('m'):
            self.show_mask = not self.show_mask
        elif key == ord('q'):
            raise KeyboardInterrupt


def main():
    rclpy.init()
    node = Preview()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
