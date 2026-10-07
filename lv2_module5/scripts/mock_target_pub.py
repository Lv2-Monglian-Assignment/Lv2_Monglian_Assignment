#!/usr/bin/env python3
"""모의 인지 입력 (모터 출력 끈 시험용). /target 과 /target/position_cam 을 같은 stamp로 발행한다.

ROS 환경(source install/setup.bash) 뒤 python3로 실행한다. tracker_controller 패키지의 geometry를 쓴다.

mode:=fixed  (문제 2 다섯 입력)
  python3 scripts/mock_target_pub.py --ros-args -p ex:=0.4 -p area:=0.05
  -p area:=0.0 -> 미검출,  -p duration_s:=5.0 -> 5 s 후 발행 중단(토픽 침묵),
  -p lost_after_s:=3.0 -> 3 s 후부터 미검출(z=0)을 계속 발행

mode:=virtual  (폐루프 시험: control.launch.py dry_run:=true와 함께)
  기준 좌표에 가상의 물체를 두고 /pan_tilt/joint_states 의 카메라 자세로 영상 위치를 계산한다.
  시야 밖이거나 hide 구간이면 미검출을 발행한다 -> TRACKING / LOST / SEARCHING 을 실물 없이 확인.
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import JointState

from tracker_controller import geometry as geo

NAN = float('nan')


class MockTarget(Node):
    def __init__(self):
        super().__init__('mock_target')
        p = lambda name, default: self.declare_parameter(name, default).value  # noqa: E731
        self.mode = p('mode', 'fixed')
        self.ex, self.ey, self.area = p('ex', 0.0), p('ey', 0.0), p('area', 0.05)
        self.range_m = p('range_m', 0.6)
        self.duration = p('duration_s', 0.0)        # 0: 계속
        self.lost_after = p('lost_after_s', 0.0)    # 0: 사용 안 함
        self.frame_id = p('frame_id', 'camera_color_optical_frame')        # #todo 인지 담당과 합의
        self.hfov, self.vfov = math.radians(p('hfov_deg', 69.0)), math.radians(p('vfov_deg', 42.0))
        # virtual 모드 (기준 좌표 [m], tracker와 같은 direction·기하 값을 넣는다)
        self.obj = [p('obj_x_m', 0.8), p('obj_y_m', -0.2), p('obj_z_m', 0.0)]
        self.obj_vy = p('obj_vy_m_s', 0.0)          # 좌우 이동 속도 (왼쪽 +)
        self.hide = (p('hide_from_s', -1.0), p('hide_to_s', -1.0))
        self.dirs = (p('pan_direction', 1), p('tilt_direction', 1))
        self.cam_forward, self.cam_up = p('cam_forward_m', 0.0), p('cam_up_m', 0.0)
        self.joint = None
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=1)
        self.pub = self.create_publisher(PointStamped, p('topic', '/target'), qos)
        self.pos_pub = self.create_publisher(PointStamped, p('position_topic', '/target/position_cam'), qos)
        self.create_subscription(JointState, p('joint_state_topic', '/pan_tilt/joint_states'), self.on_joint, 10)
        self.start = self.get_clock().now()
        self.silent = False
        self.create_timer(1.0 / p('rate_hz', 30.0), self.tick)

    def on_joint(self, msg):
        if len(msg.position) >= 2:
            self.joint = (math.degrees(msg.position[0]), math.degrees(msg.position[1]))

    def observe(self, t):
        if self.mode != 'virtual':
            if self.lost_after > 0 and t >= self.lost_after or self.area <= 0:
                return NAN, NAN, 0.0, None
            return self.ex, self.ey, self.area, geo.ray_point(self.ex, self.ey, self.range_m, self.hfov, self.vfov)
        if self.joint is None:
            return NAN, NAN, 0.0, None
        obj = (self.obj[0], self.obj[1] + self.obj_vy * t, self.obj[2])
        pan_g = -self.dirs[0] * math.radians(self.joint[0])
        tilt_g = -self.dirs[1] * math.radians(self.joint[1])
        p_opt = geo.base_to_cam(obj, pan_g, tilt_g, self.cam_forward, self.cam_up)
        n = geo.to_normalized(p_opt, self.hfov, self.vfov)
        hidden = self.hide[0] <= t < self.hide[1]
        if n is None or hidden or abs(n[0]) > 1 or abs(n[1]) > 1:
            return NAN, NAN, 0.0, None
        return n[0], n[1], 0.02, p_opt

    def tick(self):
        now = self.get_clock().now()
        t = (now - self.start).nanoseconds * 1e-9
        if self.duration > 0 and t >= self.duration:
            if not self.silent:
                self.get_logger().info('publishing stopped (topic silence test)')
                self.silent = True
            return
        ex, ey, area, p_opt = self.observe(t)
        msg = PointStamped()
        msg.header.stamp, msg.header.frame_id = now.to_msg(), self.frame_id
        msg.point.x, msg.point.y, msg.point.z = (ex, ey, area) if area > 0 else (0.0, 0.0, 0.0)
        pos = PointStamped()
        pos.header = msg.header
        pos.point.x, pos.point.y, pos.point.z = p_opt if p_opt is not None else (NAN, NAN, NAN)
        self.pub.publish(msg)
        self.pos_pub.publish(pos)


def main():
    rclpy.init()
    node = MockTarget()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
