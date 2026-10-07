#!/usr/bin/env python3
"""팬·틸트 추적 제어 노드 (라즈베리파이에서 실행). 시리얼은 tracker_bridge/opencr_bridge가 맡는다.

규약: lv2_module5/docs/interface.md
입력 (같은 Pi의 인지 노드 -> 이 노드)
  /target              PointStamped  x=ex, y=ey(정규화 중심 오차), z=면적비(0=미검출)   매 영상
  /target/position_cam PointStamped  카메라 광학 좌표 [m] (깊이 무효·미검출이면 NaN)   매 영상, /target과 같은 stamp
  /pan_tilt/joint_states JointState  모터 각도 [rad] (opencr_bridge 발행, dry_run이면 pan_sim·tilt_sim)
  인지와 제어가 같은 Pi에서 돌아 시계가 같으므로, 영상 stamp 시각의 모터 각도를 그대로 찾아 쓴다.
출력
  /pan_tilt/command    Vector3Stamped x=팬, y=틸트 [deg/s]  50 Hz (IDLE·LOST에서도 0을 계속 발행)
  /tracking_status     String "상태:사유"
동작
  TRACKING : 영상 중심 오차 P 제어 (발제 문제 3 식). 정규화 오차를 카메라 각도로 바꿔 각도 Kp [1/s]를 곱한다
             각도 오차 = atan(ex x tan(hfov/2)), 명령 = clamp(direction x Kp x 각도 오차) [deg/s]
  LOST     : 즉시 정지
  SEARCHING: (심화) 기준 좌표에 기억한 목표의 예측 위치로 카메라를 돌린다
"""
import csv
import math
import os
import time
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import PointStamped, Vector3Stamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String

from tracker_controller import geometry as geo
from tracker_controller.tracking_logic import AxisConfig, SearchConfig, TrackingLogic

COMMAND_UNIT = 'deg/s'
PAIR_WAIT_S = 0.15     # /target과 /target/position_cam 짝을 기다리는 시간


class TrackerNode(Node):
    def __init__(self):
        super().__init__('tracker_controller')
        p = lambda name, default: self.declare_parameter(name, default).value  # noqa: E731

        # ---- 토픽 (#todo 토픽 이름은 인지·통합 담당과 합의) ----
        rate = p('control_rate_hz', 50.0)
        target_topic = p('target_topic', '/target')                          # 발제 기본 인터페이스
        position_topic = p('position_topic', '/target/position_cam')         # #todo 인지 담당과 합의
        enable_topic = p('enable_topic', '/tracking_enable')                 # #todo 합의
        status_topic = p('status_topic', '/tracking_status')                 # #todo 합의
        command_topic = p('command_topic', '/pan_tilt/command')              # #todo 합의
        joint_topic = p('joint_state_topic', '/pan_tilt/joint_states')       # #todo 합의
        pred_topic = p('predicted_topic', '/tracker/predicted_target')       # #todo 인지 담당과 합의
        base_topic = p('target_base_topic', '/tracker/target_base')          # #todo 합의
        self.base_frame = p('base_frame', 'pan_tilt_base')                   # #todo 합의
        auto_enable = p('auto_enable', False)

        # ---- 제어: Kp는 각도 오차 [deg] -> 속도 [deg/s] 단위 [1/s] (report 3-1 계단 응답과 같은 단위) ----
        hfov_deg = p('hfov_deg', 55.7)                      # CameraInfo: 2*atan(W/(2*fx))
        vfov_deg = p('vfov_deg', 43.2)                      # CameraInfo: 2*atan(H/(2*fy))
        self.pan_cfg = AxisConfig(kp=p('pan_kp', 2.0), direction=int(p('pan_direction', 1)),
                                  speed_limit=p('pan_speed_limit_deg_s', 30.0), deadband=p('pan_deadband', 0.03),
                                  half_fov_deg=hfov_deg / 2)
        self.tilt_cfg = AxisConfig(kp=p('tilt_kp', 2.5), direction=int(p('tilt_direction', 1)),
                                   speed_limit=p('tilt_speed_limit_deg_s', 20.0), deadband=p('tilt_deadband', 0.05),
                                   enabled=p('tilt_enabled', True), half_fov_deg=vfov_deg / 2)
        search = SearchConfig(enabled=p('search_enabled', False), delay_s=p('search_delay_s', 0.3),
                              timeout_s=p('search_timeout_s', 3.0), kp=p('search_kp', 2.0),
                              speed_limit=p('search_speed_limit_deg_s', 20.0),
                              tolerance_deg=p('search_tolerance_deg', 1.0),
                              pan_max_deg=p('search_pan_max_deg', 80.0), tilt_max_deg=p('search_tilt_max_deg', 25.0))
        self.logic = TrackingLogic(self.pan_cfg, self.tilt_cfg, search,
                                   input_timeout_s=p('input_timeout_s', 0.5),
                                   recover_frames=int(p('recover_frames', 3)),
                                   require_increasing_stamp=p('require_increasing_stamp', True))

        # ---- 카메라·기구 기하 (#todo 측정) ----
        self.hfov = math.radians(hfov_deg)
        self.vfov = math.radians(vfov_deg)
        self.cam_forward = p('cam_forward_m', 0.0)          # #todo 틸트 축 -> 광학 중심, 앞 방향 [m]
        self.cam_up = p('cam_up_m', 0.0)                    # #todo 틸트 축 -> 광학 중심, 위 방향 [m]
        self.pose_time_source = p('pose_time_source', 'stamp')  # stamp: 영상 stamp 시각의 자세 / receive: 수신 시각-지연
        self.latency = p('camera_latency_s', 0.05)          # receive 모드에서만 사용 [s] #todo
        self.default_range = p('default_range_m', 0.6)      # 깊이 무효일 때 쓰는 거리 [m] #todo
        self.memory = geo.TargetMemory(window_s=p('memory_window_s', 0.5),
                                       max_speed_m_s=p('memory_max_speed_m_s', 0.5),
                                       horizon_s=p('memory_horizon_s', 1.0),
                                       max_age_s=p('memory_max_age_s', 3.0))
        self.joints = geo.JointHistory()
        self.joint_vel = (math.nan, math.nan)   # [deg/s] 펌웨어 측정 속도 (기록용)

        # ---- 기록 ----
        run_id = p('run_id', 'auto')
        self.run_id = run_id if run_id not in ('', 'auto') else datetime.now().strftime('run_%Y%m%d_%H%M%S')
        log_dir = os.path.expanduser(p('log_dir', '~/lv2_module5_logs'))
        os.makedirs(log_dir, exist_ok=True)
        self.csv_file = open(os.path.join(log_dir, f'{self.run_id}.csv'), 'w', newline='')
        self.csv = csv.writer(self.csv_file)
        self.csv.writerow(['run_id', 'time_s', 'ros_time_s', 'target_seq', 'stamp_s', 'frame_id', 'detected',
                           'ex', 'ey', 'area_ratio', 'target_age_s', 'depth_valid',
                           'state', 'reason', 'pan_cmd', 'tilt_cmd', 'command_unit',
                           'meas_source', 'pan_deg', 'tilt_deg', 'pan_dps', 'tilt_dps',
                           'base_x_m', 'base_y_m', 'base_z_m', 'pred_ex', 'pred_ey',
                           'desired_pan_deg', 'desired_tilt_deg', 'search_elapsed_s',
                           'joint_name'])
        self.t0 = time.monotonic()
        self.last_frame_id, self.last_stamp_s = '', math.nan
        self.prev_state = None
        self.pending = {}     # stamp_ns -> (rx_mono, rx_ros, ex, ey, area)   /target 대기
        self.positions = {}   # stamp_ns -> (rx, (x, y, z))      /target/position_cam 대기
        self.last_depth_valid = False
        self.joint_name = ''

        # ---- ROS 인터페이스 ----
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=1)
        self.create_subscription(PointStamped, target_topic, self.on_target, qos)
        self.create_subscription(PointStamped, position_topic, self.on_position, qos)
        self.create_subscription(Bool, enable_topic, self.on_enable, 10)
        self.create_subscription(JointState, joint_topic, self.on_joint, 10)
        self.status_pub = self.create_publisher(String, status_topic, 10)
        self.cmd_pub = self.create_publisher(Vector3Stamped, command_topic, 10)
        self.pred_pub = self.create_publisher(PointStamped, pred_topic, qos)
        self.base_pub = self.create_publisher(PointStamped, base_topic, 10)
        self.create_timer(1.0 / rate, self.on_timer)
        # 기록은 1 s마다 디스크에 내보낸다: kill -9(제어 통신 중단 시험)·전원 차단 때도 그 직전까지 남게
        self.create_timer(1.0, self.flush_logs)
        self.get_logger().info(
            f'run_id={self.run_id} search={search.enabled} '
            f'pan(kp={self.pan_cfg.kp}/s, dir={self.pan_cfg.direction}, limit={self.pan_cfg.speed_limit}) '
            f'tilt(kp={self.tilt_cfg.kp}/s, dir={self.tilt_cfg.direction}, limit={self.tilt_cfg.speed_limit})')
        if auto_enable:
            self.set_enabled(True)

    # ================= 각도 변환 =================
    def motor_to_geo(self, pan_deg, tilt_deg):
        """모터 각도 [deg] -> 기하 각도 [rad] (왼쪽·위 +). geo = -direction x motor"""
        return (-self.pan_cfg.direction * math.radians(pan_deg),
                -self.tilt_cfg.direction * math.radians(tilt_deg))

    def geo_to_motor(self, pan_geo, tilt_geo):
        return (-self.pan_cfg.direction * math.degrees(pan_geo),
                -self.tilt_cfg.direction * math.degrees(tilt_geo))

    # ================= 입력 =================
    def set_enabled(self, on):
        self.logic.set_enabled(on)
        self.memory.clear()
        self.get_logger().info(f'tracking enabled={on}')

    def on_enable(self, msg):
        self.set_enabled(bool(msg.data))

    def on_joint(self, msg):
        """opencr_bridge의 모터 각도 [rad]. header.stamp = 브리지가 상태 줄을 받은 ROS 시각"""
        if len(msg.position) < 2:
            return
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.joints.add(t, math.degrees(msg.position[0]), math.degrees(msg.position[1]))
        self.joint_name = msg.name[0] if msg.name else ''
        if len(msg.velocity) >= 2:
            self.joint_vel = (math.degrees(msg.velocity[0]), math.degrees(msg.velocity[1]))

    def ros_now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    @staticmethod
    def stamp_ns(msg):
        return msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec

    def on_target(self, msg):
        now = time.monotonic()
        ns = self.stamp_ns(msg)
        if not self.logic.on_target(msg.point.x, msg.point.y, msg.point.z, ns, now):
            return
        self.last_frame_id, self.last_stamp_s = msg.header.frame_id, ns * 1e-9
        if self.logic.last[3]:                       # 검출된 프레임만 기억에 넣는다
            self.pending[ns] = (now, self.ros_now(), *self.logic.last[:3])
            self.try_pair(ns)

    def on_position(self, msg):
        ns = self.stamp_ns(msg)
        self.positions[ns] = (time.monotonic(), (msg.point.x, msg.point.y, msg.point.z))
        self.try_pair(ns)

    def try_pair(self, ns, force=False):
        """같은 stamp의 /target과 위치가 모이면 기준 좌표 기억을 갱신한다."""
        if ns not in self.pending:
            return
        pos = self.positions.pop(ns, None)
        if pos is None and not force:
            return
        rx, rx_ros, ex, ey, _ = self.pending.pop(ns)
        t_cap = ns * 1e-9 if self.pose_time_source == 'stamp' and ns > 0 else rx_ros - self.latency
        joint = self.joints.at(t_cap)                  # 촬영 시각의 카메라 자세
        if joint is None:
            return
        mem_t = rx - max(0.0, rx_ros - t_cap)          # 기억은 단조 시계 기준 촬영 시각으로 저장
        depth_ok = pos is not None and all(math.isfinite(c) for c in pos[1]) and pos[1][2] > 0
        p_opt = pos[1] if depth_ok else geo.ray_point(ex, ey, self.default_range, self.hfov, self.vfov)
        pan_g, tilt_g = self.motor_to_geo(*joint)
        self.memory.add(mem_t, geo.cam_to_base(p_opt, pan_g, tilt_g, self.cam_forward, self.cam_up), depth_ok)
        self.last_depth_valid = depth_ok

    def flush_pending(self, now):
        for ns in [k for k, v in self.pending.items() if now - v[0] > PAIR_WAIT_S]:
            self.try_pair(ns, force=True)            # 위치가 안 오면 광선+기본 거리로 대체
        for ns in [k for k, v in self.positions.items() if now - v[0] > 1.0]:
            del self.positions[ns]

    # ================= 주기 처리 =================
    def current_joint(self):
        latest = self.joints.latest()
        return latest[1:] if latest and self.ros_now() - latest[0] < 0.2 else None

    def on_timer(self):
        now = time.monotonic()
        self.flush_pending(now)

        joint = self.current_joint()
        pred_base = self.memory.predict(now)
        desired, pred_norm = None, None
        if pred_base is not None:
            desired = self.geo_to_motor(*geo.look_at(pred_base, self.cam_up))
            if joint is not None:
                pan_g, tilt_g = self.motor_to_geo(*joint)
                pred_norm = geo.to_normalized(
                    geo.base_to_cam(pred_base, pan_g, tilt_g, self.cam_forward, self.cam_up), self.hfov, self.vfov)

        out = self.logic.step(now, joint, desired)
        if out.state != self.prev_state:
            self.get_logger().info(f'state {self.prev_state} -> {out.state} ({out.reason})')
            self.prev_state = out.state
        self.publish(out, joint, pred_base, pred_norm)
        self.write_csv(now, out, joint, pred_base, pred_norm, desired)

    def publish(self, out, joint, pred_base, pred_norm):
        stamp = self.get_clock().now().to_msg()
        self.status_pub.publish(String(data=f'{out.state}:{out.reason}'))
        cmd = Vector3Stamped()
        cmd.header.stamp = stamp
        cmd.vector.x, cmd.vector.y = out.pan_cmd, out.tilt_cmd           # [deg/s] IDLE·LOST에서도 0을 계속 보내 정지를 확정
        self.cmd_pub.publish(cmd)
        if pred_base is not None:
            b = PointStamped()
            b.header.stamp, b.header.frame_id = stamp, self.base_frame
            b.point.x, b.point.y, b.point.z = pred_base                      # [m]
            self.base_pub.publish(b)
        if pred_norm is not None:
            pr = PointStamped()
            pr.header.stamp, pr.header.frame_id = stamp, 'normalized_image'
            pr.point.x, pr.point.y = pred_norm                               # |값| > 1 이면 시야 밖
            pr.point.z = 1.0 if self.memory.depth_valid else 0.5             # 1: 깊이 기반, 0.5: 광선 추정
            self.pred_pub.publish(pr)

    def write_csv(self, now, out, joint, pred_base, pred_norm, desired):
        f = lambda v, n=3: '' if v is None or not math.isfinite(v) else f'{v:.{n}f}'  # noqa: E731
        last = self.logic.last or (math.nan, math.nan, math.nan, False)
        pb = pred_base or (None, None, None)
        pn = pred_norm or (None, None)
        de = desired or (None, None)
        jt = joint or (None, None)
        self.csv.writerow([self.run_id, f(now - self.t0), f(self.ros_now(), 6), self.logic.seq, f(self.last_stamp_s, 6), self.last_frame_id,
                           int(last[3]), f(last[0], 4), f(last[1], 4), f(last[2], 5),
                           f(self.logic.target_age(now)), int(self.last_depth_valid),
                           out.state, out.reason, f(out.pan_cmd), f(out.tilt_cmd), COMMAND_UNIT,
                           'joint_states', f(jt[0]), f(jt[1]), f(self.joint_vel[0]), f(self.joint_vel[1]),
                           f(pb[0]), f(pb[1]), f(pb[2]), f(pn[0]), f(pn[1]), f(de[0]), f(de[1]),
                           f(out.search_elapsed), self.joint_name])

    def flush_logs(self):
        self.csv_file.flush()

    def close(self):
        self.get_logger().info(f'stats {self.logic.stats} stale_inputs={self.logic.stale_count}')
        self.csv_file.close()


def main():
    rclpy.init()
    node = TrackerNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):   # Ctrl+C / launch 종료(SIGINT·SIGTERM)
        pass
    except Exception:
        if rclpy.ok():   # 종료 신호로 context가 닫힌 뒤 콜백이 발행하다 난 오류만 무시
            raise
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
