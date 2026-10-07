#!/usr/bin/env python3
"""OpenCR 시리얼 브리지 노드 (라즈베리파이에서 실행). 펌웨어: firmware/opencr_tracker (작업 이슈 #14).

입력
  /pan_tilt/command    Vector3Stamped  x=팬, y=틸트 [deg/s] (tracker_controller 50 Hz)
출력
  Pi -> OpenCR  "V <pan_dps> <tilt_dps>" 50 Hz. 명령이 cmd_timeout_s 동안 안 오면 "V 0.00 0.00"
                (제어 노드가 죽어도 0을 보내 정지. 브리지가 죽으면 보드 타임아웃 300 ms가 정지시킨다)
                종료 시 "X"(속도 0, 토크 유지), torque_off_on_exit면 "O"(토크 OFF)
  /pan_tilt/joint_states  JointState  OpenCR 상태 줄 "S <ms> <pan_deg> <tilt_deg> <pan_dps> <tilt_dps> <state>"를
                position [rad]·velocity [rad/s]로. header.stamp = 줄을 받은 ROS 시각
dry_run:=true 이면 시리얼을 열지 않고 관절 각도를 명령 적분으로 흉내 낸다(모터 출력 끈 시험·bag 재현).
  이때 이름은 pan_sim·tilt_sim: target_detector(번호 유지 회전 보정)가 실제 카메라 자세로 착각하지 않게 한다.
"""
import math
import os
import time
from datetime import datetime

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3Stamped
from sensor_msgs.msg import JointState

from tracker_bridge.protocol import parse_status


class OpencrBridge(Node):
    def __init__(self):
        super().__init__('opencr_bridge')
        p = lambda name, default: self.declare_parameter(name, default).value  # noqa: E731
        self.port = p('serial_port', '/dev/ttyACM0')                   # #todo 실제 OpenCR 포트 (/dev/serial/by-id/... 권장)
        self.baud = int(p('serial_baud', 115200))
        self.dry_run = p('dry_run', False)
        rate = p('bridge_rate_hz', 50.0)                               # [Hz] 펌웨어 CMD_TIMEOUT_MS(300 ms)보다 충분히 짧게
        self.cmd_timeout = p('cmd_timeout_s', 0.2)                     # [s] 제어 명령이 이보다 오래되면 0 전송 #todo
        self.torque_off_on_exit = p('torque_off_on_exit', False)       # true: 종료 시 토크 OFF (틸트 처짐 주의)
        self.sim_limits = (p('sim_pan_limit_deg', 180.0), p('sim_tilt_limit_deg', 40.0))  # 펌웨어 LIMIT_DEG와 같게
        log_dir = os.path.expanduser(p('log_dir', '~/lv2_module5_logs'))
        run_id = p('run_id', '') or datetime.now().strftime('bridge_%Y%m%d_%H%M%S')
        os.makedirs(log_dir, exist_ok=True)
        self.serial_log = open(os.path.join(log_dir, f'{run_id}_serial.log'), 'w')

        self.t0 = time.monotonic()
        self.cmd, self.cmd_rx = (0.0, 0.0), None
        self.sim_joint = [0.0, 0.0]
        self.last_tick = None
        self.fw_state = None

        self.ser, self.rx_buf = None, b''
        if not self.dry_run:
            import serial  # python3-serial
            self.ser = serial.Serial(self.port, self.baud, timeout=0)

        self.create_subscription(Vector3Stamped, p('command_topic', '/pan_tilt/command'), self.on_command, 10)
        self.joint_pub = self.create_publisher(JointState, p('joint_state_topic', '/pan_tilt/joint_states'), 10)
        self.create_timer(1.0 / rate, self.on_timer)
        self.create_timer(1.0, self.serial_log.flush)   # kill -9·전원 차단 때도 직전까지 남게
        self.get_logger().info(f'run_id={run_id} dry_run={self.dry_run} port={self.port}')

    def on_command(self, msg):
        self.cmd, self.cmd_rx = (msg.vector.x, msg.vector.y), time.monotonic()

    def send(self, line):
        self.serial_log.write(f'{time.monotonic() - self.t0:.3f}\t>>> {line}\n')
        if self.ser:
            self.ser.write((line + '\n').encode())

    def read_serial(self):
        waiting = self.ser.in_waiting
        if waiting:
            self.rx_buf += self.ser.read(waiting)
        while b'\n' in self.rx_buf:
            raw, self.rx_buf = self.rx_buf.split(b'\n', 1)
            line = raw.decode(errors='replace').strip()
            if not line:
                continue
            self.serial_log.write(f'{time.monotonic() - self.t0:.3f}\t{line}\n')
            st = parse_status(line)
            if st:
                if st[4] != self.fw_state:
                    self.get_logger().info(f'OpenCR state {self.fw_state} -> {st[4]}')
                    self.fw_state = st[4]
                self.publish_joint(['pan', 'tilt'], st[0:2], st[2:4])
            elif line.startswith('E '):
                self.get_logger().error(f'OpenCR {line}')
            elif line.startswith('READY'):
                self.get_logger().info(f'OpenCR {line}')

    def publish_joint(self, names, pos_deg, vel_dps):
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()   # 영상 stamp와 같은 시계(ROS 시간)
        js.name = names
        js.position = [math.radians(v) for v in pos_deg]
        js.velocity = [math.radians(v) for v in vel_dps]
        self.joint_pub.publish(js)

    def on_timer(self):
        now = time.monotonic()
        dt = 0.0 if self.last_tick is None else now - self.last_tick
        self.last_tick = now
        fresh = self.cmd_rx is not None and now - self.cmd_rx < self.cmd_timeout
        pan, tilt = self.cmd if fresh else (0.0, 0.0)
        self.send(f'V {pan:.2f} {tilt:.2f}')
        if self.dry_run:   # 시뮬레이션: 명령을 적분 (펌웨어 각도 제한과 같게 자른다)
            for i, (c, lim) in enumerate(zip((pan, tilt), self.sim_limits)):
                self.sim_joint[i] = max(-lim, min(lim, self.sim_joint[i] + c * dt))
            self.publish_joint(['pan_sim', 'tilt_sim'], self.sim_joint, (pan, tilt))
        else:
            self.read_serial()

    def close(self):
        try:
            if self.ser:
                self.send('X')
                if self.torque_off_on_exit:
                    self.send('O')
                self.ser.flush()
                self.ser.close()
        finally:
            self.serial_log.close()


def main():
    rclpy.init()
    node = OpencrBridge()
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
