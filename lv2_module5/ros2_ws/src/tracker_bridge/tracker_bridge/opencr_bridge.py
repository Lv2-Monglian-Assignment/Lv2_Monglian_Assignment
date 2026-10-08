#!/usr/bin/env python3
"""OpenCR 시리얼 브리지 노드 (라즈베리파이에서 실행). 펌웨어: firmware/opencr_tracker (작업 이슈 #14, 프로토콜 #7).

입력
  /pan_tilt/command    Vector3Stamped  x=팬, y=틸트 [deg/s] (tracker_controller 50 Hz)
출력
  Pi -> OpenCR  "V <pan_dps> <tilt_dps>" 50 Hz. 명령이 cmd_timeout_s 동안 안 오면 "V 0.00 0.00"
                (제어 노드가 죽어도 0을 보내 정지. 브리지가 죽으면 보드 타임아웃 300 ms가 정지시킨다)
                종료 시 "X"(속도 0, 토크 유지), torque_off_on_exit면 "O"(토크 OFF)
                시작 시 "B <pan_tick> <tilt_tick>": config/device.yaml의 home_ticks(장비마다 다른 기준 자세, scripts/test/pose_tool.py의 h 키로 저장)
                home_on_start면 이어서 "I"(기준 자세로 이동). 도착할 때까지 제어 명령 대신 "V 0.00 0.00"(생존 신호)만 보낸다
                FAULT면 2 s 뒤 "R"(복구)을 최대 3번 보내고, 그래도 FAULT면 수동 복구를 요청한다(2026-10-08 팀 결정).
                복구되면(R OK) home_on_start일 때 다시 "I"로 기준 자세로 이동해 토크를 켠다
  /pan_tilt/joint_states  JointState  OpenCR 상태 줄 "S <ms> <pan_deg> <tilt_deg> <pan_dps> <tilt_dps> <state>"를
                position [rad]·velocity [rad/s]로. header.stamp = 보드가 각도를 읽은 시각(보드 ms를 Pi ROS 시각으로 환산)
  /pan_tilt/board_state   String      보드 상태 OFF·HOLD·TRACK·HOMING·FAULT. 상태 줄이 board_silent_s 동안 없으면 NO_STATUS,
                자동 복구를 포기하면 FAULT_MANUAL (제어 노드가 받아 LOST로 표시하고 명령 0)
dry_run:=true 이면 시리얼을 열지 않고 관절 각도를 명령 적분으로 흉내 낸다(모터 출력 끈 시험·bag 재현).
  이때 이름은 pan_sim·tilt_sim: target_detector(번호 유지 회전 보정)가 실제 카메라 자세로 착각하지 않게 한다.
"""
import math
import os
import time
from datetime import datetime

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from geometry_msgs.msg import Vector3Stamped
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from tracker_bridge.protocol import BoardClock, FaultRetry, parse_status, sim_step, status_ms

HOME_WAIT_S = 20.0        # 시작 시 기준 자세 이동을 기다리는 최대 시간 (펌웨어 HOME_TIMEOUT 15 s + 여유)
HOME_START_S = 2.0        # 이 안에 HOMING이 시작되지 않으면(토크 켜기 거부 등) 기다리지 않는다


def keep_previous(path):
    """같은 이름의 기록이 있으면 지우지 않고 <이름>.prevN으로 옮긴다(같은 run_id를 다시 써도 이전 기록 보존)."""
    if os.path.exists(path):
        n = 1
        while os.path.exists(f'{path}.prev{n}'):
            n += 1
        os.rename(path, f'{path}.prev{n}')
    return path


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
        self.home_ticks = list(p('home_ticks', [-1, -1]))              # 기준 자세 tick. 음수면 펌웨어 기본값 사용
        self.sim_limits = (p('sim_pan_limit_deg', 180.0), p('sim_tilt_limit_deg', 40.0))  # 펌웨어 LIMIT_DEG와 같게
        self.home_on_start = p('home_on_start', True)                 # 시작 때 기준 자세로 이동 후 추적
        self.board_silent_s = p('board_silent_s', 0.5)                # 상태 줄이 이보다 오래 없으면 NO_STATUS
        log_dir = os.path.expanduser(p('log_dir', '~/lv2_module5_logs'))
        run_id = p('run_id', 'auto')
        run_id = run_id if run_id not in ('', 'auto') else datetime.now().strftime('bridge_%Y%m%d_%H%M%S')
        os.makedirs(log_dir, exist_ok=True)
        self.serial_log = open(keep_previous(os.path.join(log_dir, f'{run_id}_serial.log')), 'w')

        self.t0 = time.monotonic()
        self.cmd, self.cmd_rx = (0.0, 0.0), None
        self.sim_joint = [0.0, 0.0]
        self.last_tick = None
        self.fw_state = None
        self.last_status = None                 # 마지막 상태 줄을 받은 단조 시각
        self.silent_warned = False
        self.clock = BoardClock()
        self.retry = FaultRetry(max_tries=3, delay_s=2.0, healthy_s=60.0)
        self.manual_reported = False
        self.homing = None                      # 시작 시 기준 자세 이동: None(안 함·끝남) 또는 {'t0', 'seen'}

        self.ser, self.rx_buf = None, b''
        if not self.dry_run:
            import serial  # python3-serial
            try:
                self.ser = serial.Serial(self.port, self.baud, timeout=0)
            except Exception:
                self.serial_log.close()
                raise
            self.send('')                  # 포트를 열 때 섞이는 잡음 바이트를 줄바꿈으로 비운다
            if len(self.home_ticks) == 2 and min(self.home_ticks) >= 0:
                self.send(f'B {int(self.home_ticks[0])} {int(self.home_ticks[1])}')   # 회신 'B ...'를 기록에서 확인
            if self.home_on_start:
                self.send('I')             # 기준 자세로 이동 (토크가 꺼져 있으면 켠다. 170°·45° 밖이면 보드가 거부)
                self.homing = {'t0': time.monotonic(), 'seen': False}

        self.create_subscription(Vector3Stamped, p('command_topic', '/pan_tilt/command'), self.on_command, 10)
        self.joint_pub = self.create_publisher(JointState, p('joint_state_topic', '/pan_tilt/joint_states'), 10)
        self.state_pub = self.create_publisher(String, p('board_state_topic', '/pan_tilt/board_state'), 10)
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
            rx = time.monotonic()
            self.serial_log.write(f'{rx - self.t0:.3f}\t{line}\n')
            st = parse_status(line)
            if st:
                self.on_status(st, status_ms(line), rx)
            elif line.startswith('E '):
                self.get_logger().error(f'OpenCR {line}')
                if self.homing and line.startswith(('E 3', 'E 4', 'E 5')):   # 거부·FAULT·시간 초과: 기다리지 않는다
                    self.end_homing(f'중단 ({line})')
            elif line.startswith(('READY', 'B ', 'R ')):
                self.get_logger().info(f'OpenCR {line}')
                if line.startswith('R OK') and self.home_on_start:
                    # 복구 직후 보드는 토크 OFF: 기준 자세로 이동시켜 토크를 켠다(목표가 중앙이라 명령 0이면 계속 처지므로)
                    self.send('I')
                    self.homing = {'t0': time.monotonic(), 'seen': False}

    def on_status(self, st, board_ms, rx):
        state = st[4]
        self.last_status, self.silent_warned = rx, False
        if state != self.fw_state:
            self.get_logger().info(f'OpenCR state {self.fw_state} -> {state}')
            self.fw_state = state
        if self.homing:
            if state == 'HOMING':
                self.homing['seen'] = True
            elif self.homing['seen'] and state == 'HOLD':
                self.end_homing('도착')
        if self.retry.update(state, rx):
            self.get_logger().warning(f'OpenCR FAULT: 자동 복구 R ({self.retry.tries}/{self.retry.max_tries})')
            self.send('R')
        if self.retry.manual and not self.manual_reported:
            self.get_logger().error('OpenCR FAULT 반복: 자동 복구를 멈춥니다. 케이블·모터 전원을 확인한 뒤 '
                                    '추적을 다시 시작(브리지 재시작)하거나 OpenCR을 리셋하세요')
            self.manual_reported = True
        elif not self.retry.manual:
            self.manual_reported = False
        self.state_pub.publish(String(data='FAULT_MANUAL' if self.retry.manual and state == 'FAULT' else state))
        ros_now = self.get_clock().now().nanoseconds * 1e-9
        stamp = self.clock.stamp(rx, board_ms) + (ros_now - rx) if board_ms is not None else ros_now
        self.publish_joint(['pan', 'tilt'], st[0:2], st[2:4], stamp)

    def end_homing(self, why):
        self.get_logger().info(f'시작 시 기준 자세 이동 {why}: {time.monotonic() - self.homing["t0"]:.1f} s')
        self.homing = None

    def publish_joint(self, names, pos_deg, vel_dps, stamp_s=None):
        js = JointState()
        if stamp_s is None:
            js.header.stamp = self.get_clock().now().to_msg()   # 영상 stamp와 같은 시계(ROS 시간)
        else:   # 보드가 각도를 읽은 시각 (ROS 시간으로 환산)
            js.header.stamp.sec, js.header.stamp.nanosec = int(stamp_s), int((stamp_s % 1.0) * 1e9)
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
        if self.homing:
            waited = now - self.homing['t0']
            if waited > HOME_WAIT_S or (not self.homing['seen'] and waited > HOME_START_S):
                self.end_homing('시간 초과(추적 시작)')
            else:
                pan, tilt = 0.0, 0.0       # 이동 중: 0은 보드에서 생존 신호로만 쓰여 이동을 끊지 않는다
        self.send(f'V {pan:.2f} {tilt:.2f}')
        if self.dry_run:   # 시뮬레이션: 펌웨어와 같은 속도 상한·한계 감속으로 적분, 실제 움직인 속도를 보고
            vel = [0.0, 0.0]
            for i, (c, lim) in enumerate(zip((pan, tilt), self.sim_limits)):
                self.sim_joint[i], vel[i] = sim_step(self.sim_joint[i], c, dt, lim)
            self.publish_joint(['pan_sim', 'tilt_sim'], self.sim_joint, vel)
            self.state_pub.publish(String(data='SIM'))
        else:
            self.read_serial()
            if self.last_status is None or now - self.last_status > self.board_silent_s:
                self.state_pub.publish(String(data='NO_STATUS'))
                if not self.silent_warned and now - self.t0 > 2.0:
                    self.get_logger().warning(f'OpenCR 상태 줄이 {self.board_silent_s} s 넘게 없습니다 '
                                              '(보드 리셋·USB·펌웨어 확인)')
                    self.silent_warned = True

    def close(self):
        try:
            if self.ser:
                self.send('X')
                if self.torque_off_on_exit:
                    self.send('O')
                self.ser.flush()
                self.ser.close()
        except OSError as e:     # USB가 빠진 뒤 종료: 남은 정리를 계속한다(보드는 300 ms 타임아웃으로 정지)
            self.get_logger().warning(f'시리얼 닫기 실패: {e}')
        finally:
            self.serial_log.close()


def main():
    rclpy.init()
    node = OpencrBridge()
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
