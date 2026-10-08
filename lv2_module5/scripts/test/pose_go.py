#!/usr/bin/env python3
"""팬·틸트를 지정 자세로 천천히 옮긴다 (브리지만 떠 있을 때). /pan_tilt/joint_states를 보고 /pan_tilt/command로 P 제어.
  python3 scripts/test/pose_go.py --pan -10.02 --tilt 26.28 [--max-dps 15] [--tol 0.5] [--timeout 25]
  python3 scripts/test/pose_go.py --show      # 지금 자세만 출력
종료 코드 0 = 도착, 1 = 실패(각도 수신 없음·시간 초과). 끝나면 0 명령을 보내고 나간다.
"""
import argparse
import math
import sys
import time

import rclpy
from geometry_msgs.msg import Vector3Stamped
from sensor_msgs.msg import JointState

ap = argparse.ArgumentParser()
ap.add_argument('--pan', type=float)
ap.add_argument('--tilt', type=float)
ap.add_argument('--max-dps', type=float, default=15.0)
ap.add_argument('--kp', type=float, default=1.5)
ap.add_argument('--tol', type=float, default=0.5)
ap.add_argument('--timeout', type=float, default=25.0)
ap.add_argument('--show', action='store_true')
a = ap.parse_args()

rclpy.init()
node = rclpy.create_node('pose_go')
pub = node.create_publisher(Vector3Stamped, '/pan_tilt/command', 10)
cur = {}


def on_js(m):
    if list(m.name[:2]) == ['pan', 'tilt']:          # 실제 모터 각도만 (dry_run의 pan_sim은 무시)
        cur['p'] = (math.degrees(m.position[0]), math.degrees(m.position[1]))


node.create_subscription(JointState, '/pan_tilt/joint_states', on_js, 10)


def send(px, ty):
    m = Vector3Stamped()
    m.header.stamp = node.get_clock().now().to_msg()
    m.vector.x, m.vector.y = px, ty
    pub.publish(m)


t0 = time.time()
while 'p' not in cur and time.time() - t0 < 10:
    rclpy.spin_once(node, timeout_sec=0.1)
if 'p' not in cur:
    print('관절 각도를 받지 못했습니다 (브리지 확인)')
    sys.exit(1)
print(f'지금 자세: pan {cur["p"][0]:.2f}°  tilt {cur["p"][1]:.2f}°')
if a.show:
    sys.exit(0)

def lim(e):
    """P 제어 + 최소 2 deg/s (펌웨어 최소 단위 1.374 deg/s보다 작으면 움직이지 않음) + 최대 max_dps"""
    if abs(e) < a.tol:
        return 0.0
    return math.copysign(min(a.max_dps, max(2.0, a.kp * abs(e))), e)
ok, t0, last = False, time.time(), 0.0
while time.time() - t0 < a.timeout:
    rclpy.spin_once(node, timeout_sec=0.02)
    if time.time() - last < 0.02:                   # 50 Hz
        continue
    last = time.time()
    ep, et = a.pan - cur['p'][0], a.tilt - cur['p'][1]
    if abs(ep) < a.tol and abs(et) < a.tol:
        ok = True
        break
    send(lim(ep), lim(et))
    print(f'\r  이동 중: pan {cur["p"][0]:7.2f}° → {a.pan:.2f}°   tilt {cur["p"][1]:7.2f}° → {a.tilt:.2f}°   ', end='', flush=True)
for _ in range(10):
    send(0.0, 0.0)
    rclpy.spin_once(node, timeout_sec=0.02)
print(f'\n{"도착" if ok else "시간 초과"}: pan {cur["p"][0]:.2f}°  tilt {cur["p"][1]:.2f}°')
node.destroy_node()
rclpy.shutdown()
sys.exit(0 if ok else 1)
