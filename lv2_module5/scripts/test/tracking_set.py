#!/usr/bin/env python3
"""추적 켜기/끄기: 컨트롤러가 /tracking_enable을 구독할 때까지 기다렸다 보내고, /tracking_status로 바뀐 것을 확인한다.
  python3 scripts/test/tracking_set.py on|off [--timeout 15]
종료 코드 0 = 확인됨, 1 = 컨트롤러 없음·상태가 안 바뀜.
"""
import argparse
import sys
import time

import rclpy
from std_msgs.msg import Bool, String

ap = argparse.ArgumentParser()
ap.add_argument('mode', choices=('on', 'off'))
ap.add_argument('--timeout', type=float, default=15.0)
a = ap.parse_args()
want = a.mode == 'on'

rclpy.init()
node = rclpy.create_node('tracking_set')
pub = node.create_publisher(Bool, '/tracking_enable', 10)
st = {'s': ''}
node.create_subscription(String, '/tracking_status', lambda m: st.__setitem__('s', m.data), 10)


def done():
    s = st['s']
    return (not s.startswith('IDLE')) if want else s.startswith('IDLE')


t0, ok, sent = time.time(), False, 0
while time.time() - t0 < a.timeout:
    rclpy.spin_once(node, timeout_sec=0.1)
    if pub.get_subscription_count() > 0:
        pub.publish(Bool(data=want))          # 같은 값을 반복해 보내도 안전
        sent += 1
        if st['s'] and done():
            ok = True
            break
print(f'추적 {"켜기" if want else "끄기"}: {"확인" if ok else "실패"} (상태 {st["s"] or "수신 없음"}, '
      f'구독 {pub.get_subscription_count()}, 전송 {sent}회, {time.time() - t0:.1f} s)')
node.destroy_node()
rclpy.shutdown()
sys.exit(0 if ok else 1)
