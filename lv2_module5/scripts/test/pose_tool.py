#!/usr/bin/env python3
"""기준 자세 설정 도구: 키보드로 카메라를 돌려 정면·수평에 맞추고, 그 자세의 tick을 config/device.yaml에 저장한다.

Pi의 SSH·tmux 창에서 실행한다(ROS 없이 시리얼만 사용, 제어 launch·시리얼 모니터는 끈 상태). 펌웨어: firmware/opencr_tracker
  python3 scripts/test/pose_tool.py
키 (한글 입력 상태의 같은 자판도 됨, Enter 없이 바로 동작)
  a / d : 팬 왼쪽 / 오른쪽        w / s : 틸트 위 / 아래        (누르고 있으면 계속 움직임)
  스페이스 : 정지                  + / - : 속도 바꾸기 (5 · 15 · 30 deg/s)
  i : 기준 자세로 복귀 (I)        h : 지금 자세를 기준 자세로 저장 (H → device.yaml home_ticks)
  o : 토크 OFF (손으로 맞출 때, 카메라를 받칠 것). 이동 키를 누르면 다시 켜짐
  q : 끝내기 (속도 0, 토크 유지)
기준 자세는 장비마다 다르다. 저장한 home_ticks는 브리지(opencr_bridge)가 시작할 때마다 펌웨어에 B로 보낸다.
이동 방향(왼쪽·위)은 device.yaml의 pan_direction·tilt_direction으로 정한다(scripts/test/direction_test.py로 확인한 값).
모든 송수신은 ~/lv2_module5_logs/pose_tool_<시각>.log 에 남는다.
"""
import argparse
import os
import re
import select
import sys
import termios
import threading
import time
import tty

import serial

HERE = os.path.dirname(os.path.abspath(__file__))
ap = argparse.ArgumentParser(description='기준 자세 설정 도구')
ap.add_argument('--port', default='/dev/ttyACM0')
ap.add_argument('--device-yaml', default=os.path.join(HERE, '..', '..', 'config', 'device.yaml'))
args = ap.parse_args()

HANGUL = {'ㅁ': 'a', 'ㅇ': 'd', 'ㅈ': 'w', 'ㄴ': 's', 'ㅑ': 'i', 'ㅗ': 'h', 'ㅐ': 'o', 'ㅂ': 'q'}
SPEEDS = (5.0, 15.0, 30.0)
HOLD_S = 0.2            # 키 한 번에 움직이는 시간 [s]. 누르고 있으면 자동 반복으로 계속 갱신
STATUS = re.compile(r'^S (\d+) (-?[\d.]+) (-?[\d.]+) (-?[\d.]+) (-?[\d.]+) (OFF|HOLD|TRACK|HOMING|FAULT)$')
HOME_RE = re.compile(r'^\s*home_ticks:\s*\[\s*(-?\d+)\s*,\s*(-?\d+)\s*\]', re.M)

LOG_DIR = os.path.expanduser('~/lv2_module5_logs')
os.makedirs(LOG_DIR, exist_ok=True)
log = open(os.path.join(LOG_DIR, time.strftime('pose_tool_%Y%m%d_%H%M%S.log')), 'w')
lock = threading.Lock()
t0 = time.time()
st = {'pos': (float('nan'), float('nan')), 'state': '?', 'home': None, 'home_at': 0.0, 'msg': ''}
stop = threading.Event()


def record(tag, text):
    with lock:
        log.write(f'{time.time() - t0:9.3f} {tag} {text}\n')
        log.flush()


def read_yaml():
    """direction은 설정 폴더 전체(main 배치: control.yaml /**)에서, home_ticks는 device.yaml에서 읽는다."""
    cfg_dir = os.path.dirname(os.path.abspath(args.device_yaml))
    text = ''.join(open(os.path.join(cfg_dir, n)).read() for n in sorted(os.listdir(cfg_dir)) if n.endswith('.yaml'))
    d = {k: int(m.group(1)) for k in ('pan_direction', 'tilt_direction')
         for m in [re.search(rf'^\s*{k}:\s*(-?\d+)', text, re.M)] if m}
    m = HOME_RE.search(open(args.device_yaml).read())
    return d.get('pan_direction', -1), d.get('tilt_direction', 1), (int(m.group(1)), int(m.group(2))) if m else None


def save_home(ticks):
    text = open(args.device_yaml).read()
    if not HOME_RE.search(text):
        return False
    text = HOME_RE.sub(lambda m: m.group(0).split('[')[0] + f'[{ticks[0]}, {ticks[1]}]', text, count=1)
    open(args.device_yaml, 'w').write(text)
    return True


def reader(ser):
    buf = b''
    while not stop.is_set():
        buf += ser.read(256)
        while b'\n' in buf:
            raw, buf = buf.split(b'\n', 1)
            line = raw.decode(errors='replace').strip()
            if not line:
                continue
            m = STATUS.match(line)
            if m:
                st['pos'], st['state'] = (float(m.group(2)), float(m.group(3))), m.group(6)
                continue
            record('<<', line)
            if line.startswith('B '):
                parts = line.split()
                st['home'], st['home_at'] = (int(parts[1]), int(parts[2])), time.time()
            elif line.startswith('E '):
                st['msg'] = f'OpenCR {line}'


def send(ser, text):
    ser.write((text + '\n').encode())
    record('>>', text)


pan_dir, tilt_dir, saved_home = read_yaml()
ser = serial.Serial(args.port, 115200, timeout=0.02)
threading.Thread(target=reader, args=(ser,), daemon=True).start()
send(ser, '')                     # 포트를 열 때 섞이는 잡음 바이트를 비운다
time.sleep(0.2)
if saved_home and min(saved_home) >= 0:
    send(ser, f'B {saved_home[0]} {saved_home[1]}')   # 저장된 기준 자세를 먼저 적용 (i가 이 자세로 돌아가게)
print(__doc__.split('키 (')[1].split('기준 자세는')[0].replace('한글 입력 상태의 같은 자판도 됨, Enter 없이 바로 동작)', ''))
print(f'device.yaml: home_ticks {list(saved_home) if saved_home else "없음"}, pan_direction {pan_dir:+d}, tilt_direction {tilt_dir:+d}')

old_tty = termios.tcgetattr(sys.stdin)
speed_i, move, until = 1, (0.0, 0.0), 0.0
try:
    tty.setcbreak(sys.stdin.fileno())
    while True:
        now = time.time()
        if select.select([sys.stdin], [], [], 0.02)[0]:
            k = sys.stdin.read(1)
            k = HANGUL.get(k, k.lower())
            speed = SPEEDS[speed_i]
            moves = {'a': (-pan_dir * speed, 0.0), 'd': (pan_dir * speed, 0.0),     # 왼쪽 = 오른쪽의 반대
                     'w': (0.0, -tilt_dir * speed), 's': (0.0, tilt_dir * speed)}   # 아래 = tilt_direction
            if k in moves:
                move, until = moves[k], now + HOLD_S
            elif k == ' ':
                move, until = (0.0, 0.0), 0.0
                send(ser, 'X')
            elif k in '+=':
                speed_i = min(speed_i + 1, len(SPEEDS) - 1)
            elif k == '-':
                speed_i = max(speed_i - 1, 0)
            elif k == 'i':
                move, until = (0.0, 0.0), 0.0
                send(ser, 'I')
            elif k == 'o':
                move, until = (0.0, 0.0), 0.0
                send(ser, 'O')
                st['msg'] = '토크 OFF: 손으로 맞춘 뒤 h로 저장'
            elif k == 'h':
                move, until = (0.0, 0.0), 0.0
                if st['state'] not in ('OFF', 'HOLD'):
                    send(ser, 'X')
                    time.sleep(0.1)
                asked = time.time()
                send(ser, 'H')
                while time.time() - asked < 1.0 and st['home_at'] < asked:
                    time.sleep(0.02)
                if st['home_at'] >= asked and save_home(st['home']):
                    st['msg'] = f'저장: home_ticks {list(st["home"])} → {os.path.relpath(args.device_yaml)}'
                    record('##', st['msg'])
                else:
                    st['msg'] = '저장 실패: 펌웨어 회신(B) 없음 또는 device.yaml에 home_ticks 줄 없음'
            elif k == 'q':
                break
        if now < until:
            send(ser, f'V {move[0]:.2f} {move[1]:.2f}')
        elif move != (0.0, 0.0):
            move = (0.0, 0.0)
            send(ser, 'X')
        elif st['state'] == 'HOMING':
            send(ser, 'V 0.00 0.00')   # 기준 자세 복귀 중 생존 신호 (300 ms 명령 없으면 펌웨어가 복귀를 멈춤)
        pan, tilt = st['pos']
        home = list(st['home']) if st['home'] else '?'
        sys.stdout.write(f'\r[{st["state"]:6s}] 팬 {pan:+7.2f}°  틸트 {tilt:+7.2f}°  속도 {SPEEDS[speed_i]:4.0f}°/s  '
                         f'기준 tick {home}  {st["msg"][:60]:60s}')
        sys.stdout.flush()
        time.sleep(0.02)
finally:
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_tty)
    send(ser, 'X')
    time.sleep(0.2)
    stop.set()
    print('\n끝: 속도 0, 토크 유지. 기록:', log.name)
    with lock:
        log.close()
