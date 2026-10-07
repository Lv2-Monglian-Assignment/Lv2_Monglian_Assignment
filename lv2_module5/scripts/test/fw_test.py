#!/usr/bin/env python3
"""opencr_tracker 펌웨어 안전 정지 단독 시험 (Pi의 SSH·tmux 창에서 실행, ROS 없이 시리얼만 사용).

제어 노드·브리지·시리얼 모니터는 끈 상태에서 실행한다. 방향 확인은 scripts/test/direction_test.py.
  python3 scripts/test/fw_test.py                    # 상태 확인 → 보드 타임아웃(300 ms) → X 정지
  python3 scripts/test/fw_test.py --hold-check      # + 명령 없이 6 s 두어 토크가 유지되는지(HOLD) 확인
  python3 scripts/test/fw_test.py --home             # + I(기준 자세 복귀) 확인
  python3 scripts/test/fw_test.py --off              # O(토크 OFF)만 보내고 끝
각 단계 전에 Enter를 기다린다. 기다리는 동안에도 별도 스레드가 시리얼을 계속 읽는다.
연결이 끊기면 펌웨어는 300 ms 뒤 속도 0으로 멈추고 토크는 유지한다(2026-10-06 팀 결정). Ctrl+C: X(정지, 토크 유지).
모든 송수신은 ~/lv2_module5_logs/fw_test_<시각>.log 에 시각과 함께 남는다.
"""
import argparse
import os
import re
import threading
import time

import serial

ap = argparse.ArgumentParser(description='opencr_tracker 안전 정지 시험')
ap.add_argument('--port', default='/dev/ttyACM0')
ap.add_argument('--hold-check', action='store_true', help='명령 없이 6 s 두어 토크 유지(HOLD) 확인')
ap.add_argument('--home', action='store_true', help='I(기준 자세 복귀) 확인')
ap.add_argument('--off', action='store_true', help='O(토크 OFF)만 보내고 끝')
args = ap.parse_args()

LOG_DIR = os.path.expanduser('~/lv2_module5_logs')
os.makedirs(LOG_DIR, exist_ok=True)
log = open(os.path.join(LOG_DIR, time.strftime('fw_test_%Y%m%d_%H%M%S.log')), 'w')
log_lock, write_lock = threading.Lock(), threading.Lock()
t0 = time.time()
STATUS = re.compile(r'^S (\d+) (-?[\d.]+) (-?[\d.]+) (-?[\d.]+) (-?[\d.]+) (OFF|HOLD|TRACK|HOMING|FAULT)$')
st = {'pos': None, 'state': None, 'stamp': 0.0, 'events': []}   # events: (시각, 'E ...' 줄)
stop = threading.Event()
keepalive_on = threading.Event()


def record(tag, text):
    with log_lock:
        log.write(f'{time.time() - t0:9.3f} {tag} {text}\n')
        log.flush()


def reader(ser):
    while not stop.is_set():
        line = ser.readline().decode(errors='replace').strip()
        if not line:
            continue
        record('<<', line)
        m = STATUS.match(line)
        if m:
            if m.group(6) != st['state']:
                print(f'  상태 {st["state"]} → {m.group(6)}')
            st.update(pos=(float(m.group(2)), float(m.group(3))), state=m.group(6), stamp=time.time())
        else:
            st['events'].append((time.time(), line))
            print('  OpenCR:', line)


def send(ser, text):
    with write_lock:
        ser.write(text.encode())
    record('>>', text.strip())


def keepalive(ser):
    while not stop.is_set():
        if keepalive_on.is_set() and st['state'] in ('HOLD', 'TRACK', 'HOMING'):   # HOMING도 300 ms 타임아웃이 걸림
            send(ser, 'V 0.00 0.00\n')
        time.sleep(0.1)


def wait_status(after, timeout=1.0):
    while time.time() - after < timeout:
        if st['stamp'] > after:
            return st['pos']
        time.sleep(0.01)
    return None


def step(title):
    input(f'\n▶ {title}\n  Enter를 누르면 실행 (중단: Ctrl+C) ')
    record('##', title)


ser = serial.Serial(args.port, 115200, timeout=0.05)
threading.Thread(target=reader, args=(ser,), daemon=True).start()
threading.Thread(target=keepalive, args=(ser,), daemon=True).start()
send(ser, '\n')            # 포트를 열 때 섞이는 잡음 바이트를 줄바꿈으로 비운다(안 하면 첫 명령이 'bad command')
time.sleep(0.2)
try:
    if wait_status(time.time()) is None:
        raise SystemExit('상태 줄(S ...)이 오지 않습니다. opencr_tracker 펌웨어·포트·다른 프로그램 사용 여부를 확인하세요.')
    print(f'상태: {st["state"]}  각도: 팬 {st["pos"][0]:.1f}°, 틸트 {st["pos"][1]:.1f}° (IDLE 기준)')
    if st['state'] == 'FAULT':
        raise SystemExit('FAULT 상태입니다. OpenCR RESET(또는 재업로드) 후 다시 실행하세요.')
    if args.off:
        step('O: 토크 OFF (카메라를 손으로 받쳐 주세요)')
        send(ser, 'O\n')
        time.sleep(0.5)
        raise SystemExit(0)

    step("보드 타임아웃: 'V 5 0'을 한 번만 보내고 1초 동안 아무 명령도 안 보냄 (토크가 꺼져 있으면 이때 켜짐)")
    keepalive_on.clear()
    before = wait_status(time.time())
    sent = time.time()
    send(ser, 'V 5.00 0.00\n')
    time.sleep(1.0)
    after = wait_status(time.time())
    timeout_evt = [t for t, line in st['events'] if t > sent and 'command timeout; stop' in line]
    moved = after[0] - before[0]
    ms = (timeout_evt[0] - sent) * 1000 if timeout_evt else None
    ok = ms is not None and abs(moved) < 3.0 and st['state'] == 'HOLD'
    print(f'  timeout 알림: {"%.0f ms 후" % ms if ms else "없음"} / 팬 이동 {moved:+.2f}° / 상태 {st["state"]}'
          f'  → {"PASS" if ok else "CHECK"} (기대: 약 300 ms, 5°/s x 0.3 s ≈ 1.5°, HOLD)')
    record('##', f'timeout_ms={ms and round(ms)} pan_moved_deg={moved:+.2f} state={st["state"]} verdict={"PASS" if ok else "CHECK"}')
    keepalive_on.set()

    if args.home:
        step('I: 기준 자세(팬 0°, 틸트 0°)로 복귀 (최대 30°/s, 15 s 제한)')
        sent = time.time()
        send(ser, 'I\n')
        while time.time() - sent < 1.0 and st['state'] != 'HOMING':   # 상태 줄에 HOMING이 보일 때까지
            time.sleep(0.02)
        while time.time() - sent < 16 and st['state'] == 'HOMING':
            time.sleep(0.05)
        print(f'  복귀 후 각도: 팬 {st["pos"][0]:.2f}°, 틸트 {st["pos"][1]:.2f}° (허용 0.5°)')
        record('##', f'home pos={st["pos"]} state={st["state"]}')

    if args.hold_check:
        step('토크 유지: 명령을 끊고 6초 기다림 (기대: HOLD 유지, 토크 OFF 알림 없음, 각도 변화 1° 이하)')
        keepalive_on.clear()
        silent = time.time()
        start = wait_status(silent)
        time.sleep(6.0)
        off_evt = [t for t, line in st['events'] if t > silent and 'torque off' in line]
        drift = max(abs(a - b) for a, b in zip(st['pos'], start))
        ok = not off_evt and st['state'] == 'HOLD' and drift <= 1.0
        print(f'  상태 {st["state"]} / 토크 OFF 알림 {"있음" if off_evt else "없음"} / 각도 변화 {drift:.2f}°  → {"PASS" if ok else "CHECK"}')
        record('##', f'hold_check state={st["state"]} torque_off_event={bool(off_evt)} drift_deg={drift:.2f} verdict={"PASS" if ok else "CHECK"}')
    send(ser, 'X\n')
    time.sleep(0.3)
    print('\n시험 끝. 속도 0, 토크 유지(HOLD).')
except KeyboardInterrupt:
    keepalive_on.clear()
    with write_lock:
        ser.write(b'X\n')
    record('>>', 'X (Ctrl+C)')
    time.sleep(0.3)
    print('\n중단: 정지 (토크 유지)')
finally:
    stop.set()
    print('기록:', log.name)
    with log_lock:
        log.close()
