#!/usr/bin/env python3
"""모터 방향 정의 시험: 속도 명령 부호와 실제 카메라 회전 방향을 확인해 device.yaml의 direction 값을 정한다.

Pi의 SSH·tmux 창에서 실행한다(ROS 없이 시리얼만 사용, 제어 노드·브리지·시리얼 모니터는 끈 상태).
  python3 scripts/test/direction_test.py                    # 펌웨어 자동 판별, 15 deg/s로 2초씩 (약 30°, 회전이 눈에 보이는 크기)
  python3 scripts/test/direction_test.py --speed 10 --seconds 2 --port /dev/ttyACM0

순서: 상태 확인 → (opencr_pan_tilt면 E로 토크 on) → 팬 + → 팬 −(제자리) → 틸트 + → 틸트 −(제자리) → 정지.
움직일 때마다 카메라 뒤에서(카메라가 보는 방향으로) 본 방향을 키로 입력한다.
  팬: l = 왼쪽, r = 오른쪽, n = 안 움직임        틸트: u = 위, d = 아래, n = 안 움직임
판정
  pan_direction : 오른쪽 목표(ex > 0)를 줄이려면 카메라가 오른쪽으로 돌아야 한다 → 팬 +가 왼쪽이면 −1, 오른쪽이면 +1
  tilt_direction: 아래 목표(ey > 0)를 줄이려면 카메라가 아래로 돌아야 한다 → 틸트 +가 아래면 +1, 위면 −1
  tick 부호     : + 명령에 tick(각도)이 늘어야 한다. 반대면 펌웨어·모터 설정(Drive Mode 등)을 확인
결과를 config/*.yaml의 direction 값(main 배치에서는 control.yaml)과 비교해 보여 준다(파일은 고치지 않는다).
지원 펌웨어: firmware/opencr_tracker (V/X/O, 상태 줄 S, 첫 V에서 토크 on), 이전 opencr_pan_tilt (E/V/S/?/x)
Ctrl+C·정상 종료 모두 속도 0, 토크 유지(카메라가 처지지 않게). 이전 펌웨어 opencr_pan_tilt는 Ctrl+C 때 토크 off.
모든 송수신과 판정은 ~/lv2_module5_logs/direction_test_<시각>.log 에 남는다.
"""
import argparse
import os
import re
import sys
import threading
import time

import serial

DEG_PER_TICK = 360.0 / 4096.0
SEND_HZ = 50.0
HERE = os.path.dirname(os.path.abspath(__file__))

ap = argparse.ArgumentParser(description='모터 방향 정의 시험')
ap.add_argument('--port', default='/dev/ttyACM0')
ap.add_argument('--speed', type=float, default=15.0, help='시험 속도 [deg/s] (기본 15)')
ap.add_argument('--seconds', type=float, default=2.0, help='한 방향 이동 시간 [s] (기본 2)')
ap.add_argument('--config-dir', default=os.path.join(HERE, '..', '..', 'config'), help='direction 값을 찾을 설정 폴더 (*.yaml)')
args = ap.parse_args()
if not (0 < args.speed <= 30 and 0 < args.seconds <= 5 and args.speed * args.seconds <= 35):
    sys.exit('속도 0~30 deg/s, 시간 0~5 s, 한 번 이동 35° 이하로 지정하세요(틸트 범위 ±40° 보호).')

LOG_DIR = os.path.expanduser('~/lv2_module5_logs')
os.makedirs(LOG_DIR, exist_ok=True)
log = open(os.path.join(LOG_DIR, time.strftime('direction_test_%Y%m%d_%H%M%S.log')), 'w')
log_lock = threading.Lock()
t0 = time.time()
stop_reader = threading.Event()
jogging = threading.Event()
write_lock = threading.Lock()
# 펌웨어 상태: fw = 'pan_tilt' | 'tracker', pos = 기준 자세에서 잰 (팬, 틸트) [deg, tick 증가 방향 +], state = 펌웨어 상태 이름
# (opencr_pan_tilt는 이전 기준 tick 0·2048, opencr_tracker는 펌웨어 IDLE_TICKS 기준)
st = {'fw': None, 'pos': None, 'state': None, 'stamp': 0.0}
PAN_TILT_RE = re.compile(r'^ST\t.*state:(\w+).*pan_tick:(-?\d+).*tilt_tick:(-?\d+)')
TRACKER_RE = re.compile(r'^S (\d+) (-?[\d.]+) (-?[\d.]+) (-?[\d.]+) (-?[\d.]+) (OFF|HOLD|TRACK|HOMING|FAULT)$')


def record(tag, text):
    with log_lock:
        log.write(f'{time.time() - t0:9.3f} {tag} {text}\n')
        log.flush()


def reader(ser):
    """시리얼을 쉬지 않고 읽는다(안 읽으면 OpenCR 송신 버퍼가 차서 제어 루프가 늦어진다 — 2026-10-05 FAULT 원인)."""
    while not stop_reader.is_set():
        line = ser.readline().decode(errors='replace').strip()
        if not line:
            continue
        record('<<', line)
        m = PAN_TILT_RE.match(line)
        if m:
            st.update(fw='pan_tilt', state=m.group(1),
                      pos=(int(m.group(2)) * DEG_PER_TICK, (int(m.group(3)) - 2048) * DEG_PER_TICK), stamp=time.time())
            if 'fault:' in line:
                print('  OpenCR FAULT 원인:', line.split('fault:')[1])
            continue
        m = TRACKER_RE.match(line)
        if m:
            st.update(fw='tracker', state=m.group(6), pos=(float(m.group(2)), float(m.group(3))), stamp=time.time())
            continue
        print('  OpenCR:', line)


def send(ser, text):
    with write_lock:
        ser.write(text.encode())
    record('>>', text.strip() or repr(text))


def keepalive(ser):
    """입력을 기다리는 동안 생존 신호(V 0 0)를 보낸다(펌웨어 TORQUE_OFF_AFTER_MS를 다시 켜도 토크가 꺼지지 않게)."""
    while not stop_reader.is_set():
        if st['fw'] == 'tracker' and st['state'] != 'OFF' and not jogging.is_set():
            send(ser, 'V 0.00 0.00\n')
        time.sleep(0.1)


def position(ser):
    """명령 뒤에 도착한 최신 각도 [deg]. opencr_pan_tilt는 '?'로 묻고, opencr_tracker는 50 Hz 상태 줄을 기다린다."""
    asked = time.time()
    if st['fw'] != 'tracker':
        send(ser, '?\n')
    while time.time() - asked < 0.6:
        if st['stamp'] > asked:
            return st['pos']
        time.sleep(0.01)
    return None


def stop_cmd(ser):
    send(ser, 'S\n' if st['fw'] == 'pan_tilt' else 'X\n')   # 속도 0, 토크 유지


def jog(ser, pan, tilt):
    jogging.set()
    try:
        end = time.time() + args.seconds
        while time.time() < end:
            send(ser, f'V {pan:.2f} {tilt:.2f}\n')
            time.sleep(1.0 / SEND_HZ)
        stop_cmd(ser)
    finally:
        jogging.clear()
    time.sleep(0.5)   # 감속·정지 후 위치를 읽는다


HANGUL_KEYS = {'ㅣ': 'l', 'ㄱ': 'r', 'ㅕ': 'u', 'ㅇ': 'd', 'ㅜ': 'n'}   # 한글 입력 상태에서 같은 자판 위치


def ask(prompt, choices):
    while True:
        a = input(f'  ★ {prompt} [{"/".join(choices)}] ').strip().lower()
        a = HANGUL_KEYS.get(a, a)
        if a in choices:
            record('##', f'{prompt} -> {a}')
            return a
        print('    다시 입력하세요:', ', '.join(choices))


def step(title):
    input(f'\n▶ {title}\n  Enter를 누르면 실행 (중단: Ctrl+C) ')
    record('##', title)


def config_directions(config_dir):
    """설정 폴더의 *.yaml에서 pan_direction·tilt_direction과 그 파일 이름을 찾는다(main 배치: control.yaml의 /**)."""
    vals = {}
    for name in sorted(os.listdir(config_dir)) if os.path.isdir(config_dir) else []:
        if not name.endswith('.yaml'):
            continue
        text = open(os.path.join(config_dir, name)).read()
        for key in ('pan_direction', 'tilt_direction'):
            m = re.search(rf'^\s*{key}:\s*(-?\d+)', text, re.M)
            if m and key not in vals:
                vals[key] = (int(m.group(1)), name)
    return vals if len(vals) == 2 else None


ser = serial.Serial(args.port, 115200, timeout=0.05)
threading.Thread(target=reader, args=(ser,), daemon=True).start()
threading.Thread(target=keepalive, args=(ser,), daemon=True).start()
send(ser, '\n')            # 포트를 열 때 섞일 수 있는 잡음 바이트를 줄바꿈으로 비운다
time.sleep(0.4)            # opencr_tracker면 이 사이에 상태 줄이 들어와 판별된다
results = {}
try:
    if position(ser) is None:
        sys.exit('OpenCR 응답이 없습니다. 포트·전원·다른 프로그램이 포트를 쓰는지 확인하세요.')
    fw_name = {'pan_tilt': 'opencr_pan_tilt', 'tracker': 'opencr_tracker'}[st['fw']]
    print(f'펌웨어: {fw_name}  상태: {st["state"]}  기준 자세에서: 팬 {st["pos"][0]:.1f}°, 틸트 {st["pos"][1]:.1f}°')
    record('##', f'firmware={fw_name} state={st["state"]}')
    if st['state'] == 'FAULT':
        sys.exit('FAULT 상태입니다. OpenCR RESET(또는 재업로드) 후 다시 실행하세요.')

    if st['fw'] == 'pan_tilt':
        step('E: 토크 on (팬·틸트 + 연결된 13·14·15 고정)')
        send(ser, 'E\n')
        time.sleep(0.5)
    else:
        print('\nopencr_tracker는 첫 V 명령에서 토크가 켜집니다(IDLE에서 팬 170°·틸트 45°보다 멀면 거부).')

    move = args.speed * args.seconds
    for key, pan, tilt, prompt, choices in (
        ('pan+', args.speed, 0.0, '카메라가 어느 쪽으로 돌았나요? 왼쪽 l / 오른쪽 r / 안 움직임 n', ('l', 'r', 'n')),
        ('pan-', -args.speed, 0.0, None, None),
        ('tilt+', 0.0, args.speed, '카메라가 어느 쪽을 보게 됐나요? 위 u / 아래 d / 안 움직임 n', ('u', 'd', 'n')),
        ('tilt-', 0.0, -args.speed, None, None),
    ):
        axis = 0 if key.startswith('pan') else 1
        step(f'{key}: V {pan:+.1f} {tilt:+.1f} deg/s로 {args.seconds:.1f}초 (약 {move:.0f}°)'
             + ('' if prompt else ' — 제자리로 돌아오기'))
        before = position(ser)
        jog(ser, pan, tilt)
        after = position(ser)
        if before is None or after is None:
            sys.exit('각도를 읽지 못했습니다. 기록 파일을 확인하세요.')
        delta = after[axis] - before[axis]
        print(f'  {"팬" if axis == 0 else "틸트"} 각도 변화 {delta:+.1f}° (tick 증가 방향 +), 다른 축 {after[1 - axis] - before[1 - axis]:+.1f}°')
        record('##', f'{key} before={before} after={after} delta={delta:+.2f}')
        results[key] = {'delta': delta, 'seen': ask(prompt, choices) if prompt else None}

    stop_cmd(ser)
    time.sleep(0.3)

    # ---- 판정 ----
    print('\n========== 판정 ==========')
    ok = True
    for key, name in (('pan+', '팬'), ('tilt+', '틸트')):
        d = results[key]['delta']
        if abs(d) < 0.3 * move:
            print(f'✗ {name} + 명령에 {d:+.1f}°만 움직임 (기대 약 {move:.0f}°): 토크·전원·속도 제한 확인')
            ok = False
        elif d < 0:
            print(f'✗ {name} + 명령에 tick이 줄어듦 ({d:+.1f}°): 펌웨어 속도 부호 또는 모터 Drive Mode 확인')
            ok = False
        else:
            print(f'✓ {name} + 명령 → tick 증가 ({d:+.1f}°)')
    seen_pan, seen_tilt = results['pan+']['seen'], results['tilt+']['seen']
    pan_dir = {'l': -1, 'r': 1}.get(seen_pan)
    tilt_dir = {'d': 1, 'u': -1}.get(seen_tilt)
    if pan_dir is None or tilt_dir is None:
        print('✗ 움직임이 안 보인 축이 있어 direction을 정할 수 없습니다.')
        ok = False
    else:
        print(f'팬 + = {"왼쪽" if seen_pan == "l" else "오른쪽"}  → pan_direction = {pan_dir:+d}')
        print(f'틸트 + = {"아래" if seen_tilt == "d" else "위"}  → tilt_direction = {tilt_dir:+d}')
        cfg = config_directions(args.config_dir)
        if cfg:
            for key, val in (('pan_direction', pan_dir), ('tilt_direction', tilt_dir)):
                same = cfg[key][0] == val
                ok = ok and same
                print(f'{"✓" if same else "✗"} config/{cfg[key][1]} {key}: {cfg[key][0]} '
                      + ('(일치)' if same else f'→ {val:+d}로 바꿔야 함'))
        else:
            print(f'(설정에서 direction을 찾지 못해 비교 생략: {args.config_dir})')
    verdict = 'PASS' if ok else 'CHECK'
    print(f'결과: {verdict}')
    record('##', f'verdict={verdict} pan_direction={pan_dir} tilt_direction={tilt_dir} results={results}')
    print('토크는 켜진 상태(속도 0)로 유지합니다.')
except KeyboardInterrupt:
    stop_reader.set()   # 생존 신호를 멈춘다
    with write_lock:
        ser.write(b'X\n' if st['fw'] == 'tracker' else b'x')
    record('>>', 'stop (Ctrl+C)')
    time.sleep(0.3)
    print('\n중단: 정지' + (' (토크 유지)' if st['fw'] == 'tracker' else ' 후 토크 off'))
finally:
    stop_reader.set()
    print('기록:', log.name)
    with log_lock:
        log.close()
