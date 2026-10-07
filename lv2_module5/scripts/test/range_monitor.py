#!/usr/bin/env python3
"""토크 OFF 상태에서 손으로 돌린 ID 11(수평)·12(수직)의 위치를 실시간 표시하고, 끝 위치를 키로 표시해 저장한다.

Pi의 SSH(tmux) 창에서 실행한다. OpenCR에는 motor_check 펌웨어('p' 명령으로 위치 출력)가 올라가 있어야 한다.
  a / d : ID 11 왼쪽 끝 / 오른쪽 끝 표시
  w / s : ID 12 위쪽 끝 / 아래쪽 끝 표시
  q     : 저장 후 종료 (Ctrl+C도 저장)
XM430은 토크 OFF에서 여러 바퀴를 누적해 센다. 그래서 각도는 tick 2048 기준 누적값(360도 넘을 수 있음)으로 보여 준다.
모터 설정(EEPROM)은 읽기만 하고 바꾸지 않는다.
"""
import csv
import os
import re
import select
import sys
import termios
import time
import tty

import serial

PORT = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyACM0"
LOG_DIR = os.path.expanduser("~/lv2_module5_logs")
RUN = time.strftime("range_%Y%m%d_%H%M%S")
LINE = re.compile(r"id=(\d+) mode=(-?\d+) torque=(-?\d+) pos=(-?\d+)")
KEYS = {"a": (11, "pan_left"), "d": (11, "pan_right"), "w": (12, "tilt_up"), "s": (12, "tilt_down")}


def deg(tick):
    return (tick - 2048) * 360.0 / 4096.0


def read_positions(ser):
    ser.write(b"p")
    got = {}
    deadline = time.time() + 0.3
    while time.time() < deadline and len(got) < 2:
        m = LINE.search(ser.readline().decode(errors="replace"))
        if m:
            got[int(m.group(1))] = (int(m.group(3)), int(m.group(4)))
    return got


os.makedirs(LOG_DIR, exist_ok=True)
ser = serial.Serial(PORT, 115200, timeout=0.2)
time.sleep(0.5)
ser.reset_input_buffer()
ser.write(b"x")  # 시작 전에 토크 OFF 보장
rows, marks = [], {}
t0 = time.time()
old_tty = termios.tcgetattr(sys.stdin)
tty.setcbreak(sys.stdin.fileno())
print("a/d: 수평 왼쪽/오른쪽 끝, w/s: 수직 위/아래 끝, q: 저장 후 종료\n")
try:
    while True:
        got = read_positions(ser)
        now = round(time.time() - t0, 2)
        for mid, (torque, pos) in got.items():
            rows.append((now, mid, torque, pos, round(deg(pos), 1)))
        key = sys.stdin.read(1) if select.select([sys.stdin], [], [], 0)[0] else ""
        if key == "q":
            break
        if key in KEYS and KEYS[key][0] in got:
            mid, name = KEYS[key]
            marks[name] = got[mid][1]
            print(f"\n[표시] {name} = tick {got[mid][1]} ({deg(got[mid][1]):+.1f}°)")
        parts = []
        for mid in (11, 12):
            if mid in got:
                parts.append(f"ID{mid}: tick {got[mid][1]:6d} ({deg(got[mid][1]):+7.1f}°) 토크={got[mid][0]}")
            else:
                parts.append(f"ID{mid}: 응답 없음")
        print("\r" + " | ".join(parts) + "   ", end="", flush=True)
        time.sleep(0.1)
except KeyboardInterrupt:
    pass
finally:
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_tty)

with open(os.path.join(LOG_DIR, RUN + ".csv"), "w", newline="") as f:
    w = csv.writer(f)
    w.writerow(["time_s", "id", "torque", "pos_tick", "deg_from_2048"])
    w.writerows(rows)
with open(os.path.join(LOG_DIR, RUN + "_marks.txt"), "w") as f:
    for name in ("pan_left", "pan_right", "tilt_up", "tilt_down"):
        if name in marks:
            f.write(f"{name} tick={marks[name]} deg={deg(marks[name]):+.1f}\n")
        else:
            f.write(f"{name} 표시 안 함\n")
print(f"\nRANGE_SAVED {LOG_DIR}/{RUN}_marks.txt")
