#!/usr/bin/env python3
"""웹 관제 화면 (Pi에서 실행): 카메라(컨투어 포함) · 실시간 토픽 · 로그 · 문제 1~5 시험 버튼.

PC와 Pi의 DDS 설정(RMW·DOMAIN)이 달라도, 원본 640x480 rgb8(약 27 MB/s)을 Wi-Fi로 보내지 않고 볼 수 있게
Pi 안에서 토픽을 받아 JPEG·JSON으로 줄여 HTTP로 내보낸다.
화면은 target_detector와 같은 검출 함수·같은 config/*.yaml 값으로 다시 계산해 컨투어를 그린다(보기 전용, 깊이 없이).
추적 실행(tmux 'tracker')·시험 작업은 이 프로세스가 띄우고 멈춘다. 이 프로세스는 tmux 'webui'에서 돈다.

  python3 scripts/web_view.py [--port 8080] [--hz 8] [--start real|dry|none]
  PC 브라우저: http://monglian.local:8080/        (스트림만: /stream.mjpg, 한 장: /snapshot.jpg)
"""
import argparse
import collections
import glob
import json
import os
import shutil
import signal
import subprocess
import threading
import time
from datetime import datetime
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse

import cv2
import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import PointStamped, Vector3Stamped
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import GetParameters, SetParameters
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import Bool, String

from target_detector.detection import DetectorConfig, detect, draw_overlay

M5 = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))   # lv2_module5
CFG_DIR = os.path.join(M5, 'config')
LOG_DIR = os.path.expanduser('~/lv2_module5_logs')
BAG_DIR = os.path.expanduser('~/lv2_module5_bags')
SNAP_DIR = os.path.expanduser('~/lv2_module5_results/images')
ENV = (f'source /opt/ros/lyrical/setup.bash && source {M5}/ros2_ws/install/setup.bash '
       f'&& export ROS_DOMAIN_ID={os.environ.get("ROS_DOMAIN_ID", "28")}')
CTRL = '/tracker_controller'
# 슬라이더: (파라미터, 표시 이름, 최소, 최대, 간격). 범위는 controller_node.LIVE_PARAMS 안쪽
SLIDERS = [('pan_speed_limit_deg_s', '팬 최대 속도 [deg/s]', 0, 120, 1),
           ('pan_kp', '팬 Kp [deg/s / 오차]', 0, 120, 1),
           ('tilt_speed_limit_deg_s', '틸트 최대 속도 [deg/s]', 0, 120, 1),
           ('tilt_kp', '틸트 Kp [deg/s / 오차]', 0, 120, 1)]
BAG_TOPICS = ['/camera/camera/color/image_raw', '/camera/camera/aligned_depth_to_color/image_raw',
              '/camera/camera/color/camera_info', '/target', '/target/position_cam', '/target_depth',
              '/tracking_status', '/tracking_enable', '/pan_tilt/command', '/pan_tilt/joint_states']
BAG_MAX_S = 38.0          # ros2 CLI 시작 약 7 s + 녹화 약 30 s. 컬러+깊이 약 45 MB/s -> 약 1.4 GB. SD 여유가 적어 자동 정지
BAG_MIN_FREE_GB = 0.8     # 여유 공간이 이보다 적으면 녹화 정지·시작 거부


def sh(cmd, timeout=10):
    r = subprocess.run(['bash', '-c', cmd], capture_output=True, text=True, timeout=timeout)
    return r.stdout + r.stderr


def tracker_running():
    return subprocess.run(['tmux', 'has-session', '-t', 'tracker'], capture_output=True).returncode == 0


def free_gb():
    return shutil.disk_usage(os.path.expanduser('~')).free / 1e9


def newest(pattern):
    files = glob.glob(pattern)
    return max(files, key=os.path.getmtime) if files else None


def load_params():
    params = {}
    for name in sorted(os.listdir(CFG_DIR)):   # 노드와 같게: 공통(/**) + target_detector 값
        if name.endswith('.yaml'):
            doc = yaml.safe_load(open(os.path.join(CFG_DIR, name))) or {}
            for node in ('/**', 'target_detector'):
                params.update((doc.get(node) or {}).get('ros__parameters', {}))
    return params


class Proc:
    """백그라운드 작업 하나 (출력은 파일로). 프로세스 그룹째 멈춘다."""

    def __init__(self, log_name):
        self.log_path = os.path.join(LOG_DIR, log_name)
        self.p, self.name, self.t0 = None, '', 0.0

    def running(self):
        return self.p is not None and self.p.poll() is None

    def start(self, name, cmd):
        if self.running():
            raise RuntimeError(f'"{self.name}" 실행 중 — 끝나거나 멈춘 뒤 다시')
        log = open(self.log_path, 'w')
        log.write(f'# {datetime.now():%H:%M:%S} {name}\n$ {cmd}\n\n')
        log.flush()
        self.p = subprocess.Popen(['bash', '-c', f'{ENV} && {cmd}'], stdout=log, stderr=subprocess.STDOUT,
                                  start_new_session=True, cwd=M5)
        self.name, self.t0 = name, time.monotonic()

    def stop(self):
        if self.running():   # INT 먼저(bag 파일 정리), 3 s 뒤에도 남으면 TERM (bash 배경 작업은 INT를 무시)
            pid = self.p.pid
            os.killpg(pid, signal.SIGINT)
            threading.Timer(3.0, lambda: self.running() and self.p.pid == pid and os.killpg(pid, signal.SIGTERM)).start()

    def info(self):
        if self.p is None:
            return {'name': '', 'state': '없음'}
        rc = self.p.poll()
        return {'name': self.name, 'elapsed': round(time.monotonic() - self.t0, 1),
                'state': '실행 중' if rc is None else ('완료' if rc == 0 else f'종료 코드 {rc}')}

    def tail(self, n=60):
        try:
            return ''.join(collections.deque(open(self.log_path, errors='replace'), n))
        except OSError:
            return ''


class Rate:
    def __init__(self):
        self.n, self.t0, self.hz = 0, time.monotonic(), 0.0

    def tick(self):
        self.n += 1

    def update(self, now):
        if now - self.t0 >= 1.0:
            self.hz, self.n, self.t0 = self.n / (now - self.t0), 0, now


class WebView(Node):
    def __init__(self, hz):
        super().__init__('web_view')
        params = load_params()
        keys = ('morph_kernel', 'min_area_px', 'selection', 'depth_min_valid_ratio', 'depth_erode_px', 'size_check',
                'obj_area_min_cm2', 'obj_area_max_cm2', 'similar_area_ratio', 'similar_depth_m', 'similar_depth_ratio')
        self.cfg = DetectorConfig(hsv_lower=tuple(params['hsv_lower']), hsv_upper=tuple(params['hsv_upper']),
                                  **{k: params[k] for k in keys if k in params})
        self.color_topic = params.get('color_topic', '/camera/camera/color/image_raw')
        self.lock = threading.Lock()
        self.jpeg, self.seq = None, 0
        self.frame, self.frame_t = None, 0.0
        self.target, self.target_t = None, 0.0
        self.joint, self.joint_t = None, 0.0
        self.cmd, self.cmd_t = None, 0.0
        self.status, self.status_t = '-', 0.0
        self.timeline = collections.deque(maxlen=20)   # (시각, 상태) 상태 바뀔 때만
        self.show_mask = False
        self.mode = 'none'                             # tracker를 띄운 방식: real / dry
        self.rates = {k: Rate() for k in ('camera', 'target', 'joint', 'command')}
        self.job = Proc('webui_job.log')               # 시험·분석 작업 (한 번에 하나)
        self.bag = Proc('webui_bag.log')               # bag 녹화
        self.bag_path = ''
        self.run_id = ''                               # 마지막으로 띄운 tracker의 실행 ID
        # raw=True: 직렬화된 바이트만 받아 두고, 그릴 프레임만 역직렬화한다 (30 Hz 전부 풀면 Pi CPU를 먹어 검출 FPS가 떨어짐)
        self.create_subscription(Image, self.color_topic, self.on_image, qos_profile_sensor_data, raw=True)
        self.create_subscription(PointStamped, params.get('target_topic', '/target'), self.on_target,
                                 qos_profile_sensor_data)
        self.create_subscription(String, '/tracking_status', self.on_status, 10)
        self.create_subscription(JointState, '/pan_tilt/joint_states', self.on_joint, 10)
        self.create_subscription(Vector3Stamped, '/pan_tilt/command', self.on_cmd, 10)
        self.enable_pub = self.create_publisher(Bool, '/tracking_enable', 10)
        self.snap_pub = self.create_publisher(String, params.get('snapshot_topic', '/target/save_snapshot'), 10)
        self.get_cli = self.create_client(GetParameters, f'{CTRL}/get_parameters')
        self.set_cli = self.create_client(SetParameters, f'{CTRL}/set_parameters')
        self.create_timer(1.0 / hz, self.render)
        self.create_timer(1.0, self.guard_bag)
        self.get_logger().info(f'{self.color_topic} {hz:.0f} Hz, HSV {self.cfg.hsv_lower}~{self.cfg.hsv_upper}')

    # ---- 토픽 ----
    def on_image(self, raw):
        self.rates['camera'].tick()
        self.frame, self.frame_t = raw, time.monotonic()

    def on_target(self, msg):
        self.rates['target'].tick()
        self.target, self.target_t = msg.point, time.monotonic()

    def on_status(self, msg):
        self.status_t = time.monotonic()
        if msg.data.split(':')[0] != self.status.split(':')[0]:
            self.timeline.appendleft((datetime.now().strftime('%H:%M:%S.%f')[:-3], msg.data))
        self.status = msg.data

    def on_joint(self, msg):
        self.rates['joint'].tick()
        self.joint, self.joint_t = msg, time.monotonic()

    def on_cmd(self, msg):
        self.rates['command'].tick()
        self.cmd, self.cmd_t = msg.vector, time.monotonic()

    def render(self):
        now = time.monotonic()
        for r in self.rates.values():
            r.update(now)
        if self.frame is None:
            img = np.zeros((480, 640, 3), np.uint8)
            cv2.putText(img, f'waiting for {self.color_topic}', (20, 240),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 200, 255), 2)
        else:
            msg = deserialize_message(self.frame, Image)
            buf = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.step)[:, :msg.width * 3]
            img = buf.reshape(msg.height, msg.width, 3)
            img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR) if msg.encoding == 'rgb8' else img.copy()
            det, mask = detect(img, 'bgr8', self.cfg)
            img = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR) if self.show_mask else draw_overlay(img, det)
            h, w = img.shape[:2]
            t = self.target
            if t is not None and now - self.target_t < 0.5 and t.z > 0:   # 검출 노드가 실제로 발행한 값
                txt = f'/target ex {t.x:+.2f} ey {t.y:+.2f} area {t.z:.4f}'
            else:
                txt = '/target NO TARGET'
            stale = now - self.frame_t > 1.0
            cv2.putText(img, txt, (8, 46), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
            cv2.putText(img, f'{self.status}  cam {self.rates["camera"].hz:.1f} Hz' + ('  STALE' if stale else ''),
                        (8, h - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 255) if stale else (0, 255, 255), 2)
            if self.bag.running():
                cv2.circle(img, (w - 20, 20), 8, (0, 0, 255), -1)
                cv2.putText(img, 'REC', (w - 70, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 255), 2)
        ok, enc = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, 70])
        if ok:
            with self.lock:
                self.jpeg, self.seq = enc.tobytes(), self.seq + 1

    def guard_bag(self):
        if self.bag.running() and (time.monotonic() - self.bag.t0 > BAG_MAX_S or free_gb() < BAG_MIN_FREE_GB):
            self.get_logger().warning('bag 자동 정지 (시간 상한 또는 디스크 여유 부족)')
            self.bag.stop()

    # ---- 추적 실행 관리 (tmux 'tracker') ----
    def start_tracker(self, mode):
        if tracker_running():
            raise RuntimeError('이미 실행 중 — 먼저 "전체 정지"')
        if self.job.running():
            raise RuntimeError(f'작업 "{self.job.name}" 실행 중 — 먼저 작업 멈춤')
        dry = 'true' if mode == 'dry' else 'false'
        self.run_id = datetime.now().strftime('run_%Y%m%d_%H%M%S')   # 세 노드 기록과 bag 이름을 같은 실행 ID로 묶는다
        cmd = f'{ENV} && ros2 launch tracker_bringup full.launch.py dry_run:={dry} run_id:={self.run_id}; exec bash'
        subprocess.run(['tmux', 'new-session', '-d', '-s', 'tracker', '-n', 'full', f"bash -c '{cmd}'"], check=True)
        self.mode = mode
        self.timeline.clear()

    def stop_tracker(self):
        self.bag.stop()
        if tracker_running():
            subprocess.run(['tmux', 'send-keys', '-t', 'tracker:full', 'C-c'])
            for _ in range(40):   # 브리지가 X를 보내고 끝날 때까지
                time.sleep(0.2)
                if not sh("pgrep -f 'opencr_bridge|controller_node|target_detector|realsense2_camera_node'").strip():
                    break
            subprocess.run(['tmux', 'kill-session', '-t', 'tracker'])
        self.mode = 'none'
        self.status = '-'

    def require_stopped(self, what):
        if tracker_running():
            raise RuntimeError(f'{what}: 추적 실행 중에는 못 함 — 먼저 "전체 정지"')

    def require_running(self, what):
        if not tracker_running():
            raise RuntimeError(f'{what}: 추적이 실행 중이어야 함 — 먼저 "전체 실행"')

    # ---- 파라미터 (HTTP 스레드에서 호출) ----
    def call(self, cli, req, timeout=2.0):
        if not cli.wait_for_service(timeout_sec=timeout):
            raise RuntimeError('tracker_controller 응답 없음')
        fut = cli.call_async(req)
        end = time.monotonic() + timeout
        while not fut.done():
            if time.monotonic() > end:
                raise RuntimeError('timeout')
            time.sleep(0.01)
        return fut.result()

    def get_params(self):
        names = [s[0] for s in SLIDERS]
        res = self.call(self.get_cli, GetParameters.Request(names=names), timeout=0.5)
        return {n: v.double_value for n, v in zip(names, res.values)}

    def set_param(self, name, value):
        prm = Parameter(name=name, value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=value))
        r = self.call(self.set_cli, SetParameters.Request(parameters=[prm])).results[0]
        if not r.successful:
            raise RuntimeError(r.reason)

    # ---- 상태 ----
    def firmware_state(self):
        """가장 최근 브리지 시리얼 기록의 마지막 상태 줄·오류 줄."""
        path = newest(os.path.join(LOG_DIR, '*_serial.log'))
        if not path or not tracker_running() or self.mode == 'dry':
            return {'state': '-', 'error': ''}
        with open(path, 'rb') as f:
            f.seek(max(0, os.path.getsize(path) - 6000))
            lines = f.read().decode(errors='replace').splitlines()
        state, err = '-', ''
        for line in reversed(lines):
            body = line.split('\t', 1)[-1]
            if state == '-' and body.startswith('S '):
                state = body.split()[-1]
            if not err and body.startswith('E '):
                err = body
            if state != '-' and err:
                break
        return {'state': state, 'error': err}

    def snapshot_state(self):
        now = time.monotonic()
        t, j, c = self.target, self.joint, self.cmd
        joint = None
        if j is not None and now - self.joint_t < 1.0:
            joint = {n: [round(np.degrees(p), 2), round(np.degrees(v), 2)]
                     for n, p, v in zip(j.name, j.position, j.velocity or [0.0] * len(j.name))}
        bag = self.bag.info()
        bag['path'] = self.bag_path
        if self.bag_path and os.path.isdir(self.bag_path):
            bag['size_mb'] = round(sum(os.path.getsize(p) for p in glob.glob(self.bag_path + '/*')) / 1e6, 1)
        return {
            'tracker': tracker_running(), 'mode': self.mode,
            'status': self.status if now - self.status_t < 1.0 else f'{self.status} (수신 끊김)',
            'target': None if t is None or now - self.target_t > 0.5 else [round(t.x, 3), round(t.y, 3), round(t.z, 4)],
            'joint': joint,
            'command': None if c is None or now - self.cmd_t > 0.5 else [round(c.x, 2), round(c.y, 2)],
            'rates': {k: round(r.hz, 1) for k, r in self.rates.items()},
            'firmware': self.firmware_state(), 'timeline': list(self.timeline),
            'job': self.job.info(), 'bag': bag, 'free_gb': round(free_gb(), 2), 'mask': self.show_mask,
        }

    # ---- 버튼 동작 ----
    def action(self, a, q):
        latest_run = 'ls -t ~/lv2_module5_logs/run_*.csv 2>/dev/null | grep -vE "_summary|_detect" | head -1'
        if a == 'start_real':
            self.start_tracker('real')
        elif a == 'start_dry':
            self.start_tracker('dry')
        elif a == 'stop_all':
            self.stop_tracker()
        elif a == 'restart':
            mode = self.mode if self.mode != 'none' else 'real'
            self.stop_tracker()
            self.start_tracker(mode)
        elif a == 'reset_fw':
            self.require_stopped('OpenCR 리셋')
            self.job.start('OpenCR 리셋 (펌웨어 재업로드)',
                           'cd ~/pa-opencr-build && ./uploader-src/arduino/opencr_develop/opencr_ld/opencr_ld '
                           '/dev/ttyACM0 115200 output_tracker/opencr_tracker.ino.bin 1')
        elif a == 'job_stop':
            self.job.stop()
        # 문제 1 검출
        elif a == 'mask':
            self.show_mask = not self.show_mask
        elif a in ('snap_normal', 'snap_empty', 'snap_occluded'):
            self.require_running('스냅샷')
            self.snap_pub.publish(String(data=a[5:]))
        elif a == 'list_snaps':
            self.job.start('저장된 스냅샷 목록', f'ls -lt {SNAP_DIR} 2>/dev/null | head -25')
        # 문제 2 인터페이스·모의 입력
        elif a == 'echo_target':
            self.job.start('/target 1개 확인 (ros2 CLI 시작에 약 7 s)', 'timeout 20 ros2 topic echo --once /target')
        elif a == 'hz_target':
            self.job.start('/target 주기 측정 (약 15 s, CLI 시작 포함)', 'timeout -s INT 15 ros2 topic hz /target; true')
        elif a == 'mock_test':
            self.require_stopped('모의 입력 시험 (모터 출력 OFF)')
            self.job.start('모의 입력 시험 c1~c6 (약 2분, 모터 출력 OFF)', 'bash scripts/mock_inputs_test.sh')
        # 문제 3 추적 제어
        elif a in ('enable', 'disable'):
            self.require_running('추적 시작/정지')
            self.enable_pub.publish(Bool(data=a == 'enable'))
        elif a == 'set':
            if q.get('name') not in [s[0] for s in SLIDERS]:
                raise RuntimeError('unknown parameter')
            self.set_param(q['name'], float(q['value']))
        elif a == 'summarize':
            self.job.start('최근 추적 기록 요약 (마지막 30 s)',
                           f'f=$({latest_run}); echo "$f"; python3 scripts/summarize_run.py "$f" --last 30 --save')
        # 문제 4 안전 정지
        elif a == 'kill_controller':
            self.require_running('제어 노드 중단 시험')
            self.job.start('제어 노드 강제 종료 → 브리지가 0.2 s 뒤 V 0 0 전송',
                           f'date +%T.%N; pkill -9 -f lib/tracker_controlle[r]/controller_node && echo killed; sleep 1.5; '
                           f's=$(ls -t {LOG_DIR}/*_serial.log | head -1); echo "$s"; grep ">>> V" "$s" | tail -5')
        elif a == 'kill_bridge':
            self.require_running('브리지 중단 시험')
            self.job.start('브리지 강제 종료 → OpenCR 300 ms 타임아웃 정지(HOLD), 5 s 더 지나면 토크 OFF',
                           f'date +%T.%N; pkill -9 -f lib/tracker_bridg[e]/opencr_bridge && echo killed; sleep 1; '
                           'python3 -c "import serial,time;s=serial.Serial(\'/dev/ttyACM0\',115200,timeout=0.2);'
                           't=time.time();b=b\'\'\nwhile time.time()-t<7: b+=s.read(4096)\n'
                           'L=[l for l in b.decode(errors=\'replace\').splitlines() if l.strip()];'
                           'print(*(L[:3]+[\'...\']+[l for l in L if l.startswith(\'E \')]+L[-3:]),sep=chr(10))"')
        # 문제 5 기록·재현
        elif a == 'bag_start':
            self.require_running('bag 녹화')
            if free_gb() < BAG_MIN_FREE_GB + 0.5:
                raise RuntimeError(f'디스크 여유 {free_gb():.1f} GB — 오래된 bag을 지운 뒤 다시')
            os.makedirs(BAG_DIR, exist_ok=True)
            # <run_id>_HHMMSS: 같은 실행의 <run_id>.csv·_detect.csv·_serial.log와 짝 (한 실행에 장면별 bag 여러 개)
            self.bag_path = os.path.join(BAG_DIR, f"{self.run_id or 'bag'}_{datetime.now():%H%M%S}")
            self.bag.start(f'bag 녹화 (최대 {BAG_MAX_S:.0f} s)',
                           f'ros2 bag record -s mcap -o {self.bag_path} --topics {" ".join(BAG_TOPICS)}')
        elif a == 'bag_stop':
            self.bag.stop()
        elif a == 'bag_list':
            self.job.start('bag 목록·최근 bag 정보',
                           f'du -sh {BAG_DIR}/* 2>/dev/null; echo; b=$(ls -td {BAG_DIR}/*/ 2>/dev/null | head -1); '
                           f'[ -n "$b" ] && ros2 bag info "$b"; echo; df -h ~ | tail -1')
        elif a == 'replay':
            self.require_stopped('bag 재처리 (모터 출력 없음)')
            self.job.start('최근 bag 재처리 → /target_replay (모터·카메라 노드 없음)',
                           f'b=$(ls -td {BAG_DIR}/*/ 2>/dev/null | head -1); [ -z "$b" ] && {{ echo "bag 없음"; exit 1; }}; '
                           'echo "bag: $b"; ros2 launch tracker_bringup replay.launch.py > ~/lv2_module5_logs/replay_launch.log 2>&1 & L=$!; '
                           'sleep 6; (timeout -s INT 20 ros2 topic hz /target_replay &); '
                           'timeout -s INT 90 ros2 bag play "$b" --clock --disable-keyboard-controls --progress-bar-update-rate 0 '
                           '< /dev/null --topics /camera/camera/color/image_raw '
                           '/camera/camera/aligned_depth_to_color/image_raw /camera/camera/color/camera_info; '
                           'sleep 1; kill -TERM $L; wait $L; echo; tail -5 ~/lv2_module5_logs/replay_launch.log')
        elif a == 'bag_delete_oldest':
            bags = sorted(glob.glob(os.path.join(BAG_DIR, '*', 'metadata.yaml')), key=os.path.getmtime)
            bags = [os.path.dirname(b) for b in bags]
            if self.bag.running() or not bags:
                raise RuntimeError('지울 bag이 없거나 녹화 중')
            shutil.rmtree(bags[0])
            return {'note': f'삭제: {os.path.basename(bags[0])}'}
        else:
            raise RuntimeError(f'unknown action {a}')
        return {}

    def logs(self, src):
        if src == 'nodes':
            return sh('tmux capture-pane -p -J -t tracker:full -S -80 2>/dev/null | grep -v "^$" | tail -60') \
                or '(추적 실행 안 함)'
        if src == 'job':
            return self.job.tail() or '(작업 없음)'
        if src == 'serial':
            path = newest(os.path.join(LOG_DIR, '*_serial.log'))
            if not path:
                return '(기록 없음)'
            with open(path, 'rb') as f:
                f.seek(max(0, os.path.getsize(path) - 20000))
                lines = f.read().decode(errors='replace').splitlines()[1:]
            keep = [ln for ln in lines if '>>> V 0.00 0.00' not in ln]   # 생존 신호는 빼고
            return path + '\n' + '\n'.join(keep[-50:])
        if src == 'bag':
            return self.bag.tail()
        return ''


PAGE = r"""<!doctype html><html lang="ko"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1"><title>Monglian Tracker</title><style>
:root{--bg:#0f1115;--card:#181b22;--line:#2a2f3a;--tx:#e3e6ec;--mut:#8a93a3;--ok:#2fa36b;--bad:#d24b4b;--warn:#d9a33a;--acc:#3b82f6}
*{box-sizing:border-box}body{margin:0;background:var(--bg);color:var(--tx);font:14px/1.45 system-ui,sans-serif}
header{display:flex;flex-wrap:wrap;align-items:center;gap:10px;padding:10px 16px;border-bottom:1px solid var(--line)}
header h1{font-size:16px;margin:0 8px 0 0}.pill{padding:2px 10px;border-radius:99px;background:#252a35;font-size:13px}
.pill.ok{background:#173b2a;color:#7ee2ad}.pill.bad{background:#4a1d1d;color:#ffaaaa}.pill.warn{background:#4a3a14;color:#ffd98a}
main{display:grid;grid-template-columns:minmax(0,660px) minmax(0,1fr);gap:14px;padding:14px 16px}
@media(max-width:1100px){main{grid-template-columns:1fr}}
.card{background:var(--card);border:1px solid var(--line);border-radius:10px;padding:12px;margin-bottom:14px}
.card h2{font-size:14px;margin:0 0 8px}.card h2 small{color:var(--mut);font-weight:400}
img.cam{width:100%;border-radius:6px;display:block;background:#000}
table{border-collapse:collapse;width:100%;font-variant-numeric:tabular-nums}td{padding:3px 6px;border-bottom:1px solid var(--line)}
td:first-child{color:var(--mut);width:42%}
button{font:inherit;padding:6px 11px;margin:3px 4px 3px 0;border:1px solid var(--line);border-radius:6px;background:#252a35;color:var(--tx);cursor:pointer}
button:hover{border-color:var(--acc)}button.go{background:#1d4f37;border-color:#2c7353}button.stop{background:#5a2222;border-color:#8a3434}
button.tab{background:none}button.tab.on{border-color:var(--acc);color:#9cc1ff}
.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(300px,1fr));gap:14px}
.hint{color:var(--mut);font-size:12.5px;margin:4px 0 6px}
.row label{display:flex;justify-content:space-between;font-size:13px}input[type=range]{width:100%}
pre{background:#0a0c10;border:1px solid var(--line);border-radius:6px;padding:8px;height:300px;overflow:auto;font:12px/1.4 ui-monospace,monospace;white-space:pre-wrap;margin:6px 0 0}
#msg{min-height:1.2em;color:#ffaaaa}#msg.ok{color:#7ee2ad}
.tl{font:12px ui-monospace,monospace;max-height:150px;overflow:auto}
</style></head><body>
<header><h1>Monglian 팬·틸트 추적</h1>
<span class="pill" id="p_run">-</span><span class="pill" id="p_status">-</span><span class="pill" id="p_fw">OpenCR -</span>
<span class="pill" id="p_bag">bag -</span><span class="pill" id="p_disk">-</span>
<span style="flex:1"></span>
<button class="go" onclick="act('start_real')">전체 실행 (모터 ON)</button>
<button onclick="act('start_dry')">전체 실행 (모터 OFF)</button>
<button onclick="act('restart')">재시작</button>
<button class="stop" onclick="act('stop_all')">전체 정지</button>
<button onclick="act('reset_fw')" title="FAULT일 때. 전체 정지 후">OpenCR 리셋</button>
</header>
<div style="padding:0 16px"><div id="msg"></div></div>
<main>
<section>
 <div class="card"><img class="cam" src="/stream.mjpg" alt="camera">
  <div style="margin-top:8px"><button onclick="act('mask')" id="b_mask">마스크 화면</button>
  <button class="go" onclick="act('enable')">추적 시작</button><button class="stop" onclick="act('disable')">추적 정지</button></div></div>
 <div class="card"><h2>실시간 토픽</h2><table id="topics"></table></div>
 <div class="card"><h2>상태 변화 <small>/tracking_status</small></h2><div class="tl" id="timeline">-</div></div>
</section>
<section>
 <div class="grid">
  <div class="card"><h2>문제 1 · HSV·컨투어 검출</h2>
   <div class="hint">같은 설정으로 정상·대상 없음·가림 세 장면을 저장 (원본·마스크·검출 이미지, ~/lv2_module5_results/images)</div>
   <button onclick="act('snap_normal')">정상 장면 저장</button><button onclick="act('snap_empty')">대상 없음 저장</button>
   <button onclick="act('snap_occluded')">가림 장면 저장</button><button onclick="act('list_snaps')">저장 목록</button>
   <button onclick="act('mask')">마스크/검출 전환</button></div>
  <div class="card"><h2>문제 2 · /target 인터페이스·모의 입력</h2>
   <div class="hint">x=ex, y=ey, z=면적비(0=미검출). 모의 입력 시험은 전체 정지 후 실행 (브리지 없이 → 모터 출력 OFF)</div>
   <button onclick="act('echo_target')">/target 1개 보기</button><button onclick="act('hz_target')">/target 주기</button>
   <button class="go" onclick="act('mock_test')">모의 입력 시험 c1~c6</button></div>
  <div class="card"><h2>문제 3 · 중심 오차 P 추적 제어</h2>
   <div class="hint">명령 = clamp(Kp × 오차, ±최대 속도). 값은 실행 중에만 바뀜 (재시작하면 config/control.yaml)</div>
   <button class="go" onclick="act('enable')">추적 시작</button><button class="stop" onclick="act('disable')">추적 정지</button>
   <button onclick="act('summarize')">최근 기록 요약</button>
   <div id="sliders"></div></div>
  <div class="card"><h2>문제 4 · 안전 정지·복귀</h2>
   <div class="hint">목표 소실 → LOST 즉시 정지, 연속 3프레임 재검출 → TRACKING 복귀 (물체를 가렸다 보이며 '상태 변화' 확인).
   아래 두 시험 뒤에는 '재시작'.</div>
   <button onclick="act('disable')">IDLE (추적 정지)</button>
   <button class="stop" onclick="confirm('제어 노드를 강제 종료합니다')&&act('kill_controller')">제어 노드 중단 시험</button>
   <button class="stop" onclick="confirm('브리지를 강제 종료합니다')&&act('kill_bridge')">통신(브리지) 중단 시험</button></div>
  <div class="card"><h2>문제 5 · bag 기록·재현</h2>
   <div class="hint">영상·깊이·/target·상태·명령·관절 토픽을 mcap으로 녹화 (최대 약 30 s, 시작까지 약 7 s, ~/lv2_module5_bags).
   재처리는 전체 정지 후: 최근 bag 영상으로 검출만 다시 → /target_replay</div>
   <button class="go" onclick="act('bag_start')">녹화 시작</button><button class="stop" onclick="act('bag_stop')">녹화 정지</button>
   <button onclick="act('bag_list')">bag 목록·정보</button><button onclick="act('replay')">최근 bag 재처리</button>
   <button onclick="confirm('가장 오래된 bag을 삭제합니다')&&act('bag_delete_oldest')">가장 오래된 bag 삭제</button></div>
 </div>
 <div class="card"><h2>로그 <small id="jobinfo"></small></h2>
  <button class="tab on" data-src="job">작업 출력</button><button class="tab" data-src="nodes">노드 로그</button>
  <button class="tab" data-src="serial">OpenCR 시리얼</button><button class="tab" data-src="bag">bag 녹화</button>
  <button onclick="act('job_stop')">작업 멈춤</button>
  <pre id="log"></pre></div>
</section></main>
<script>
const S=__SLIDERS__;let src='job',busy=false;
const $=id=>document.getElementById(id);
for(const [n,l,mi,ma,st] of S){$('sliders').insertAdjacentHTML('beforeend',
 `<div class="row"><label>${l}<b id="v_${n}">-</b></label><input type="range" id="${n}" min="${mi}" max="${ma}" step="${st}"
  oninput="$('v_${n}').textContent=this.value" onchange="act('set','&name=${n}&value='+this.value)"></div>`);}
document.querySelectorAll('.tab').forEach(b=>b.onclick=()=>{document.querySelectorAll('.tab').forEach(x=>x.classList.remove('on'));
 b.classList.add('on');src=b.dataset.src;pollLog();});
function msg(t,ok){$('msg').textContent=t||'';$('msg').className=ok?'ok':'';}
async function act(a,extra=''){msg('… '+a,true);try{const j=await (await fetch('/api/act?a='+a+extra)).json();
 msg(j.error||j.note||('완료: '+a),!j.error);if(['reset_fw','mock_test','summarize','list_snaps','echo_target','hz_target',
 'kill_controller','kill_bridge','bag_list','replay'].includes(a)){document.querySelector('.tab[data-src=job]').click();}}
 catch(e){msg(String(e));}poll();}
function pill(id,t,c){const e=$(id);e.textContent=t;e.className='pill '+(c||'');}
const f=(v,d=2)=>v==null?'-':(+v).toFixed(d);
async function poll(){if(busy)return;busy=true;try{const s=await (await fetch('/api/state')).json();
 pill('p_run',s.tracker?('실행 중 · '+(s.mode=='dry'?'모터 OFF':'모터 ON')):'정지',s.tracker?'ok':'');
 const st=s.status.split(':')[0];pill('p_status',s.status,st=='TRACKING'?'ok':(st=='LOST'?'warn':''));
 const fw=s.firmware.state;pill('p_fw','OpenCR '+fw,fw=='FAULT'||/FAULT/.test(s.firmware.error)?'bad':(fw=='TRACK'?'ok':''));
 if(fw=='FAULT'||/RESET required/.test(s.firmware.error))msg('OpenCR FAULT — 전체 정지 → OpenCR 리셋 → 전체 실행');
 pill('p_bag',s.bag.state=='실행 중'?`REC ${s.bag.elapsed}s ${s.bag.size_mb||0}MB`:'bag 대기',s.bag.state=='실행 중'?'bad':'');
 pill('p_disk','여유 '+s.free_gb+' GB',s.free_gb<1.5?'warn':'');
 $('b_mask').textContent=s.mask?'검출 화면':'마스크 화면';
 const t=s.target,j=s.joint||{},c=s.command,r=s.rates,jn=Object.keys(j);
 $('topics').innerHTML=[
  ['카메라 '+'<small>/camera/.../image_raw</small>',r.camera+' Hz'],
  ['/target (ex, ey, 면적비)',t?`${f(t[0],3)}, ${f(t[1],3)}, ${f(t[2],4)}`+(t[2]>0?'':' (미검출)'):'-',],
  ['/target 주기',r.target+' Hz'],
  ['/pan_tilt/command (팬, 틸트) deg/s',c?`${f(c[0])}, ${f(c[1])}`:'-'],
  ['명령 주기',r.command+' Hz'],
  ...jn.map(n=>[`관절 ${n} 각도·속도`,`${f(j[n][0])}° · ${f(j[n][1])}°/s`]),
  ['/pan_tilt/joint_states 주기',r.joint+' Hz'],
  ['OpenCR 마지막 오류',s.firmware.error||'-'],
 ].map(([a,b])=>`<tr><td>${a}</td><td>${b}</td></tr>`).join('');
 $('timeline').innerHTML=s.timeline.length?s.timeline.map(([t,v])=>`${t}  ${v}`).join('<br>'):'-';
 $('jobinfo').textContent=s.job.name?`· ${s.job.name} — ${s.job.state}${s.job.elapsed!=null?' ('+s.job.elapsed+' s)':''}`:'';
 for(const n in (s.params||{})){const e=$(n);if(document.activeElement!==e){e.value=s.params[n];$('v_'+n).textContent=s.params[n];}}
}catch(e){pill('p_run','웹 서버 응답 없음','bad');}busy=false;}
async function pollLog(){try{const t=await (await fetch('/api/logs?src='+src)).text();const p=$('log');
 const end=p.scrollTop+p.clientHeight>=p.scrollHeight-20;p.textContent=t;if(end)p.scrollTop=p.scrollHeight;}catch(e){}}
poll();pollLog();setInterval(poll,1000);setInterval(pollLog,2000);
</script></body></html>"""


def make_handler(view):
    page = PAGE.replace('__SLIDERS__', json.dumps(SLIDERS)).encode()

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *args):
            pass

        def reply(self, body, ctype):
            self.send_response(200)
            self.send_header('Content-Type', ctype)
            self.send_header('Cache-Control', 'no-store')
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            url = urlparse(self.path)
            q = {k: v[0] for k, v in parse_qs(url.query).items()}
            if url.path == '/api/state':
                out = view.snapshot_state()
                if out['tracker'] and view.mode != 'none':
                    try:
                        out['params'] = view.get_params()
                    except Exception:   # noqa: BLE001  제어 노드가 아직 안 떴거나 죽음
                        pass
                self.reply(json.dumps(out).encode(), 'application/json')
            elif url.path == '/api/act':
                try:
                    out = view.action(q.get('a', ''), q) or {}
                except Exception as e:   # noqa: BLE001  화면에 이유를 보여준다
                    out = {'error': str(e)}
                self.reply(json.dumps(out).encode(), 'application/json')
            elif url.path == '/api/logs':
                self.reply(view.logs(q.get('src', 'job')).encode(), 'text/plain; charset=utf-8')
            elif url.path == '/stream.mjpg':
                self.send_response(200)
                self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
                self.end_headers()
                last = -1
                try:
                    while True:
                        with view.lock:
                            jpeg, seq = view.jpeg, view.seq
                        if jpeg is None or seq == last:
                            time.sleep(0.01)
                            continue
                        last = seq
                        self.wfile.write(b'--frame\r\nContent-Type: image/jpeg\r\nContent-Length: '
                                         + str(len(jpeg)).encode() + b'\r\n\r\n' + jpeg + b'\r\n')
                except (BrokenPipeError, ConnectionResetError):
                    return
            elif url.path == '/snapshot.jpg' and view.jpeg is not None:
                self.reply(view.jpeg, 'image/jpeg')
            else:
                self.reply(page, 'text/html; charset=utf-8')
    return Handler


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--port', type=int, default=8080)
    ap.add_argument('--hz', type=float, default=8.0)
    ap.add_argument('--start', choices=('real', 'dry', 'none'), default='none', help='켜면서 추적 실행')
    args = ap.parse_args()
    os.makedirs(LOG_DIR, exist_ok=True)
    rclpy.init()
    view = WebView(args.hz)
    if tracker_running():
        view.mode = 'dry' if 'dry_run:=true' in sh('pgrep -af "tracker_bringup full.launch.py"') else 'real'
    elif args.start != 'none':
        view.start_tracker(args.start)
    server = ThreadingHTTPServer(('0.0.0.0', args.port), make_handler(view))
    server.daemon_threads = True
    threading.Thread(target=server.serve_forever, daemon=True).start()
    view.get_logger().info(f'http://0.0.0.0:{args.port}/')
    executor = MultiThreadedExecutor(num_threads=2)   # 서비스 응답을 받는 동안에도 렌더 타이머가 돈다
    executor.add_node(view)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        view.bag.stop()
        view.job.stop()
        server.shutdown()
        view.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
