#!/usr/bin/env python3
"""웹 관제 화면 — 보기 전용 (Pi에서 실행): 추적 카메라 화면 · 실시간 토픽 · 상태 변화 · 로그.

로봇 동작에 끼어들지 않게:
  - 발행·서비스 호출·프로세스 실행이 없다 (버튼 없음). 토픽 구독과 로그 파일 읽기만 한다.
  - 모든 구독은 best-effort라 발행 노드에 ACK·재전송 부담을 주지 않는다.
  - 카메라 원본(640x480 rgb8, 약 27 MB/s)은 브라우저가 화면을 볼 때만 구독하고, 아무도 안 보면 3 s 뒤 구독을 끊는다.
  - 낮은 우선순위(nice 10)·OpenCV 스레드 1개로 돌아 검출·제어 노드의 CPU를 뺏지 않는다.
  - 화면은 /target 값(인지 노드가 실제 발행한 값)으로 목표 위치를 그린다. 검출을 다시 돌리지 않는다
    (--contour를 주면 같은 검출 함수로 컨투어를 다시 그리지만 Pi CPU를 더 쓴다).
네트워크 부담을 줄이게:
  - 영상은 Pi 안에서 JPEG으로 줄여 HTTP로만 내보낸다 (기본 5 Hz · 폭 640 · 품질 60 ≈ 150 KB/s, 브라우저 1개 기준).
  - 브라우저 탭이 가려지면 스트림·폴링을 멈춘다. 동시 스트림은 --max-clients개까지.
  - --local-dds: 이 노드의 DDS 탐색을 Pi 안(localhost)으로만 한정해 Wi-Fi에 DDS 패킷을 더하지 않는다.

  python3 scripts/web_view.py [--port 8080] [--hz 5] [--width 640] [--quality 60] [--contour] [--local-dds]
  PC 브라우저: http://papimon.local:8080/        (스트림만: /stream.mjpg, 한 장: /snapshot.jpg)
  (실행 전: source /opt/ros/lyrical/setup.bash && source ros2_ws/install/setup.bash
           && export ROS_DOMAIN_ID=28 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp)
"""
import argparse
import collections
import glob
import json
import math
import os
import threading
import time
from datetime import datetime
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse

import cv2
import numpy as np
import yaml

M5 = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))   # lv2_module5
CFG_DIR = os.path.join(M5, 'config')
LOG_DIR = os.path.expanduser('~/lv2_module5_logs')
IDLE_UNSUB_S = 3.0        # 화면을 보는 브라우저가 없으면 이 시간 뒤 카메라 구독 해제


def load_params():
    params = {}
    for name in sorted(os.listdir(CFG_DIR)):   # 노드와 같게: 공통(/**) + target_detector 값
        if name.endswith('.yaml'):
            with open(os.path.join(CFG_DIR, name)) as f:
                doc = yaml.safe_load(f) or {}
            for node in ('/**', 'target_detector'):
                params.update((doc.get(node) or {}).get('ros__parameters', {}))
    return params


def newest(*patterns):
    files = [f for p in patterns for f in glob.glob(p, recursive=True)]
    return max(files, key=os.path.getmtime) if files else None


def tail(path, nbytes=20000):
    with open(path, 'rb') as f:
        f.seek(max(0, os.path.getsize(path) - nbytes))
        return f.read().decode(errors='replace').splitlines()[1:]   # 첫 줄은 잘렸을 수 있어 버린다


class Rate:
    def __init__(self):
        self.n, self.t0, self.hz = 0, time.monotonic(), 0.0

    def tick(self):
        self.n += 1

    def update(self, now):
        if now - self.t0 >= 1.0:
            self.hz, self.n, self.t0 = self.n / (now - self.t0), 0, now


def build_node(args):
    # rclpy는 --local-dds 환경 변수를 정한 뒤에 불러온다
    from geometry_msgs.msg import PointStamped, Vector3Stamped
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data as BEST_EFFORT
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import Image, JointState
    from std_msgs.msg import String

    class WebView(Node):
        def __init__(self):
            super().__init__('web_view')
            params = load_params()
            self.args = args
            self.color_topic = params.get('color_topic', '/camera/camera/color/image_raw')
            self.cfg = None
            if args.contour:
                from target_detector.detection import DetectorConfig
                keys = ('morph_kernel', 'min_area_px', 'selection', 'depth_min_valid_ratio', 'depth_erode_px',
                        'size_check', 'obj_area_min_cm2', 'obj_area_max_cm2', 'similar_area_ratio',
                        'similar_depth_m', 'similar_depth_ratio', 'detect_scale')
                self.cfg = DetectorConfig(hsv_lower=tuple(params['hsv_lower']), hsv_upper=tuple(params['hsv_upper']),
                                          **{k: params[k] for k in keys if k in params})
            self.lock = threading.Lock()
            self.jpeg, self.seq = None, 0
            self.viewers, self.last_view = 0, 0.0      # 스트림을 보는 HTTP 연결 수 (HTTP 스레드가 바꾼다)
            self.image_sub = None
            self.frame, self.frame_t = None, 0.0
            self.target, self.target_t = None, 0.0
            self.depth, self.depth_t = None, 0.0
            self.joint, self.joint_t = None, 0.0
            self.cmd, self.cmd_t = None, 0.0
            self.status, self.status_t = '-', 0.0
            self.timeline = collections.deque(maxlen=30)   # (시각, 상태) 상태가 바뀔 때만
            self.rates = {k: Rate() for k in ('camera', 'target', 'joint', 'command')}
            self._Image, self._deserialize, self._best_effort = Image, deserialize_message, BEST_EFFORT

            self.create_subscription(PointStamped, params.get('target_topic', '/target'), self.on_target, BEST_EFFORT)
            self.create_subscription(PointStamped, params.get('depth_out_topic', '/target_depth'), self.on_depth,
                                     BEST_EFFORT)
            self.create_subscription(String, params.get('status_topic', '/tracking_status'), self.on_status,
                                     BEST_EFFORT)
            self.create_subscription(JointState, params.get('joint_state_topic', '/pan_tilt/joint_states'),
                                     self.on_joint, BEST_EFFORT)
            self.create_subscription(Vector3Stamped, params.get('command_topic', '/pan_tilt/command'), self.on_cmd,
                                     BEST_EFFORT)
            self.create_timer(1.0 / args.hz, self.render)
            self.create_timer(0.5, self.housekeeping)
            self.get_logger().info(f'보기 전용 · {self.color_topic} (화면을 볼 때만 구독) · {args.hz:g} Hz · '
                                   f'{"컨투어 재계산" if args.contour else "/target 표시"}')

        # ---- 토픽 ----
        def on_image(self, raw):     # raw=True: 바이트만 받아 두고 그릴 프레임만 역직렬화 (30 Hz 전부 풀지 않음)
            self.rates['camera'].tick()
            self.frame, self.frame_t = raw, time.monotonic()

        def on_target(self, msg):
            self.rates['target'].tick()
            self.target, self.target_t = msg.point, time.monotonic()

        def on_depth(self, msg):
            self.depth, self.depth_t = msg.point, time.monotonic()

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

        # ---- 카메라 구독은 보는 사람이 있을 때만 ----
        def housekeeping(self):
            now = time.monotonic()
            for r in self.rates.values():
                r.update(now)
            watching = self.viewers > 0 or now - self.last_view < IDLE_UNSUB_S
            if watching and self.image_sub is None:
                self.image_sub = self.create_subscription(self._Image, self.color_topic, self.on_image,
                                                          self._best_effort, raw=True)
                self.get_logger().info('브라우저 연결 → 카메라 구독 시작')
            elif not watching and self.image_sub is not None:
                self.destroy_subscription(self.image_sub)
                self.image_sub, self.frame = None, None
                self.rates['camera'] = Rate()
                with self.lock:
                    self.jpeg = None     # 다음 브라우저에게 지난 화면을 보이지 않게
                self.get_logger().info('보는 브라우저 없음 → 카메라 구독 해제')

        def render(self):
            if self.image_sub is None:
                return
            now = time.monotonic()
            w_out = self.args.width
            if self.frame is None:
                img = np.zeros((w_out * 3 // 4, w_out, 3), np.uint8)
                cv2.putText(img, f'waiting for {self.color_topic}', (20, img.shape[0] // 2),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 200, 255), 2)
            else:
                msg = self._deserialize(self.frame, self._Image)
                buf = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.step)[:, :msg.width * 3]
                img = buf.reshape(msg.height, msg.width, 3)
                if msg.encoding == 'rgb8':
                    img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
                img = self.draw(img, now)
            ok, enc = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, self.args.quality])
            if ok:
                with self.lock:
                    self.jpeg, self.seq = enc.tobytes(), self.seq + 1

        def draw(self, img, now):
            if self.cfg is not None:     # --contour: 원본 크기에서 같은 검출 함수로 다시 계산 (보기 전용, 깊이 없이)
                from target_detector.detection import detect, draw_overlay
                img = draw_overlay(img, detect(img, 'bgr8', self.cfg)[0])
            h0, w0 = img.shape[:2]
            if w0 != self.args.width:
                img = cv2.resize(img, (self.args.width, round(h0 * self.args.width / w0)), interpolation=cv2.INTER_AREA)
            h, w = img.shape[:2]
            t = self.target
            fresh = t is not None and now - self.target_t < 0.5
            if self.cfg is None:
                cv2.drawMarker(img, (w // 2, h // 2), (255, 255, 255), cv2.MARKER_CROSS, 24, 1)   # 영상 중심
                if fresh and t.z > 0:    # /target: ex·ey 정규화 중심, z 면적비 -> 같은 면적의 정사각형으로 표시
                    p = (int((t.x + 1) * w / 2), int((t.y + 1) * h / 2))
                    half = int(math.sqrt(t.z * w * h) / 2)
                    cv2.rectangle(img, (p[0] - half, p[1] - half), (p[0] + half, p[1] + half), (0, 255, 0), 2)
                    cv2.drawMarker(img, p, (0, 0, 255), cv2.MARKER_TILTED_CROSS, 16, 2)
                    cv2.line(img, (w // 2, h // 2), p, (0, 0, 255), 1)
            if fresh and t.z > 0:
                txt = f'/target ex {t.x:+.2f} ey {t.y:+.2f} area {t.z:.4f}'
                d = self.depth
                if d is not None and now - self.depth_t < 0.5:
                    txt += f'  Z {d.z:.2f} m' if d.x > 0.5 else '  Z invalid'
            else:
                txt = '/target NO TARGET' if fresh else '/target (no message)'
            stale = now - self.frame_t > 1.0
            status = self.status if now - self.status_t < 1.0 else f'{self.status} (no msg)'
            cv2.putText(img, txt, (8, 46), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2)
            cv2.putText(img, f'{status}  cam {self.rates["camera"].hz:.1f} Hz' + ('  STALE' if stale else ''),
                        (8, h - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 255) if stale else (0, 255, 255), 2)
            return img

        # ---- 상태·로그 (HTTP 스레드에서 호출, 읽기만) ----
        def firmware_state(self):
            """가장 최근 브리지 시리얼 기록의 마지막 상태 줄·오류 줄 (5 s 넘게 안 바뀐 기록은 지난 실행으로 본다)."""
            path = newest(os.path.join(LOG_DIR, '*_serial.log'))
            if not path or time.time() - os.path.getmtime(path) > 5.0:
                return {'state': '-', 'error': ''}
            state, err = '-', ''
            for line in reversed(tail(path, 6000)):
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
            t, j, c, d = self.target, self.joint, self.cmd, self.depth
            joint = None
            if j is not None and now - self.joint_t < 1.0:
                joint = {n: [round(math.degrees(p), 2), round(math.degrees(v), 2)]
                         for n, p, v in zip(j.name, j.position, j.velocity or [0.0] * len(j.name))}
            return {
                'status': self.status if now - self.status_t < 1.0 else f'{self.status} (수신 끊김)',
                'live': now - self.status_t < 1.0,
                'target': None if t is None or now - self.target_t > 0.5 else [round(t.x, 3), round(t.y, 3), round(t.z, 4)],
                'depth': None if d is None or now - self.depth_t > 0.5 else [d.x > 0.5, round(d.y, 2), round(d.z, 3)],
                'joint': joint,
                'command': None if c is None or now - self.cmd_t > 0.5 else [round(c.x, 2), round(c.y, 2)],
                'rates': {k: round(r.hz, 1) for k, r in self.rates.items()},
                'camera_sub': self.image_sub is not None,
                'firmware': self.firmware_state(), 'timeline': list(self.timeline),
            }

    return WebView()


def read_log(src):
    if src == 'launch':      # 통합 메뉴(assignment/main.py)·과제 프로그램이 남기는 launch 출력 = 노드 로그
        path = newest(os.path.join(LOG_DIR, '*_launch.log'), os.path.join(M5, 'results', '**', '*_launch.log'))
        keep = None
    elif src == 'serial':    # 브리지 시리얼 기록. 50 Hz 생존 신호(V 0 0)는 뺀다
        path = newest(os.path.join(LOG_DIR, '*_serial.log'))
        keep = lambda ln: '>>> V 0.00 0.00' not in ln   # noqa: E731
    elif src == 'control':   # 제어 노드 CSV (최근 줄)
        path = newest(os.path.join(LOG_DIR, 'run_*.csv'), os.path.join(LOG_DIR, 'idle_*.csv'))
        keep = None
    else:
        return ''
    if not path:
        return '(기록 없음)'
    lines = tail(path)
    if keep:
        lines = [ln for ln in lines if keep(ln)]
    age = time.time() - os.path.getmtime(path)
    return f'{path}  (마지막 기록 {age:.0f} s 전)\n' + '\n'.join(lines[-60:])


PAGE = r"""<!doctype html><html lang="ko"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1"><title>Monglian Tracker View</title><style>
:root{--bg:#0f1115;--card:#181b22;--line:#2a2f3a;--tx:#e3e6ec;--mut:#8a93a3;--acc:#3b82f6}
*{box-sizing:border-box}body{margin:0;background:var(--bg);color:var(--tx);font:14px/1.45 system-ui,sans-serif}
header{display:flex;flex-wrap:wrap;align-items:center;gap:10px;padding:10px 16px;border-bottom:1px solid var(--line)}
header h1{font-size:16px;margin:0 8px 0 0}.pill{padding:2px 10px;border-radius:99px;background:#252a35;font-size:13px}
.pill.ok{background:#173b2a;color:#7ee2ad}.pill.bad{background:#4a1d1d;color:#ffaaaa}.pill.warn{background:#4a3a14;color:#ffd98a}
.ro{color:var(--mut);font-size:12.5px}
main{display:grid;grid-template-columns:minmax(0,660px) minmax(0,1fr);gap:14px;padding:14px 16px}
@media(max-width:1100px){main{grid-template-columns:1fr}}
.card{background:var(--card);border:1px solid var(--line);border-radius:10px;padding:12px;margin-bottom:14px}
.card h2{font-size:14px;margin:0 0 8px}.card h2 small{color:var(--mut);font-weight:400}
img.cam{width:100%;border-radius:6px;display:block;background:#000;aspect-ratio:4/3}
table{border-collapse:collapse;width:100%;font-variant-numeric:tabular-nums}td{padding:3px 6px;border-bottom:1px solid var(--line)}
td:first-child{color:var(--mut);width:42%}
button.tab{font:inherit;padding:5px 11px;margin:0 4px 0 0;border:1px solid var(--line);border-radius:6px;background:none;color:var(--tx);cursor:pointer}
button.tab.on{border-color:var(--acc);color:#9cc1ff}
pre{background:#0a0c10;border:1px solid var(--line);border-radius:6px;padding:8px;height:420px;overflow:auto;font:12px/1.4 ui-monospace,monospace;white-space:pre-wrap;margin:8px 0 0}
.tl{font:12px ui-monospace,monospace;max-height:220px;overflow:auto}
</style></head><body>
<header><h1>Monglian 팬·틸트 추적</h1>
<span class="pill" id="p_status">-</span><span class="pill" id="p_fw">OpenCR -</span><span class="pill" id="p_cam">카메라 -</span>
<span style="flex:1"></span><span class="ro">보기 전용 · 로봇 제어 없음</span>
</header>
<main>
<section>
 <div class="card"><img class="cam" id="cam" alt="camera"></div>
 <div class="card"><h2>실시간 토픽</h2><table id="topics"></table></div>
</section>
<section>
 <div class="card"><h2>상태 변화 <small>/tracking_status</small></h2><div class="tl" id="timeline">-</div></div>
 <div class="card"><h2>로그 <small>~/lv2_module5_logs (읽기만)</small></h2>
  <button class="tab on" data-src="launch">노드 로그</button><button class="tab" data-src="serial">OpenCR 시리얼</button>
  <button class="tab" data-src="control">제어 CSV</button>
  <pre id="log"></pre></div>
</section></main>
<script>
let src='launch',busy=false,timers=[];
const $=id=>document.getElementById(id);
document.querySelectorAll('.tab').forEach(b=>b.onclick=()=>{document.querySelectorAll('.tab').forEach(x=>x.classList.remove('on'));
 b.classList.add('on');src=b.dataset.src;pollLog();});
function pill(id,t,c){const e=$(id);e.textContent=t;e.className='pill '+(c||'');}
const f=(v,d=2)=>v==null?'-':(+v).toFixed(d);
async function poll(){if(busy)return;busy=true;try{const s=await (await fetch('/api/state')).json();
 const st=s.status.split(':')[0];pill('p_status',s.status,!s.live?'':(st=='TRACKING'?'ok':(st=='LOST'?'warn':'')));
 const fw=s.firmware.state;pill('p_fw','OpenCR '+fw,fw=='FAULT'||/FAULT/.test(s.firmware.error)?'bad':(fw=='TRACK'?'ok':''));
 const r=s.rates;pill('p_cam',s.camera_sub?`카메라 ${r.camera} Hz`:'카메라 구독 안 함',s.camera_sub&&r.camera<1?'warn':'');
 const t=s.target,d=s.depth,j=s.joint||{},c=s.command,jn=Object.keys(j);
 $('topics').innerHTML=[
  ['/target (ex, ey, 면적비)',t?`${f(t[0],3)}, ${f(t[1],3)}, ${f(t[2],4)}`+(t[2]>0?'':' (미검출)'):'-'],
  ['/target 주기',r.target+' Hz'],
  ['/target_depth (유효, 비율, Z m)',d?`${d[0]?'유효':'무효'}, ${f(d[1])}, ${f(d[2],3)}`:'-'],
  ['/pan_tilt/command (팬, 틸트) deg/s',c?`${f(c[0])}, ${f(c[1])}`:'-'],
  ['명령 주기',r.command+' Hz'],
  ...jn.map(n=>[`관절 ${n} 각도·속도`,`${f(j[n][0])}° · ${f(j[n][1])}°/s`]),
  ['/pan_tilt/joint_states 주기',r.joint+' Hz'],
  ['OpenCR 마지막 오류',s.firmware.error||'-'],
 ].map(([a,b])=>`<tr><td>${a}</td><td>${b}</td></tr>`).join('');
 $('timeline').innerHTML=s.timeline.length?s.timeline.map(([t,v])=>`${t}  ${v}`).join('<br>'):'-';
}catch(e){pill('p_status','웹 서버 응답 없음','bad');}busy=false;}
async function pollLog(){try{const t=await (await fetch('/api/logs?src='+src)).text();const p=$('log');
 const end=p.scrollTop+p.clientHeight>=p.scrollHeight-20;p.textContent=t;if(end)p.scrollTop=p.scrollHeight;}catch(e){}}
// 탭이 가려지면 스트림·폴링을 멈춰 네트워크·Pi CPU를 쓰지 않는다
function start(){$('cam').src='/stream.mjpg?'+Date.now();poll();pollLog();timers=[setInterval(poll,1000),setInterval(pollLog,3000)];}
function stop(){$('cam').removeAttribute('src');timers.forEach(clearInterval);timers=[];}
document.addEventListener('visibilitychange',()=>document.hidden?stop():start());
start();
</script></body></html>"""


def make_handler(view, max_clients):
    page = PAGE.encode()

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *args):
            pass

        def reply(self, body, ctype, code=200):
            self.send_response(code)
            self.send_header('Content-Type', ctype)
            self.send_header('Cache-Control', 'no-store')
            self.send_header('Content-Length', str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            url = urlparse(self.path)
            q = {k: v[0] for k, v in parse_qs(url.query).items()}
            if url.path == '/api/state':
                self.reply(json.dumps(view.snapshot_state()).encode(), 'application/json')
            elif url.path == '/api/logs':
                self.reply(read_log(q.get('src', 'launch')).encode(), 'text/plain; charset=utf-8')
            elif url.path == '/stream.mjpg':
                self.stream()
            elif url.path == '/snapshot.jpg':
                view.last_view = time.monotonic()     # 한 장 요청도 잠시 구독을 켠다
                with view.lock:
                    jpeg = view.jpeg
                if jpeg is None:
                    self.reply('카메라 구독 시작 중 — 1~2 s 뒤 다시'.encode(), 'text/plain; charset=utf-8', 503)
                else:
                    self.reply(jpeg, 'image/jpeg')
            elif url.path in ('/', '/index.html'):
                self.reply(page, 'text/html; charset=utf-8')
            else:
                self.reply(b'not found', 'text/plain', 404)

        def stream(self):
            with view.lock:
                if view.viewers >= max_clients:
                    full = True
                else:
                    full, view.viewers = False, view.viewers + 1
            if full:
                self.reply(f'동시 스트림 {max_clients}개 초과'.encode(), 'text/plain; charset=utf-8', 503)
                return
            try:
                self.send_response(200)
                self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
                self.send_header('Cache-Control', 'no-store')
                self.end_headers()
                last = -1
                while True:
                    view.last_view = time.monotonic()
                    with view.lock:
                        jpeg, seq = view.jpeg, view.seq
                    if jpeg is None or seq == last:
                        time.sleep(0.02)
                        continue
                    last = seq
                    self.wfile.write(b'--frame\r\nContent-Type: image/jpeg\r\nContent-Length: '
                                     + str(len(jpeg)).encode() + b'\r\n\r\n' + jpeg + b'\r\n')
            except (BrokenPipeError, ConnectionResetError, TimeoutError):
                pass
            finally:
                with view.lock:
                    view.viewers -= 1
                    view.last_view = time.monotonic()

    return Handler


def main():
    ap = argparse.ArgumentParser(description='보기 전용 웹 관제 화면 (로봇 제어 없음)')
    ap.add_argument('--port', type=int, default=8080)
    ap.add_argument('--hz', type=float, default=5.0, help='스트림 프레임 수 [Hz] (기본 5)')
    ap.add_argument('--width', type=int, default=640, help='스트림 폭 [px] (320이면 전송량 약 1/3)')
    ap.add_argument('--quality', type=int, default=60, help='JPEG 품질 (기본 60)')
    ap.add_argument('--max-clients', type=int, default=2, help='동시 스트림 수 상한')
    ap.add_argument('--contour', action='store_true', help='컨투어를 다시 계산해 그림 (Pi CPU 더 씀)')
    ap.add_argument('--local-dds', action='store_true',
                    help='이 노드의 DDS 탐색을 localhost로 한정 (Pi에서 모든 노드가 돌 때만)')
    args = ap.parse_args()
    if args.local_dds:
        os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'
    try:
        os.nice(10)            # 검출·제어·브리지 노드보다 낮은 우선순위
    except OSError:
        pass
    cv2.setNumThreads(1)       # OpenCV 작업 스레드가 다른 노드의 코어를 뺏지 않게

    import rclpy
    from rclpy.executors import ExternalShutdownException
    rclpy.init()
    view = build_node(args)
    server = ThreadingHTTPServer(('0.0.0.0', args.port), make_handler(view, args.max_clients))
    server.daemon_threads = True
    threading.Thread(target=server.serve_forever, daemon=True).start()
    view.get_logger().info(f'http://0.0.0.0:{args.port}/')
    try:
        rclpy.spin(view)
    except (KeyboardInterrupt, ExternalShutdownException):   # Ctrl+C·SIGTERM
        pass
    finally:
        server.shutdown()
        view.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
