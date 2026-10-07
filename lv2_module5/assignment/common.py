"""assignment 공용 도구: 경로·설정 변형·노드 실행/종료·기록 읽기·지표·평가 프레임·그래프·표.

assignment*.py는 Pi에서 ROS 워크스페이스를 source한 셸에서 실행한다.
  cd ~/git/Lv2_Monglian_Assignment/lv2_module5 && source ros2_ws/install/setup.bash
  python3 assignment/assignment1.py --help
지표 산식은 발제 문제 4의 측정 지표 표를 따르고, 각 함수 설명에 분모·조건을 적었다.
"""
import csv
import glob
import math
import os
import re
import shutil
import signal
import subprocess
import sys
import time

LV2 = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CONFIG = os.path.join(LV2, 'config')
RESULTS = os.path.join(LV2, 'results')
LOG_DIR = os.path.expanduser('~/lv2_module5_logs')        # 노드 기록 (제어 <run_id>.csv, 인지 <run_id>_detect.csv)
SNAP_DIR = os.path.expanduser('~/lv2_module5_results/images')
NAN = float('nan')

# 자식 프로세스(ros2 launch)가 SIGINT를 무시한 채 시작되지 않도록 기본 처리로 되돌린다
# (비대화형 셸에서 백그라운드로 실행되면 SIGINT가 무시 상태로 물려받아져 launch가 Ctrl+C에 반응하지 않았다, 2026-10-06)
signal.signal(signal.SIGINT, signal.default_int_handler)


# ================= 경로·기록 =================
def out_dir(*parts):
    path = os.path.join(RESULTS, *parts)
    os.makedirs(path, exist_ok=True)
    return path


def tag():
    return time.strftime('%Y%m%d_%H%M%S')


def git_commit():
    try:
        h = subprocess.run(['git', '-C', LV2, 'rev-parse', '--short', 'HEAD'], capture_output=True, text=True).stdout.strip()
        dirty = subprocess.run(['git', '-C', LV2, 'status', '--porcelain', '--', '.'], capture_output=True, text=True).stdout.strip()
        return h + ('-dirty' if dirty else '')
    except OSError:
        return 'unknown'


def fnum(v):
    try:
        return float(v)
    except (TypeError, ValueError):
        return NAN


def read_csv(path):
    with open(path, newline='') as f:
        return list(csv.DictReader(f))


def write_csv(path, rows, header=None):
    header = header or (list(rows[0].keys()) if rows else [])
    with open(path, 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=header, extrasaction='ignore')
        w.writeheader()
        for r in rows:
            w.writerow({k: fmt(r.get(k, '')) for k in header})
    return path


def fmt(v):
    if isinstance(v, float):
        if math.isnan(v):
            return ''
        return f'{v:.3f}' if abs(v) >= 1000 else f'{v:.4g}'     # 시각(1.79e9 s)은 소수 3자리까지 보존
    return v


def md_table(rows, header):
    lines = ['| ' + ' | '.join(header) + ' |', '|' + '---|' * len(header)]
    lines += ['| ' + ' | '.join(str(fmt(r.get(k, ''))) for k in header) + ' |' for r in rows]
    return '\n'.join(lines)


def update_metrics(rows):
    """results/metrics.csv(회차별 성능표)를 (test, run_id) 기준으로 갱신한다. 다른 시험의 행은 그대로 둔다."""
    path = os.path.join(RESULTS, 'metrics.csv')
    old = read_csv(path) if os.path.exists(path) and os.path.getsize(path) > 0 else []
    keys = {(r['test'], r['run_id']) for r in rows}
    merged = [r for r in old if (r.get('test'), r.get('run_id')) not in keys] + rows
    header = ['test', 'run_id', 'condition', 'date']
    for r in merged:
        header += [k for k in r if k not in header]
    write_csv(path, merged, header)
    return path


def node_logs(run_id):
    """노드 기록 파일 경로: 제어 CSV, 인지 CSV, 브리지 시리얼 로그"""
    return {'control': os.path.join(LOG_DIR, f'{run_id}.csv'),
            'detect': os.path.join(LOG_DIR, f'{run_id}_detect.csv'),
            'serial': os.path.join(LOG_DIR, f'{run_id}_serial.log')}


def keep_logs(run_id, dest):
    """노드 기록을 results/ 아래로 복사한다(원본은 ~/lv2_module5_logs에 그대로 둔다)."""
    os.makedirs(dest, exist_ok=True)
    copied = []
    for p in node_logs(run_id).values():
        if os.path.exists(p):
            shutil.copy2(p, dest)
            copied.append(os.path.join(dest, os.path.basename(p)))
    return copied


# ================= 설정 변형 =================
def config_variant(dest, overrides):
    """config/*.yaml을 dest로 복사하고 overrides {키: 값}을 바꾼다(주석 유지). 바뀐 설정 폴더 경로를 돌려준다.
    키는 파일 안에서 한 번만 나오는 이름이어야 한다(pan_kp, pan_deadband, recover_frames, relock_after_s ...)."""
    os.makedirs(dest, exist_ok=True)
    for name in os.listdir(CONFIG):
        if name.endswith('.yaml'):
            shutil.copy2(os.path.join(CONFIG, name), dest)
    for key, value in overrides.items():
        hit = False
        for name in os.listdir(dest):
            p = os.path.join(dest, name)
            text = open(p).read()
            pat = re.compile(rf'^(\s*{re.escape(key)}:\s*)([^#\n]*?)(\s*#.*)?$', re.M)
            if pat.search(text):
                val = str(value).lower() if isinstance(value, bool) else str(value)
                text = pat.sub(lambda m: m.group(1) + val + (m.group(3) or ''), text, count=1)
                open(p, 'w').write(text)
                hit = True
                break
        if not hit:
            raise SystemExit(f'설정 키를 찾지 못함: {key}')
    return dest


def read_param(key, config_dir=CONFIG):
    for name in sorted(os.listdir(config_dir)):
        if name.endswith('.yaml'):
            m = re.search(rf'^\s*{re.escape(key)}:\s*([^#\n]+)', open(os.path.join(config_dir, name)).read(), re.M)
            if m:
                return m.group(1).strip()
    return None


# ================= 프로세스 =================
class Proc:
    """ros2 launch/run을 새 프로세스 그룹으로 띄우고, 끝낼 때 SIGINT → SIGTERM → SIGKILL 순서로 정리한다."""

    def __init__(self, cmd, log_path):
        self.cmd, self.log_path = cmd, log_path
        self.log = open(log_path, 'w')
        self.p = subprocess.Popen(cmd, stdout=self.log, stderr=subprocess.STDOUT, start_new_session=True)

    def alive(self):
        return self.p.poll() is None

    def stop(self, timeout=20.0):
        if self.alive():
            os.killpg(self.p.pid, signal.SIGINT)
            try:
                self.p.wait(timeout)
            except subprocess.TimeoutExpired:
                os.killpg(self.p.pid, signal.SIGTERM)
                try:
                    self.p.wait(5)
                except subprocess.TimeoutExpired:
                    os.killpg(self.p.pid, signal.SIGKILL)
        self.log.close()
        return self.p.returncode


def launch(file, log_path, **args):
    """ros2 launch tracker_bringup <file> key:=value ..."""
    cmd = ['ros2', 'launch', 'tracker_bringup', file] + [f'{k}:={str(v).lower() if isinstance(v, bool) else v}'
                                                         for k, v in args.items()]
    return Proc(cmd, log_path)


def ros_args(node, config_dir=CONFIG, params=None):
    """--ros-args: config/*.yaml 전부(launch와 같게, 노드는 자기 이름과 /** 항목만 읽음) + params.
    -p 값은 /** 항목이 되어 yaml의 노드 이름 항목에 진다(auto_enable 등). 그래서 launch의 override와 같이
    노드 이름으로 쓴 임시 yaml을 마지막 파일로 넘긴다."""
    import tempfile
    import yaml
    args = ['--ros-args']
    for f in sorted(glob.glob(os.path.join(config_dir, '*.yaml'))):
        args += ['--params-file', f]
    if params:
        f = tempfile.NamedTemporaryFile('w', prefix=f'{node}_run_', suffix='.yaml', delete=False)
        yaml.safe_dump({node: {'ros__parameters': dict(params)}}, f)
        f.close()
        args += ['--params-file', f.name]
    return args


def run_node(pkg, exe, node, log_path, config_dir=CONFIG, params=None):
    """ros2 run <pkg> <exe> (node = 노드 이름, 설정 yaml의 항목 이름)"""
    return Proc(['ros2', 'run', pkg, exe] + ros_args(node, config_dir, params), log_path)


def run_mock(log_path, params=None):
    """모의 /target 발행기 scripts/mock_target_pub.py (패키지 밖 스크립트라 ros2 run 대신 python3로 실행)"""
    return Proc([sys.executable, os.path.join(LV2, 'scripts', 'mock_target_pub.py'), '--ros-args']
                + [a for k, v in (params or {}).items() for a in ('-p', f'{k}:={str(v).lower() if isinstance(v, bool) else v}')],
                log_path)


def node_params(node, config_dir=CONFIG):
    """config/*.yaml에서 /** 공통 값과 node 항목을 합친 파라미터 dict (launch로 띄운 노드가 받는 값과 같음)"""
    import yaml
    out = {}
    for f in sorted(glob.glob(os.path.join(config_dir, '*.yaml'))):
        data = yaml.safe_load(open(f)) or {}
        for key in ('/**', node):
            out.update((data.get(key) or {}).get('ros__parameters') or {})
    return out


def wait_bag_ready(log_path, topics, timeout=20.0):
    """ros2 bag record가 토픽을 실제로 구독할 때까지 기다린다(Pi에서 시작에 수 초 걸림, 2026-10-06 6 s 기록이 1.56 s만 남음).
    로그의 "Subscribed to topic '<토픽>'" 줄로 판단한다. 구독한 토픽 집합을 돌려준다."""
    end = time.time() + timeout
    got = set()
    while time.time() < end:
        try:
            text = open(log_path, errors='replace').read()
        except OSError:
            text = ''
        got = {t for t in topics if f"topic '{t}'" in text or f'topic: {t}' in text}
        if len(got) == len(topics):
            break
        time.sleep(0.2)
    return got


def pids(pattern):
    out = subprocess.run(['pgrep', '-f', pattern], capture_output=True, text=True).stdout.split()
    return [int(p) for p in out if int(p) != os.getpid()]


def kill9(pattern):
    """패턴에 맞는 프로세스를 SIGKILL로 끊는다(통신 중단 시험). 끊은 시각(time.time())과 PID를 돌려준다."""
    found = pids(pattern)
    t = time.time()
    for p in found:
        os.kill(p, signal.SIGKILL)
    return t, found


NODE_PATTERNS = {   # 실행 파일 경로로 찾는다(명령 줄에 같은 글자가 들어간 셸을 잘못 끊지 않게)
    'detector': 'lib/target_detector/target_detector',
    'controller': 'lib/tracker_controller/controller_node',
    'bridge': 'lib/tracker_bridge/opencr_bridge',
}


def check_ros():
    if shutil.which('ros2') is None:
        raise SystemExit('ros2가 없습니다: source /opt/ros/lyrical/setup.bash && source ros2_ws/install/setup.bash')
    for n in ('ROS_DOMAIN_ID', 'RMW_IMPLEMENTATION'):
        if not os.environ.get(n):
            print(f'주의: {n}가 비어 있습니다(팀 기준 28 / rmw_cyclonedds_cpp)')


def port_free(port='/dev/ttyACM0'):
    r = subprocess.run(['fuser', port], capture_output=True, text=True)
    return r.returncode != 0


# ================= 진행 안내 =================
def ask(msg):
    return input(f'\n▶ {msg}\n  Enter: 진행 (중단: Ctrl+C) ')


def countdown(label, seconds):
    """'label' 구간을 seconds 동안 1초 단위로 보여 준다. 구간 시작 시각(time.time())을 돌려준다."""
    start = time.time()
    while True:
        left = seconds - (time.time() - start)
        if left <= 0:
            break
        sys.stdout.write(f'\r  \a{label}: {left:4.1f} s 남음   ' if left > seconds - 0.2 else f'\r  {label}: {left:4.1f} s 남음   ')
        sys.stdout.flush()
        time.sleep(0.1)
    sys.stdout.write(f'\r  {label}: 끝{" " * 20}\n')
    return start


# ================= ROS 도우미 =================
def make_probe(name='assignment_probe'):
    """/tracking_status·/target·/pan_tilt/command를 듣고 /tracking_enable을 보내는 노드. (node, state) 반환."""
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from geometry_msgs.msg import PointStamped, Vector3Stamped
    from std_msgs.msg import Bool, String
    if not rclpy.ok():
        rclpy.init()
    node = rclpy.create_node(name)
    st = {'status': None, 'status_t': 0.0, 'target': None, 'target_t': 0.0, 'cmd': None, 'cmd_t': 0.0,
          'history': []}

    def on_status(m):
        st['status'], st['status_t'] = m.data, time.time()
        st['history'].append((time.time(), m.data))

    def on_target(m):
        st['target'], st['target_t'] = (m.point.x, m.point.y, m.point.z), time.time()

    def on_cmd(m):
        st['cmd'], st['cmd_t'] = (m.vector.x, m.vector.y), time.time()

    node.create_subscription(String, '/tracking_status', on_status, 10)
    node.create_subscription(PointStamped, '/target', on_target, qos_profile_sensor_data)
    node.create_subscription(Vector3Stamped, '/pan_tilt/command', on_cmd, 10)
    st['enable_pub'] = node.create_publisher(Bool, '/tracking_enable', 10)
    return node, st


def spin_for(node, seconds):
    import rclpy
    end = time.time() + seconds
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.02)


def wait_until(node, cond, timeout):
    import rclpy
    end = time.time() + timeout
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.05)
        if cond():
            return True
    return False


def set_tracking(node, st, on):
    from std_msgs.msg import Bool
    for _ in range(3):        # 구독 연결 직후 한 번은 놓칠 수 있어 세 번 보낸다(같은 값이라 안전)
        st['enable_pub'].publish(Bool(data=on))
        spin_for(node, 0.1)


def start_stack(run_id, dest, dry_run=False, config_dir=CONFIG, camera=True, timeout=40.0):
    """full.launch(카메라·인지·제어·브리지)를 띄우고 /tracking_status와 /target이 들어올 때까지 기다린다."""
    check_ros()
    if not dry_run and not port_free():
        raise SystemExit('/dev/ttyACM0을 다른 프로그램이 쓰고 있습니다(시리얼 모니터·pose_tool·다른 launch를 끄세요).')
    proc = launch('full.launch.py', os.path.join(dest, f'{run_id}_launch.log'), run_id=run_id, dry_run=dry_run,
                  config_dir=config_dir, camera=camera)
    node, st = make_probe()
    ok = wait_until(node, lambda: st['status'] is not None and (st['target'] is not None or not camera), timeout)
    if not ok:
        proc.stop()
        raise SystemExit(f'노드가 준비되지 않았습니다. 기록: {proc.log_path}')
    print(f'  준비됨: 상태 {st["status"]}, 기록 이름 {run_id}')
    return proc, node, st


# ================= 지표 (발제 문제 4 측정 지표) =================
def in_window(t, t0, t1):
    return (t0 is None or t >= t0) and (t1 is None or t <= t1)


def processing_fps(det_rows, t0=None, t1=None):
    """처리 FPS = 처리 완료 프레임 수 / 실제 경과 초 (인지 노드 발행 시각 pub_time_s 기준). 카메라 FPS(stamp 기준)와 구분."""
    rows = [r for r in det_rows if in_window(fnum(r['pub_time_s']), t0, t1)]
    if len(rows) < 2:
        return NAN, NAN, len(rows)
    pub = [fnum(r['pub_time_s']) for r in rows]
    stamp = [fnum(r['stamp_s']) for r in rows]
    fps = (len(rows) - 1) / (pub[-1] - pub[0]) if pub[-1] > pub[0] else NAN
    cam = (len(rows) - 1) / (stamp[-1] - stamp[0]) if stamp[-1] > stamp[0] else NAN
    return fps, cam, len(rows)


def tracking_metrics(ctl_rows, t0=None, t1=None):
    """제어 기록(50 Hz)에서 추적 지표를 계산한다. 구간: 추적을 켠 행(IDLE 제외) 중 [t0, t1] (ros_time_s).
    - 수평 RMSE = sqrt(mean(ex^2)), 검출·TRACKING 행만 사용 (제외 행 수와 비율 병기)
    - 유효 추적 비율 = TRACKING 행 / 전체 행
    - 노드 검출 비율 = detected 행 / 전체 행 (사람 대조 검출률과 다름)
    - 흔들림 = TRACKING 구간 팬 명령 부호 바뀜 횟수 / TRACKING 시간 [회/s]"""
    rows = [r for r in ctl_rows if r['state'] != 'IDLE' and in_window(fnum(r['ros_time_s']), t0, t1)]
    if not rows:
        return {}
    trk = [r for r in rows if r['state'] == 'TRACKING' and r['detected'] == '1']
    ex = [fnum(r['ex']) for r in trk]
    flips, prev = 0, 0
    for r in trk:
        c = fnum(r['pan_cmd'])
        s = (c > 0) - (c < 0)
        if s and prev and s != prev:
            flips += 1
        if s:
            prev = s
    dur = fnum(rows[-1]['ros_time_s']) - fnum(rows[0]['ros_time_s'])
    trk_time = len(trk) / max(len(rows), 1) * dur
    return {'duration_s': dur, 'rows': len(rows),
            'rmse_ex': math.sqrt(sum(e * e for e in ex) / len(ex)) if ex else NAN,
            'mean_abs_ex': sum(abs(e) for e in ex) / len(ex) if ex else NAN,
            'max_abs_ex': max((abs(e) for e in ex), default=NAN),
            'rmse_rows': len(ex), 'excluded_rows': len(rows) - len(ex),
            'tracking_ratio': len(trk) / len(rows),
            'node_detect_ratio': sum(r['detected'] == '1' for r in rows) / len(rows),
            'cmd_flips_per_s': flips / trk_time if trk_time > 0 else NAN}


def state_segments(ctl_rows, by_state=False):
    """(시작 시각, 끝 시각, 상태, 사유) 목록. 같은 상태·사유(by_state면 상태만)가 이어지면 한 구간."""
    segs = []
    for r in ctl_rows:
        t, key = fnum(r['ros_time_s']), (r['state'], '' if by_state else r['reason'])
        if segs and tuple(segs[-1][2:]) == key:
            segs[-1][1] = t
        else:
            segs.append([t, t, *key])
    return segs


def recovery_trials(ctl_rows, det_rows, marks, limit_s=3.0):
    """가림 후 재등장 판정. marks = [(가림 안내 시각, 재등장 안내 시각)] (time.time(), 같은 시스템 시계).
    - 가림 확인: 안내 구간에 후보가 없는 영상(n_candidates=0)이 있어야 한다(없으면 '가림 미확인')
    - 재등장 시각: 후보 없는 마지막 영상 다음의 첫 후보 영상 stamp (사람이 판단한 시각이 아님을 보고서에 적는다)
    - 복귀 시각: 재등장 이후 첫 TRACKING 행 / 성공: 복귀 - 재등장 <= limit_s
    - 정지 확인: 가림 중 미검출 행의 최대 |명령| (0이어야 함)"""
    out = []
    for i, (hide_t, show_t) in enumerate(marks, 1):
        win = [r for r in det_rows if hide_t - 0.5 <= fnum(r['stamp_s']) <= show_t + limit_s + 3.0]
        empty = [j for j, r in enumerate(win) if r['n_candidates'] == '0']
        res = {'trial': i, 'hide_prompt_t': hide_t, 'show_prompt_t': show_t}
        if not empty:
            res.update(result='가림 미확인', reappear_t=NAN, tracking_t=NAN, recovery_s=NAN)
            out.append(res)
            continue
        after = [r for r in win[empty[-1] + 1:] if r['n_candidates'] != '0']
        lost_rows = [r for r in ctl_rows if fnum(win[empty[0]]['stamp_s']) <= fnum(r['ros_time_s'])
                     and r['detected'] != '1' and r['state'] != 'IDLE']
        res['max_cmd_while_lost'] = max((max(abs(fnum(r['pan_cmd'])), abs(fnum(r['tilt_cmd']))) for r in lost_rows
                                         if not after or fnum(r['ros_time_s']) <= fnum(after[0]['stamp_s'])), default=NAN)
        res['occluded_s'] = fnum(win[empty[-1]]['stamp_s']) - fnum(win[empty[0]]['stamp_s'])
        if not after:
            res.update(result='재등장 미검출', reappear_t=NAN, tracking_t=NAN, recovery_s=NAN)
            out.append(res)
            continue
        t_re = fnum(after[0]['stamp_s'])
        trk = [r for r in ctl_rows if fnum(r['ros_time_s']) >= t_re and r['state'] == 'TRACKING']
        t_trk = fnum(trk[0]['ros_time_s']) if trk else NAN
        rec = t_trk - t_re
        res.update(reappear_t=t_re, tracking_t=t_trk, recovery_s=rec,
                   result='성공' if rec == rec and rec <= limit_s else '실패')
        out.append(res)
    return out


# ================= 평가 프레임 (검출률·배경 오검출) =================
def detector_config(config_dir=CONFIG):
    from target_detector.detection import DetectorConfig
    params = node_params('target_detector', config_dir)
    keys = ('morph_kernel', 'min_area_px', 'selection', 'depth_min_valid_ratio', 'depth_erode_px', 'size_check',
            'obj_area_min_cm2', 'obj_area_max_cm2', 'similar_area_ratio', 'similar_depth_m', 'similar_depth_ratio',
            'detect_scale', 'depth_min_m', 'depth_max_m')
    return DetectorConfig(hsv_lower=tuple(params['hsv_lower']), hsv_upper=tuple(params['hsv_upper']),
                          **{k: params[k] for k in keys if k in params}), params


def capture_eval(dest, phases, interval_s=0.5, configs=None):
    """카메라 토픽에서 평가 프레임을 저장하고 같은 프레임에 검출을 실행한다(인지 노드와 같은 detect 함수·설정).
    phases = [('target', 30), ('empty', 10)]: 단계마다 안내 후 interval_s 간격으로 저장(고르게 고르기).
    configs = {'base': config_dir, ...}: 같은 원본 프레임에 설정별 검출 결과를 따로 기록(도전 A 비교).
    labels.csv의 correct 열은 사람이 *_detect.png를 보고 채운다(목표 위 검출 1, 다른 물체 0)."""
    import cv2
    import rclpy
    from cv_bridge import CvBridge
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import CameraInfo, Image
    from target_detector.detection import detect, draw_overlay
    configs = configs or {'base': CONFIG}
    cfgs = {name: detector_config(d)[0] for name, d in configs.items()}
    os.makedirs(dest, exist_ok=True)
    if not rclpy.ok():
        rclpy.init()
    node = rclpy.create_node('assignment_eval')
    bridge = CvBridge()
    st = {'color': None, 'depth': None, 'k': None, 'n': 0}
    node.create_subscription(Image, '/camera/camera/color/image_raw',
                             lambda m: st.update(color=m, n=st['n'] + 1), qos_profile_sensor_data)
    node.create_subscription(Image, '/camera/camera/aligned_depth_to_color/image_raw',
                             lambda m: st.update(depth=m), qos_profile_sensor_data)
    node.create_subscription(CameraInfo, '/camera/camera/color/camera_info',
                             lambda m: st.update(k=(m.k[0], m.k[4], m.k[2], m.k[5])), qos_profile_sensor_data)
    if not wait_until(node, lambda: st['color'] is not None, 15):
        raise SystemExit('카메라 영상이 없습니다(perception.launch 또는 full.launch를 먼저 실행).')
    rows = []
    for phase, count in phases:
        ask(f'[{phase}] {"목표가 보이게 두세요(위치·방향을 조금씩 바꿔 가며)" if phase == "target" else "목표를 시야 밖으로 치우세요"}'
            f' — {count}장을 {interval_s} s 간격으로 저장합니다')
        for i in range(count):
            spin_for(node, interval_s)
            msg = st['color']
            img = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            depth = bridge.imgmsg_to_cv2(st['depth'], desired_encoding='passthrough') if st['depth'] is not None else None
            name = f'{phase}_{i + 1:03d}'
            cv2.imwrite(os.path.join(dest, name + '_raw.png'), img)
            for cname, cfg in cfgs.items():
                t = time.perf_counter()
                det, _ = detect(img, 'bgr8', cfg, None, depth, 0.001, st['k'])
                ms = (time.perf_counter() - t) * 1000
                view = draw_overlay(img.copy(), det)
                h, w = view.shape[:2]
                cv2.drawMarker(view, (w // 2, h // 2), (255, 255, 255), cv2.MARKER_CROSS, 20, 1)
                cv2.imwrite(os.path.join(dest, f'{name}_{cname}_detect.png'), view)
                auto = '' if (phase == 'target' and det.detected) else ('0' if phase == 'empty' and det.detected else '')
                rows.append({'frame': name, 'config': cname, 'phase': phase, 'target_visible': int(phase == 'target'),
                             'detected': int(det.detected), 'cx': det.cx, 'cy': det.cy, 'ex': det.ex,
                             'area_ratio': det.area_ratio, 'n_candidates': det.n_candidates, 'proc_ms': ms,
                             'stamp_s': msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
                             'image': f'{name}_{cname}_detect.png', 'correct': auto})
            sys.stdout.write(f'\r  {name} 저장 ({i + 1}/{count})   ')
            sys.stdout.flush()
        print()
    node.destroy_node()
    path = write_csv(os.path.join(dest, 'labels.csv'), rows)
    print(f'  저장: {path}\n  다음: 목표가 보이는데 검출된 행(correct 빈칸)을 *_detect.png로 보고 1(목표 위) 또는 0(다른 물체)을 채운 뒤 score 실행')
    return path


def score_eval(labels_path):
    """사람 대조 검출률 = 올바른 검출 / 목표가 실제 보이는 평가 프레임 x100, 배경 오검출 = 목표 없는 프레임의 검출 수.
    처리 속도 = 1000 / 평균 처리 시간 [ms] (같은 함수로 같은 프레임을 처리한 값)."""
    rows = read_csv(labels_path)
    out = []
    for cname in sorted({r['config'] for r in rows}):
        rs = [r for r in rows if r['config'] == cname]
        vis = [r for r in rs if r['target_visible'] == '1']
        emp = [r for r in rs if r['target_visible'] == '0']
        unlabeled = [r['frame'] for r in vis if r['detected'] == '1' and r['correct'] == '']
        correct = sum(r['detected'] == '1' and r['correct'] == '1' for r in vis)
        ms = [fnum(r['proc_ms']) for r in rs]
        out.append({'config': cname, 'visible_frames': len(vis), 'correct': correct,
                    'detection_rate_pct': 100 * correct / len(vis) if vis else NAN,
                    'missed': sum(r['detected'] == '0' for r in vis),
                    'wrong_object': sum(r['detected'] == '1' and r['correct'] == '0' for r in vis),
                    'empty_frames': len(emp), 'false_detections': sum(r['detected'] == '1' for r in emp),
                    'mean_proc_ms': sum(ms) / len(ms) if ms else NAN,
                    'proc_fps': 1000 / (sum(ms) / len(ms)) if ms else NAN,
                    'unlabeled': len(unlabeled)})
        if unlabeled:
            print(f'  주의 [{cname}] correct 미입력 {len(unlabeled)}개: {unlabeled[:5]} ... (미입력은 올바른 검출로 세지 않음)')
    return out


# ================= 그래프 (OpenCV, matplotlib 없이) =================
COLORS = [(200, 80, 30), (40, 140, 230), (60, 170, 60), (40, 40, 200), (160, 60, 160), (30, 160, 160)]


def line_plot(path, series, title, xlabel, ylabel, hlines=(), size=(1100, 520), ylim=None):
    """series = [(이름, xs, ys)]. 글자는 OpenCV 기본 글꼴이라 영문만 쓴다."""
    import cv2
    import numpy as np
    w, h = size
    left, right, top, bottom = 80, 200, 40, 60
    img = np.full((h, w, 3), 255, np.uint8)
    xs_all = [x for _, xs, ys in series for x, y in zip(xs, ys) if x == x and y == y]
    ys_all = [y for _, xs, ys in series for x, y in zip(xs, ys) if x == x and y == y] + [v for v, _ in hlines]
    if not xs_all:
        return None
    x0, x1 = min(xs_all), max(xs_all)
    y0, y1 = ylim or (min(ys_all), max(ys_all))
    if y1 - y0 < 1e-9:
        y0, y1 = y0 - 1, y1 + 1
    if x1 - x0 < 1e-9:
        x1 = x0 + 1
    pad = (y1 - y0) * 0.05
    y0, y1 = y0 - pad, y1 + pad

    def px(x, y):
        return (int(left + (x - x0) / (x1 - x0) * (w - left - right)), int(top + (y1 - y) / (y1 - y0) * (h - top - bottom)))

    cv2.rectangle(img, (left, top), (w - right, h - bottom), (0, 0, 0), 1)
    for i in range(6):
        yv = y0 + (y1 - y0) * i / 5
        p = px(x0, yv)
        cv2.line(img, (left, p[1]), (w - right, p[1]), (225, 225, 225), 1)
        cv2.putText(img, f'{yv:.2f}', (5, p[1] + 4), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 0), 1)
        xv = x0 + (x1 - x0) * i / 5
        p = px(xv, y0)
        cv2.putText(img, f'{xv:.1f}', (p[0] - 15, h - bottom + 18), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 0), 1)
    for v, label in hlines:
        a, b = px(x0, v), px(x1, v)
        cv2.line(img, a, b, (120, 120, 120), 1, cv2.LINE_AA)
        cv2.putText(img, label, (b[0] + 4, b[1] + 4), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (80, 80, 80), 1)
    for i, (name, xs, ys) in enumerate(series):
        c = COLORS[i % len(COLORS)]
        prev = None
        for x, y in zip(xs, ys):
            if x != x or y != y:          # NaN(예: TRACKING이 아닌 구간)에서는 선을 끊는다
                prev = None
                continue
            p = px(x, y)
            if prev is not None:
                cv2.line(img, prev, p, c, 1, cv2.LINE_AA)
            prev = p
        cv2.putText(img, name[:24], (w - right + 10, top + 20 + 18 * i), cv2.FONT_HERSHEY_SIMPLEX, 0.45, c, 1)
    cv2.putText(img, title, (left, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 1)
    cv2.putText(img, xlabel, ((w - right) // 2, h - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 0), 1)
    cv2.putText(img, ylabel, (5, top - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 0), 1)
    cv2.imwrite(path, img)
    return path


def series_from(ctl_rows, key, t0=None, only_tracking=False):
    rows = [r for r in ctl_rows if r['state'] != 'IDLE']
    if not rows:
        return [], []
    base = t0 if t0 is not None else fnum(rows[0]['ros_time_s'])
    xs, ys = [], []
    for r in rows:
        v = fnum(r[key])
        if only_tracking and r['state'] != 'TRACKING':
            v = NAN
        xs.append(fnum(r['ros_time_s']) - base)
        ys.append(v)
    return xs, ys


def bar_plot(path, groups, title, ylabel, size=(900, 480)):
    """groups = [(이름, 값)]. 막대 위에 값을 적는다(영문 이름)."""
    import cv2
    import numpy as np
    w, h = size
    left, bottom, top = 70, 90, 40
    img = np.full((h, w, 3), 255, np.uint8)
    vals = [v for _, v in groups if v == v]
    vmax = max(vals + [1e-9]) * 1.15
    bw = (w - left - 20) / max(len(groups), 1)
    cv2.line(img, (left, h - bottom), (w - 10, h - bottom), (0, 0, 0), 1)
    for i, (name, v) in enumerate(groups):
        x0 = int(left + i * bw + bw * 0.15)
        x1 = int(left + (i + 1) * bw - bw * 0.15)
        if v == v:
            y = int(h - bottom - v / vmax * (h - bottom - top))
            cv2.rectangle(img, (x0, y), (x1, h - bottom), COLORS[i % len(COLORS)], -1)
            cv2.putText(img, f'{v:.3g}', (x0, y - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 0), 1)
        cv2.putText(img, name[:18], (x0, h - bottom + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 0), 1)
    cv2.putText(img, title, (left, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 1)
    cv2.putText(img, ylabel, (5, top - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 0), 1)
    cv2.imwrite(path, img)
    return path
