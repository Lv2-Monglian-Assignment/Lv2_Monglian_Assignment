#!/usr/bin/env python3
"""Lv2 모듈 5 통합 실행 메뉴: 숫자 키 하나로 문제·도전 실습 프로그램을 실행한다 (Raspberry Pi).

부팅 때 tmux 세션 lv2 안에서 자동 실행된다(scripts/test/install_autostart.sh). SSH로 접속해 붙는다:
  tmux attach -t lv2          (또는 설치 후 lv2)
직접 실행:
  cd ~/git/Lv2_Monglian_Assignment/lv2_module5 && source ros2_ws/install/setup.bash && python3 assignment/main.py

입력이 없으면(IDLE) 10 s 뒤 최종 추적(full.launch: 카메라·인지·제어·브리지, 추적 켬)이 백그라운드로 실행된다.
다른 키를 누르면 추적을 먼저 멈추고(카메라·시리얼 공유) 그 프로그램을 실행한다. 끝나면 바로 다시 추적한다.
키 (Enter 없이 바로): 화면의 입출력 표 참고. 스페이스 = 대기 추적 끄기/켜기, z = 화면 나가기(메뉴·추적 계속), x = 끝내기.
실행 중인 프로그램은 Ctrl+C로 멈추고 메뉴로 돌아온다(메뉴는 Ctrl+C에 끝나지 않음).
환경 변수: LV2_IDLE_DELAY(대기 초, 기본 10), LV2_MENU_ECHO=1(명령만 보여 주는 확인 모드)
"""
import glob
import os
import re
import select
import signal
import subprocess
import sys
import termios
import time
import tty
import unicodedata

LV2 = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
A = os.path.join(LV2, 'assignment')
T = os.path.join(LV2, 'scripts', 'test')
PY = sys.executable

IDLE_DELAY_S = float(os.environ.get('LV2_IDLE_DELAY', 10))
ECHO = bool(os.environ.get('LV2_MENU_ECHO'))
LOGS = os.path.expanduser('~/lv2_module5_logs')

# 입출력 표: (키, 입력하면 실행되는 것, 출력·결과 위치)
TABLE = [
    ('(없음)', f'입력 없이 {IDLE_DELAY_S:g} s → 최종 추적 (카메라·인지·제어·브리지, 추적 켬)', '모터 추적 · ~/lv2_module5_logs/idle_<시각>*.csv'),
    ('1', '문제 1  세 장면(정상·없음·가림) 검출 기록', 'results/assignment1/ · results/images/'),
    ('2', '문제 2  모의 입력 7종 판정 (모터 출력 없음)', 'results/assignment2/ (PASS/FAIL 표)'),
    ('3', '문제 3  Kp 시험 r / 분석 a', 'results/assignment3/ · plots/assignment3_*'),
    ('4', '문제 4  정상 n·검출률 e/s·가림 o·입력중단 t·통신중단 c/b', 'results/assignment4/ · metrics.csv'),
    ('5', '문제 5  bag 기록 s/l · 재처리 p · 재분석 a · 재현 w', 'recordings/ · results/assignment5/'),
    ('6', '도전 A  조건별 평가 프레임 c / 비교 s', 'results/assignment_A/'),
    ('7', '도전 B  인터페이스 i / SEARCHING s (모터 없음)', 'results/assignment_B/'),
    ('8', '도전 C  데드밴드 시험 r / 분석 a', 'results/assignment_C/'),
    ('9', '도전 D  가림 반복 r / 비교 a', 'results/assignment_D/'),
    ('0', '도전 E  고정 bag 회귀 비교 (모터 없음)', 'results/assignment_E/'),
    ('t', '대기 없이 바로 최종 추적 시작', '(없음)과 같음'),
    ('스페이스', '대기 추적 끄기 / 켜기', '추적 정지: X 전송, 토크 유지'),
    ('p', '기준 자세 설정 (pose_tool)', 'config/device.yaml home_ticks'),
    ('d · f', '방향 시험 · 정지 시험', '~/lv2_module5_logs/direction_test_* · fw_test_*'),
    ('e', '환경 확인', 'results/logs/env_<시각>.txt'),
    ('z · x', '원래 세션으로 돌아가기(메뉴·추적 계속) · 메뉴 끝내기(추적 정지)', ''),
]
PROGRAM_KEYS = set('1234567890pdfe')
HANGUL = {'ㅅ': 't', 'ㅔ': 'p', 'ㅇ': 'd', 'ㄹ': 'f', 'ㄷ': 'e', 'ㅋ': 'z', 'ㅌ': 'x'}


def width(text):
    return sum(2 if unicodedata.east_asian_width(c) in 'WF' else 1 for c in text)


def pad(text, n):
    return text + ' ' * max(0, n - width(text))


# ---------------- 입력 도우미 ----------------
def ask(prompt, default=None, cast=str):
    """기본값이 있으면 Enter로 받는다. 잘못된 값이면 다시 묻는다."""
    while True:
        s = input(f'  {prompt}' + (f' [{default}]' if default is not None else '') + ': ').strip()
        if not s and default is not None:
            s = str(default)
        try:
            return cast(s)
        except ValueError:
            print('    다시 입력하세요')


def choose(prompt, options):
    """options = [(키, 설명)]. 한 글자를 입력받아 키를 돌려준다(빈 입력은 취소)."""
    for k, d in options:
        print(f'    {k}  {d}')
    while True:
        s = input(f'  {prompt} (Enter: 취소): ').strip().lower()
        if not s:
            return None
        if s in [k for k, _ in options]:
            return s
        print('    목록의 글자를 입력하세요')


def yes(prompt, default=False):
    s = input(f'  {prompt} [{"Y/n" if default else "y/N"}]: ').strip().lower()
    return default if not s else s in ('y', 'yes', 'ㅛ')


def pick(prompt, paths, multi=False):
    """최근 순 목록에서 번호로 고른다."""
    paths = sorted(paths, key=os.path.getmtime, reverse=True)[:15]
    if not paths:
        print('    선택할 항목이 없습니다')
        return [] if multi else None
    for i, p in enumerate(paths, 1):
        print(f'    {i:2d}  {os.path.relpath(p, LV2)}')
    s = input(f'  {prompt}' + (' (여러 개는 공백으로)' if multi else '') + ': ').split()
    try:
        sel = [paths[int(x) - 1] for x in s]
    except (ValueError, IndexError):
        print('    번호가 올바르지 않습니다')
        return [] if multi else None
    return sel if multi else (sel[0] if sel else None)


def bags():
    return [d for d in glob.glob(os.path.join(LV2, 'recordings', '*')) if os.path.isdir(d)]


# ---------------- 메뉴 항목 → 실행할 명령 ----------------
def cmd_for(key):
    if key == '1':
        return [PY, f'{A}/assignment1.py']
    if key == '2':
        return [PY, f'{A}/assignment2.py']
    if key == '3':
        k = choose('3', [('r', '1회 시험 (왼쪽→중앙→오른쪽→중앙, 각 3 s)'), ('a', '분석: 회차별 지표·Kp별 평균·그래프')])
        if k == 'r':
            c = [PY, f'{A}/assignment3.py', 'run', '--pan-kp', str(ask('팬 Kp [1/s]', 2.0, float)),
                 '--trial', str(ask('회차', 1, int))]
            return c + (['--dry-run'] if yes('모터 출력 없이(dry_run)?') else [])
        return [PY, f'{A}/assignment3.py', 'analyze'] if k == 'a' else None
    if key == '4':
        k = choose('4', [('n', '정상 추적 30 s 이상'), ('e', '검출률 평가 프레임 저장 (목표 30 + 없음 10)'),
                         ('s', '검출률 점수 (labels.csv 대조 후)'), ('o', '가림 후 재등장 5회'),
                         ('t', '인지 입력 중단 (검출 노드 kill -9)'), ('c', '제어 통신 중단: 제어 노드 kill -9'),
                         ('b', '제어 통신 중단: 브리지 kill -9 (OpenCR 타임아웃)')])
        if k == 'n':
            return [PY, f'{A}/assignment4.py', 'normal', '--seconds', str(ask('기록 시간 [s]', 35, float))]
        if k == 'e':
            return [PY, f'{A}/assignment4.py', 'eval']
        if k == 's':
            p = pick('labels.csv 번호', glob.glob(os.path.join(LV2, 'results', 'assignment4', 'eval_*', 'labels.csv')))
            return [PY, f'{A}/assignment4.py', 'score', p] if p else None
        if k == 'o':
            return [PY, f'{A}/assignment4.py', 'occlusion', '--trials', str(ask('회수', 5, int))]
        if k == 't':
            return [PY, f'{A}/assignment4.py', 'topic-stop']
        if k in ('c', 'b'):
            return [PY, f'{A}/assignment4.py', 'control-stop', '--node', 'controller' if k == 'c' else 'bridge']
        return None
    if key == '5':
        k = choose('5', [('s', '성공 장면 bag 기록'), ('l', '소실·복귀 장면 bag 기록'), ('p', '입력 재처리 (bag → /target_replay)'),
                         ('a', '결과 재분석 (bag의 /target·상태·명령)'), ('w', '다른 팀원 재현 기록')])
        if k in ('s', 'l'):
            return [PY, f'{A}/assignment5.py', 'record', '--name', 'success' if k == 's' else 'lost',
                    '--seconds', str(ask('기록 시간 [s] (10~30)', 20, float))]
        if k in ('p', 'a'):
            b = pick('bag 번호', bags())
            return [PY, f'{A}/assignment5.py', 'replay' if k == 'p' else 'reanalyze', b] if b else None
        if k == 'w':
            return [PY, f'{A}/assignment5.py', 'reproduce', '--who', ask('확인자 이름')]
        return None
    if key == '6':
        k = choose('A', [('c', '조건 하나에서 평가 프레임 저장'), ('s', '조건별 점수·비교 (labels.csv 대조 후)')])
        if k == 'c':
            c = [PY, f'{A}/assignment_A.py', 'capture', '--condition', ask('조건 이름(영문, 예: dim_0.6m)')]
            if yes('개선 설정(검출 설정 하나 변경)도 같은 프레임에 적용?'):
                c += ['--param', ask('설정 키', 'obj_area_min_cm2'), '--value', ask('값')]
            return c
        if k == 's':
            ds = pick('비교할 조건 폴더 번호', glob.glob(os.path.join(LV2, 'results', 'assignment_A', '*_2*')), multi=True)
            return [PY, f'{A}/assignment_A.py', 'score', *ds] if ds else None
        return None
    if key == '7':
        k = choose('B', [('i', '인터페이스 재확인 (모의 입력)'), ('s', 'SEARCHING 성공·취소·미발견')])
        return [PY, f'{A}/assignment_B.py', {'i': 'interface', 's': 'search'}[k]] if k else None
    if key == '8':
        k = choose('C', [('r', '1회 시험'), ('a', '분석')])
        param = 'pan_deadband'
        if k:
            param = {'p': 'pan_deadband', 't': 'tilt_deadband'}[choose('바꿀 값', [('p', '팬 데드밴드 (기본 0.03)'),
                                                                                   ('t', '틸트 데드밴드 (기본 0.05)')]) or 'p']
        if k == 'r':
            c = [PY, f'{A}/assignment_C.py', 'run', '--param', param, '--value', str(ask('값', 0.03, float)),
                 '--trial', str(ask('회차', 1, int))]
            return c + (['--dry-run'] if yes('모터 출력 없이(dry_run)?') else [])
        return [PY, f'{A}/assignment_C.py', 'analyze', '--param', param] if k == 'a' else None
    if key == '9':
        k = choose('D', [('r', '가림 반복 시험'), ('a', '설정별 비교')])
        if k == 'r':
            c = [PY, f'{A}/assignment_D.py', 'run', '--trials', str(ask('회수', 10, int))]
            p = choose('바꿀 조건(Enter: 기본 설정)', [('1', 'recover_frames (기본 3)'), ('2', 'relock_after_s (기본 3.0)'),
                                                    ('3', 'input_timeout_s (기본 0.5)')])
            if p:
                c += ['--param', {'1': 'recover_frames', '2': 'relock_after_s', '3': 'input_timeout_s'}[p],
                      '--value', str(ask('값', None, float))]
            return c
        return [PY, f'{A}/assignment_D.py', 'analyze'] if k == 'a' else None
    if key == '0':
        b = pick('기준 bag 번호', bags())
        if not b:
            return None
        return [PY, f'{A}/assignment_E.py', '--bag', b, '--param', ask('바꿀 검출 설정 키', 'min_area_px'),
                '--value', ask('변경 값')]
    if key == 'p':
        return [PY, f'{T}/pose_tool.py']
    if key == 'd':
        return [PY, f'{T}/direction_test.py']
    if key == 'f':
        return [PY, f'{T}/fw_test.py', '--home', '--hold-check']
    if key == 'e':
        return ['bash', os.path.join(LV2, 'scripts', 'check_env.sh')]
    return None


# ---------------- 실행 ----------------
def child_env():
    """메뉴 세션의 최신 DISPLAY(lv2 명령·tmux 접속 때 갱신)를 자식 프로그램에 넘긴다. 메뉴는 부팅 때 화면 없이 시작했기 때문."""
    env = os.environ.copy()
    if os.environ.get('TMUX'):
        r = subprocess.run(['tmux', 'show-environment', 'DISPLAY'], capture_output=True, text=True).stdout.strip()
        if r.startswith('DISPLAY='):
            env['DISPLAY'] = r.split('=', 1)[1]
        elif r == '-DISPLAY':
            env.pop('DISPLAY', None)
    return env


def run(cmd):
    """자식은 기본 SIGINT 처리(Ctrl+C로 멈춤), 메뉴는 그동안 SIGINT를 무시한다."""
    print(f'\n$ {" ".join(cmd)}\n')
    if os.environ.get('LV2_MENU_ECHO'):          # 확인용: 실행하지 않고 명령만 보여 줌
        print('[확인 모드: 실행 안 함] 3 s 뒤 추적 상태로 돌아갑니다')
        getkey(timeout=3.0)
        return
    old = signal.signal(signal.SIGINT, signal.SIG_IGN)
    t0 = time.time()
    try:
        env = child_env()
        if env.get('DISPLAY'):
            print(f'(화면 {env["DISPLAY"]}: 이미지 창은 ssh -X로 접속한 PC에 뜸)')
        rc = subprocess.run(cmd, cwd=LV2, env=env,
                            preexec_fn=lambda: signal.signal(signal.SIGINT, signal.SIG_DFL)).returncode
    except FileNotFoundError as e:
        rc = f'실행 실패: {e}'
    finally:
        signal.signal(signal.SIGINT, old)
    # 끝나면 키를 기다리지 않고 메뉴(추적 상태)로 돌아간다. 결과를 읽을 수 있게 3 s만 보여 준다(아무 키로 바로 넘어감)
    print(f'\n[끝: 종료 코드 {rc}, {time.time() - t0:.0f} s] 3 s 뒤 추적 상태로 돌아갑니다')
    getkey(timeout=3.0)


def getkey(timeout=None):
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)
        if timeout is not None and not select.select([sys.stdin], [], [], timeout)[0]:
            return None
        return sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


class IdleTracker:
    """입력이 없을 때 도는 최종 추적(full.launch, auto_enable). 새 프로세스 그룹으로 띄우고 SIGINT로 정리한다."""

    STATE_RE = re.compile(r'\[tracker_controller\]: state (\S+) -> (\S+) \(([^)]*)\)')

    def __init__(self):
        self.p, self.run_id, self.log_path, self.started = None, None, None, 0.0
        self.paused = False

    def running(self):
        return ECHO and self.run_id is not None or (self.p is not None and self.p.poll() is None)

    def start(self):
        self.run_id = f'idle_{time.strftime("%Y%m%d_%H%M%S")}'
        os.makedirs(LOGS, exist_ok=True)
        self.log_path = os.path.join(LOGS, f'{self.run_id}_launch.log')
        self.started = time.time()
        if ECHO:
            return
        self.p = subprocess.Popen(['ros2', 'launch', 'tracker_bringup', 'full.launch.py', f'run_id:={self.run_id}',
                                   'auto_enable:=true'], cwd=LV2, stdout=open(self.log_path, 'w'), stderr=subprocess.STDOUT,
                                  start_new_session=True, preexec_fn=lambda: signal.signal(signal.SIGINT, signal.SIG_DFL))

    def stop(self):
        if ECHO:
            self.run_id = None
            return
        if self.p is not None and self.p.poll() is None:
            os.killpg(self.p.pid, signal.SIGINT)        # 출력보다 먼저 보낸다(세션이 닫혀 터미널이 없으면 print가 실패함)
            try:
                print('  최종 추적 정지 중 (브리지가 X를 보내고 토크 유지)...', flush=True)
            except OSError:
                pass
            try:
                self.p.wait(20)
            except subprocess.TimeoutExpired:
                os.killpg(self.p.pid, signal.SIGTERM)
                try:
                    self.p.wait(5)
                except subprocess.TimeoutExpired:
                    os.killpg(self.p.pid, signal.SIGKILL)
        self.p = None

    def state(self):
        """제어 기록(CSV, 1 s마다 저장)의 마지막 줄 상태:사유. 노드가 죽었으면 launch 기록의 오류."""
        if ECHO:
            return '확인 모드(실행 안 함)'
        try:
            with open(os.path.join(LOGS, f'{self.run_id}.csv'), errors='replace') as f:
                header = f.readline().strip().split(',')
                f.seek(max(0, os.path.getsize(f.name) - 4000))
                last = f.read().strip().splitlines()[-1].split(',')
            if len(last) == len(header) and last[0] == self.run_id:
                row = dict(zip(header, last))
                live = f'{row["state"]} ({row["reason"]})'
            else:
                live = ''
        except (OSError, IndexError, KeyError):
            live = ''
        try:
            with open(self.log_path, errors='replace') as f:
                f.seek(max(0, os.path.getsize(self.log_path) - 20000))
                tail = f.read()
        except OSError:
            return ''
        died = [l for l in tail.splitlines() if 'process has died' in l]
        if died:
            return '노드 종료됨: ' + died[-1].split(']: ')[-1][:60]
        if live:
            return live
        st = self.STATE_RE.findall(tail)          # 제어 기록이 아직 없으면 launch 기록의 마지막 상태 전이
        return f'{st[-1][1]} ({st[-1][2]})' if st else '시작 중'


def devices_ready():
    """최종 추적을 띄울 수 있는지: OpenCR 포트가 있고 비어 있으며 카메라가 연결됨"""
    if ECHO:
        return True, ''
    if not os.path.exists('/dev/ttyACM0'):
        return False, 'OpenCR 없음'
    if subprocess.run(['fuser', '/dev/ttyACM0'], capture_output=True).returncode == 0:
        return False, '/dev/ttyACM0 사용 중(다른 프로그램)'
    if subprocess.run(['bash', '-c', 'lsusb | grep -q 8086:0b'], capture_output=True).returncode != 0:
        return False, '카메라 없음'
    return True, ''


def status_line():
    parts = [f'ROS_DOMAIN_ID {os.environ.get("ROS_DOMAIN_ID", "-")}']
    if 'tracker_bringup' not in os.environ.get('AMENT_PREFIX_PATH', ''):
        parts.append('워크스페이스 미적용(source ros2_ws/install/setup.bash)')
    return ' · '.join(parts)


def draw(trk, idle_since, note):
    print('\033[2J\033[H', end='')
    print('━━━━━━━━━━ Lv2 모듈 5 비전 객체 추적 · 통합 메뉴 ━━━━━━━━━━')
    print(f'  {status_line()}')
    if trk.running():
        print(f'  ● 최종 추적 실행 중 [{trk.run_id}] {int(time.time() - trk.started)} s · 상태 {trk.state()}  (스페이스: 끄기)')
    elif trk.paused:
        print('  ○ 대기 추적 꺼짐 (스페이스: 켜기, t: 바로 시작)')
    else:
        left = max(0.0, IDLE_DELAY_S - (time.time() - idle_since))
        print(f'  ○ 입력이 없으면 {left:4.1f} s 뒤 최종 추적 시작' + (f'  — 대기: {note}' if note else ''))
    print()
    print('  ' + pad('키', 9) + pad('입력하면', 66) + '출력·결과')
    print('  ' + '─' * 110)
    for k, what, out in TABLE:
        print('  ' + pad(k, 9) + pad(what, 66) + out)
    print('\n키를 누르세요: ', end='', flush=True)


def main():
    os.chdir(LV2)
    signal.signal(signal.SIGINT, lambda *_: print('\n  (메뉴는 x로 끝냅니다)'))
    # tmux 세션이 닫히거나(SIGHUP) 종료 요청(SIGTERM)을 받아도 아래 finally에서 최종 추적을 멈추고 끝낸다
    # (추적은 별도 프로세스 그룹이라 메뉴만 죽으면 노드가 남아 포트를 계속 잡음, 2026-10-07)
    def closing(*_):
        sys.stdout = sys.stderr = open(os.devnull, 'w')   # 닫힌 터미널에 쓰다 정리가 중단되지 않게
        raise SystemExit(0)
    for sig in (signal.SIGHUP, signal.SIGTERM):
        signal.signal(sig, closing)
    trk = IdleTracker()
    idle_since = time.time()
    note = ''
    try:
        while True:
            if not trk.running() and not trk.paused and time.time() - idle_since >= IDLE_DELAY_S:
                ok, note = devices_ready()
                if ok:
                    trk.start()
                else:
                    idle_since = time.time()           # 장치가 준비될 때까지 대기 시간을 다시 센다
            draw(trk, idle_since, note)
            k = getkey(timeout=1.0)
            if k is None:
                continue
            k = HANGUL.get(k, k.lower())
            idle_since = time.time()
            if k == ' ':
                trk.paused = not trk.paused
                if trk.paused:
                    trk.stop()
                else:
                    idle_since = time.time() - IDLE_DELAY_S     # 켜면 바로 시작
                continue
            if k == 't':
                trk.paused = False
                if not trk.running():
                    ok, note = devices_ready()
                    trk.start() if ok else None
                continue
            if k == 'x':
                print(k)
                if yes('메뉴를 끝낼까요? (최종 추적도 멈춤)'):
                    break
                continue
            if k == 'z':      # lv2 명령으로 전환해 왔으면 원래 세션(예: pi)으로, 아니면 tmux에서 나감
                if os.environ.get('TMUX') and subprocess.run(['tmux', 'switch-client', '-l'], capture_output=True).returncode != 0:
                    subprocess.run(['tmux', 'detach-client'])
                continue
            if k not in PROGRAM_KEYS:
                continue
            print(k)
            if trk.running():
                trk.stop()
            signal.signal(signal.SIGINT, signal.default_int_handler)    # 하위 질문은 Ctrl+C로 취소
            try:
                cmd = cmd_for(k)
            except (KeyboardInterrupt, EOFError):
                print('\n  취소')
                cmd = None
            finally:
                signal.signal(signal.SIGINT, lambda *_: print('\n  (메뉴는 x로 끝냅니다)'))
            if cmd:
                run(cmd)
                idle_since = time.time() - IDLE_DELAY_S    # 프로그램이 끝나면 기다리지 않고 바로 최종 추적
            else:
                idle_since = time.time()
    finally:
        trk.stop()


if __name__ == '__main__':
    main()
