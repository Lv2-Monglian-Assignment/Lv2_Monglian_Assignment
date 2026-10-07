#!/usr/bin/env python3
"""문제 1 — HSV·Contour 검출 파이프라인: 세 장면(정상·대상 없음·일부 가림) 증거와 카메라·설정 기록.

같은 설정(config/hsv.yaml)으로 인지 노드(target_detector)가 실제로 처리한 영상을 저장한다.
장면마다 원본·마스크·검출(컨투어·목표 중심·영상 중심 표시) 이미지와 그 프레임의 /target 값을 남긴다.

  python3 assignment/assignment1.py                 # 카메라+인지를 띄우고 세 장면 안내 (모터 사용 안 함)
  python3 assignment/assignment1.py --no-launch     # 이미 perception/full launch가 떠 있을 때
결과: results/assignment1/<시각>/ (scenes.csv, camera.txt, config/, 이미지) · results/images/assignment1_*.png
심화(조명·거리 조건 변경 비교)는 assignment_A.py
"""
import argparse
import getpass
import glob
import os
import shutil
import socket
import subprocess
import sys
import time

import common as C


SCENES = [('normal', '목표가 잘 보이는 장면'), ('empty', '목표가 없는 장면(시야 밖으로 치움)'),
          ('occluded', '목표 일부를 손이나 물체로 가린 장면')]


def camera_record(node, dest):
    """D435 프로파일·encoding·CameraInfo(K)·frame_id·stamp·버전 기록 (발제 평가표 3번)"""
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import CameraInfo, Image
    got = {'color': [], 'depth': [], 'info': None}
    node.create_subscription(Image, '/camera/camera/color/image_raw',
                             lambda m: got['color'].append((time.time(), m.header, m.encoding, m.width, m.height)),
                             qos_profile_sensor_data)
    node.create_subscription(Image, '/camera/camera/aligned_depth_to_color/image_raw',
                             lambda m: got['depth'].append((time.time(), m.encoding, m.width, m.height)),
                             qos_profile_sensor_data)
    node.create_subscription(CameraInfo, '/camera/camera/color/camera_info', lambda m: got.update(info=m),
                             qos_profile_sensor_data)
    C.spin_for(node, 3.0)
    lines = [f'측정: {time.strftime("%Y-%m-%d %H:%M:%S")}, 커밋 {C.git_commit()}']
    if got['color']:
        t, hdr, enc, w, h = got['color'][-1]
        n = len(got['color'])
        rate = (n - 1) / (got['color'][-1][0] - got['color'][0][0]) if n > 1 else float('nan')
        stamp = hdr.stamp.sec + hdr.stamp.nanosec * 1e-9
        lines += [f'Color: {w}x{h} {enc}, 수신 {rate:.1f} Hz (3 s), frame_id {hdr.frame_id}',
                  f'  stamp {stamp:.3f} / 수신 시각 {t:.3f} (차이 {t - stamp:+.3f} s, 시스템 시계 기준이면 작음)']
    else:
        lines.append('Color: 수신 없음')
    if got['depth']:
        t, enc, w, h = got['depth'][-1]
        n = len(got['depth'])
        rate = (n - 1) / (got['depth'][-1][0] - got['depth'][0][0]) if n > 1 else float('nan')
        lines.append(f'정렬 Depth: {w}x{h} {enc} (mm), 수신 {rate:.1f} Hz')
    else:
        lines.append('정렬 Depth: 수신 없음')
    if got['info']:
        k = got['info'].k
        lines.append(f'CameraInfo: {got["info"].width}x{got["info"].height}, fx {k[0]:.2f} fy {k[4]:.2f} '
                     f'cx {k[2]:.2f} cy {k[5]:.2f}, 왜곡 {got["info"].distortion_model} {list(got["info"].d)}')
    pk = subprocess.run(['dpkg-query', '-W', '-f', '${Package} ${Version}\n', 'ros-lyrical-realsense2-camera',
                         'ros-lyrical-librealsense2', 'python3-opencv'], capture_output=True, text=True).stdout
    lines += ['패키지:'] + ['  ' + s for s in pk.strip().splitlines()]
    usb = subprocess.run(['bash', '-c', 'lsusb -t | grep -B1 -i "uvcvideo" | head -4'], capture_output=True, text=True).stdout
    lines += ['USB (lsusb -t, 5000M = USB 3):'] + ['  ' + s for s in usb.strip().splitlines()]
    path = os.path.join(dest, 'camera.txt')
    open(path, 'w').write('\n'.join(lines) + '\n')
    print('\n'.join('  ' + s for s in lines))
    return path


def montage(dest, rows):
    """세 장면의 검출 이미지를 한 장으로 붙인다(장면 이름·검출 결과 표시). 경로를 돌려준다."""
    import cv2
    import numpy as np
    tiles = []
    for r in rows:
        f = [x for x in r['images'].split() if x.endswith('_detect.png')]
        img = cv2.imread(os.path.join(dest, f[0])) if f else None
        if img is None:
            continue
        img = cv2.resize(img, (426, 320))
        cv2.rectangle(img, (0, 0), (426, 26), (0, 0, 0), -1)
        cv2.putText(img, f'{r["scene"]}  detected={r.get("detected", "?")}  area={r.get("area_ratio", "?")}', (6, 18),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        tiles.append(img)
    if not tiles:
        return None
    path = os.path.join(dest, 'scenes_montage.png')
    cv2.imwrite(path, np.hstack(tiles))
    shutil.copy2(path, os.path.join(C.out_dir('images'), 'assignment1_scenes_montage.png'))
    return path


def show_or_point(path, dest):
    """화면이 있으면(ssh -X 등) 창으로 보여 주고, 없으면 PC에서 가져오는 명령을 알려 준다."""
    host = f'{getpass.getuser()}@{socket.gethostname()}.local'
    print(f'\n이미지 보기 (PC 터미널): scp -r {host}:{dest} ~/Downloads/')
    if path and os.environ.get('DISPLAY'):
        import select
        import cv2
        title = 'assignment1 scenes (normal / empty / occluded)'
        print('  창이 열립니다. 창에서 아무 키를 누르면 닫힘 (창 닫기 버튼·터미널 Enter·Ctrl+C로도 닫힘)')
        cv2.imshow(title, cv2.imread(path))
        try:
            # waitKey(0)은 창 키만 기다리고 Ctrl+C도 막는다(2026-10-07 메뉴가 멈춤). 짧게 나눠 기다리며 사람이 닫을 때까지 둔다
            while True:
                if cv2.waitKey(100) != -1 or cv2.getWindowProperty(title, cv2.WND_PROP_VISIBLE) < 1:
                    break
                if select.select([sys.stdin], [], [], 0)[0]:
                    sys.stdin.readline()
                    break
        finally:
            cv2.destroyAllWindows()
            cv2.waitKey(1)
    elif path:
        print(f'  (화면 없음: 세 장면 합친 이미지 {os.path.relpath(path, C.LV2)})')


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('--no-launch', action='store_true', help='이미 실행 중인 카메라·인지 노드를 사용')
    args = ap.parse_args()
    C.check_ros()
    run_id = f'assignment1_{C.tag()}'
    dest = C.out_dir('assignment1', run_id)
    shutil.copytree(C.CONFIG, os.path.join(dest, 'config'))       # 이 시험에 쓴 설정 그대로
    proc = None
    if not args.no_launch:
        proc = C.launch('perception.launch.py', os.path.join(dest, 'launch.log'), run_id=run_id)
    node, st = C.make_probe()
    from std_msgs.msg import String
    snap = node.create_publisher(String, '/target/save_snapshot', 10)
    try:
        if not C.wait_until(node, lambda: st['target'] is not None, 40):
            raise SystemExit('/target이 없습니다(카메라·인지 노드 확인)')
        det_run = run_id if proc else None
        print('카메라·설정 기록:')
        camera_record(node, dest)
        rows = []
        for scene, guide in SCENES:
            C.ask(f'[{scene}] {guide}을 만드세요')
            C.spin_for(node, 1.0)
            label = f'{scene}_{C.tag()}'
            asked = time.time()
            snap.publish(String(data=label))
            C.spin_for(node, 1.5)
            files = sorted(glob.glob(os.path.join(C.SNAP_DIR, f'*_{label}*.png')))
            files = [f for f in files if os.path.getmtime(f) >= asked - 1]
            if not files:
                print('  이미지가 저장되지 않았습니다(인지 노드 save_dir 확인)')
                continue
            for f in files:
                shutil.copy2(f, dest)
                kind = f.rsplit('_', 1)[-1]          # raw.png / mask.png / detect.png
                shutil.copy2(f, os.path.join(C.out_dir('images'), f'assignment1_{scene}_{kind}'))
            seq = int(os.path.basename(files[0]).split('_f')[-1].split('_')[0]) if '_f' in files[0] else None
            row = {'scene': scene, 'label': label, 'images': ' '.join(os.path.basename(f) for f in files), 'frame_seq': seq}
            det_csv = None
            if det_run:
                det_csv = C.node_logs(det_run)['detect']
            else:
                cands = sorted(glob.glob(os.path.join(C.LOG_DIR, '*_detect.csv')), key=os.path.getmtime)
                det_csv = cands[-1] if cands else None
            if det_csv and seq is not None:
                C.spin_for(node, 1.2)      # 인지 CSV는 1 s마다 저장된다
                hit = [r for r in C.read_csv(det_csv) if r['frame_seq'] == str(seq)]
                if hit:
                    row.update({k: hit[0][k] for k in ('detected', 'cx_px', 'cy_px', 'ex', 'ey', 'area_ratio',
                                                       'n_candidates', 'n_rejected', 'depth_valid', 'z_m', 'proc_ms')})
            rows.append(row)
            print(f'  {scene}: detected={row.get("detected", "?")} ex={row.get("ex", "?")} '
                  f'area={row.get("area_ratio", "?")} 후보 {row.get("n_candidates", "?")}개 → {len(files)}장 저장')
        C.write_csv(os.path.join(dest, 'scenes.csv'), rows)
        hsv = C.read_param('hsv_lower'), C.read_param('hsv_upper'), C.read_param('min_area_px')
        summary = [f'# 문제 1 세 장면 ({run_id})', '',
                   f'- 설정: HSV {hsv[0]}~{hsv[1]}, 최소 면적 {hsv[2]} px², 크기 검증 {C.read_param("obj_area_min_cm2")}'
                   f'~{C.read_param("obj_area_max_cm2")} cm², 선택 {C.read_param("selection")} (config/ 복사본)',
                   '- 미검출 전달: 정상 영상에서 목표가 없으면 /target z=0 (x=y=0) 발행, 이전 좌표 재사용 없음', '',
                   C.md_table(rows, ['scene', 'detected', 'ex', 'ey', 'area_ratio', 'n_candidates', 'n_rejected', 'z_m', 'images']),
                   '', '카메라 기록: camera.txt']
        open(os.path.join(dest, 'summary.md'), 'w').write('\n'.join(summary) + '\n')
        print(f'\n결과: {dest}')
        show_or_point(montage(dest, rows), dest)
    finally:
        node.destroy_node()
        if proc:
            proc.stop()


if __name__ == '__main__':
    main()
