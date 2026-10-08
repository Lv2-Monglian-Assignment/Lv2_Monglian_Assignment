"""bag 입력 재처리: bag의 영상으로 target_detector만 다시 돌리고 결과는 /target_replay로 분리한다.
카메라·제어·브리지는 띄우지 않는다(모터 출력 없음). 저장된 /target과 섞이지 않게 출력 토픽 3개를 모두 바꾼다.

터미널 1: ros2 launch tracker_bringup replay.launch.py
터미널 2: ros2 bag play recordings/<run_id> --clock
선택 인자: bag:=<bag 폴더>(주면 이 launch가 영상·CameraInfo·정렬 Depth·모터 각도만 --clock으로 함께 재생),
          run_id:=<이름>(재처리 기록 <run_id>_detect.csv, 기본 auto), config_dir:=<폴더>(다른 설정으로 재처리, 회귀 비교),
          save_every_n:=<N>(N 프레임마다 재처리 원본·마스크·검출 이미지 저장, 기본 0=끔)
"""
import atexit
import os
import tempfile
from glob import glob

import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

DEFAULT_CONFIG = os.path.join(get_package_share_directory('tracker_bringup'), 'config')


def override(node, **values):
    """launch 인자 값을 config와 같은 노드 이름 항목의 임시 yaml로 써서 맨 뒤 파일로 넘긴다.
    dict(와일드카드 /**)나 ros_arguments의 -p로 넘기면 config/*.yaml의 노드 이름 항목에 덮여 무시된다(2026-10-07 시험)."""
    f = tempfile.NamedTemporaryFile('w', prefix=f'{node}_launch_', suffix='.yaml', delete=False)
    atexit.register(lambda path=f.name: os.path.exists(path) and os.remove(path))   # launch가 끝나면 지운다(/tmp에 쌓임)
    yaml.safe_dump({node: {'ros__parameters': values}}, f)
    f.close()
    return f.name


def as_bool(text):
    return str(text).lower() in ('true', '1', 'yes')

PLAY_TOPICS = ['/camera/camera/color/image_raw', '/camera/camera/color/camera_info',
               '/camera/camera/aligned_depth_to_color/image_raw', '/pan_tilt/joint_states']


def _setup(context):
    configs = sorted(glob(os.path.join(LaunchConfiguration('config_dir').perform(context), '*.yaml')))
    actions = [Node(package='target_detector', executable='target_detector', name='target_detector',
                    parameters=configs + [override('target_detector', use_sim_time=True,
                                                   target_topic='/target_replay',
                                                   depth_out_topic='/target_replay/depth',
                                                   position_topic='/target_replay/position_cam',
                                                   run_id=LaunchConfiguration('run_id').perform(context),
                                                   save_every_n=int(LaunchConfiguration('save_every_n').perform(context)))],
                    output='screen')]
    bag = LaunchConfiguration('bag').perform(context)
    if bag:   # 저장된 /target·상태·명령은 재생하지 않는다(새 검출 결과와 섞지 않음)
        actions.append(ExecuteProcess(cmd=['ros2', 'bag', 'play', bag, '--clock', '--topics', *PLAY_TOPICS], output='screen'))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('bag', default_value='', description='함께 재생할 bag 폴더 (비우면 검출기만 실행)'),
        DeclareLaunchArgument('run_id', default_value='auto', description='재처리 기록 이름'),
        DeclareLaunchArgument('save_every_n', default_value='0', description='N 프레임마다 재처리 이미지 저장, 0=끔'),
        DeclareLaunchArgument('config_dir', default_value=DEFAULT_CONFIG, description='설정 폴더 (*.yaml 전부 사용)'),
        OpaqueFunction(function=_setup),
    ])
