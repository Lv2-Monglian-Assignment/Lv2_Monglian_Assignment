"""tracker_controller(추적 제어)와 opencr_bridge(OpenCR 시리얼)를 함께 실행한다.

ros2 launch tracker_bringup control.launch.py                 # 실제 모터
ros2 launch tracker_bringup control.launch.py dry_run:=true   # 시리얼 미사용, 관절 각도 시뮬레이션
추적 시작: ros2 topic pub --once /tracking_enable std_msgs/msg/Bool "{data: true}"
선택 인자: auto_enable:=true(시작하자마자 추적), run_id:=<이름>(제어 <run_id>.csv·브리지 <run_id>_serial.log, 기본 auto),
          config_dir:=<폴더>(다른 설정 폴더의 *.yaml 사용, 예: Kp 비교 시험)
"""
import atexit
import os
import tempfile
from glob import glob

import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
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


def _setup(context):
    configs = sorted(glob(os.path.join(LaunchConfiguration('config_dir').perform(context), '*.yaml')))
    arg = lambda name: LaunchConfiguration(name).perform(context)  # noqa: E731
    return [
        Node(package='tracker_controller', executable='controller_node', name='tracker_controller',
             parameters=configs + [override('tracker_controller', auto_enable=as_bool(arg('auto_enable')),
                                                     run_id=arg('run_id'))], output='screen'),
        Node(package='tracker_bridge', executable='opencr_bridge', name='opencr_bridge',
             parameters=configs + [override('opencr_bridge', dry_run=as_bool(arg('dry_run')), run_id=arg('run_id'))],
             output='screen'),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('dry_run', default_value='false'),
        DeclareLaunchArgument('auto_enable', default_value='false', description='true: 시작하자마자 추적'),
        DeclareLaunchArgument('run_id', default_value='auto', description='기록 이름 (auto: 날짜_시각)'),
        DeclareLaunchArgument('config_dir', default_value=DEFAULT_CONFIG, description='설정 폴더 (*.yaml 전부 사용)'),
        OpaqueFunction(function=_setup),
    ])
