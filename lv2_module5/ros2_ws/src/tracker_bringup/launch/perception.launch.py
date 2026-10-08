"""카메라(realsense2_camera)와 target_detector를 함께 실행한다.

ros2 launch tracker_bringup perception.launch.py               # 카메라 + 검출
ros2 launch tracker_bringup perception.launch.py camera:=false  # 검출만
선택 인자: run_id:=<이름>(기록 이름, 기본 auto = det_날짜_시각), config_dir:=<폴더>(다른 설정 폴더의 *.yaml 사용)
"""
import atexit
import os
import tempfile
from glob import glob

import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
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
    rs_launch = os.path.join(get_package_share_directory('realsense2_camera'), 'launch', 'rs_launch.py')
    return [
        # forwarding=False: 이 launch의 인자(run_id 등)를 rs_launch에 넘기지 않는다(넘기면 "not supported" 경고가 반복됨)
        GroupAction(condition=IfCondition(LaunchConfiguration('camera')), scoped=True, forwarding=False, actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(rs_launch),
                launch_arguments={'align_depth.enable': 'true',
                                  'rgb_camera.color_profile': '640x480x30',
                                  'depth_module.depth_profile': '640x480x30'}.items())]),
        Node(package='target_detector', executable='target_detector', name='target_detector',
             parameters=configs + [override('target_detector', run_id=LaunchConfiguration('run_id').perform(context))],
             output='screen'),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('camera', default_value='true'),
        DeclareLaunchArgument('run_id', default_value='auto', description='기록 이름 (auto: det_날짜_시각)'),
        DeclareLaunchArgument('config_dir', default_value=DEFAULT_CONFIG, description='설정 폴더 (*.yaml 전부 사용)'),
        OpaqueFunction(function=_setup),
    ])
