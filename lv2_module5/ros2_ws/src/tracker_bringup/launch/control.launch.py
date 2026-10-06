"""tracker_controller(추적 제어)와 opencr_bridge(OpenCR 시리얼)를 함께 실행한다.

ros2 launch tracker_bringup control.launch.py                 # 실제 모터
ros2 launch tracker_bringup control.launch.py dry_run:=true   # 시리얼 미사용, 관절 각도 시뮬레이션
추적 시작: ros2 topic pub --once /tracking_enable std_msgs/msg/Bool "{data: true}"
"""
import os
from glob import glob

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    configs = sorted(glob(os.path.join(get_package_share_directory('tracker_bringup'), 'config', '*.yaml')))
    dry_run = ParameterValue(LaunchConfiguration('dry_run'), value_type=bool)
    return LaunchDescription([
        DeclareLaunchArgument('dry_run', default_value='false'),
        Node(package='tracker_controller', executable='controller_node', name='tracker_controller',
             parameters=configs, output='screen'),
        Node(package='tracker_bridge', executable='opencr_bridge', name='opencr_bridge',
             parameters=configs + [{'dry_run': dry_run}], output='screen'),
    ])
