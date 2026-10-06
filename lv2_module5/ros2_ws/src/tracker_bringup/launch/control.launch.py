"""tracker_controller(추적 제어)를 실행한다. 모터로 가는 출력은 없다(/pan_tilt/command 토픽만 발행).

ros2 launch tracker_bringup control.launch.py
추적 시작: ros2 topic pub --once /tracking_enable std_msgs/msg/Bool "{data: true}"
"""
import os
from glob import glob

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    configs = sorted(glob(os.path.join(get_package_share_directory('tracker_bringup'), 'config', '*.yaml')))
    return LaunchDescription([
        Node(package='tracker_controller', executable='controller_node', name='tracker_controller',
             parameters=configs, output='screen'),
    ])
