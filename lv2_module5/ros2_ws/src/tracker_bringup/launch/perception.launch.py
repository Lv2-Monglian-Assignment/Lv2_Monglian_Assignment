"""카메라(realsense2_camera)와 target_detector를 함께 실행한다.

ros2 launch tracker_bringup perception.launch.py               # 카메라 + 검출
ros2 launch tracker_bringup perception.launch.py camera:=false  # 검출만
"""
import os
from glob import glob

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    configs = sorted(glob(os.path.join(get_package_share_directory('tracker_bringup'), 'config', '*.yaml')))
    rs_launch = os.path.join(get_package_share_directory('realsense2_camera'), 'launch', 'rs_launch.py')
    return LaunchDescription([
        DeclareLaunchArgument('camera', default_value='true'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(rs_launch), condition=IfCondition(LaunchConfiguration('camera')),
            launch_arguments={'align_depth.enable': 'true',
                              'rgb_camera.color_profile': '640x480x30',
                              'depth_module.depth_profile': '640x480x30'}.items()),
        Node(package='target_detector', executable='target_detector', name='target_detector',
             parameters=configs, output='screen'),
    ])
