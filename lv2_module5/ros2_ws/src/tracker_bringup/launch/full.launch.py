"""전체 실행: perception.launch.py + control.launch.py

ros2 launch tracker_bringup full.launch.py
ros2 launch tracker_bringup full.launch.py dry_run:=true   # 카메라·검출은 실물, 모터 출력 끔
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    launch_dir = os.path.join(get_package_share_directory('tracker_bringup'), 'launch')
    include = lambda name, args: IncludeLaunchDescription(  # noqa: E731
        PythonLaunchDescriptionSource(os.path.join(launch_dir, name)), launch_arguments=args.items())
    return LaunchDescription([
        DeclareLaunchArgument('camera', default_value='true'),
        DeclareLaunchArgument('dry_run', default_value='false'),
        include('perception.launch.py', {'camera': LaunchConfiguration('camera')}),
        include('control.launch.py', {'dry_run': LaunchConfiguration('dry_run')}),
    ])
