"""전체 실행: perception.launch.py + control.launch.py

ros2 launch tracker_bringup full.launch.py
ros2 launch tracker_bringup full.launch.py dry_run:=true   # 카메라·검출은 실물, 모터 출력 끔
선택 인자: auto_enable·run_id·config_dir (perception·control launch와 같은 뜻, run_id는 인지·제어·브리지 기록에 함께 씀)
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
        DeclareLaunchArgument('auto_enable', default_value='false'),
        DeclareLaunchArgument('run_id', default_value='auto'),
        DeclareLaunchArgument('config_dir',
                              default_value=os.path.join(get_package_share_directory('tracker_bringup'), 'config')),
        include('perception.launch.py', {'camera': LaunchConfiguration('camera'), 'run_id': LaunchConfiguration('run_id'),
                                         'config_dir': LaunchConfiguration('config_dir')}),
        include('control.launch.py', {'dry_run': LaunchConfiguration('dry_run'),
                                      'auto_enable': LaunchConfiguration('auto_enable'),
                                      'run_id': LaunchConfiguration('run_id'),
                                      'config_dir': LaunchConfiguration('config_dir')}),
    ])
