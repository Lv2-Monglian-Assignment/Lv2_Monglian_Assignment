"""bag 입력 재처리: bag의 영상으로 target_detector만 다시 돌리고 결과는 /target_replay로 분리한다.
카메라·제어·브리지는 띄우지 않는다(모터 출력 없음). 저장된 /target과 섞이지 않게 출력 토픽 3개를 모두 바꾼다.

터미널 1: ros2 launch tracker_bringup replay.launch.py
터미널 2: ros2 bag play recordings/<run_id> --clock
"""
import os
from glob import glob

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    configs = sorted(glob(os.path.join(get_package_share_directory('tracker_bringup'), 'config', '*.yaml')))
    return LaunchDescription([
        Node(package='target_detector', executable='target_detector', name='target_detector',
             parameters=configs + [{'use_sim_time': True,
                                    'target_topic': '/target_replay',
                                    'depth_out_topic': '/target_replay/depth',
                                    'position_topic': '/target_replay/position_cam'}],
             output='screen'),
    ])
