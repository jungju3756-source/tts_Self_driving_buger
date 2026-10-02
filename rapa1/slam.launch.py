#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
slam.launch.py  —  라파1 지도 만들기 (bringup + 스캔 재샘플 + slam_toolbox)

    ros2 launch ~/tts_Self_driving_buger/rapa1/slam.launch.py

    OpenCR ─► turtlebot3_node ─► /odom, TF odom→base_footprint
    LDS-02 ─► ld08_driver ─► /scan ─► scan_resample.py ─► /scan_fixed
                                                             │
                                          slam_toolbox ◄─────┘ ─► /map, TF map→odom

OpenCR 포트는 /dev/serial/by-id 로 고정한다. OpenCR 전원을 껐다 켜면
ttyACM0 → ttyACM1 처럼 번호가 바뀌어 기본값(ttyACM0)으로는 못 연다.

지도 저장:
    ros2 run nav2_map_server map_saver_cli -f ~/maps/<이름>
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

HERE = os.path.dirname(os.path.realpath(__file__))
ROOT = os.path.dirname(HERE)
OPENCR = '/dev/serial/by-id/usb-ROBOTIS_OpenCR_Virtual_ComPort_in_FS_Mode_FFFFFFFEFFFF-if00'


def generate_launch_description():
    usb_port = LaunchConfiguration('usb_port')
    slam_params = LaunchConfiguration('slam_params_file')

    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('turtlebot3_bringup'), 'launch', 'robot.launch.py')),
        launch_arguments={'usb_port': usb_port}.items())

    resample = ExecuteProcess(
        cmd=['python3', os.path.join(HERE, 'scan_resample.py')],
        output='screen')

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py')),
        launch_arguments={'slam_params_file': slam_params}.items())

    return LaunchDescription([
        DeclareLaunchArgument('usb_port', default_value=OPENCR),
        DeclareLaunchArgument('slam_params_file',
                              default_value=os.path.join(ROOT, 'config', 'slam_toolbox.yaml')),
        bringup,
        resample,
        slam,
    ])
