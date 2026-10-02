#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
localize.launch.py  —  저장한 지도 위에서 위치 추정 + 주행 (bringup + map_server + AMCL + Nav2)

    ros2 launch ~/tts_Self_driving_buger/rapa1/localize.launch.py
    ros2 launch ~/tts_Self_driving_buger/rapa1/localize.launch.py map:=$HOME/maps/<이름>.yaml

    OpenCR ─► turtlebot3_node ─► /odom, TF odom→base_footprint
    LDS-02 ─► ld08_driver ─► /scan ─► AMCL ─► TF map→odom
    ~/maps/<이름>.yaml ─► map_server ─► /map

지도 작성(carto.launch.py)이 끝난 뒤 장소 지정·주행할 때 쓴다.
Cartographer 로 만든 지도도 map 좌표가 그대로라 places.yaml 좌표와 맞는다.

시작 후 RViz '2D Pose Estimate' 로 로봇 위치를 한 번 찍어줘야 AMCL 이 자리를 잡는다.
(지도 작성 시작 지점에 그대로 있으면 initial_pose (0,0,0) 이 맞다)
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

OPENCR = '/dev/serial/by-id/usb-ROBOTIS_OpenCR_Virtual_ComPort_in_FS_Mode_FFFFFFFEFFFF-if00'
HERE = os.path.dirname(os.path.realpath(__file__))
NAV2_PARAMS = os.path.join(os.path.dirname(HERE), 'config', 'nav2_waffle_pi.yaml')


def generate_launch_description():
    usb_port = LaunchConfiguration('usb_port')
    map_yaml = LaunchConfiguration('map')

    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('turtlebot3_bringup'), 'launch', 'robot.launch.py')),
        launch_arguments={'usb_port': usb_port}.items())

    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('nav2_bringup'), 'launch', 'localization_launch.py')),
        launch_arguments={
            'map': map_yaml,
            'params_file': NAV2_PARAMS,
            'use_composition': 'False',
            'autostart': 'True',
        }.items())

    # 주행(planner·controller·bt_navigator 등) — nav:=false 면 위치 추정만
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('nav2_bringup'), 'launch', 'navigation_launch.py')),
        launch_arguments={
            'params_file': NAV2_PARAMS,
            'use_composition': 'False',
            'autostart': 'True',
        }.items(),
        condition=IfCondition(LaunchConfiguration('nav')))

    return LaunchDescription([
        DeclareLaunchArgument('nav', default_value='true'),
        DeclareLaunchArgument('usb_port', default_value=OPENCR),
        DeclareLaunchArgument('map', default_value=os.path.expanduser('~/maps/lab_v2.yaml')),
        bringup,
        localization,
        navigation,
    ])
