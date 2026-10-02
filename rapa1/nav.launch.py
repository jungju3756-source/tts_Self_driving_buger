#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
nav.launch.py  —  라파1 캡스톤 주행 한 번에 (지도 작성 이후 평소에 띄우는 것)

    ros2 launch ~/tts_Self_driving_buger/rapa1/nav.launch.py
    ros2 launch ~/tts_Self_driving_buger/rapa1/nav.launch.py map:=$HOME/maps/<이름>.yaml

    localize.launch.py   bringup + map_server + AMCL + Nav2
    joy_node             8BitDo Micro (D 모드) → /joy
    teleop_twist_joy     /joy → /cmd_vel_joy
    pad_gate.py          /cmd_vel_joy → /cmd_vel (누를 때만 — Nav2 와 안 싸우게)
    place_manager.py     장소 저장·마커, 패드 A(버튼 0) 1초 → 현재 위치 저장
    pose_keeper.py       마지막 위치 기억 → 다음 실행 때 AMCL 초기 위치로 자동 입력

패드(/dev/input/event*)는 input 그룹 권한이 필요하다 (jungju 는 2026-10-02 추가됨, 재로그인 후 적용).
로봇을 껐던 자리가 아닌 곳에서 켰으면 RViz '2D Pose Estimate' 로 위치를 다시 찍을 것.
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

HERE = os.path.dirname(os.path.realpath(__file__))
ROOT = os.path.dirname(HERE)


def generate_launch_description():
    localize = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(HERE, 'localize.launch.py')),
        launch_arguments={'map': LaunchConfiguration('map')}.items())

    joy = Node(package='joy', executable='joy_node', name='joy_node',
               parameters=[{'autorepeat_rate': 20.0}], output='screen')

    teleop = Node(package='teleop_twist_joy', executable='teleop_node', name='teleop_twist_joy_node',
                  parameters=[os.path.join(ROOT, 'config', 'teleop_8bitdo.yaml')],
                  remappings=[('/cmd_vel', '/cmd_vel_joy')], output='screen')

    gate = ExecuteProcess(cmd=['python3', os.path.join(HERE, 'pad_gate.py')], output='screen')

    places = ExecuteProcess(
        cmd=['python3', os.path.join(HERE, 'place_manager.py'), '--ros-args',
             '-p', 'save_button:=0'],
        output='screen')

    keeper = ExecuteProcess(cmd=['python3', os.path.join(HERE, 'pose_keeper.py')], output='screen')

    return LaunchDescription([
        DeclareLaunchArgument('map', default_value=os.path.expanduser('~/maps/lab_v2.yaml')),
        localize,
        joy,
        teleop,
        gate,
        places,
        keeper,
    ])
