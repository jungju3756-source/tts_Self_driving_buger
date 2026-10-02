#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
carto.launch.py  —  라파1 지도 만들기, Cartographer 판 (캡스톤 RFP 지정 SLAM)

    ros2 launch ~/tts_Self_driving_buger/rapa1/carto.launch.py

    OpenCR ─► turtlebot3_node ─► /odom, TF odom→base_footprint
    LDS-02 ─► ld08_driver ─► /scan ─► cartographer_node ─► TF map→odom
                                            │
                     cartographer_occupancy_grid_node ─► /map

slam.launch.py(slam_toolbox) 와 달리 scan_resample 이 필요 없다.
Cartographer 는 스캔마다 점 개수가 달라도 그대로 받는다.
그래도 /scan_fixed 로 돌려보고 싶으면 scan_topic:=/scan_fixed 를 주면
scan_resample 도 같이 띄운다.

지도 저장 (Cartographer 는 저장 전에 궤적을 끝내는 게 깔끔하다):
    ros2 service call /finish_trajectory cartographer_ros_msgs/srv/FinishTrajectory "{trajectory_id: 0}"
    ros2 run nav2_map_server map_saver_cli -f ~/maps/<이름> --ros-args -p save_map_timeout:=15.0
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

HERE = os.path.dirname(os.path.realpath(__file__))
ROOT = os.path.dirname(HERE)
OPENCR = '/dev/serial/by-id/usb-ROBOTIS_OpenCR_Virtual_ComPort_in_FS_Mode_FFFFFFFEFFFF-if00'


def generate_launch_description():
    usb_port = LaunchConfiguration('usb_port')
    scan_topic = LaunchConfiguration('scan_topic')
    resolution = LaunchConfiguration('resolution')

    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('turtlebot3_bringup'), 'launch', 'robot.launch.py')),
        launch_arguments={'usb_port': usb_port}.items())

    resample = ExecuteProcess(
        cmd=['python3', os.path.join(HERE, 'scan_resample.py')],
        output='screen',
        condition=IfCondition(PythonExpression(["'", scan_topic, "' == '/scan_fixed'"])))

    cartographer = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        arguments=['-configuration_directory', os.path.join(ROOT, 'config'),
                   '-configuration_basename', 'cartographer_lds02.lua'],
        remappings=[('scan', scan_topic)])

    grid = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='cartographer_occupancy_grid_node',
        output='screen',
        arguments=['-resolution', resolution, '-publish_period_sec', '1.0'])

    return LaunchDescription([
        DeclareLaunchArgument('usb_port', default_value=OPENCR),
        DeclareLaunchArgument('scan_topic', default_value='/scan'),
        DeclareLaunchArgument('resolution', default_value='0.05'),
        bringup,
        resample,
        cartographer,
        grid,
    ])
