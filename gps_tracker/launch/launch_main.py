#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, TimerAction
import os

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='gps_tracker',
            executable='gps_can_pub',
            name='gps_can_pub_node',
            output='screen',
            parameters=[],
            respawn=True,
            respawn_delay=2.0
        ),

        Node(
            package='gps_tracker',
            executable='gps_csv_logger',
            name='gps_csv_logger_node',
            output='screen',
            parameters=[],
            respawn=True,
            respawn_delay=2.0
        ),

        Node(
            package='gps_tracker',
            executable='gps_logger',
            name='gps_logger_node',
            output='screen',
            parameters=[],
            respawn=True,
            respawn_delay=2.0
        ),

        Node(
            package='gps_tracker',
            executable='timer_can_pub',
            name='timer_node',
            output='screen',
            parameters=[],
            respawn=True,
            respawn_delay=2.0
        ),

        # 顯示資訊
        ExecuteProcess(
            cmd=['echo', 'GPS Tracker 已啟動！ gps_can_pub_node , gps_logger_node 和 timer_node 正在運行...'],
            output='screen'
        )
    ])