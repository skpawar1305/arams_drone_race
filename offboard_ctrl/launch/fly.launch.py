#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='offboard_ctrl',
            executable='automatic_ctrl',
            name='automatic_ctrl',
            output='log'),
    ])