#!/usr/bin/env python3
"""
Launch file for TOS Teachbot Recorder GUI
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get package share directory
    pkg_share = get_package_share_directory('teachbot_recorder')
    default_config = os.path.join(pkg_share, 'config', 'ur.yaml')

    # Declare launch arguments
    config_arg = DeclareLaunchArgument(
        'config',
        default_value=default_config,
        description='Path to configuration YAML file'
    )

    sim_arg = DeclareLaunchArgument(
        'sim',
        default_value='false',
        description='Use simulation controller'
    )

    # Recorder GUI node
    recorder_node = Node(
        package='teachbot_recorder',
        executable='recorder',
        name='teachbot_recorder_gui',
        output='screen',
        parameters=[
            LaunchConfiguration('config'),
            {'use_sim': LaunchConfiguration('sim')}
        ]
    )

    return LaunchDescription([
        config_arg,
        sim_arg,
        recorder_node,
    ])
