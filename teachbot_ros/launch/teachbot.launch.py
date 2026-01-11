#!/usr/bin/env python3
# teachbot.launch.py
"""
Launch file for TOS Teachbot ROS2 node.

Usage:
    ros2 launch teachbot_ros teachbot.launch.py
    ros2 launch teachbot_ros teachbot.launch.py remote_ip:=192.168.100.152
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def expand_path(path):
    """Expand ~ and environment variables in path."""
    return os.path.expanduser(os.path.expandvars(path))


def launch_setup(context, *args, **kwargs):
    """Generate nodes with expanded paths."""
    # Expand the target config file path
    target_config = LaunchConfiguration('target_config_file').perform(context)
    target_config_expanded = expand_path(target_config)
    
    # Create the teachbot node
    teachbot_node = Node(
        package='teachbot_ros',
        executable='teachbot_publisher',
        name='teachbot_publisher',
        output='screen',
        parameters=[LaunchConfiguration('config_file')],
        remappings=[]
    )

    # Joint State Remapper (teachbot -> target robot)
    joint_state_remapper_node = Node(
        package='teachbot_ros',
        executable='joint_state_remapper',
        name='joint_state_remapper',
        output='screen',
        parameters=[
            target_config_expanded,
            {
                'target_config_file': target_config_expanded
            }
        ]
    )
    
    return [
        teachbot_node,
        joint_state_remapper_node
    ]


def generate_launch_description():
    # Get package share directory
    pkg_share = get_package_share_directory('teachbot_ros')
    
    # Default config file path
    default_config = os.path.join(pkg_share, 'config', 'teachbot_params.yaml')
    
    # Declare launch arguments
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=default_config,
        description='Path to the configuration YAML file'
    )

    target_config = os.path.join(pkg_share, 'config', 'target_robots', 'ur.yaml')
    target_config_arg = DeclareLaunchArgument(
        'target_config_file',
        default_value=target_config,
        description='Path to the target robot configuration YAML file'
    )    
    
    remote_ip_arg = DeclareLaunchArgument(
        'remote_ip',
        default_value='',
        description='Override teachbot IP address (empty = use config file value)'
    )
    
    publish_rate_arg = DeclareLaunchArgument(
        'publish_rate',
        default_value='',
        description='Override publish rate in Hz (empty = use config file value)'
    )
    
    return LaunchDescription([
        config_file_arg,
        target_config_arg,
        remote_ip_arg,
        publish_rate_arg,
        OpaqueFunction(function=launch_setup)
    ])
