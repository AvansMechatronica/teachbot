#!/usr/bin/env python3
"""
Launch file for teachbot follower system using action client
Starts the teachbot follower action node
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
    # Expand the config file path
    #config_file = LaunchConfiguration('config_file').perform(context)
    #config_file_expanded = expand_path(config_file)
    
    # Get sim parameter
    sim = LaunchConfiguration('sim').perform(context)
    
    # Teachbot follower action node
    teachbot_follower_node = Node(
        package='teachbot_follower',
        executable='follower_action',
        name='teachbot_follower',
        output='screen',
            parameters=[LaunchConfiguration('config_file'),
            {'sim': sim == 'true'}
        ]
    )
    
    # Teachbot enable GUI node
    teachbot_enable_gui_node = Node(
        package='teachbot_follower',
        executable='teachbot_enable_from_button',
        name='teachbot_enable_from_button',
        output='screen',
        parameters=[LaunchConfiguration('config_file')]
    )

    return [teachbot_follower_node, teachbot_enable_gui_node]


def generate_launch_description():
    # Get package directory
    teachbot_follower_dir = get_package_share_directory('teachbot_follower')
    
    # Path to config file
    config_file = os.path.join(
        teachbot_follower_dir,
        'config',
        'ur.yaml'
    )
    
    # Declare launch arguments
    declared_arguments = []
    
    declared_arguments.append(
        DeclareLaunchArgument(
            'config_file',
            default_value=config_file,
            description='Path to the configuration file'
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            'sim',
            default_value='false',
            description='Use simulation controller (true/false)'
        )
    )
    
    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
