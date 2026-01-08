#!/usr/bin/env python3
"""
Launch file for teachbot follower system with enable GUI
Starts both the teachbot follower action node and the enable GUI
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
from launch_ros.actions import Node


def generate_launch_description():
    # Get package directory
    teachbot_follower_dir = get_package_share_directory('teachbot_follower')
    
    # Default config file path
    default_config_file = os.path.join(
        teachbot_follower_dir,
        'config',
        'ur.yaml'
    )
    
    # Declare launch arguments
    declared_arguments = []
    
    declared_arguments.append(
        DeclareLaunchArgument(
            'config_file',
            default_value=default_config_file,
            description='Path to the configuration file (optional, can use individual parameters instead)'
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            'controller_name',
            default_value='joint_trajectory_controller',
            description='Name of the trajectory controller to use'
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            'update_rate',
            default_value='0.5',
            description='Update rate in seconds for sending trajectories'
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            'enable_mode',
            default_value='button',
            choices=['gui', 'button', 'none'],
            description='Enable mode: "gui" for manual GUI button, "button" for teachbot button control, "none" for no enable control'
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_config_file',
            default_value='true',
            choices=['true', 'false'],
            description='If true, load parameters from config file; if false, use individual parameters'
        )
    )

    # Get launch configurations
    config_file = LaunchConfiguration('config_file')
    controller_name = LaunchConfiguration('controller_name')
    update_rate = LaunchConfiguration('update_rate')
    enable_mode = LaunchConfiguration('enable_mode')
    use_config_file = LaunchConfiguration('use_config_file')

    # Teachbot follower action node with config file
    teachbot_follower_node_from_config = Node(
        package='teachbot_follower',
        executable='follower_action',
        name='teachbot_follower_action',
        output='screen',
        parameters=[config_file],
        condition=IfCondition(
            PythonExpression(["'", use_config_file, "' == 'true'"])
        )
    )
    
    # Teachbot follower action node with individual parameters
    teachbot_follower_node_from_params = Node(
        package='teachbot_follower',
        executable='follower_action',
        name='teachbot_follower_action',
        output='screen',
        parameters=[{
            'controller_name': controller_name,
            'update_rate': update_rate,
        }],
        condition=IfCondition(
            PythonExpression(["'", use_config_file, "' == 'false'"])
        )
    )

    # Enable GUI node (manual button control)
    enable_gui_node = Node(
        package='teachbot_follower',
        executable='teachbot_enable_gui',
        name='teachbot_enable_gui',
        output='screen',
        condition=IfCondition(
            PythonExpression(["'", enable_mode, "' == 'gui'"])
        )
    )
    
    # Enable from button node (teachbot button control)
    enable_button_node = Node(
        package='teachbot_follower',
        executable='teachbot_enable_from_button',
        name='teachbot_enable_from_button',
        output='screen',
        condition=IfCondition(
            PythonExpression(["'", enable_mode, "' == 'button'"])
        )
    )

    nodes_to_launch = [
        teachbot_follower_node_from_config,
        teachbot_follower_node_from_params,
        enable_gui_node,
        enable_button_node,
    ]

    return LaunchDescription(declared_arguments + nodes_to_launch)
