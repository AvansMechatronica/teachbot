#!/usr/bin/env python3
"""
Launch file for teachbot follower system with enable GUI
Starts both the teachbot follower action node and the enable GUI
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
from launch_ros.actions import Node

def expand_path(path):
    """Expand ~ and environment variables in path."""
    return os.path.expanduser(os.path.expandvars(path))


def launch_setup(context, *args, **kwargs):
    """Generate nodes with expanded paths."""
    # Expand the config file path
    config_file = LaunchConfiguration('config_file').perform(context)
    config_file_expanded = expand_path(config_file)
    
    controller_name = LaunchConfiguration('controller_name')
    update_rate = LaunchConfiguration('update_rate')
    use_config_file = LaunchConfiguration('use_config_file')
    
    # Activate the scaled_joint_trajectory_controller
    activate_controller = ExecuteProcess(
        cmd=[
            'ros2', 'service', 'call',
            '/controller_manager/switch_controller',
            'controller_manager_msgs/srv/SwitchController',
            "{activate_controllers: ['scaled_joint_trajectory_controller'], deactivate_controllers: [], strictness: 2, activate_asap: true}"
        ],
        output='screen',
        shell=False
    )
    
    # Teachbot follower action node with config file
    teachbot_follower_node_from_config = Node(
        package='teachbot_follower',
        executable='follower_action',
        name='teachbot_follower_action',
        output='screen',
        parameters=[config_file_expanded],
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
    
    return [
        activate_controller,
        teachbot_follower_node_from_config,
        teachbot_follower_node_from_params,
    ]


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
            'use_config_file',
            default_value='true',
            choices=['true', 'false'],
            description='If true, load parameters from config file; if false, use individual parameters'
        )
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
