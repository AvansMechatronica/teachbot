#!/usr/bin/env python3
# teachbot_rviz.launch.py
"""
Launch file for TOS Teachbot with official UR5e visualization.

Uses the official ur_description package for accurate UR5e model.

Usage:
    ros2 launch teachbot_ros teachbot_rviz.launch.py
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, Command, PythonExpression
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def expand_path(path):
    """Expand ~ and environment variables in path."""
    return os.path.expanduser(os.path.expandvars(path))


def launch_setup(context, *args, **kwargs):
    """Generate nodes with expanded paths."""
    # Expand the target config file path  
    config_file = LaunchConfiguration('config_file').perform(context)
    config_file_expanded = expand_path(config_file)
    
    
    # Get package share directories
    pkg_share = get_package_share_directory('teachbot_ros')
    my_robot_share = get_package_share_directory('my_robot_description')
    rviz_config = os.path.join(pkg_share, 'rviz', 'teachbot.rviz')
    
    # Generate URDF from xacro for custom robot
    urdf_xacro = os.path.join(my_robot_share, 'urdf', 'my_robot.urdf.xacro')
    robot_description = ParameterValue(Command(['xacro ', urdf_xacro]), value_type=str)
    
    # Teachbot publisher node
    teachbot_node = Node(
        package='teachbot_ros',
        executable='teachbot_publisher',
        name='teachbot_publisher',
        output='screen',
        parameters=[config_file_expanded],
        remappings=[]
    )
    
    # Robot state publisher (publishes TF from URDF + joint states)
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'publish_frequency': 250.0,
        }],
        remappings=[
            ('/joint_states', '/teachbot/joint_states'),
        ]
    )
    
    # RViz node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config]
    )
    
    # Teachbot Monitor GUI (optional)
    monitor_gui_node = Node(
        package='teachbot_ros',
        executable='teachbot_monitor_gui',
        name='teachbot_monitor_gui',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_monitor_gui'))
    )
    




    # ...existing code...
    return [
        teachbot_node,
        robot_state_publisher_node,
        rviz_node,
        monitor_gui_node,
    ]


def generate_launch_description():
    # Get package share directories
    pkg_share = get_package_share_directory('teachbot_ros')
    
    # File paths
    default_config = os.path.join(pkg_share, 'config', 'teachbot_params.yaml')
    target_config = os.path.join(pkg_share, 'config', 'target_robots', 'ur.yaml')
    rviz_config = os.path.join(pkg_share, 'rviz', 'teachbot.rviz')
    
    # Declare launch arguments
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=default_config,
        description='Path to the configuration YAML file'
    )
    
    target_config_arg = DeclareLaunchArgument(
        'target_config_file',
        default_value=target_config,
        description='Path to the target robot configuration YAML file'
    )
    
    use_monitor_gui_arg = DeclareLaunchArgument(
        'use_monitor_gui',
        default_value='true',
        description='Launch the teachbot control monitor GUI'
    )
    
    
    return LaunchDescription([
        config_file_arg,
        target_config_arg,
        use_monitor_gui_arg,
        OpaqueFunction(function=launch_setup)
    ])
