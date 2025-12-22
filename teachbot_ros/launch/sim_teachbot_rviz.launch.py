#!/usr/bin/env python3
# sim_teachbot_rviz.launch.py
"""
Launch file for TOS Teachbot simulation with official UR5e visualization.

Uses the official ur_description package for accurate UR5e model.
Uses joint_state_publisher_gui to simulate teachbot movements.

Usage:
    ros2 launch teachbot_ros sim_teachbot_rviz.launch.py
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node


def generate_launch_description():
    # Get package share directories
    pkg_share = get_package_share_directory('teachbot_ros')
    ur_description_share = get_package_share_directory('ur_description')
    
    # File paths
    default_config = os.path.join(pkg_share, 'config', 'teachbot_params.yaml')
    default_target_config = os.path.join(pkg_share, 'config', 'target_robots', 'ur.yaml')
    sim_initial_positions = os.path.join(pkg_share, 'config', 'sim_initial_positions.yaml')
    rviz_config = os.path.join(pkg_share, 'rviz', 'teachbot.rviz')
    
    # Generate URDF from xacro for UR5e
    urdf_xacro = os.path.join(ur_description_share, 'urdf', 'ur.urdf.xacro')
    robot_description = Command([
        'xacro ', urdf_xacro,
        ' ur_type:=ur5e',
        ' name:=ur5e',
        ' tf_prefix:=teachbot/',
        ' generate_ros2_control_tag:=false',
    ])
    
    # Declare launch arguments
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=default_config,
        description='Path to the configuration YAML file'
    )
    
    target_config_file_arg = DeclareLaunchArgument(
        'target_config_file',
        default_value=default_target_config,
        description='Path to the target robot configuration YAML file'
    )

    use_monitor_gui_arg = DeclareLaunchArgument(
        'use_monitor_gui',
        default_value='true',
        description='Launch the teachbot control monitor GUI'
    )

    # Joint State Publisher GUI node
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
        parameters=[
            sim_initial_positions,
            {'robot_description': robot_description}
        ],
        remappings=[
            ('/joint_states', '/teachbot/joint_states'),
            ('/robot_description', '/teachbot/robot_description')
        ]
    )
    
    # Joint State Remapper (teachbot -> target robot)
    joint_state_remapper_node = Node(
        package='teachbot_ros',
        executable='joint_state_remapper',
        name='joint_state_remapper',
        output='screen',
        parameters=[{
            'target_config_file': LaunchConfiguration('target_config_file')
        }]
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
            ('/robot_description', '/teachbot/robot_description')
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

    # TeachbotState Publisher GUI
    teachbot_state_publisher_gui_node = Node(
        package='teachbot_ros',
        executable='teachbot_state_publisher_gui',
        name='teachbot_state_publisher_gui',
        output='screen'
    )
    
    return LaunchDescription([
        config_file_arg,
        target_config_file_arg,
        use_monitor_gui_arg,
        joint_state_publisher_gui_node,
        joint_state_remapper_node,
        robot_state_publisher_node,
        rviz_node,
        teachbot_state_publisher_gui_node,
        monitor_gui_node
    ])
