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
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, Command, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def expand_path(path):
    """Expand ~ and environment variables in path."""
    return os.path.expanduser(os.path.expandvars(path))


def launch_setup(context, *args, **kwargs):
    """Generate nodes with expanded paths."""
    # Node to convert /teachbot/joint_states_sim to /teachbot/joint_states with offsets
    pkg_share = get_package_share_directory('teachbot_ros')
    sim_offsets_path = os.path.join(pkg_share, 'config', 'sim_offsets.yaml')

    # Expand the target config file path

    
    # Robot state publisher (publishes TF from URDF + joint states)
    pkg_share = get_package_share_directory('teachbot_ros')
    teachbot_description_share = get_package_share_directory('teachbot_description')
    urdf_xacro = os.path.join(teachbot_description_share, 'urdf', 'teachbot.urdf.xacro')
    robot_description = ParameterValue(
        Command([
            'xacro ', urdf_xacro,
        ]),
        value_type=str
    )
    sim_initial_positions = os.path.join(pkg_share, 'config', 'sim_initial_positions.yaml')
    
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
            ('/joint_states', '/teachbot/joint_states_sim'),
            ('/robot_description', '/teachbot/robot_description')
        ]
    )
    
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
            ('/joint_states', '/teachbot/joint_states_sim'),
            ('/robot_description', '/teachbot/robot_description')
        ]
    )
    
    # RViz node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rviz_config')]
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

    # TeachbotState Publisher GUI
    teachbot_sim_jointstate_publishers_node = Node(
        package='teachbot_ros',
        executable='teachbot_sim_jointstate_publishers',
        name='sim_jointstate_publisher',
        output='screen',
        parameters=[os.path.join(pkg_share, 'config', 'sim_parameters.yaml')]
    )
    
    return [
        joint_state_publisher_gui_node,
        robot_state_publisher_node,
        rviz_node,
        teachbot_state_publisher_gui_node,
        teachbot_sim_jointstate_publishers_node,
        monitor_gui_node,
    ]
    


def generate_launch_description():
    # Get package share directories
    pkg_share = get_package_share_directory('teachbot_ros')
    teachbot_description_share = get_package_share_directory('teachbot_description')
    
    # File paths
    default_config = os.path.join(pkg_share, 'config', 'teachbot_params.yaml')
    sim_initial_positions = os.path.join(pkg_share, 'config', 'sim_initial_positions.yaml')
    rviz_config = os.path.join(pkg_share, 'rviz', 'teachbot.rviz')
    
    # Generate URDF from xacro for Teachbot
    urdf_xacro = os.path.join(teachbot_description_share, 'urdf', 'teachbot.urdf.xacro')
    robot_description = ParameterValue(
        Command([
            'xacro ', urdf_xacro,
        ]),
        value_type=str
    )
    
    # Declare launch arguments
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=default_config,
        description='Path to the configuration YAML file'
    )
    
    def process_target_config(context):
        """Process and expand the target config file path."""
        config_path = LaunchConfiguration('target_config_file').perform(context)
        expanded_path = expand_path(config_path)
        return expanded_path
    

    use_monitor_gui_arg = DeclareLaunchArgument(
        'use_monitor_gui',
        default_value='true',
        description='Launch the teachbot control monitor GUI'
    )


    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=rviz_config,
        description='Path to the RViz configuration file'
    )

    return LaunchDescription([
        config_file_arg,
        use_monitor_gui_arg,
        rviz_config_arg,
        OpaqueFunction(function=launch_setup)
    ])
