#!/usr/bin/env python3
"""
Joint State Remapper Node

Subscribes to /teachbot/joint_states and republishes to /teachbot/<target_robot>/joint_states
with target-specific joint names and angle offsets applied.

This allows the teachbot to control different target robots with appropriate transformations.

Author: Gerard Harkema
Date: 2025-12
Initial version: 2025-12-22
License: CC BY-NC-SA 4.0
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
import yaml
import os
from ament_index_python.packages import get_package_share_directory
import math


class JointStateRemapper(Node):
    def __init__(self):
        super().__init__('jointstate_remapper_to_target_robot')
        
        # Declare parameters
        self.declare_parameter('target_config_file', '')
        
        # Initialize enabled state
        self.enabled = False
        
        # Load target robot configuration
        self.load_target_config()
        
        # Create subscriber to teachbot enable topic
        self.enable_subscription = self.create_subscription(
            Bool,
            '/teachbot/enable',
            self.enable_callback,
            10
        )
        
        # Create subscriber to teachbot joint states
        self.subscription = self.create_subscription(
            JointState,
            '/teachbot/joint_states',
            self.joint_state_callback,
            10
        )
        
        # Create publisher for target robot joint states
        target_topic = f'/teachbot/{self.target_robot_name}/joint_states'
        self.publisher = self.create_publisher(JointState, target_topic, 10)
        
        self.get_logger().info(
            f'Joint State Remapper initialized: /teachbot/joint_states -> {target_topic}'
        )
        self.get_logger().info(f'Target robot: {self.target_robot_name}')
        self.get_logger().info(f'Target joint names: {self.target_joint_names}')
        self.get_logger().info(f'Target degree offsets: {self.target_degree_offsets}')
    
    def load_target_config(self):
        """Load target robot configuration from YAML file."""
        config_file = self.get_parameter('target_config_file').get_parameter_value().string_value
        
        if not config_file:
            # Use default ur.yaml if not specified
            pkg_share = get_package_share_directory('teachbot_ros')
            config_file = os.path.join(pkg_share, 'config', 'target_robots', 'ur.yaml')
        
        # Expand user home directory (~) and environment variables
        config_file = os.path.expanduser(config_file)
        config_file = os.path.expandvars(config_file)
        
        self.get_logger().info(f'Loading target config from: {config_file}')
        
        # Default values
        self.target_robot_name = 'not_specified'
        self.target_joint_names = [
            'joint_1',
            'joint_2',
            'joint_3',
            'joint_4',
            'joint_5',
            'joint_6'
        ]
        self.target_degree_offsets = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        
        try:
            # Check if file exists first
            if not os.path.exists(config_file):
                raise FileNotFoundError(f'Config file not found: {config_file}')
            
            with open(config_file, 'r') as f:
                config = yaml.safe_load(f)
            
            # Check if config is None (empty or invalid YAML)
            if config is None:
                raise ValueError(f'Config file is empty or invalid: {config_file}')
            
            # Debug: print the loaded config
            self.get_logger().info(f'Loaded config structure: {config}')
            self.get_logger().info(f'Config type: {type(config)}')
            
            # Validate config is a dictionary
            if not isinstance(config, dict):
                raise ValueError(f'Config must be a dictionary, got {type(config)}')
            
            # Log available keys
            self.get_logger().info(f'Available config keys: {list(config.keys())}')
            
            # Navigate to the nested parameters structure
            # First try to find parameters for this node's actual name
            node_name = self.get_name()
            if node_name in config and 'ros__parameters' in config[node_name]:
                params = config[node_name]['ros__parameters']
                self.target_robot_name = params.get('target_robot_name', self.target_robot_name)
                self.target_joint_names = params.get('target_joint_names', self.target_joint_names)
                self.target_degree_offsets = params.get('target_degree_offsets', self.target_degree_offsets)
            # Fallback to 'jointstate_remapper_to_target_robot' (default node name)
            elif 'jointstate_remapper_to_target_robot' in config and 'ros__parameters' in config['jointstate_remapper_to_target_robot']:
                params = config['jointstate_remapper_to_target_robot']['ros__parameters']
                self.target_robot_name = params.get('target_robot_name', self.target_robot_name)
                self.target_joint_names = params.get('target_joint_names', self.target_joint_names)
                self.target_degree_offsets = params.get('target_degree_offsets', self.target_degree_offsets)
            # Another fallback to 'joint_state_remapper' (alternate name)
            elif 'joint_state_remapper' in config and 'ros__parameters' in config['joint_state_remapper']:
                params = config['joint_state_remapper']['ros__parameters']
                self.target_robot_name = params.get('target_robot_name', self.target_robot_name)
                self.target_joint_names = params.get('target_joint_names', self.target_joint_names)
                self.target_degree_offsets = params.get('target_degree_offsets', self.target_degree_offsets)
            # Fallback to 'teachbot_publisher' (legacy)
            elif 'teachbot_publisher' in config and 'ros__parameters' in config['teachbot_publisher']:
                params = config['teachbot_publisher']['ros__parameters']
                self.target_robot_name = params.get('target_robot_name', self.target_robot_name)
                self.target_joint_names = params.get('target_joint_names', self.target_joint_names)
                self.target_degree_offsets = params.get('target_degree_offsets', self.target_degree_offsets)
            else:
                # Final fallback: try to get directly from config root
                self.target_robot_name = config.get('target_robot_name', self.target_robot_name)
                self.target_joint_names = config.get('target_joint_names', self.target_joint_names)
                self.target_degree_offsets = config.get('target_degree_offsets', self.target_degree_offsets)
            
            # Log what was actually loaded
            self.get_logger().info(f'Loaded target_robot_name: {self.target_robot_name}')
            self.get_logger().info(f'Loaded target_joint_names: {self.target_joint_names}')
            self.get_logger().info(f'Loaded target_degree_offsets: {self.target_degree_offsets}')
        
        except Exception as e:
            self.get_logger().error(f'Failed to load target config: {e}')
            import traceback
            self.get_logger().error(f'Traceback: {traceback.format_exc()}')
            self.get_logger().warn('Using default UR configuration')
    
    def enable_callback(self, msg):
        """Callback for enable/disable messages."""
        self.enabled = msg.data
        state = "enabled" if self.enabled else "disabled"
        self.get_logger().info(f'Joint state remapper {state}')
    
    def joint_state_callback(self, msg):
        """
        Callback for incoming joint states.
        Apply target-specific transformations and republish.
        """
        # Only publish if enabled
        if not self.enabled:
            return
        
        # Create new message for target robot
        target_msg = JointState()
        target_msg.header = msg.header
        target_msg.name = self.target_joint_names
        
        # Apply angle offsets (convert deg to rad)
        if len(msg.position) == 6:
            target_msg.position = [
                msg.position[i] + math.radians(self.target_degree_offsets[i])
                for i in range(6)
            ]
        else:
            self.get_logger().warn(
                f'Expected 6 joint positions, got {len(msg.position)}. Skipping.'
            )
            return
        
        # Copy velocities and efforts if available
        if msg.velocity:
            target_msg.velocity = list(msg.velocity)
        if msg.effort:
            target_msg.effort = list(msg.effort)
        
        # Publish transformed message
        self.publisher.publish(target_msg)


def main(args=None):
    rclpy.init(args=args)
    node = JointStateRemapper()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()