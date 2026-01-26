#!/usr/bin/env python3
"""
Node to subscribe to /teachbot/joint_states_sim and publish to /teachbot/joint_states
with offsets from a YAML config file.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import yaml
import os
import math

class JointStateSimToRealPublisher(Node):
    def __init__(self, config_path):
        super().__init__('publish_jointstates_from_sim')
        self.declare_parameter('config_file', config_path)
        config_file = self.get_parameter('config_file').value
        self.offsets = []
        self.joint_names = []
        self._load_offsets(config_file)

        self.sub = self.create_subscription(
            JointState,
            '/teachbot/joint_states_sim',
            self.joint_state_callback,
            10
        )
        self.pub = self.create_publisher(JointState, '/teachbot/joint_states', 10)
        self.get_logger().info(f'Loaded offsets: {self.offsets}')
        self.get_logger().info(f'Loaded joint names: {self.joint_names}')

    def _load_offsets(self, config_file):
        if not os.path.isfile(config_file):
            self.get_logger().error(f'Config file not found: {config_file}')
            return
        with open(config_file, 'r') as f:
            config = yaml.safe_load(f)
        # Expecting keys: degree_offsets, joint_names
        degree_offsets = config.get('degree_offsets', [])
        self.offsets = [math.radians(deg) for deg in degree_offsets]
        self.joint_names = config.get('joint_names', [])

    def joint_state_callback(self, msg):
        # Apply offsets if joint names match
        if len(msg.position) != len(self.offsets):
            self.get_logger().warn('Mismatch in joint count, passing through without offset.')
            self.pub.publish(msg)
            return
        new_msg = JointState()
        new_msg.header = msg.header
        new_msg.name = list(msg.name)
        new_msg.position = [p + o for p, o in zip(msg.position, self.offsets)]
        new_msg.velocity = list(msg.velocity)
        new_msg.effort = list(msg.effort)
        self.pub.publish(new_msg)


def main(args=None):
    rclpy.init(args=args)
    import sys
    import os
    # Use sim_offsets.yaml by default
    config_path = sys.argv[1] if len(sys.argv) > 1 else os.path.abspath(os.path.join(
        os.path.dirname(__file__), '..', '..', 'config', 'sim_offsets.yaml'))
    node = JointStateSimToRealPublisher(config_path)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
