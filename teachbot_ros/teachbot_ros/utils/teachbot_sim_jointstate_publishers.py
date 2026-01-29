#!/usr/bin/env python3
"""
Node to subscribe to /teachbot/joint_states_sim and publish to /teachbot/joint_states
with offsets from a YAML config file.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from rclpy.parameter import Parameter

class JointStateSimToRealPublisher(Node):
    def __init__(self):
        super().__init__('sim_jointstate_publisher')
        # Declare and get parameters (populated from launch file)

        self.declare_parameter('gui_joint_topic', value='/teachbot/joint_states_sim')
        self.declare_parameter('teachbot_joint_topic', value='/teachbot/joint_states')
        self.declare_parameter('joint_names', value=['joint', 'joint', 'joint', 
                                                'joint', 'joint', 'joint'])
        self.declare_parameter('degree_offsets', value=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

        self.gui_joint_topic = self.get_parameter('gui_joint_topic').value
        self.teachbot_joint_topic = self.get_parameter('teachbot_joint_topic').value
        self.joint_names = list(self.get_parameter('joint_names').value)
        self.offsets = list(self.get_parameter('degree_offsets').value)
        self.rad_offsets = [offset * 3.141592653589793 / 180.0 for offset in self.offsets]

        self.sub = self.create_subscription(
            JointState,
            self.gui_joint_topic,
            self.joint_state_callback,
            10
        )
        self.pub = self.create_publisher(JointState, self.teachbot_joint_topic, 10)
        self.get_logger().info(f'Loaded offsets: {self.offsets}')
        self.get_logger().info(f'Loaded joint names: {self.joint_names}')


    def joint_state_callback(self, msg):
        # Apply offsets if joint names match
        if len(msg.position) != len(self.offsets):
            self.get_logger().warn(
                f'position length does not match offsets length: msg.position={len(msg.position)}, offsets={len(self.offsets)}'
            )
            self.pub.publish(msg)
            return
        new_msg = JointState()
        new_msg.header = msg.header
        new_msg.name = list(msg.name)
        new_msg.position = [p + o for p, o in zip(msg.position, self.rad_offsets)]
        new_msg.velocity = list(msg.velocity)
        new_msg.effort = list(msg.effort)
        self.pub.publish(new_msg)


def main(args=None):
    rclpy.init(args=args)

    node = JointStateSimToRealPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
