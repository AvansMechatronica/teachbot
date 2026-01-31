#!/usr/bin/env python3
"""
UR Robot TeachBot Follower Node (Action Client)

This node subscribes to joint states from a teachbot device and commands
the target robot to follow those positions using action client for trajectory execution.

Author: Gerard Harkema
Date: 2025-12
Initial version: 2025-12-22
License: CC BY-NC-SA 4.0
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from control_msgs.msg import JointTolerance
import threading
import math


class URTeachBotFollowerAction(Node):
    """Node that follows teachbot joint commands using action client."""

    def __init__(self):
        super().__init__('teachbot_follower')

        # Declare parameters FIRST
        self.declare_parameter('teachbot_topic', '/teachbot/joint_states')
        self.declare_parameter('enable_topic', '/teachbot/enable')
        self.declare_parameter('controller_name', 'not_specified_controller')
        self.declare_parameter('controller_name_sim', 'not_specified_controller_sim')
        self.declare_parameter('sim', False)  # Use simulation controller
        self.declare_parameter('update_rate', 0.5)  # seconds between updates
        self.declare_parameter('position_tolerance', 0.01)  # radians
        self.declare_parameter('trajectory_duration', 2.0)  # seconds - increased to avoid path tolerance violations
        self.declare_parameter('degree_offsets', [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.declare_parameter('joint_scale_factors', [1.0, 1.0, 1.0, 1.0, 1.0, 1.0])
        # Joint names for JointState message, fill wih dummy names by default
        self.declare_parameter('teachbot_joint_names', ['joint', 'joint', 'joint', 
                                                'joint', 'joint', 'joint'])
        self.declare_parameter('target_joint_names', ['joint', 'joint', 'joint', 
                                                'joint', 'joint', 'joint'])

        # Now get parameters
        teachbot_topic = self.get_parameter('teachbot_topic').value
        enable_topic = self.get_parameter('enable_topic').value
        controller_name = self.get_parameter('controller_name').value
        controller_name_sim = self.get_parameter('controller_name_sim').value
        sim = self.get_parameter('sim').value
        self.update_rate = self.get_parameter('update_rate').value
        self.position_tolerance = self.get_parameter('position_tolerance').value
        # Initialize action client for FollowJointTrajectory
        controller_ns = controller_name_sim if sim else controller_name
        self._action_client = ActionClient(
            self,
            FollowJointTrajectory,
            f'/{controller_ns}/follow_joint_trajectory'
        )
        self.trajectory_duration = self.get_parameter('trajectory_duration').value
        self.degree_offsets = list(self.get_parameter('degree_offsets').value)
        self.rad_offsets = [offset * 3.14159265 / 180.0 for offset in self.degree_offsets]
        self.joint_scale_factors = list(self.get_parameter('joint_scale_factors').value)
        self.teachbot_joint_names = list(self.get_parameter('teachbot_joint_names').value)
        self.target_joint_names = list(self.get_parameter('target_joint_names').value)

        self.get_logger().info(f'Teachbot joint names: {self.teachbot_joint_names}')
        self.get_logger().info(f'Target joint names: {self.target_joint_names}')

        # Subscribe to /teachbot/joint_states
        self.latest_joint_states = None
        self.lock = threading.Lock()
        self.enabled = False
        self.is_executing = False
        self.last_commanded_positions = None
        self.current_robot_state = None
        self.robot_state_subscription = self.create_subscription(
            JointState,
            teachbot_topic,
            self.joint_states_callback,
            10
        )
        # Subscribe to enable topic
        self.enable_subscription = self.create_subscription(
            Bool,
            enable_topic,
            self.enable_callback,
            10
        )
        # Timer to periodically update robot position
        self.timer = self.create_timer(self.update_rate, self.update_robot_position)

        # Wait for action server to be available
        self.get_logger().info('Waiting for action server...')
        self._action_client.wait_for_server()
        self.get_logger().info('Action server available!')

    def joint_states_callback(self, msg):
        """Callback for /teachbot/joint_states subscription."""
        #self.get_logger().info('Received joint states') 
        with self.lock:
            #self.get_logger().info('Received joint states after lock')
            self.latest_joint_states = msg

    def enable_callback(self, msg):
        """Callback for enable topic subscription."""
        was_enabled = self.enabled
        self.enabled = msg.data
        if self.enabled and not was_enabled:
            self.get_logger().info('TeachBot following ENABLED')
        elif not self.enabled and was_enabled:
            self.get_logger().info('TeachBot following DISABLED')
        # ...existing code...
    
    
    def update_robot_position(self):
        """Periodically update the robot position based on teachbot input."""
        # Check if enabled
        if not self.enabled:
            self.get_logger().debug('Update skipped: following disabled')
            return

        # Don't send new commands while executing
        if self.is_executing:
            self.get_logger().debug('Update skipped: already executing')
            return

        with self.lock:
            if self.latest_joint_states is None:
                self.get_logger().debug('Update skipped: no joint states received yet')
                return
            joint_states = self.latest_joint_states

        try:
            # Check if we need to update (positions changed significantly)
            if self.should_update_position(joint_states):
                self.get_logger().info('Position changed, sending goal...')
                self.send_goal(joint_states)
            else:
                self.get_logger().debug('Position change below tolerance, not sending goal')
        except Exception as e:
            self.get_logger().error(f'Error updating robot position: {str(e)}')
    
    def should_update_position(self, joint_states):
        """Check if the position has changed enough to warrant a new command."""
        if self.last_commanded_positions is None:
            return True
        
        # Check if any joint has moved beyond the tolerance
        for i, pos in enumerate(joint_states.position):
            if i < len(self.last_commanded_positions):
                if abs(pos - self.last_commanded_positions[i]) > self.position_tolerance:
                    return True
        
        return False
    def send_goal(self, joint_states):
        """Send a trajectory goal via action client, applying offsets and scale factors from YAML, using target_joint_names."""
        self.get_logger().info('Preparing to send goal to action server...')
        self.is_executing = True

        # Strip tf_prefix from joint names (e.g., 'teachbot/shoulder_pan_joint' -> 'shoulder_pan_joint')
        teachbot_joint_names = [name.split('/')[-1] for name in joint_states.name]

        # Log joint name mapping for debugging
        if any('/' in name for name in joint_states.name):
            self.get_logger().debug(
                f'Stripped tf_prefix: {list(joint_states.name)} -> {teachbot_joint_names}'
            )

        # Map teachbot joint states to target joint order
        # Build a mapping from teachbot_joint_names to their positions
        teachbot_joint_map = {name: pos for name, pos in zip(teachbot_joint_names, joint_states.position)}


        # Map teachbot_joint_names to target_joint_names by index
        target_positions = []
        for i, target_joint in enumerate(self.target_joint_names):
            # Find the teachbot value by index, not by name
            if i < len(self.teachbot_joint_names) and i < len(joint_states.position):
                teachbot_value = joint_states.position[self.teachbot_joint_names.index(self.teachbot_joint_names[i])]
            else:
                teachbot_value = 0.0
            # Apply offset and scaler if available
            offset = self.rad_offsets[i] if i < len(self.rad_offsets) else 0.0
            scaler = self.joint_scale_factors[i] if i < len(self.joint_scale_factors) else 1.0
            target_positions.append((teachbot_value + offset) * scaler)

        # Get current robot positions for these joints
        current_positions = None
        with self.lock:
            if self.current_robot_state is not None:
                robot_joint_map = {name: pos for name, pos in zip(
                    self.current_robot_state.name,
                    self.current_robot_state.position
                )}
                current_positions = [robot_joint_map.get(name, 0.0) for name in self.target_joint_names]

        # Create trajectory message
        trajectory = JointTrajectory()
        trajectory.joint_names = self.target_joint_names

        # If we have current position, create a 2-point trajectory (smooth motion)
        if current_positions is not None:
            point0 = JointTrajectoryPoint()
            point0.positions = current_positions
            point0.velocities = [0.0] * len(self.target_joint_names)
            point0.time_from_start = Duration(sec=0, nanosec=100000000)  # 0.1 seconds
            trajectory.points.append(point0)

            point1 = JointTrajectoryPoint()
            point1.positions = target_positions
            point1.velocities = [0.0] * len(self.target_joint_names)
            point1.time_from_start = Duration(
                sec=int(self.trajectory_duration),
                nanosec=int((self.trajectory_duration % 1) * 1e9)
            )
            trajectory.points.append(point1)
        else:
            point = JointTrajectoryPoint()
            point.positions = target_positions
            point.velocities = [0.0] * len(self.target_joint_names)
            point.time_from_start = Duration(
                sec=int(self.trajectory_duration),
                nanosec=int((self.trajectory_duration % 1) * 1e9)
            )
            trajectory.points = [point]

        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory = trajectory

        for joint_name in trajectory.joint_names:
            tol = JointTolerance()
            tol.name = joint_name
            tol.position = 0.5
            tol.velocity = 0.5
            tol.acceleration = 1.0
            goal_msg.path_tolerance.append(tol)

        for joint_name in trajectory.joint_names:
            tol = JointTolerance()
            tol.name = joint_name
            tol.position = 0.05
            tol.velocity = 0.1
            tol.acceleration = 0.0
            goal_msg.goal_tolerance.append(tol)

        goal_msg.goal_time_tolerance = Duration(sec=2, nanosec=0)

        self.get_logger().info(
            f'Sending goal: {[f"{p:.3f}" for p in target_positions]}'
        )
        self.get_logger().info(f'Joint names: {self.target_joint_names}')
        self.get_logger().info(f'Trajectory points: {len(trajectory.points)}')

        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)

        # Update last commanded positions (in teachbot joint order)
        self.last_commanded_positions = [teachbot_joint_map.get(name, 0.0) for name in self.teachbot_joint_names]
    
    def goal_response_callback(self, future):
        """Handle goal response."""
        try:
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.get_logger().warn('Goal rejected by action server')
                self.get_logger().warn('Possible reasons: controller not ready, invalid trajectory, or joints mismatch')
                self.get_logger().info('Check that the controller is running: ros2 control list_controllers')
                self.is_executing = False
                return
            
            self.get_logger().info('Goal accepted')
        except Exception as e:
            self.get_logger().error(f'Exception in goal response: {str(e)}')
            self.is_executing = False
            return
        
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)
    
    def get_result_callback(self, future):
        """Handle result."""
        from control_msgs.action import FollowJointTrajectory as FJT
        result = future.result().result
        
        # Map error codes to messages
        error_messages = {
            FJT.Result.SUCCESSFUL: 'SUCCESSFUL',
            FJT.Result.INVALID_GOAL: 'INVALID_GOAL',
            FJT.Result.INVALID_JOINTS: 'INVALID_JOINTS',
            FJT.Result.OLD_HEADER_TIMESTAMP: 'OLD_HEADER_TIMESTAMP',
            FJT.Result.PATH_TOLERANCE_VIOLATED: 'PATH_TOLERANCE_VIOLATED',
            FJT.Result.GOAL_TOLERANCE_VIOLATED: 'GOAL_TOLERANCE_VIOLATED'
        }
        
        error_msg = error_messages.get(result.error_code, f'UNKNOWN({result.error_code})')
        
        if result.error_code == FJT.Result.SUCCESSFUL:
            self.get_logger().info(f'Result: {error_msg}')
        else:
            self.get_logger().warn(f'Result: {error_msg} (code: {result.error_code})')
        
        self.is_executing = False
    
    def feedback_callback(self, feedback_msg):
        """Handle feedback (optional)."""
        pass


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = URTeachBotFollowerAction()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error: {e}')
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
