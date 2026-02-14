#!/usr/bin/env python3
"""
Recording GUI application for Teachbot
Records joint states and pistol state during robot movement.
Can playback recorded trajectories using action interface.
"""

import tkinter as tk
from tkinter import ttk, filedialog, messagebox, scrolledtext
import threading
import json
import os
from datetime import datetime
import logging
import time
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from rclpy.action import ActionClient
from teachbot_interfaces.msg import TeachbotState, TeachbotPistolState
from control_msgs.msg import JointTolerance


class TextWidgetLogHandler(logging.Handler):
    """Custom logging handler that outputs to a tkinter Text widget"""
    
    def __init__(self, text_widget):
        super().__init__()
        self.text_widget = text_widget
    
    def emit(self, record):
        """Emit a log record to the text widget"""
        try:
            msg = self.format(record)
            self.text_widget.config(state=tk.NORMAL)
            self.text_widget.insert(tk.END, msg + '\n')
            self.text_widget.see(tk.END)  # Auto-scroll to bottom
            self.text_widget.update()
        except Exception:
            self.handleError(record)


class LogWindow:
    """Separate window for debug logging"""
    
    def __init__(self, parent, logger):
        self.logger = logger
        self.window = None
        self.log_text = None
        self.text_handler = None
        self.is_visible = False
        self.parent = parent
    
    def create_window(self):
        """Create the log window"""
        if self.window is None:
            self.window = tk.Toplevel(self.parent)
            self.window.title('Teachbot Recorder - Debug Log')
            self.window.geometry('800x400')
            self.window.protocol('WM_DELETE_WINDOW', self.hide_window)
            
            # Create main frame
            main_frame = ttk.Frame(self.window, padding='10')
            main_frame.pack(fill=tk.BOTH, expand=True)
            
            # Title
            title = ttk.Label(main_frame, text='Debug Log', font=('Arial', 12, 'bold'))
            title.pack(pady=5)
            
            # Create text widget with scrollbar
            self.log_text = scrolledtext.ScrolledText(main_frame, height=20, width=100)
            self.log_text.pack(fill=tk.BOTH, expand=True)
            
            # Add text handler for logging
            self.text_handler = TextWidgetLogHandler(self.log_text)
            self.text_handler.setLevel(logging.DEBUG)
            formatter = logging.Formatter('[%(asctime)s] [%(levelname)s] %(message)s', datefmt='%H:%M:%S')
            self.text_handler.setFormatter(formatter)
            self.logger.addHandler(self.text_handler)
            
            self.is_visible = True
    
    def show_window(self):
        """Show the log window"""
        if self.window is None:
            self.create_window()
        else:
            self.window.deiconify()
        self.is_visible = True
    
    def hide_window(self):
        """Hide the log window"""
        if self.window is not None:
            self.window.withdraw()
        self.is_visible = False
    
    def toggle_window(self):
        """Toggle log window visibility"""
        if self.is_visible:
            self.hide_window()
        else:
            self.show_window()


class RecordingNode(Node):
    """ROS 2 node for recording and playback of joint trajectories"""

    def __init__(self):
        super().__init__('teachbot_recorder_gui')
        
        # Declare parameters
        self.declare_parameter('robot_joint_topic', value='/joint_states')
        self.declare_parameter('enable_topic', value='/teachbot/enable')
        self.declare_parameter('teachbot_state_topic', value='/teachbot/state')
        self.declare_parameter('pistol_topic', value='/teachbot/pistol_state')
        self.declare_parameter('controller_name', value='scaled_joint_trajectory_controller')
        self.declare_parameter('controller_name_sim', value='joint_trajectory_controller')
        self.declare_parameter('recording_rate', value=100.0)
        self.declare_parameter('record_pistol', value=True)
        self.declare_parameter('use_sim', value=False)
        self.declare_parameter('trajectory_duration', value=2.0)
        self.declare_parameter('robot_joint_names', value=[
            'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
            'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'
        ])

        # Get parameters
        self.robot_joint_topic = self.get_parameter('robot_joint_topic').value
        self.enable_topic = self.get_parameter('enable_topic').value
        self.teachbot_state_topic = self.get_parameter('teachbot_state_topic').value
        self.pistol_topic = self.get_parameter('pistol_topic').value
        self.controller_name = self.get_parameter('controller_name').value
        self.controller_name_sim = self.get_parameter('controller_name_sim').value
        self.recording_rate = self.get_parameter('recording_rate').value
        self.record_pistol = self.get_parameter('record_pistol').value
        self.use_sim = self.get_parameter('use_sim').value
        self.trajectory_duration = self.get_parameter('trajectory_duration').value
        self.robot_joint_names = list(self.get_parameter('robot_joint_names').value)

        # Recording state
        self.is_recording = False
        self.is_playing = False
        self.is_paused = False
        self.is_moving = False  # Track if robot is currently moving
        self.recording_time = 0.0
        self.recording_start_time = None  # Track actual time when recording started
        self.playback_start_time = None  # Track when playback started for timeout
        self.recorded_data = []
        self.pistol_state = {}
        self.current_joint_state = None
        self.recording_mode = 'button'  # 'button' or 'topic'
        self.recording_stopped_externally = False  # Track if stopped by enable_topic
        self.last_record_time = None  # Throttle sampling to recording_rate
        self.current_point_index = 0  # Track current point for navigation

        # Create callback groups
        self.record_group = MutuallyExclusiveCallbackGroup()
        self.control_group = ReentrantCallbackGroup()

        # Subscriptions
        self.joint_state_sub = self.create_subscription(
            JointState,
            self.robot_joint_topic,
            self.joint_state_callback,
            10,
            callback_group=self.record_group
        )

        self.enable_sub = self.create_subscription(
            Bool,
            self.enable_topic,
            self.enable_callback,
            10,
            callback_group=self.control_group
        )

        self.teachbot_state_sub = self.create_subscription(
            TeachbotState,
            self.teachbot_state_topic,
            self.teachbot_state_callback,
            10,
            callback_group=self.record_group
        )

        # Action client for playback
        self.action_client = ActionClient(
            self,
            FollowJointTrajectory,
            f'{self.controller_name_sim if self.use_sim else self.controller_name}/follow_joint_trajectory'
        )

        # Publisher for pistol state
        self.pistol_pub = self.create_publisher(
            TeachbotPistolState,
            self.pistol_topic,
            10
        )

        self.get_logger().info('RecordingNode initialized')
        
        # Log action server connection info
        self.get_logger().info(f'Action server target: {self.controller_name_sim if self.use_sim else self.controller_name}/follow_joint_trajectory')

    def joint_state_callback(self, msg):
        """Record joint states during recording"""
        if self.is_recording and not self.is_paused:
            self.current_joint_state = msg
            # Calculate elapsed time since recording started
            if self.recording_start_time is not None:
                elapsed_time = time.time() - self.recording_start_time
            else:
                elapsed_time = 0.0
            # Throttle sampling to recording_rate
            if self.recording_rate and self.recording_rate > 0:
                min_interval = 1.0 / self.recording_rate
                if self.last_record_time is not None:
                    if (elapsed_time - self.last_record_time) < min_interval:
                        return
            self.last_record_time = elapsed_time
            
            # Capture joint names from message (handles any joint order from source)
            joint_names_from_msg = list(msg.name)
            
            # Record data point with joint names for ordering verification
            data_point = {
                'time': elapsed_time,
                'joint_names': joint_names_from_msg,  # Store joint names from message
                'positions': list(msg.position),
                'velocities': list(msg.velocity),
                'efforts': list(msg.effort),
                'pistol_state': self.pistol_state.copy()
            }
            self.recorded_data.append(data_point)
            
            # Log joint name mismatch on first recording
            if len(self.recorded_data) == 1:
                if joint_names_from_msg != self.robot_joint_names:
                    self.get_logger().warn(f'Joint order mismatch detected!')
                    self.get_logger().warn(f'  Message joint_names: {joint_names_from_msg}')
                    self.get_logger().warn(f'  YAML joint_names:    {self.robot_joint_names}')
                    self.get_logger().info('Will reorder positions during playback to match YAML configuration.')
                else:
                    self.get_logger().info(f'Joint order matches YAML: {joint_names_from_msg}')

    def enable_callback(self, msg):
        """Start recording when enable signal is received"""
        if self.recording_mode == 'topic':
            if msg.data and not self.is_recording:
                self.start_recording()
                self.recording_stopped_externally = False
            elif not msg.data and self.is_recording:
                self.stop_recording()
                self.recording_stopped_externally = True

    def teachbot_state_callback(self, msg):
        """Capture pistol state data"""
        self.pistol_state = {
            'pot_raw': msg.pistol.pot_raw,
            'pot_percent': msg.pistol.pot_percent,
            'btn1': msg.pistol.btn1,
            'btn2': msg.pistol.btn2,
        }
        if self.is_recording and self.record_pistol:
            pistol_msg = TeachbotPistolState()
            pistol_msg.pot_raw = msg.pistol.pot_raw
            pistol_msg.pot_percent = msg.pistol.pot_percent
            pistol_msg.btn1 = msg.pistol.btn1
            pistol_msg.btn2 = msg.pistol.btn2
            self.pistol_pub.publish(pistol_msg)

    def start_recording(self):
        """Start recording trajectory"""
        self.is_recording = True
        self.is_paused = False
        self.recording_time = 0.0
        self.recording_start_time = time.time()  # Record actual start time
        self.last_record_time = None
        self.recorded_data = []
        self.get_logger().info('Recording started')

    def pause_recording(self):
        """Pause recording"""
        self.is_paused = True
        self.get_logger().info('Recording paused')

    def resume_recording(self):
        """Resume recording"""
        self.is_paused = False
        self.last_record_time = None
        self.get_logger().info('Recording resumed')

    def stop_recording(self):
        """Stop recording"""
        self.is_recording = False
        self.is_paused = False
        self.get_logger().info(f'Recording stopped. {len(self.recorded_data)} points recorded.')

    def save_recording(self, filepath):
        """Save recording to JSON file"""
        try:
            # Extract joint names from first recorded point (if available)
            recorded_joint_names = None
            if self.recorded_data:
                recorded_joint_names = self.recorded_data[0].get('joint_names')
            
            data = {
                'timestamp': datetime.now().isoformat(),
                'controller_name': self.controller_name_sim if self.use_sim else self.controller_name,
                'robot_joint_names': self.robot_joint_names,  # YAML configuration
                'recorded_joint_names': recorded_joint_names,  # Joint order from /joint_states
                'recording_rate': self.recording_rate,
                'trajectory_duration': self.trajectory_duration,
                'points': self.recorded_data
            }
            with open(filepath, 'w') as f:
                json.dump(data, f, indent=2)
            self.get_logger().info(f'Recording saved to {filepath}')
            if recorded_joint_names and recorded_joint_names != self.robot_joint_names:
                self.get_logger().info(f'  (Note: Recording has different joint order than YAML - will be reordered on playback)')
            return True
        except Exception as e:
            self.get_logger().error(f'Failed to save recording: {e}')
            return False

    def load_recording(self, filepath):
        """Load recording from JSON file"""
        try:
            with open(filepath, 'r') as f:
                data = json.load(f)
            self.recorded_data = data.get('points', [])
            
            # Check for joint order mismatch
            recorded_joint_names = data.get('recorded_joint_names')
            if recorded_joint_names and recorded_joint_names != self.robot_joint_names:
                self.get_logger().warn(f'Loaded recording has different joint order than current YAML:')
                self.get_logger().warn(f'  Recorded: {recorded_joint_names}')
                self.get_logger().warn(f'  YAML:     {self.robot_joint_names}')
                self.get_logger().info('Positions will be reordered during playback to match YAML joints.')
                # Reorder all recorded positions to match YAML joint order
                self._reorder_recorded_data(recorded_joint_names)
            elif recorded_joint_names:
                self.get_logger().info(f'Joint order matches: {recorded_joint_names}')
            else:
                self.get_logger().warn('Loaded recording has no joint_names metadata - assuming order matches YAML')
            
            self.get_logger().info(f'Loaded {len(self.recorded_data)} points from {filepath}')
            return True
        except Exception as e:
            self.get_logger().error(f'Failed to load recording: {e}')
            return False
    
    def _reorder_recorded_data(self, source_joint_names):
        """Reorder positions/velocities in recorded data to match robot_joint_names"""
        if not source_joint_names or source_joint_names == self.robot_joint_names:
            return
        
        # Build mapping from source order to YAML order
        try:
            reorder_indices = [source_joint_names.index(joint) for joint in self.robot_joint_names]
        except ValueError as e:
            self.get_logger().error(f'Joint name mismatch - cannot reorder: {e}')
            return
        
        # Reorder all recorded points
        for data_point in self.recorded_data:
            if 'positions' in data_point and len(data_point['positions']) == len(reorder_indices):
                data_point['positions'] = [data_point['positions'][i] for i in reorder_indices]
            if 'velocities' in data_point and len(data_point['velocities']) == len(reorder_indices):
                data_point['velocities'] = [data_point['velocities'][i] for i in reorder_indices]
            if 'efforts' in data_point and len(data_point['efforts']) == len(reorder_indices):
                data_point['efforts'] = [data_point['efforts'][i] for i in reorder_indices]
        
        self.get_logger().info(f'Reordered {len(self.recorded_data)} points from {source_joint_names} to {self.robot_joint_names}')

    def start_playback(self):
        """Start playback of recorded trajectory"""
        if not self.recorded_data:
            self.get_logger().warn('No recording to play')
            return False

        self.is_playing = True
        self.is_paused = False
        self.is_moving = True
        self.playback_start_time = time.time()  # Track when playback started

        # Execute each datapoint individually
        playback_thread = threading.Thread(target=self._playback_points_thread, daemon=True)
        playback_thread.start()
        return True

    def _playback_points_thread(self):
        """Playback by executing each recorded datapoint as an individual goal"""
        action_server = f'{self.controller_name_sim if self.use_sim else self.controller_name}/follow_joint_trajectory'
        self.get_logger().info(f'Checking action server availability: {action_server}')

        if not self.action_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error(f'Action server not reachable: {action_server}')
            self.get_logger().info('Available action servers can be listed with: ros2 action list')
            self.is_playing = False
            self.is_moving = False
            return

        last_time = None
        for i, data_point in enumerate(self.recorded_data):
            if not self.is_playing:
                break

            while self.is_paused and self.is_playing:
                time.sleep(0.05)

            # Update current point index for GUI display
            self.current_point_index = i

            # Update pistol state for GUI display and publish if available
            if 'pistol_state' in data_point and data_point['pistol_state']:
                self.pistol_state = data_point['pistol_state'].copy()
                # Publish pistol state - use very defensive type handling
                try:
                    ps = data_point['pistol_state']
                    pistol_msg = TeachbotPistolState()
                    
                    # Ensure values are proper Python types (not numpy, not string, etc.)
                    raw_pot_raw = ps.get('pot_raw', 0)
                    raw_pot_percent = ps.get('pot_percent', 0.0)
                    raw_btn1 = ps.get('btn1', False)
                    raw_btn2 = ps.get('btn2', False)
                    
                    # Convert to native Python types, being very explicit
                    pistol_msg.pot_raw = int(raw_pot_raw) if raw_pot_raw is not None else 0
                    pistol_msg.pot_percent = int(raw_pot_percent) if raw_pot_percent is not None else 0
                    pistol_msg.btn1 = bool(raw_btn1) if raw_btn1 is not None else False
                    pistol_msg.btn2 = bool(raw_btn2) if raw_btn2 is not None else False
                    
                    # Verify types before publishing
                    assert isinstance(pistol_msg.pot_raw, int), f"pot_raw is {type(pistol_msg.pot_raw)}, not int"
                    assert isinstance(pistol_msg.pot_percent, int), f"pot_percent is {type(pistol_msg.pot_percent)}, not int"
                    assert isinstance(pistol_msg.btn1, bool), f"btn1 is {type(pistol_msg.btn1)}, not bool"
                    assert isinstance(pistol_msg.btn2, bool), f"btn2 is {type(pistol_msg.btn2)}, not bool"
                    
                    self.pistol_pub.publish(pistol_msg)
                except Exception as e:
                    self.get_logger().error(f'Failed to publish pistol state at point {i}: {e}')

            # Compute per-point duration
            if last_time is None:
                dt = 0.05
            else:
                dt = max(0.01, data_point['time'] - last_time)
            last_time = data_point['time']

            trajectory = JointTrajectory()
            trajectory.joint_names = self.robot_joint_names

            point = JointTrajectoryPoint()
            point.positions = data_point['positions']
            point.velocities = [0.0] * len(self.robot_joint_names)
            point.effort = []
            point.time_from_start.sec = int(dt)
            point.time_from_start.nanosec = int((dt % 1.0) * 1e9)
            trajectory.points.append(point)

            if not self._send_single_point_goal(trajectory, i):
                self.is_playing = False
                self.is_moving = False
                # Reset pistol state to zero on failure
                self._reset_pistol_state()
                return

        self.is_playing = False
        self.is_moving = False
        # Reset pistol state to zero at end of playback
        self._reset_pistol_state()

    def _send_single_point_goal(self, trajectory, index):
        """Send a single-point trajectory goal and wait for result"""
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory

        goal.path_tolerance = [
            JointTolerance(name=joint, position=6.0, velocity=10.0)
            for joint in self.robot_joint_names
        ]
        goal.goal_tolerance = [
            JointTolerance(name=joint, position=0.1, velocity=0.5, time_from_start=rclpy.time.Duration(seconds=2))
            for joint in self.robot_joint_names
        ]

        done_event = threading.Event()
        goal_handle_container = {}

        def _goal_done(future):
            goal_handle_container['handle'] = future.result()
            done_event.set()

        future = self.action_client.send_goal_async(goal)
        future.add_done_callback(_goal_done)
        if not done_event.wait(timeout=5.0):
            self.get_logger().error(f'Point {index}: goal request timed out')
            return False

        goal_handle = goal_handle_container.get('handle')
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error(f'Point {index}: goal rejected')
            return False

        result_event = threading.Event()
        result_container = {}

        def _result_done(result_future):
            result_container['result'] = result_future.result()
            result_event.set()

        goal_handle.get_result_async().add_done_callback(_result_done)
        if not result_event.wait(timeout=15.0):
            self.get_logger().error(f'Point {index}: result timed out')
            return False

        return True

    def _reset_pistol_state(self):
        """Reset pistol state to zero/false"""
        pistol_msg = TeachbotPistolState()
        pistol_msg.pot_raw = 0
        pistol_msg.pot_percent = 0
        pistol_msg.btn1 = False
        pistol_msg.btn2 = False
        self.pistol_pub.publish(pistol_msg)
        # Update internal state for GUI display
        self.pistol_state = {
            'pot_raw': 0,
            'pot_percent': 0,
            'btn1': False,
            'btn2': False
        }
        self.get_logger().info('Pistol state reset to zero')

    def pause_playback(self):
        """Pause playback"""
        self.is_paused = True
        self.get_logger().info('Playback paused')

    def resume_playback(self):
        """Resume playback"""
        self.is_paused = False
        self.get_logger().info('Playback resumed')

    def stop_playback(self):
        """Stop playback"""
        self.is_playing = False
        self.is_paused = False
        self.is_moving = False
        self._reset_pistol_state()
        self.get_logger().info('Playback stopped')


    def go_to_first_point(self):
        """Move robot to first recorded point"""
        if not self.recorded_data:
            self.get_logger().warn('No recording available')
            return False
        self.current_point_index = 0
        return self.go_to_point(0)

    def go_to_previous_point(self):
        """Move robot to previous recorded point"""
        if not self.recorded_data:
            self.get_logger().warn('No recording available')
            return False
        if self.current_point_index > 0:
            self.current_point_index -= 1
            return self.go_to_point(self.current_point_index)
        else:
            self.get_logger().info('Already at first point')
            return False

    def go_to_next_point(self):
        """Move robot to next recorded point"""
        if not self.recorded_data:
            self.get_logger().warn('No recording available')
            return False
        if self.current_point_index < len(self.recorded_data) - 1:
            self.current_point_index += 1
            return self.go_to_point(self.current_point_index)
        else:
            self.get_logger().info('Already at last point')
            return False

    def go_to_point(self, index):
        """Move robot to specific point by index"""
        if not self.recorded_data or index < 0 or index >= len(self.recorded_data):
            self.get_logger().warn('Invalid point index')
            return False

        # Create a single-point trajectory with specified point
        trajectory = JointTrajectory()
        trajectory.joint_names = self.robot_joint_names

        point_data = self.recorded_data[index]
        point = JointTrajectoryPoint()
        point.positions = point_data['positions']
        point.velocities = [0.0] * len(self.robot_joint_names)
        point.effort = []
        # Use 2.0 seconds for point-to-point navigation to allow smooth execution
        point.time_from_start.sec = 2
        point.time_from_start.nanosec = 0
        trajectory.points.append(point)

        self.get_logger().info(f'Moving to point {index}: {point_data["positions"]}')
        
        # Set moving flag and use shared _send_single_point_goal function
        self.is_moving = True
        result = self._send_single_point_goal(trajectory, index)
        self.is_moving = False
        return result

    def go_forward_by_points(self, num_points):
        """Move robot forward by specified number of points"""
        if not self.recorded_data:
            self.get_logger().warn('No recording available')
            return False
        new_index = min(self.current_point_index + num_points, len(self.recorded_data) - 1)
        self.current_point_index = new_index
        return self.go_to_point(new_index)

    def go_backward_by_points(self, num_points):
        """Move robot backward by specified number of points"""
        if not self.recorded_data:
            self.get_logger().warn('No recording available')
            return False
        new_index = max(self.current_point_index - num_points, 0)
        self.current_point_index = new_index
        return self.go_to_point(new_index)


class RecordingGUI:
    """GUI for recording and playback control"""

    def __init__(self, root, ros_node):
        self.root = root
        self.node = ros_node
        self.root.title('Teachbot Recorder')
        self.root.geometry('800x1000')

        # Threading
        self.ros_thread = None
        self.update_timer = None
        
        # Setup logging to GUI
        self.setup_logging()
        
        # Create log window
        self.log_window = LogWindow(self.root, self.logger)
        self.log_window.show_window()

        # Create GUI
        self.create_widgets()

        # Start ROS spin in background thread
        self.start_ros_thread()

        # Schedule periodic updates
        self.update_gui()
    
    def setup_logging(self):
        """Setup logging to capture output in GUI"""
        # Get logger
        self.logger = logging.getLogger('teachbot_recorder')
        self.logger.setLevel(logging.DEBUG)
        
        # Remove existing handlers
        self.logger.handlers = []
        
        # Create console handler for debugging
        console_handler = logging.StreamHandler()
        console_handler.setLevel(logging.INFO)
        formatter = logging.Formatter('[%(levelname)s] %(message)s')
        console_handler.setFormatter(formatter)
        self.logger.addHandler(console_handler)

    def create_widgets(self):
        """Create GUI widgets"""
        # Main frame
        main_frame = ttk.Frame(self.root, padding='10')
        main_frame.pack(fill=tk.BOTH, expand=True)

        # Title
        title = ttk.Label(main_frame, text='Teachbot Recorder', font=('Arial', 16, 'bold'))
        title.pack(pady=10)

        # Recording mode selection
        mode_frame = ttk.LabelFrame(main_frame, text='Recording Start Method', padding='10')
        mode_frame.pack(fill=tk.X, pady=10)

        self.recording_mode_var = tk.StringVar(value='button')
        ttk.Radiobutton(mode_frame, text='GUI Button', variable=self.recording_mode_var, value='button', command=self.on_mode_changed).pack(anchor=tk.W)
        ttk.Radiobutton(mode_frame, text='Enable Topic', variable=self.recording_mode_var, value='topic', command=self.on_mode_changed).pack(anchor=tk.W)

        # Recording section
        recording_frame = ttk.LabelFrame(main_frame, text='Recording', padding='10')
        recording_frame.pack(fill=tk.X, pady=10)

        self.rec_button = ttk.Button(recording_frame, text='Start Recording', command=self.start_recording, state=tk.NORMAL)
        self.rec_button.pack(side=tk.LEFT, padx=5)

        self.pause_rec_button = ttk.Button(recording_frame, text='Pause', command=self.pause_recording, state=tk.DISABLED)
        self.pause_rec_button.pack(side=tk.LEFT, padx=5)

        self.stop_rec_button = ttk.Button(recording_frame, text='Stop', command=self.stop_recording, state=tk.DISABLED)
        self.stop_rec_button.pack(side=tk.LEFT, padx=5)

        self.save_button = ttk.Button(recording_frame, text='Save Recording', command=self.save_recording, state=tk.DISABLED)
        self.save_button.pack(side=tk.LEFT, padx=5)

        # Playback section
        playback_frame = ttk.LabelFrame(main_frame, text='Playback', padding='10')
        playback_frame.pack(fill=tk.X, pady=10)

        self.load_button = ttk.Button(playback_frame, text='Load Recording', command=self.load_recording)
        self.load_button.pack(side=tk.LEFT, padx=5)

        self.play_button = ttk.Button(playback_frame, text='Play', command=self.start_playback, state=tk.DISABLED)
        self.play_button.pack(side=tk.LEFT, padx=5)

        self.pause_play_button = ttk.Button(playback_frame, text='Pause', command=self.pause_playback, state=tk.DISABLED)
        self.pause_play_button.pack(side=tk.LEFT, padx=5)

        self.stop_play_button = ttk.Button(playback_frame, text='Stop', command=self.stop_playback, state=tk.DISABLED)
        self.stop_play_button.pack(side=tk.LEFT, padx=5)

        self.go_first_button = ttk.Button(playback_frame, text='Go to First Point', command=self.go_to_first_point, state=tk.DISABLED)
        self.go_first_button.pack(side=tk.LEFT, padx=5)

        self.prev_button = ttk.Button(playback_frame, text='< Previous', command=self.go_to_previous_point, state=tk.DISABLED)
        self.prev_button.pack(side=tk.LEFT, padx=5)

        self.next_button = ttk.Button(playback_frame, text='Next >', command=self.go_to_next_point, state=tk.DISABLED)
        self.next_button.pack(side=tk.LEFT, padx=5)

        # Jump navigation section
        jump_frame = ttk.LabelFrame(main_frame, text='Jump Navigation', padding='10')
        jump_frame.pack(fill=tk.X, pady=10)

        ttk.Label(jump_frame, text='Step Size:', font=('Arial', 9, 'bold')).pack(side=tk.LEFT, padx=5)
        
        self.step_size_var = tk.StringVar(value='5')
        self.step_size_combo = ttk.Combobox(jump_frame, textvariable=self.step_size_var, values=['2', '5', '10', '20', '50', '100'], 
                                            state='readonly', width=5)
        self.step_size_combo.pack(side=tk.LEFT, padx=5)
        
        self.back_button = ttk.Button(jump_frame, text='<< Back', command=self.jump_backward_from_dropdown, state=tk.DISABLED, width=10)
        self.back_button.pack(side=tk.LEFT, padx=10)
        
        self.fwd_button = ttk.Button(jump_frame, text='Forward >>', command=self.jump_forward_from_dropdown, state=tk.DISABLED, width=10)
        self.fwd_button.pack(side=tk.LEFT, padx=10)

        # Status section
        status_frame = ttk.LabelFrame(main_frame, text='Status', padding='10')
        status_frame.pack(fill=tk.X, pady=10)

        ttk.Label(status_frame, text='Recording Status:').pack(anchor=tk.W)
        self.rec_status_label = ttk.Label(status_frame, text='Stopped', foreground='gray')
        self.rec_status_label.pack(anchor=tk.W, padx=20)

        ttk.Label(status_frame, text='Recording Time:').pack(anchor=tk.W, pady=(10, 0))
        self.rec_time_label = ttk.Label(status_frame, text='0.00 s', foreground='blue')
        self.rec_time_label.pack(anchor=tk.W, padx=20)

        ttk.Label(status_frame, text='Recorded Points:').pack(anchor=tk.W, pady=(10, 0))
        self.point_count_label = ttk.Label(status_frame, text='0', foreground='blue')
        self.point_count_label.pack(anchor=tk.W, padx=20)

        ttk.Label(status_frame, text='Current Point:').pack(anchor=tk.W, pady=(10, 0))
        self.current_point_label = ttk.Label(status_frame, text='0 / 0', foreground='green')
        self.current_point_label.pack(anchor=tk.W, padx=20)

        ttk.Label(status_frame, text='Playback Status:').pack(anchor=tk.W, pady=(10, 0))
        self.play_status_label = ttk.Label(status_frame, text='Stopped', foreground='gray')
        self.play_status_label.pack(anchor=tk.W, padx=20)

        ttk.Label(status_frame, text='Playback Progress:').pack(anchor=tk.W, pady=(10, 0))
        self.play_progress = ttk.Progressbar(status_frame, mode='determinate', length=400)
        self.play_progress.pack(anchor=tk.W, padx=20, pady=5)

        # Pistol state section
        ttk.Label(status_frame, text='Pistol State:').pack(anchor=tk.W, pady=(10, 0))
        self.pistol_state_label = ttk.Label(status_frame, text='pot_raw=0, pot_percent=0.0, btn1=False, btn2=False', foreground='purple')
        self.pistol_state_label.pack(anchor=tk.W, padx=20)

        # Robot movement status
        ttk.Label(status_frame, text='Robot Status:').pack(anchor=tk.W, pady=(10, 0))
        self.robot_status_label = ttk.Label(status_frame, text='Idle', foreground='gray', font=('Arial', 10, 'bold'))
        self.robot_status_label.pack(anchor=tk.W, padx=20)

        button_frame = ttk.Frame(main_frame)
        button_frame.pack(fill=tk.X, pady=10)

        log_button = ttk.Button(button_frame, text='Show Debug Log', command=self.toggle_log_window)
        log_button.pack(side=tk.LEFT, padx=5)

        # Quit button
        quit_button = ttk.Button(button_frame, text='Quit', command=self.quit_app)
        quit_button.pack(side=tk.LEFT, padx=5)

    def start_ros_thread(self):
        """Start ROS node in background thread"""
        def ros_spin():
            executor = MultiThreadedExecutor(num_threads=2)
            executor.add_node(self.node)
            executor.spin()

        self.ros_thread = threading.Thread(target=ros_spin, daemon=True)
        self.ros_thread.start()
        
        # Log startup
        self.logger.info('Teachbot Recorder started')

    def toggle_log_window(self):
        """Toggle log window visibility"""
        self.log_window.toggle_window()

    def on_mode_changed(self):
        """Handle recording mode change"""
        self.node.recording_mode = self.recording_mode_var.get()
        # Update button states based on mode
        if self.node.recording_mode == 'topic':
            self.rec_button.config(state=tk.DISABLED, text='Waiting for Enable Topic...')
            self.logger.info(f'Recording mode changed to: TOPIC (waiting for /teachbot/enable signal)')
        else:
            self.rec_button.config(state=tk.NORMAL, text='Start Recording')
            self.logger.info(f'Recording mode changed to: GUI BUTTON')
        self.node.get_logger().info(f'Recording mode changed to: {self.node.recording_mode}')

    def start_recording(self):
        """Start recording"""
        if self.node.recording_mode == 'button':
            self.logger.info('Starting recording from GUI button')
            self.node.start_recording()
            self.logger.debug(f'Recording started - Topics: {self.node.robot_joint_topic}, {self.node.teachbot_state_topic}')
            self.rec_button.config(state=tk.DISABLED)
            self.pause_rec_button.config(state=tk.NORMAL)
            self.stop_rec_button.config(state=tk.NORMAL)
            self.save_button.config(state=tk.DISABLED)

    def pause_recording(self):
        """Pause/Resume recording"""
        if self.node.is_paused:
            self.node.resume_recording()
            self.pause_rec_button.config(text='Pause')
            self.logger.info('Recording resumed')
        else:
            self.node.pause_recording()
            self.pause_rec_button.config(text='Resume')
            self.logger.info('Recording paused')

    def stop_recording(self):
        """Stop recording"""
        self.logger.info(f'Stopping recording - Recorded {len(self.node.recorded_data)} points')
        self.node.stop_recording()
        if self.node.recording_mode == 'topic':
            self.rec_button.config(state=tk.DISABLED, text='Waiting for Enable Topic...')
        else:
            self.rec_button.config(state=tk.NORMAL, text='Start Recording')
        self.pause_rec_button.config(state=tk.DISABLED, text='Pause')
        self.stop_rec_button.config(state=tk.DISABLED)
        self.save_button.config(state=tk.NORMAL)

    def save_recording(self):
        """Save recording to file"""
        filepath = filedialog.asksaveasfilename(
            defaultextension='.json',
            filetypes=[('JSON files', '*.json'), ('All files', '*.*')],
            initialdir=os.path.expanduser('~')
        )
        if filepath:
            self.logger.info(f'Saving recording to: {filepath}')
            if self.node.save_recording(filepath):
                self.logger.info(f'Recording successfully saved with {len(self.node.recorded_data)} points')
                messagebox.showinfo('Success', f'Recording saved to {filepath}')
            else:
                self.logger.error('Failed to save recording')
                messagebox.showerror('Error', 'Failed to save recording')

    def load_recording(self):
        """Load recording from file"""
        filepath = filedialog.askopenfilename(
            filetypes=[('JSON files', '*.json'), ('All files', '*.*')],
            initialdir=os.path.expanduser('~')
        )
        if filepath:
            self.logger.info(f'Loading recording from: {filepath}')
            if self.node.load_recording(filepath):
                self.logger.info(f'Recording loaded successfully - {len(self.node.recorded_data)} points')
                self.play_button.config(state=tk.NORMAL)
                self.go_first_button.config(state=tk.NORMAL)
                self.prev_button.config(state=tk.NORMAL)
                self.next_button.config(state=tk.NORMAL)
                # Enable jump buttons
                self.back_button.config(state=tk.NORMAL)
                self.fwd_button.config(state=tk.NORMAL)
                messagebox.showinfo('Success', f'Recording loaded from {filepath}')
            else:
                self.logger.error('Failed to load recording')
                messagebox.showerror('Error', 'Failed to load recording')

    def start_playback(self):
        """Start playback"""
        self.logger.info(f'Starting playback of {len(self.node.recorded_data)} points')
        self.logger.debug(f'Joint names: {self.node.robot_joint_names}')
        if self.node.start_playback():
            self.logger.info('Playback started successfully')
            self.play_button.config(state=tk.DISABLED)
            self.pause_play_button.config(state=tk.NORMAL)
            self.stop_play_button.config(state=tk.NORMAL)
            self.load_button.config(state=tk.DISABLED)
        else:
            self.logger.error('Failed to start playback - check trajectory validity')

    def pause_playback(self):
        """Pause/Resume playback"""
        if self.node.is_paused:
            self.node.resume_playback()
            self.pause_play_button.config(text='Pause')
            self.logger.info('Playback resumed')
        else:
            self.node.pause_playback()
            self.pause_play_button.config(text='Resume')
            self.logger.info('Playback paused')

    def stop_playback(self):
        """Stop playback"""
        self.logger.info('Stopping playback')
        self.node.stop_playback()
        self.play_button.config(state=tk.NORMAL)
        self.pause_play_button.config(state=tk.DISABLED, text='Pause')
        self.stop_play_button.config(state=tk.DISABLED)
        self.load_button.config(state=tk.NORMAL)

    def go_to_first_point(self):
        """Move robot to first recorded point"""
        if not self.node.recorded_data:
            messagebox.showwarning('Warning', 'No recording loaded')
            return
        self.logger.info('Moving to first point')
        if self.node.go_to_first_point():
            self.logger.info(f'First point goal sent (point 0 of {len(self.node.recorded_data)})')
        else:
            self.logger.error('Failed to send first point goal')

    def go_to_previous_point(self):
        """Move robot to previous recorded point"""
        if not self.node.recorded_data:
            messagebox.showwarning('Warning', 'No recording loaded')
            return
        self.logger.info('Moving to previous point')
        if self.node.go_to_previous_point():
            self.logger.info(f'Previous point goal sent (point {self.node.current_point_index} of {len(self.node.recorded_data)})')
        else:
            self.logger.error('Cannot move to previous point')

    def go_to_next_point(self):
        """Move robot to next recorded point"""
        if not self.node.recorded_data:
            messagebox.showwarning('Warning', 'No recording loaded')
            return
        self.logger.info('Moving to next point')
        if self.node.go_to_next_point():
            self.logger.info(f'Next point goal sent (point {self.node.current_point_index} of {len(self.node.recorded_data)})')
        else:
            self.logger.error('Cannot move to next point')

    def jump_forward(self, num_points):
        """Jump forward by specified number of points"""
        if not self.node.recorded_data:
            messagebox.showwarning('Warning', 'No recording loaded')
            return
        if self.node.go_forward_by_points(num_points):
            self.logger.info(f'Jumped forward {num_points} points (now at point {self.node.current_point_index} of {len(self.node.recorded_data)})')
        else:
            self.logger.error(f'Failed to jump forward {num_points} points')

    def jump_forward_from_dropdown(self):
        """Jump forward using step size from dropdown"""
        try:
            step_size = int(self.step_size_var.get())
            self.jump_forward(step_size)
        except ValueError:
            messagebox.showerror('Error', 'Invalid step size')

    def jump_backward_from_dropdown(self):
        """Jump backward using step size from dropdown"""
        try:
            step_size = int(self.step_size_var.get())
            self.jump_backward(step_size)
        except ValueError:
            messagebox.showerror('Error', 'Invalid step size')

    def jump_backward(self, num_points):
        """Jump backward by specified number of points"""
        if not self.node.recorded_data:
            messagebox.showwarning('Warning', 'No recording loaded')
            return
        if self.node.go_backward_by_points(num_points):
            self.logger.info(f'Jumped backward {num_points} points (now at point {self.node.current_point_index} of {len(self.node.recorded_data)})')
        else:
            self.logger.error(f'Failed to jump backward {num_points} points')

    def update_gui(self):
        """Update GUI status labels"""
        # Check if recording was stopped externally (by enable_topic)
        if self.node.recording_stopped_externally and not self.node.is_recording:
            self.node.recording_stopped_externally = False
            self.logger.info('Recording stopped by enable_topic signal')
            if self.node.recording_mode == 'topic':
                self.rec_button.config(state=tk.DISABLED, text='Waiting for Enable Topic...')
            self.pause_rec_button.config(state=tk.DISABLED, text='Pause')
            self.stop_rec_button.config(state=tk.DISABLED)
            self.save_button.config(state=tk.NORMAL)

        # Update recording status
        if self.node.is_recording:
            if self.node.is_paused:
                self.rec_status_label.config(text='Paused', foreground='orange')
            else:
                self.rec_status_label.config(text='Recording', foreground='red')
            # Update recording time from actual elapsed time
            if self.node.recording_start_time is not None:
                self.node.recording_time = time.time() - self.node.recording_start_time
            self.rec_time_label.config(text=f'{self.node.recording_time:.2f} s')
        else:
            self.rec_status_label.config(text='Stopped', foreground='gray')

        # Update point count
        self.point_count_label.config(text=str(len(self.node.recorded_data)))

        # Update current point display
        if self.node.recorded_data:
            self.current_point_label.config(text=f'{self.node.current_point_index} / {len(self.node.recorded_data) - 1}')
        else:
            self.current_point_label.config(text='0 / 0')

        # Update playback status
        if self.node.is_playing:
            if self.node.is_paused:
                self.play_status_label.config(text='Paused', foreground='orange')
            else:
                self.play_status_label.config(text='Playing', foreground='green')
                # Update progress bar based on recorded trajectory duration (if available)
                if self.node.playback_start_time is not None and self.node.recorded_data:
                    elapsed = time.time() - self.node.playback_start_time
                    # Estimate total duration: last point's time + execution buffer
                    estimated_duration = self.node.recorded_data[-1]['time'] + 5.0 if self.node.recorded_data else 10.0
                    progress = min(100, (elapsed / estimated_duration) * 100)
                    self.play_progress['value'] = progress
        else:
            self.play_status_label.config(text='Stopped', foreground='gray')
            self.play_progress['value'] = 0

        # Update pistol state display
        if self.node.pistol_state:
            pot_raw = self.node.pistol_state.get('pot_raw', 0)
            pot_percent = self.node.pistol_state.get('pot_percent', 0.0)
            btn1 = self.node.pistol_state.get('btn1', False)
            btn2 = self.node.pistol_state.get('btn2', False)
            self.pistol_state_label.config(
                text=f'pot_raw={pot_raw}, pot_percent={pot_percent:.2f}, btn1={btn1}, btn2={btn2}'
            )

        # Update robot movement status
        if self.node.is_moving:
            self.robot_status_label.config(text='🤖 Robot Moving...', foreground='orange')
        else:
            self.robot_status_label.config(text='Idle', foreground='gray')

        # Schedule next update
        self.update_timer = self.root.after(100, self.update_gui)

    def quit_app(self):
        """Quit application"""
        self.root.quit()
        if self.update_timer:
            self.root.after_cancel(self.update_timer)


def main(args=None):
    """Main entry point"""
    rclpy.init(args=args)
    
    # Create ROS node
    node = RecordingNode()
    
    # Create GUI
    root = tk.Tk()
    gui = RecordingGUI(root, node)
    
    # Run GUI
    root.mainloop()
    
    # Cleanup
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
