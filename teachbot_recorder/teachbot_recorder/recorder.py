#!/usr/bin/env python3
"""
Recording GUI application for TOS Teachbot
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
            self.window.title('TOS Teachbot Recorder - Debug Log')
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
        self.recording_time = 0.0
        self.recording_start_time = None  # Track actual time when recording started
        self.playback_start_time = None  # Track when playback started for timeout
        self.recorded_data = []
        self.pistol_state = {}
        self.current_joint_state = None
        self.recording_mode = 'button'  # 'button' or 'topic'
        self.recording_stopped_externally = False  # Track if stopped by enable_topic

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

        self.pistol_sub = self.create_subscription(
            TeachbotState,
            self.teachbot_state_topic,
            self.pistol_callback,
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
            # Record data point
            data_point = {
                'time': elapsed_time,
                'positions': list(msg.position),
                'velocities': list(msg.velocity),
                'efforts': list(msg.effort),
                'pistol_state': self.pistol_state.copy()
            }
            self.recorded_data.append(data_point)

    def enable_callback(self, msg):
        """Start recording when enable signal is received"""
        if self.recording_mode == 'topic':
            if msg.data and not self.is_recording:
                self.start_recording()
                self.recording_stopped_externally = False
            elif not msg.data and self.is_recording:
                self.stop_recording()
                self.recording_stopped_externally = True

    def pistol_callback(self, msg):
        """Capture pistol state data"""
        self.pistol_state = {
            'position': msg.tcp_x,
            'tcp_y': msg.tcp_y,
            'tcp_z': msg.tcp_z,
        }

    def start_recording(self):
        """Start recording trajectory"""
        self.is_recording = True
        self.is_paused = False
        self.recording_time = 0.0
        self.recording_start_time = time.time()  # Record actual start time
        self.recorded_data = []
        self.get_logger().info('Recording started')

    def pause_recording(self):
        """Pause recording"""
        self.is_paused = True
        self.get_logger().info('Recording paused')

    def resume_recording(self):
        """Resume recording"""
        self.is_paused = False
        self.get_logger().info('Recording resumed')

    def stop_recording(self):
        """Stop recording"""
        self.is_recording = False
        self.is_paused = False
        self.get_logger().info(f'Recording stopped. {len(self.recorded_data)} points recorded.')

    def save_recording(self, filepath):
        """Save recording to JSON file"""
        try:
            data = {
                'timestamp': datetime.now().isoformat(),
                'controller_name': self.controller_name_sim if self.use_sim else self.controller_name,
                'robot_joint_names': self.robot_joint_names,
                'recording_rate': self.recording_rate,
                'trajectory_duration': self.trajectory_duration,
                'points': self.recorded_data
            }
            with open(filepath, 'w') as f:
                json.dump(data, f, indent=2)
            self.get_logger().info(f'Recording saved to {filepath}')
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
            self.get_logger().info(f'Loaded {len(self.recorded_data)} points from {filepath}')
            return True
        except Exception as e:
            self.get_logger().error(f'Failed to load recording: {e}')
            return False

    def start_playback(self):
        """Start playback of recorded trajectory"""
        if not self.recorded_data:
            self.get_logger().warn('No recording to play')
            return False

        self.is_playing = True
        self.is_paused = False
        self.playback_start_time = time.time()  # Track when playback started

        # Build trajectory from recorded data
        trajectory = self.build_trajectory()
        
        # Validate trajectory
        if not trajectory.points:
            self.get_logger().error('Built trajectory has no points')
            self.is_playing = False
            return False
        
        if len(trajectory.joint_names) != len(self.robot_joint_names):
            self.get_logger().error(
                f'Joint names mismatch: trajectory has {len(trajectory.joint_names)} joints, '
                f'but {len(self.robot_joint_names)} expected'
            )
            self.is_playing = False
            return False
        
        # Check if action server is available
        action_server = f'{self.controller_name_sim if self.use_sim else self.controller_name}/follow_joint_trajectory'
        self.get_logger().info(f'Checking action server availability: {action_server}')
        
        if not self.action_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error(f'Action server not reachable: {action_server}')
            self.get_logger().info('Available action servers can be listed with: ros2 action list')
            self.is_playing = False
            return False

        # Send goal to action server
        self.send_trajectory_goal(trajectory)
        return True

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
        self.get_logger().info('Playback stopped')

    def build_trajectory(self):
        """Build JointTrajectory message from recorded data"""
        trajectory = JointTrajectory()
        trajectory.joint_names = self.robot_joint_names
        
        if not self.recorded_data:
            return trajectory

        # Calculate time scaling
        max_time = self.recorded_data[-1]['time'] if self.recorded_data else 1.0
        time_scale = self.trajectory_duration / max(max_time, 0.1)
        self.get_logger().info(f'Time scaling: recorded duration={max_time:.2f}s, trajectory_duration={self.trajectory_duration}s, time_scale={time_scale:.3f}')

        last_time = -1.0  # Track last time to ensure strictly increasing
        min_time_increment = 0.001  # Minimum 1ms between points

        for i, data_point in enumerate(self.recorded_data):
            point = JointTrajectoryPoint()
            point.positions = data_point['positions']
            # Scale velocities proportionally with time compression
            point.velocities = [v * time_scale for v in data_point['velocities']]
            point.effort = []  # Don't include effort - controller uses position/velocity interface
            
            # Calculate scaled time
            scaled_time = data_point['time'] * time_scale
            
            # Ensure strictly increasing time
            if scaled_time <= last_time:
                scaled_time = last_time + min_time_increment
                self.get_logger().debug(f'Point {i}: adjusted time from {data_point["time"] * time_scale:.6f} to {scaled_time:.6f} to maintain strict ordering')
            
            point.time_from_start.sec = int(scaled_time)
            point.time_from_start.nanosec = int((scaled_time % 1.0) * 1e9)
            
            # Validate point data
            if any(float('nan') if isinstance(p, float) and p != p else False for p in point.positions):
                self.get_logger().error(f'Point {i} contains NaN in positions: {point.positions}')
                continue
            if any(float('inf') if isinstance(p, float) and abs(p) == float('inf') else False for p in point.positions):
                self.get_logger().error(f'Point {i} contains inf in positions: {point.positions}')
                continue
                
            trajectory.points.append(point)
            last_time = scaled_time
        
        # Ensure last point has zero velocities (safety requirement)
        if trajectory.points:
            last_point = trajectory.points[-1]
            last_point.velocities = [0.0] * len(self.robot_joint_names)
            self.get_logger().info('Set final trajectory point velocities to zero')
        
        # Log trajectory info
        self.get_logger().info(f'Built trajectory: {len(trajectory.points)} valid points, joint_names={trajectory.joint_names}')
        if trajectory.points:
            self.get_logger().debug(f'First point: pos={trajectory.points[0].positions}, vel={trajectory.points[0].velocities}, time={trajectory.points[0].time_from_start}')
            self.get_logger().debug(f'Last point: pos={trajectory.points[-1].positions}, vel={trajectory.points[-1].velocities}, time={trajectory.points[-1].time_from_start}')

        return trajectory

    def send_trajectory_goal(self, trajectory):
        """Send trajectory goal to action server"""
        # Log action server name
        action_server = f'{self.controller_name_sim if self.use_sim else self.controller_name}/follow_joint_trajectory'
        self.get_logger().info(f'Action server: {action_server}')
        self.get_logger().info(f'Sim mode: {self.use_sim}')
        self.get_logger().info(f'Trajectory has {len(trajectory.points)} points')
        
        if not self.action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error(f'Action server not available: {action_server}')
            self.is_playing = False
            return

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory
        
        # Add generous tolerances - allows robot to start from different positions
        # State tolerance (path tolerance) must be large to allow motion from current position to trajectory start
        from control_msgs.msg import JointTolerance
        goal.path_tolerance = [
            JointTolerance(name=joint, position=3.0, velocity=10.0)
            for joint in self.robot_joint_names
        ]
        # Goal tolerance - tighter tolerance at trajectory end
        goal.goal_tolerance = [
            JointTolerance(name=joint, position=0.1, velocity=0.5, time_from_start=rclpy.time.Duration(seconds=2))
            for joint in self.robot_joint_names
        ]

        self.get_logger().info(f'Sending trajectory with {len(trajectory.points)} points to action server')
        self.get_logger().debug(f'Joint names in trajectory: {trajectory.joint_names}')
        self.get_logger().debug(f'Expected joint names: {self.robot_joint_names}')
        self.get_logger().info('Calling send_goal_async...')
        future = self.action_client.send_goal_async(goal)
        self.get_logger().info(f'Goal future returned: {future}')
        future.add_done_callback(self.goal_response_callback)
        self.get_logger().info('Callback added to future')

    def goal_response_callback(self, future):
        """Handle action goal response"""
        try:
            self.get_logger().info('goal_response_callback invoked')
            goal_handle = future.result()
            self.get_logger().info(f'Goal handle received: {goal_handle}')
            if not goal_handle.accepted:
                self.get_logger().error('Goal rejected by action server')
                self.is_playing = False
                return

            self.get_logger().info('Goal accepted by action server')
            self.get_logger().info('Requesting result async...')
            goal_handle.get_result_async().add_done_callback(self.get_result_callback)
            self.get_logger().info('Result callback registered')
        except Exception as e:
            self.get_logger().error(f'Goal response error: {e}')
            import traceback
            self.get_logger().error(traceback.format_exc())
            self.is_playing = False

    def get_result_callback(self, future):
        """Handle action result"""
        try:
            result = future.result()
            self.get_logger().info('Playback completed successfully')
            self.is_playing = False
        except Exception as e:
            self.get_logger().error(f'Playback result callback error: {e}')
            self.is_playing = False
            # Force stop playback in case callback failed
            self.stop_playback()


class RecordingGUI:
    """GUI for recording and playback control"""

    def __init__(self, root, ros_node):
        self.root = root
        self.node = ros_node
        self.root.title('TOS Teachbot Recorder')
        self.root.geometry('900x800')

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
        title = ttk.Label(main_frame, text='TOS Teachbot Recorder', font=('Arial', 16, 'bold'))
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

        ttk.Label(status_frame, text='Playback Status:').pack(anchor=tk.W, pady=(10, 0))
        self.play_status_label = ttk.Label(status_frame, text='Stopped', foreground='gray')
        self.play_status_label.pack(anchor=tk.W, padx=20)

        ttk.Label(status_frame, text='Playback Progress:').pack(anchor=tk.W, pady=(10, 0))
        self.play_progress = ttk.Progressbar(status_frame, mode='determinate', length=400)
        self.play_progress.pack(anchor=tk.W, padx=20, pady=5)
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
        self.logger.info('TOS Teachbot Recorder started')

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

        # Update playback status
        if self.node.is_playing:
            if self.node.is_paused:
                self.play_status_label.config(text='Paused', foreground='orange')
            else:
                self.play_status_label.config(text='Playing', foreground='green')
                # Update progress bar
                if self.node.playback_start_time is not None:
                    elapsed = time.time() - self.node.playback_start_time
                    progress = min(100, (elapsed / self.node.trajectory_duration) * 100)
                    self.play_progress['value'] = progress
                # Check for playback timeout - if playing much longer than trajectory duration, force stop
                if self.node.playback_start_time is not None:
                    elapsed = time.time() - self.node.playback_start_time
                    max_allowed = self.node.trajectory_duration * 2.0 + 2.0  # Allow 2x duration plus 2 seconds buffer
                    if elapsed > max_allowed:
                        self.logger.warning(f'Playback timeout: elapsed {elapsed:.1f}s > max {max_allowed:.1f}s, stopping playback')
                        self.node.is_playing = False
                        self.play_button.config(state=tk.NORMAL)
                        self.pause_play_button.config(state=tk.DISABLED, text='Pause')
                        self.stop_play_button.config(state=tk.DISABLED)
                        self.load_button.config(state=tk.NORMAL)
                        self.play_progress['value'] = 0
        else:
            self.play_status_label.config(text='Stopped', foreground='gray')
            self.play_progress['value'] = 0

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
