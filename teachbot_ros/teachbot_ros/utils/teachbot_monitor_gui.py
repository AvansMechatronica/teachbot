#!/usr/bin/env python3
# teachbot_monitor_gui.py
"""
GUI Monitor for TOS Teachbot control inputs.

Displays real-time graphical status of:
  - Potentiometer percentage (0-100) with slider and numeric display
  - Button 1 state with visual indicator
  - Button 2 state with visual indicator

Usage:
    ros2 run teachbot_ros teachbot_monitor_gui
    python3 teachbot_monitor_gui.py

Author: Gerard Harkema
Date: 2025-12
Initial version: 2025-12-22
License: CC BY-NC-SA 4.0
"""

import rclpy
from rclpy.node import Node
from teachbot_interfaces.msg import TeachbotState
import tkinter as tk
from tkinter import ttk
import threading


class TeachbotMonitorGUI(Node):
    """ROS2 node with GUI to monitor teachbot control inputs."""

    def __init__(self, root):
        super().__init__('teachbot_monitor_gui')
        
        self.root = root
        self.root.title("Teachbot Control Monitor")
        self.root.geometry("500x500")
        self.root.resizable(False, False)
        
        # Configure style
        style = ttk.Style()
        style.theme_use('clam')
        
        # State variables
        self.pot_percent = tk.IntVar(value=0)
        self.pot_raw = tk.IntVar(value=0)
        self.btn1_state = tk.BooleanVar(value=False)
        self.btn2_state = tk.BooleanVar(value=False)
        self.connection_status = tk.StringVar(value="Waiting for data...")
        
        self._setup_ui()
        
        # Subscribe to teachbot state
        self.subscription = self.create_subscription(
            TeachbotState,
            '/teachbot/state',
            self.state_callback,
            10
        )
        
        self.get_logger().info('Teachbot Monitor GUI Started')
        self.get_logger().info('Monitoring: /teachbot/state')
        
    def _setup_ui(self):
        """Setup the GUI layout."""
        # Main container
        main_frame = ttk.Frame(self.root, padding="20")
        main_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        
        # Title
        title_label = ttk.Label(
            main_frame, 
            text="Teachbot Control Monitor", 
            font=('Arial', 18, 'bold')
        )
        title_label.grid(row=0, column=0, columnspan=2, pady=(0, 10))
        
        # Connection status
        self.status_label = ttk.Label(
            main_frame, 
            textvariable=self.connection_status,
            font=('Arial', 9),
            foreground='orange'
        )
        self.status_label.grid(row=1, column=0, columnspan=2, pady=(0, 20))
        
        # --- Potentiometer Section ---
        pot_frame = ttk.LabelFrame(main_frame, text="Potentiometer", padding="15")
        pot_frame.grid(row=2, column=0, columnspan=2, sticky=(tk.W, tk.E), pady=(0, 15))
        
        # Percentage display (large)
        self.pot_percent_label = ttk.Label(
            pot_frame,
            text="0%",
            font=('Arial', 48, 'bold'),
            foreground='#2196F3'
        )
        self.pot_percent_label.grid(row=0, column=0, columnspan=2, pady=(0, 10))
        
        # Progress bar
        self.pot_progress = ttk.Progressbar(
            pot_frame,
            variable=self.pot_percent,
            maximum=100,
            length=400,
            mode='determinate'
        )
        self.pot_progress.grid(row=1, column=0, columnspan=2, pady=(0, 10))
        
        # Raw value display
        ttk.Label(pot_frame, text="Raw Value:", font=('Arial', 10)).grid(
            row=2, column=0, sticky=tk.E, padx=(0, 10)
        )
        self.pot_raw_label = ttk.Label(
            pot_frame, 
            textvariable=self.pot_raw,
            font=('Arial', 10, 'bold')
        )
        self.pot_raw_label.grid(row=2, column=1, sticky=tk.W)
        
        # --- Buttons Section ---
        buttons_frame = ttk.Frame(main_frame)
        buttons_frame.grid(row=3, column=0, columnspan=2, pady=(0, 10))
        
        # Button 1
        btn1_frame = ttk.LabelFrame(buttons_frame, text="Button 1", padding="15")
        btn1_frame.grid(row=0, column=0, padx=(0, 15))
        
        self.btn1_canvas = tk.Canvas(
            btn1_frame, 
            width=100, 
            height=100, 
            bg='white', 
            highlightthickness=2,
            highlightbackground='#ccc'
        )
        self.btn1_canvas.grid(row=0, column=0)
        self.btn1_indicator = self.btn1_canvas.create_oval(
            10, 10, 90, 90, 
            fill='#e0e0e0', 
            outline='#999'
        )
        
        self.btn1_text_label = ttk.Label(
            btn1_frame, 
            text="Released",
            font=('Arial', 10, 'bold')
        )
        self.btn1_text_label.grid(row=1, column=0, pady=(5, 0))
        
        # Button 2
        btn2_frame = ttk.LabelFrame(buttons_frame, text="Button 2", padding="15")
        btn2_frame.grid(row=0, column=1)
        
        self.btn2_canvas = tk.Canvas(
            btn2_frame, 
            width=100, 
            height=100, 
            bg='white', 
            highlightthickness=2,
            highlightbackground='#ccc'
        )
        self.btn2_canvas.grid(row=0, column=0)
        self.btn2_indicator = self.btn2_canvas.create_oval(
            10, 10, 90, 90, 
            fill='#e0e0e0', 
            outline='#999'
        )
        
        self.btn2_text_label = ttk.Label(
            btn2_frame, 
            text="Released",
            font=('Arial', 10, 'bold')
        )
        self.btn2_text_label.grid(row=1, column=0, pady=(5, 0))
        
    def state_callback(self, msg: TeachbotState):
        """Callback for teachbot state messages."""
        # Update connection status
        self.connection_status.set("Connected - Receiving Data")
        self.status_label.config(foreground='green')
        
        # Update potentiometer
        self.pot_percent.set(msg.pot_percent)
        self.pot_raw.set(msg.pot_raw)
        self.pot_percent_label.config(text=f"{msg.pot_percent}%")
        
        # Update color based on percentage
        if msg.pot_percent < 33:
            color = '#4CAF50'  # Green
        elif msg.pot_percent < 66:
            color = '#FF9800'  # Orange
        else:
            color = '#F44336'  # Red
        self.pot_percent_label.config(foreground=color)
        
        # Update Button 1
        if msg.btn1:
            self.btn1_canvas.itemconfig(self.btn1_indicator, fill='#4CAF50', outline='#2E7D32')
            self.btn1_text_label.config(text="PRESSED", foreground='#4CAF50')
        else:
            self.btn1_canvas.itemconfig(self.btn1_indicator, fill='#e0e0e0', outline='#999')
            self.btn1_text_label.config(text="Released", foreground='#666')
        
        # Update Button 2
        if msg.btn2:
            self.btn2_canvas.itemconfig(self.btn2_indicator, fill='#F44336', outline='#C62828')
            self.btn2_text_label.config(text="PRESSED", foreground='#F44336')
        else:
            self.btn2_canvas.itemconfig(self.btn2_indicator, fill='#e0e0e0', outline='#999')
            self.btn2_text_label.config(text="Released", foreground='#666')


def spin_ros(node):
    """Spin ROS node in a separate thread."""
    rclpy.spin(node)


def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)
    
    # Create tkinter root
    root = tk.Tk()
    
    # Create ROS node with GUI
    node = TeachbotMonitorGUI(root)
    
    # Start ROS spinning in a separate thread
    ros_thread = threading.Thread(target=spin_ros, args=(node,), daemon=True)
    ros_thread.start()
    
    # Handle window close
    def on_closing():
        node.get_logger().info("Shutting down GUI...")
        node.destroy_node()
        rclpy.shutdown()
        root.destroy()
    
    root.protocol("WM_DELETE_WINDOW", on_closing)
    
    # Start GUI main loop
    try:
        root.mainloop()
    except KeyboardInterrupt:
        on_closing()


if __name__ == '__main__':
    main()
