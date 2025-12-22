# TOS Teachbot ROS2 Node

ROS2 Humble node for publishing TOS Teachbot joint states over ROS topics.

## Quick Start

```bash
# Terminal 1: Launch the teachbot node
source ~/teachbot_ws/install/setup.bash
ros2 launch teachbot_ros teachbot.launch.py

# Terminal 2: Monitor joint states (radians)
ros2 topic echo /teachbot/joint_states

# Terminal 3: Monitor full state (degrees, FK, buttons, pot)
ros2 topic echo /teachbot/state

# Check publishing rate (should be ~250 Hz)
ros2 topic hz /teachbot/joint_states
```

## Quick Start with RViz Visualization

```bash
source ~/teachbot_ws/install/setup.bash
ros2 launch teachbot_ros teachbot_rviz.launch.py
```

This launches the teachbot publisher with the official UR5e robot model in RViz, plus a GUI monitor for control inputs.

## Simulation Mode

For testing without hardware:

```bash
source ~/teachbot_ws/install/setup.bash
ros2 launch teachbot_ros sim_teachbot_rviz.launch.py
```

This uses `joint_state_publisher_gui` to simulate joint movements.

## Overview

This package provides a ROS2 node that:
- Connects to 6 joint encoders via TCP (AksIM-2 17-bit encoders on ports 5004-5009)
- Connects to trigger potentiometer and buttons via TCP (RS-485 interface on port 5011)
- Computes forward kinematics for TCP position
- Publishes data at 250 Hz
- Supports multiple target robot configurations

**Note:** The teachbot should be configured in TCP Server mode. This ROS node acts as a TCP client connecting to the teachbot's IP address.

## Published Topics

| Topic | Message Type | Description |
|-------|-------------|-------------|
| `/teachbot/joint_states` | `sensor_msgs/JointState` | Teachbot joint states (radians) |
| `/teachbot/{target_robot}/joint_states` | `sensor_msgs/JointState` | Target robot joint states (radians, with offsets) |
| `/teachbot/state` | `teachbot_interfaces/TeachbotState` | Full state with FK, controls |

The target robot topic name is determined by the `target_robot_name` parameter from the target configuration file.

## Prerequisites

- Ubuntu 22.04
- ROS2 Humble
- Python 3.10+
- NumPy

## Installation

### Quick Install

```bash
cd ~/teachbot_ws/src/teachbot_ros
chmod +x install_teachbot_ros.sh
./install_teachbot_ros.sh
```

### Manual Install

```bash
# Install ROS2 Humble (if not already installed)
sudo apt update
sudo apt install ros-humble-desktop python3-colcon-common-extensions

# Create workspace
mkdir -p ~/teachbot_ws/src
cd ~/teachbot_ws/src

# Clone or copy the packages
# (teachbot_interfaces and teachbot_ros)

# Build
cd ~/teachbot_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select teachbot_interfaces
source install/setup.bash
colcon build --packages-select teachbot_ros
source install/setup.bash
```

## Usage

### Run the Node (Headless)

```bash
# Source workspace (add to ~/.bashrc for convenience)
source ~/teachbot_ws/install/setup.bash

# Option 1: Use launch file (recommended)
ros2 launch teachbot_ros teachbot.launch.py

# Option 2: Run directly
ros2 run teachbot_ros teachbot_publisher
```

### Run with RViz Visualization

```bash
source ~/teachbot_ws/install/setup.bash
ros2 launch teachbot_ros teachbot_rviz.launch.py
```

This launches:
- `teachbot_publisher`: Connects to encoders via TCP and publishes joint states
- `robot_state_publisher`: Publishes TF transforms from the official UR5e URDF
- `rviz2`: 3D visualization with the UR5e robot model
- `teachbot_monitor_gui`: GUI for monitoring potentiometer and button states (optional)
- `static_transform_publisher`: Publishes world → teachbot/base_link transform

**Note:** Requires `ros-humble-ur-description` package (installed via `sudo apt install ros-humble-ur-description`).

### Monitor Topics

```bash
# List available topics
ros2 topic list

# View teachbot joint states (standard ROS format, radians)
ros2 topic echo /teachbot/joint_states

# View target robot joint states (e.g., for UR)
ros2 topic echo /teachbot/ur/joint_states

# View full state (degrees, FK, buttons, pot)
ros2 topic echo /teachbot/state

# Check publishing rate
ros2 topic hz /teachbot/joint_states

# View topic info
ros2 topic info /teachbot/joint_states
ros2 topic info /teachbot/state
```

### With Custom Configuration

```bash
# Copy and edit the config files
cp ~/teachbot_ws/install/teachbot_ros/share/teachbot_ros/config/teachbot_params.yaml ~/my_config.yaml
cp ~/teachbot_ws/install/teachbot_ros/share/teachbot_ros/config/target_robots/ur.yaml ~/my_target.yaml
# Edit files with your settings

# Launch with custom config
ros2 launch teachbot_ros teachbot.launch.py config_file:=~/my_config.yaml target_config_file:=~/my_target.yaml

# Or with RViz
ros2 launch teachbot_ros teachbot_rviz.launch.py config_file:=~/my_config.yaml target_config_file:=~/my_target.yaml
```

### Disable Monitor GUI

```bash
ros2 launch teachbot_ros teachbot_rviz.launch.py use_monitor_gui:=false
```

## Configuration Parameters

### Main Configuration ([teachbot_params.yaml](teachbot_ros/teachbot_ros/config/teachbot_params.yaml))

| Parameter | Default | Description |
|-----------|---------|-------------|
| `remote_ip` | `192.168.0.152` | IP address of teachbot (TCP server) |
| `start_port` | `5004` | First encoder TCP port |
| `ee_port` | `5011` | RS-485 trigger/button TCP port |
| `robot_model` | `Cobot_200_200_mm` | FK model name |
| `publish_rate` | `250.0` | Publishing rate (Hz) |
| `joint_names` | UR joint names | Joint names for `/teachbot/joint_states` |
| `position_offsets` | `[94465, ...]` | Encoder zero offsets |
| `degree_offsets` | `[0, 0, 0, 0, 0, 0]` | Angle offsets (deg) |
| `joint_scale_factors` | `[-1, -1, 1, -1, -1, -1]` | Direction scaling |
| `pot_idle` | `846` | Pot raw at 0% |
| `pot_full` | `78` | Pot raw at 100% |

### Target Robot Configuration (e.g., [ur.yaml](teachbot_ros/teachbot_ros/config/target_robots/ur.yaml))

| Parameter | Default | Description |
|-----------|---------|-------------|
| `target_robot_name` | `ur` | Name for target robot topic |
| `target_joint_names` | UR joint names | Joint names for target robot |
| `target_degree_offsets` | `[-90, 0, 0, 0, 0, 0]` | Target-specific angle offsets (deg) |

Available target configurations:
- [ur.yaml](teachbot_ros/teachbot_ros/config/target_robots/ur.yaml) - Universal Robots (UR5e, etc.)
- [ufLite6.yaml](teachbot_ros/teachbot_ros/config/target_robots/ufLite6.yaml) - uFactory Lite6

## Custom Message: TeachbotState

```
std_msgs/Header header
float64[6] joint_angles_deg     # Joint angles in degrees
float64 tcp_x, tcp_y, tcp_z     # TCP position (mm)
float64 tcp_rx, tcp_ry, tcp_rz  # TCP orientation (deg)
int32 pot_raw                   # Raw potentiometer (0-1023)
int32 pot_percent               # Mapped percentage (0-100)
bool btn1, btn2                 # Button states
bool[6] encoder_errors          # Encoder error flags
bool[6] encoder_warnings        # Encoder warning flags
float64[6] encoder_frequencies  # Encoder update rates
string robot_model              # Active robot model
```

## Network Setup

The teachbot operates as a TCP server. This ROS node connects to it as a client:

1. Configure the teachbot in TCP Server mode
2. Set the teachbot's IP address (default: `192.168.0.152`)
3. Set `remote_ip` in your config file to match the teachbot's IP
4. Ensure your PC can reach the teachbot (e.g., same subnet `192.168.0.x`)
5. Encoders stream on TCP ports 5004-5009
6. RS-485 interface streams on TCP port 5011

## Troubleshooting

### No data received
- Check network configuration (`ip addr`)
- Verify teachbot is powered on and in TCP Server mode
- Verify teachbot IP matches `remote_ip` in config
- Test connectivity: `ping 192.168.0.152`
- Check if TCP ports are open: `nc -zv 192.168.0.152 5004`

### Wrong joint directions
- Adjust `joint_scale_factors` in config (use -1.0 to invert)

### Joint zero offset wrong
- Calibrate `position_offsets` with teachbot at zero position

### RViz shows robot in wrong pose
- Check encoder calibration offsets in [teachbot_params.yaml](teachbot_ros/teachbot_ros/config/teachbot_params.yaml)
- Adjust `joint_scale_factors` for correct joint directions
- Verify `target_degree_offsets` in target robot config

### Connection keeps retrying
- The TCP client automatically reconnects if connection is lost
- Check teachbot power and network connection

### Monitor GUI not appearing
- Check `use_monitor_gui` launch argument is set to `true` (default)
- Verify tkinter is installed: `python3 -c "import tkinter"`

## File Structure

```
teachbot_ros/
├── teachbot_ros/
│   ├── __init__.py
│   ├── teachbot_publisher.py   # Main ROS node
│   ├── listeners.py            # TCP listener threads
│   ├── kinematics.py           # Forward kinematics
│   └── utils/
│       ├── teachbot_viewer.py  # Standalone TCP viewer GUI
│       └── teachbot_monitor_gui.py  # ROS monitor GUI
├── config/
│   ├── teachbot_params.yaml    # Main configuration
│   ├── sim_initial_positions.yaml  # Simulation initial pose
│   └── target_robots/          # Target robot configs
│       ├── ur.yaml
│       └── ufLite6.yaml
├── launch/
│   ├── teachbot.launch.py      # Headless launch (no RViz)
│   ├── teachbot_rviz.launch.py # Launch with UR5e visualization
│   └── sim_teachbot_rviz.launch.py  # Simulation mode
├── rviz/
│   └── teachbot.rviz           # RViz configuration
├── package.xml
├── setup.py
├── install_teachbot_ros.sh     # Installation script
└── README.md
```

## Launch Files

| File | Description |
|------|-------------|
| [teachbot.launch.py](teachbot_ros/teachbot_ros/launch/teachbot.launch.py) | Headless mode - publishes topics only |
| [teachbot_rviz.launch.py](teachbot_ros/teachbot_ros/launch/teachbot_rviz.launch.py) | Full visualization with RViz and monitor GUI |
| [sim_teachbot_rviz.launch.py](teachbot_ros/teachbot_ros/launch/sim_teachbot_rviz.launch.py) | Simulation mode with `joint_state_publisher_gui` |

## Utility Programs

### Teachbot Monitor GUI

Monitor potentiometer and button states with a graphical interface:

```bash
ros2 run teachbot_ros teachbot_monitor_gui
```

### Standalone Teachbot Viewer

Direct TCP connection viewer (no ROS required):

```bash
python3 ~/teachbot_ws/src/teachbot_ros/teachbot_ros/teachbot_ros/utils/teachbot_viewer.py
```

Edit `TEACHBOT_IP` in the script to match your teachbot's IP address.

## Dependencies

- ROS2 Humble
- `ros-humble-ur-description` - Official UR robot URDF models
- `joint_state_publisher_gui` - For simulation mode

```bash
sudo apt install ros-humble-ur-description ros-humble-joint-state-publisher-gui
```

## License

MIT License