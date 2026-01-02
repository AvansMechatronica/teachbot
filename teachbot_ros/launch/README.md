# Teachbot Launch Files

This directory contains ROS2 launch files for the Teachbot system. These launch files provide different configurations for running the Teachbot with or without visualization.

## Launch Files Overview

### 1. `teachbot.launch.py`
Basic launch file for running the Teachbot publisher node without visualization.

**Purpose:** Connect to a physical Teachbot device and publish joint states.

**Nodes Launched:**
- `teachbot_publisher` - Connects to Teachbot and publishes joint states
- `joint_state_remapper` - Remaps Teachbot joint states to target robot format

**Usage:**
```bash
ros2 launch teachbot_ros teachbot.launch.py
```

**Arguments:**
- `config_file` (default: `config/teachbot_params.yaml`)  
  Path to the main Teachbot configuration YAML file
  
- `target_config_file` (default: `config/target_robots/ur.yaml`)  
  Path to the target robot configuration YAML file
  
- `remote_ip` (default: `''`)  
  Override Teachbot IP address (empty = use config file value)
  
- `publish_rate` (default: `''`)  
  Override publish rate in Hz (empty = use config file value)

**Examples:**
```bash
# Use default configuration
ros2 launch teachbot_ros teachbot.launch.py

# Override IP address
ros2 launch teachbot_ros teachbot.launch.py remote_ip:=192.168.100.152

# Use custom config files
ros2 launch teachbot_ros teachbot.launch.py config_file:=/path/to/config.yaml target_config_file:=/path/to/robot.yaml

# Override publish rate
ros2 launch teachbot_ros teachbot.launch.py publish_rate:=100
```

---

### 2. `teachbot_rviz.launch.py`
Launch file for running a physical Teachbot with RViz visualization using the official UR5e model.

**Purpose:** Connect to a physical Teachbot device and visualize it in RViz with control GUIs.

**Nodes Launched:**
- `teachbot_publisher` - Connects to Teachbot and publishes joint states
- `robot_state_publisher` - Publishes TF transforms from URDF
- `rviz2` - Visualization
- `teachbot_monitor_gui` - Monitoring GUI (optional)
- `teachbot_enable_gui` - Manual enable control (conditional)
- `teachbot_enable_from_button` - Teachbot button enable control (conditional)

**Usage:**
```bash
ros2 launch teachbot_ros teachbot_rviz.launch.py
```

**Arguments:**
- `config_file` (default: `config/teachbot_params.yaml`)  
  Path to the Teachbot configuration YAML file
  
- `target_config_file` (default: `config/target_robots/ur.yaml`)  
  Path to the target robot configuration YAML file
  
- `use_monitor_gui` (default: `true`)  
  Launch the Teachbot control monitor GUI
  
- `enable_mode` (default: `none`)  
  Enable mode: `gui` for manual button, `button` for Teachbot button control, or `none`

**Examples:**
```bash
# Use default configuration
ros2 launch teachbot_ros teachbot_rviz.launch.py

# Disable monitor GUI
ros2 launch teachbot_ros teachbot_rviz.launch.py use_monitor_gui:=false

# Use GUI enable mode
ros2 launch teachbot_ros teachbot_rviz.launch.py enable_mode:=gui

# Use Teachbot button enable mode
ros2 launch teachbot_ros teachbot_rviz.launch.py enable_mode:=button
```

---

### 3. `sim_teachbot_rviz.launch.py`
Launch file for simulating Teachbot movements in RViz without a physical device.

**Purpose:** Simulate and test Teachbot behavior using a GUI slider interface without hardware.

**Nodes Launched:**
- `joint_state_publisher_gui` - GUI sliders to simulate robot movements
- `joint_state_remapper` - Remaps joint states to target robot format
- `robot_state_publisher` - Publishes TF transforms from URDF
- `rviz2` - Visualization
- `teachbot_state_publisher_gui` - Teachbot state control GUI
- `teachbot_monitor_gui` - Monitoring GUI (optional)
- `teachbot_enable_gui` - Manual enable control (conditional)
- `teachbot_enable_from_button` - Simulated button enable control (conditional)

**Usage:**
```bash
ros2 launch teachbot_ros sim_teachbot_rviz.launch.py
```

**Arguments:**
- `config_file` (default: `config/teachbot_params.yaml`)  
  Path to the configuration YAML file
  
- `target_config_file` (default: `config/target_robots/ur.yaml`)  
  Path to the target robot configuration YAML file
  
- `use_monitor_gui` (default: `true`)  
  Launch the Teachbot control monitor GUI
  
- `enable_mode` (default: `gui`, choices: `gui`, `button`)  
  Enable mode: `gui` for manual GUI button, `button` for Teachbot button control
  
- `rviz_config` (default: `rviz/teachbot.rviz`)  
  Path to the RViz configuration file

**Examples:**
```bash
# Use default configuration
ros2 launch teachbot_ros sim_teachbot_rviz.launch.py

# Use custom RViz config
ros2 launch teachbot_ros sim_teachbot_rviz.launch.py rviz_config:=/path/to/custom.rviz

# Use ufLite6 target robot
ros2 launch teachbot_ros sim_teachbot_rviz.launch.py target_config_file:=$(ros2 pkg prefix teachbot_ros)/share/teachbot_ros/config/target_robots/ufLite6.yaml

# Disable monitor GUI and use button enable mode
ros2 launch teachbot_ros sim_teachbot_rviz.launch.py use_monitor_gui:=false enable_mode:=button
```

---

## Quick Start Guide

### For Physical Hardware
1. **Basic connection (no visualization):**
   ```bash
   ros2 launch teachbot_ros teachbot.launch.py remote_ip:=<YOUR_TEACHBOT_IP>
   ```

2. **With visualization:**
   ```bash
   ros2 launch teachbot_ros teachbot_rviz.launch.py
   ```

### For Simulation/Testing
```bash
ros2 launch teachbot_ros sim_teachbot_rviz.launch.py
```

---

## Configuration Files

All launch files reference configuration files in the `config/` directory:

- **`teachbot_params.yaml`** - Main Teachbot parameters (IP, port, joint mappings, etc.)
- **`target_robots/ur.yaml`** - UR robot configuration
- **`target_robots/ufLite6.yaml`** - UFactory Lite6 robot configuration
- **`sim_initial_positions.yaml`** - Initial joint positions for simulation

To use a different target robot, specify the `target_config_file` argument.

---

## Troubleshooting

### Cannot connect to Teachbot
- Verify IP address with `remote_ip` argument
- Check network connectivity: `ping <TEACHBOT_IP>`
- Ensure Teachbot device is powered on

### RViz not displaying robot
- Check that `robot_state_publisher` is running: `ros2 node list`
- Verify joint states are published: `ros2 topic echo /teachbot/joint_states`
- Check TF tree: `ros2 run tf2_tools view_frames`

### GUI windows not appearing
- Check if `use_monitor_gui` is set to `true`
- Verify `enable_mode` is set correctly (`gui` or `button`)
- Ensure all GUI packages are installed

---

## Related Documentation

- Main package README: `../README.md`
- Installation guide: `../../INSTALL.md`
- Configuration guide: See `config/` directory
