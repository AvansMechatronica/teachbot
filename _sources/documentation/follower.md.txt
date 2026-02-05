# TeachBot Follower

This package contains ROS2 nodes that allow a target robot to follow joint positions published by a teachbot device.

## Overview

The teachbot device publishes joint states on `/teachbot/<target_robot>/joint_states`, and these nodes command the UR robot to move to match those positions.

## Quick Start

### Launch Control (Recommended)
```bash
ros2 launch teachbot_follower follower_action.launch.py
```

This starts:
- TeachBot follower action client


## Available Nodes

### 1. `follower_action` ✅ **RECOMMENDED - Action Client**

This node uses action client to send trajectory goals. Robust with feedback and result handling.

**Usage:**
```bash
ros2 launch teachbot_follower follower_action.launch.py
```

**Parameters:**
- `teachbot_topic` (default: `/teachbot/joint_states`) - Topic to subscribe to for teachbot commands
- `enable_topic` (default: `/teachbot/enable`) - Topic for enable/disable control
- `controller_name` (default: `scaled_joint_trajectory_controller`) - Name of the joint trajectory controller
- `update_rate` (default: `0.5`) - Seconds between updates
- `position_tolerance` (default: `0.01`) - Minimum position change (radians) to trigger movement
- `trajectory_duration` (default: `2.0`) - Duration for each trajectory segment

### 2. `teachbot_follower_moveit_commander` 📝 **Alternative - Direct Commands  - Experimental**

This node directly publishes trajectory commands to the robot controller.

**Usage:**
```bash
ros2 run teachbot_follower follower_moveit_commander
```

**Parameters:**
- `teachbot_topic` (default: `/teachbot/joint_states`)
- `enable_topic` (default: `/teachbot/enable`)
- `controller_name` (default: `scaled_joint_trajectory_controller`)
- `update_rate` (default: `0.5`)
- `position_tolerance` (default: `0.01`)
- `trajectory_duration` (default: `0.5`)



## Complete Workflow

1. **Build the package:**
   ```bash
   cd ~/teachbot_ws
   colcon build --packages-select teachbot_follower --symlink-install
   source install/setup.bash
   ```

2. **Launch your UR robot with MoveIt:**
   ```bash
   # For simulation:
   ros2 launch my_ur_bringup simulation.launch.py
   
   # Or for real robot:
   ros2 launch my_ur_bringup real_robot.launch.py robot_ip:=<ROBOT_IP>
   ```

3. **Start your teachbot device** (ensure it publishes to `/teachbot/joint_states`)

4. **Run the follower node:**
   ```bash
   # For simulation:
   ros2 launch teachbot_follower follower_action.launch.py sim:=true

   # Or for real robot:
   ros2 launch teachbot_follower follower_action.launch.py

   ```




## Joint Names

Make sure the joint names in the teachbot message match the UR robot joint names:
- `shoulder_pan_joint`
- `shoulder_lift_joint`
- `elbow_joint`
- `wrist_1_joint`
- `wrist_2_joint`
- `wrist_3_joint`

## Troubleshooting

### Robot doesn't move
- Check that `/teachbot/joint_states` is being published: `ros2 topic echo /teachbot/joint_states`
- Verify MoveIt is running: `ros2 node list | grep move_group`
- Check for errors: `ros2 topic echo /rosout`

### Movement is too slow/fast
- Adjust `update_rate` parameter (lower = faster updates)
- For `ur_teachbot_simple`, adjust `velocity_scaling` and `acceleration_scaling`
- For `teachbot_follower_action` or `teachbot_follower_moveit_commander`, adjust `trajectory_duration`

### Joint names don't match
- Check your teachbot joint names: `ros2 topic echo /teachbot/<target_robot>/joint_states --once`
- Verify UR joint names: `ros2 topic echo /joint_states --once`

## Safety Notes

⚠️ **Warning**: This node will command the robot to follow the teachbot positions. Ensure:
- The workspace is clear of obstacles
- Emergency stop is accessible
- The robot is in a safe starting configuration
- You start with slow update rates and small movements to test



