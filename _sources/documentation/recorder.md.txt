# Recorder

Record and playback motion of a target robot controlled by Teachbot.

## Launch

Default launch:

```bash
ros2 launch teachbot_recorder recorder.launch.py
```

Simulation mode:

```bash
ros2 launch teachbot_recorder recorder.launch.py sim:=true
```

## Using a different robot config

The default configuration is for Universal Robots in [config/ur.yaml](config/ur.yaml).

To use a different robot, pass an absolute path to the config file (the launch system does not expand `~`):

```bash
ros2 launch teachbot_recorder recorder.launch.py config:=/home/<user>/teachbot_ws/src/teachbot_ros/teachbot_recorder/config/ufLite6.yaml
```

## Notes

- The GUI includes recording, playback, and point-stepping controls.
- Pistol state is recorded and published when enabled.
