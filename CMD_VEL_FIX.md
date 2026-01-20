# SOLUTION: cmd_vel Topic Issue

## Problem Identified

Your differential drive controller is active and working, but it's listening on:
```
/diff_drive_controller/cmd_vel
```

But you (and the hardware interface log message) are trying to publish to:
```
/diff_drive_controller/cmd_vel_unstamped
```

## The Fix - Use the Correct Topic

### Option 1: Publish to the correct topic (EASIEST)

```bash
# Use THIS command (without _unstamped):
ros2 topic pub --once /diff_drive_controller/cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

### Option 2: Remap the topic in launch file

Add this to your `bringup.launch.py` in the diff_drive_spawner:

```python
diff_drive_spawner = Node(
    package="controller_manager",
    executable="spawner",
    arguments=["diff_drive_controller"],
    remappings=[
        ('/diff_drive_controller/cmd_vel', '/cmd_vel'),  # Remap to /cmd_vel
    ]
)
```

## Quick Test

### 1. Check what topics exist:
```bash
ros2 topic list | grep cmd_vel
```

You should see:
```
/diff_drive_controller/cmd_vel
```

### 2. Test with correct topic:
```bash
# Forward
ros2 topic pub --once /diff_drive_controller/cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

# Turn left
ros2 topic pub --once /diff_drive_controller/cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}"

# Stop
ros2 topic pub --once /diff_drive_controller/cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

### 3. Watch for output in your launch terminal:
```
[RoverSerial] 📤 TX #1 → FL: 0.500 | RL: 0.500 | FR: 0.500 | RR: 0.500 rad/s [18 bytes]
```

## For Joystick

If you're using a joystick, you need to remap the joystick's cmd_vel output to match the controller's input.

Check your joystick/teleop node and add a remapping:
```python
teleop_node = Node(
    package='teleop_twist_joy',
    executable='teleop_node',
    remappings=[
        ('/cmd_vel', '/diff_drive_controller/cmd_vel'),  # Remap to controller
    ]
)
```

## Why This Happened

The `use_stamped_vel` parameter controls the MESSAGE TYPE, not the topic name:
- `use_stamped_vel: false` → listens on `/diff_drive_controller/cmd_vel` (Twist message)
- `use_stamped_vel: true` → listens on `/diff_drive_controller/cmd_vel` (TwistStamped message)

The topic name stays the same! The `_unstamped` suffix in my earlier log message was misleading.

## Summary

**Just use `/diff_drive_controller/cmd_vel` as the topic name!**
