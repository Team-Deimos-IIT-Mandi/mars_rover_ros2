# Quick Testing Commands for Differential Drive Rover

## Test Commands

### 1. Forward Motion
```bash
ros2 topic pub --once /diff_drive_controller/cmd_vel_unstamped geometry_msgs/msg/Twist \
  "{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

### 2. Backward Motion
```bash
ros2 topic pub --once /diff_drive_controller/cmd_vel_unstamped geometry_msgs/msg/Twist \
  "{linear: {x: -0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

### 3. Turn Left (rotate in place)
```bash
ros2 topic pub --once /diff_drive_controller/cmd_vel_unstamped geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}"
```

### 4. Turn Right (rotate in place)
```bash
ros2 topic pub --once /diff_drive_controller/cmd_vel_unstamped geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: -0.5}}"
```

### 5. Arc Left (forward + turn)
```bash
ros2 topic pub --once /diff_drive_controller/cmd_vel_unstamped geometry_msgs/msg/Twist \
  "{linear: {x: 0.3, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.3}}"
```

### 6. Stop
```bash
ros2 topic pub --once /diff_drive_controller/cmd_vel_unstamped geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

## Debugging Steps

### Step 1: Check if hardware interface is running
```bash
ros2 node list | grep controller_manager
```
Expected: Should see `controller_manager` node

### Step 2: Check if controllers are loaded
```bash
ros2 control list_controllers
```
Expected output:
```
diff_drive_controller[diff_drive_controller/DiffDriveController] active
joint_state_broadcaster[joint_state_broadcaster/JointStateBroadcaster] active
```

### Step 3: Check topics
```bash
ros2 topic list | grep cmd_vel
```
Expected: `/diff_drive_controller/cmd_vel_unstamped`

### Step 4: Echo joint commands
```bash
ros2 topic echo /dynamic_joint_states
```
This shows what velocities the controller is commanding to each wheel

### Step 5: Monitor hardware interface logs
Look for these messages in your launch terminal:
```
[RoverSerial] ✓ Hardware interface ACTIVATED - Ready to receive commands
[RoverSerial] 📤 TX #1 → FL: 0.500 | RL: 0.500 | FR: -0.500 | RR: -0.500 rad/s [18 bytes]
```

### Step 6: Check ESP32 serial monitor
Connect to ESP32 host via serial monitor (115200 baud):
```
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
📦 RX Serial Packet #1
   FL: 0.500 | RL: 0.500 | FR: -0.500 | RR: -0.500 rad/s
   → Local motor [FL]: 50.00 RPM
   ✓ CAN TX [0x123]: speed=50, dir=1 (0.500 rad/s)
   ...
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
```

## Common Issues

### Issue: "⚠ Sending ZERO velocities" warning
**Cause**: Controller is not sending commands OR controller is not active
**Fix**:
```bash
# Check controller state
ros2 control list_controllers

# If inactive, activate it
ros2 control set_controller_state diff_drive_controller active
```

### Issue: No output from hardware interface
**Cause**: Serial port not opened or wrong port
**Fix**: Check launch file for correct serial port (should match ESP32 USB port)

### Issue: Commands sent but motors don't move
**Possible causes**:
1. ESP32 not receiving serial data (check USB connection)
2. CAN bus not working (check wiring and termination resistors)
3. Motor controllers not programmed with correct CAN IDs
4. Encoder issues (check motor controller LED status)

## Joystick Testing

If using joystick, check the topic it publishes to:
```bash
ros2 topic list | grep joy
ros2 topic echo /joy
```

Then check if there's a teleop node remapping joy to cmd_vel:
```bash
ros2 node list
ros2 node info /teleop_twist_joy  # or similar
```

Make sure joystick commands are being remapped to:
`/diff_drive_controller/cmd_vel_unstamped`
