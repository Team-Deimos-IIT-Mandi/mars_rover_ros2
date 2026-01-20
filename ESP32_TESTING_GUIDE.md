# ESP32 CAN Communication Testing Guide
Team Deimos IIT Mandi

## 📋 Quick Reference

### CAN IDs Configuration
- **Front Left (FL)**: `0x124`
- **Front Right (FR)**: `0x121`
- **Back Left (BL)**: `0x123`
- **Back Right (BR)**: `0x122`

### Files Created
1. `esp32_host_serial_can_bridge.ino` - Host ESP32 (connects to ROS2)
2. `esp32_motor_controller_template.ino` - Motor controller template

## 🔧 Setup Instructions

### Step 1: Program ESP32 Host
1. Open `esp32_host_serial_can_bridge.ino` in Arduino IDE
2. Select board: **ESP32 Dev Module**
3. Upload to your host ESP32
4. Open Serial Monitor (115200 baud)
5. You should see:
   ```
   ╔════════════════════════════════════════╗
   ║  ESP32 SERIAL-CAN BRIDGE v2.0         ║
   ║  Team Deimos IIT Mandi                 ║
   ╚════════════════════════════════════════╝
   
   ✓ CAN Driver Installed
   ✓ CAN Started @500kbps
   ✓ Ready to receive serial commands from ROS2
   ```

### Step 2: Program Motor Controllers
For **each** of the 4 motor controllers:

1. Open `esp32_motor_controller_template.ino`
2. **MODIFY THESE LINES** (around line 28-29):
   ```cpp
   #define CAN_ID_MOTOR 0x121   // Change to: 0x124, 0x121, 0x123, or 0x122
   #define WHEEL_NAME "FR"      // Change to: "FL", "FR", "BL", or "BR"
   ```

3. Upload to the motor controller ESP32
4. Open Serial Monitor - you should see:
   ```
   ╔════════════════════════════════════════╗
   ║  MOTOR CONTROLLER [FR] - v2.0          ║
   ║  Team Deimos IIT Mandi                 ║
   ╚════════════════════════════════════════╝
   CAN ID: 0x121
   
   ✓ CAN Driver Installed
   ✓ CAN Started @500kbps
   ✓ Motor Controller Ready
   [Waiting for CAN commands...]
   ```

### Step 3: Wire Connections

**Host ESP32:**
- USB → ROS2 Computer
- GPIO 5 (CAN TX) → CAN Transceiver TX
- GPIO 4 (CAN RX) → CAN Transceiver RX

**Each Motor Controller ESP32:**
- GPIO 1 (CAN TX) → CAN Transceiver TX
- GPIO 2 (CAN RX) → CAN Transceiver RX
- GPIO 10 → Motor Driver PWM
- GPIO 11 → Motor Driver DIR
- GPIO 4 → Encoder A
- GPIO 5 → Encoder B
- GPIO 6 → RGB LED (optional)

**CAN Bus:**
- All CAN transceivers connected to same CAN-H and CAN-L lines
- 120Ω termination resistors at both ends of the bus

## 🧪 Testing Procedure

### Test 1: ESP32 Host Standalone
```bash
# Connect to host ESP32
screen /dev/ttyUSB0 115200

# Expected: Startup messages and status updates every 5 seconds
```

### Test 2: Motor Controllers Standalone
```bash
# Connect to each motor controller
screen /dev/ttyUSB1 115200  # FL
screen /dev/ttyUSB2 115200  # FR
screen /dev/ttyUSB3 115200  # BL
screen /dev/ttyUSB4 115200  # BR

# Expected: Startup messages
# After 500ms: LED should blink PURPLE (CAN timeout - normal, no commands yet)
```

### Test 3: ROS2 Hardware Interface
```bash
# Terminal 1: Build and launch
cd /home/anish/test/mars_rover_ros2
colcon build --packages-select rover_hardware
source install/setup.bash
ros2 launch rover_hardware rover_hardware.launch.py

# Expected output:
# [RoverSerial] 4-Wheel Serial Interface Configured on /dev/ttyACM0
```

### Test 4: Send Test Commands
```bash
# Terminal 2: Send velocity command
ros2 topic pub /skid_steer_controller/cmd_vel_unstamped geometry_msgs/msg/Twist \
  "{linear: {x: 0.5}, angular: {z: 0.0}}" --once
```

**Expected Output Chain:**

**ROS2 Terminal:**
```
[RoverSerial] 📤 TX Command #1 → FL: 0.500 | RL: 0.500 | FR: 0.500 | RR: 0.500 rad/s
```

**Host ESP32 Serial Monitor:**
```
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
📦 RX Serial Packet #1
   FL: 0.500 rad/s
   RL: 0.500 rad/s
   FR: 0.500 rad/s
   RR: 0.500 rad/s
   ✓ CAN TX [0x124]: speed=50, dir=1 (0.500 rad/s)
   ✓ CAN TX [0x123]: speed=50, dir=1 (0.500 rad/s)
   ✓ CAN TX [0x121]: speed=50, dir=1 (0.500 rad/s)
   ✓ CAN TX [0x122]: speed=50, dir=1 (0.500 rad/s)
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
```

**Each Motor Controller Serial Monitor:**
```
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
✓ CAN RX [FL] #1
   ID: 0x124
   Speed: 50
   Direction: 1 (FWD)
   → Target RPM: 125.00
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
```

## 🐛 Troubleshooting

### Issue: No serial output from Host ESP32
**Solution:**
- Check USB connection
- Verify correct serial port in Arduino IDE
- Try different baud rate (115200)

### Issue: "CAN Driver Install FAILED"
**Solution:**
- Check GPIO pin connections (TX=5, RX=4 for host)
- Verify CAN transceiver power supply
- Check for shorts on CAN bus

### Issue: Motor controllers not receiving CAN messages
**Check:**
1. CAN bus wiring (CAN-H, CAN-L connected?)
2. Termination resistors (120Ω at both ends)
3. All devices show "CAN Started @500kbps"
4. Correct CAN IDs programmed
5. Use multimeter to check CAN-H and CAN-L voltage (~2.5V idle)

### Issue: ROS2 "Failed to open /dev/ttyACM0"
**Solution:**
```bash
# Find correct port
ls /dev/ttyACM* /dev/ttyUSB*

# Add user to dialout group
sudo usermod -a -G dialout $USER
# Log out and back in

# Or use sudo (temporary)
sudo chmod 666 /dev/ttyACM0
```

### Issue: Motors receiving commands but not moving
**Check:**
1. Encoder connections (A and B channels)
2. Motor driver power supply
3. PWM and DIR connections
4. Check motor controller debug output for "Encoder Fault"

### Issue: LED Status Meanings
- **OFF**: Normal operation, everything working
- **Blinking PURPLE**: CAN timeout (no commands received)
- **Blinking YELLOW**: Encoder fault (disconnected or stalled)
- **Blinking LIME**: Active braking (motor fighting to stop)

## 📊 Debug Output Explanation

### ROS2 Hardware Interface
```
📤 TX Command #123 → FL: 0.500 | RL: 0.500 | FR: 0.500 | RR: 0.500 rad/s
```
- Shows velocity commands being sent to ESP32 host
- Updates every 500ms
- Values in rad/s

```
📥 RX Feedback #45 | FL: 0.49 rad/s | RL: 0.48 rad/s | FR: 0.50 rad/s | RR: 0.51 rad/s
```
- Shows feedback from motors (if implemented)
- Updates every 2 seconds

### ESP32 Host
```
╔════════════════════════════════════════╗
║         SYSTEM STATUS                  ║
╚════════════════════════════════════════╝
Serial Packets RX: 234
CAN Messages TX:   936
CAN Errors:        0
Uptime:            47 seconds
CAN State:         RUNNING
```
- Printed every 5 seconds
- `CAN Messages TX` should be 4x `Serial Packets RX` (4 motors)
- `CAN State` should always be `RUNNING`

### Motor Controllers
```
╔════════════════════════════════════════╗
║  STATUS [FR]                           ║
╚════════════════════════════════════════╝
CAN Messages RX:   234
CAN Messages IGN:  12
Target RPM:        125.50
Current RPM:       124.80
PWM Output:        145
Encoder Fault:     NO
CAN Timeout:       NO
```
- Printed every 5 seconds
- `CAN Messages IGN` = messages for other motors (normal)
- `Target RPM` should match commanded velocity
- `Current RPM` should track `Target RPM` closely

## 🎯 Success Criteria

✅ **All systems working when:**
1. ROS2 shows TX commands being sent
2. Host ESP32 shows serial RX and CAN TX
3. Each motor controller shows CAN RX for its ID
4. Motors spin at commanded velocities
5. No error messages in any serial monitor
6. LEDs are OFF (or blinking lime during braking)

## 🔍 Advanced Debugging

### Enable verbose CAN debugging
In motor controller code, set:
```cpp
#define DEBUG_CAN true  // Already enabled
```

In host code, set:
```cpp
#define DEBUG_ENABLED true  // Already enabled
```

### Monitor CAN bus with logic analyzer
- Connect to CAN-H and CAN-L
- Set baud rate: 500 kbps
- Look for message IDs: 0x121, 0x122, 0x123, 0x124

### Check ROS2 topics
```bash
# List all topics
ros2 topic list

# Echo cmd_vel
ros2 topic echo /skid_steer_controller/cmd_vel_unstamped

# Check joint states
ros2 topic echo /joint_states
```
