# ESP32 4-Motor Rover Architecture
Team Deimos IIT Mandi

## System Overview

```
┌─────────────────┐
│   ROS2 Computer │
│  (cmd_vel)      │
└────────┬────────┘
         │ USB Serial
         │ (18-byte packets)
         ▼
┌─────────────────────────────────┐
│  ESP32 #1 (HOST + FL MOTOR)     │
│  - Receives serial from ROS2    │
│  - Controls FL motor directly   │
│  - Sends CAN to 3 other motors  │
└────────┬────────────────────────┘
         │ CAN Bus (500 kbps)
         │
    ┌────┴────┬────────┬────────┐
    ▼         ▼        ▼        ▼
┌─────────┐ ┌────────┐ ┌────────┐
│ ESP32 #2│ │ESP32 #3│ │ESP32 #4│
│ RL Motor│ │FR Motor│ │RR Motor│
│ ID:0x123│ │ID:0x121│ │ID:0x122│
└─────────┘ └────────┘ └────────┘
```

## Hardware Configuration

### Total ESP32s: 4 (not 5!)

| ESP32 # | Role | Motor | CAN ID | Serial | Code File |
|---------|------|-------|--------|--------|-----------|
| 1 | Host + Motor | FL | N/A (local) | ✓ USB to ROS2 | `esp32_host_serial_can_bridge_with_motor.ino` |
| 2 | Motor Only | RL | 0x123 | ✗ | `esp32_motor_controller_template.ino` |
| 3 | Motor Only | FR | 0x121 | ✗ | `esp32_motor_controller_template.ino` |
| 4 | Motor Only | RR | 0x122 | ✗ | `esp32_motor_controller_template.ino` |

## Configuration Steps

### ESP32 #1 (Host + FL Motor)

**File**: `esp32_host_serial_can_bridge_with_motor.ino`

**Configure these lines** (around line 35-41):
```cpp
#define THIS_MOTOR_INDEX 0    // 0=FL (this ESP controls FL)
#define THIS_MOTOR_NAME "FL"

// CAN IDs for the OTHER 3 motors
#define CAN_ID_MOTOR_1 0x121  // FR
#define CAN_ID_MOTOR_2 0x123  // RL  
#define CAN_ID_MOTOR_3 0x122  // RR
```

**Connections**:
- USB → ROS2 Computer
- GPIO 5 (CAN TX) → CAN Transceiver
- GPIO 4 (CAN RX) → CAN Transceiver
- GPIO 10 → FL Motor Driver PWM
- GPIO 11 → FL Motor Driver DIR
- GPIO 4 → FL Encoder A
- GPIO 5 → FL Encoder B
- GPIO 6 → RGB LED (optional)

### ESP32 #2 (RL Motor)

**File**: `esp32_motor_controller_template.ino`

**Configure** (line 28-29):
```cpp
#define CAN_ID_MOTOR 0x123
#define WHEEL_NAME "RL"
```

**Connections**:
- GPIO 1 (CAN TX) → CAN Transceiver
- GPIO 2 (CAN RX) → CAN Transceiver
- GPIO 10 → RL Motor Driver PWM
- GPIO 11 → RL Motor Driver DIR
- GPIO 4 → RL Encoder A
- GPIO 5 → RL Encoder B
- GPIO 6 → RGB LED (optional)

### ESP32 #3 (FR Motor)

**File**: `esp32_motor_controller_template.ino`

**Configure** (line 28-29):
```cpp
#define CAN_ID_MOTOR 0x121
#define WHEEL_NAME "FR"
```

**Connections**: Same as ESP32 #2, but for FR motor

### ESP32 #4 (RR Motor)

**File**: `esp32_motor_controller_template.ino`

**Configure** (line 28-29):
```cpp
#define CAN_ID_MOTOR 0x122
#define WHEEL_NAME "RR"
```

**Connections**: Same as ESP32 #2, but for RR motor

## Communication Flow

### ROS2 → ESP32 #1 (Serial)
```
ROS2 sends 18-byte packet:
[0xA5][FL_vel][RL_vel][FR_vel][RR_vel][0x5A]
       4 bytes 4 bytes 4 bytes 4 bytes
```

### ESP32 #1 Processing
1. Receives serial packet
2. Extracts FL velocity → controls local motor
3. Sends RL velocity → CAN ID 0x123
4. Sends FR velocity → CAN ID 0x121
5. Sends RR velocity → CAN ID 0x122

### ESP32 #1 → Other ESP32s (CAN)
```
CAN message (3 bytes):
[speed (int16_t)][direction (int8_t)]
```

## Debugging Output

### ESP32 #1 (Host + FL)
```
╔════════════════════════════════════════╗
║  ESP32 HOST + MOTOR CONTROLLER        ║
║  Team Deimos IIT Mandi                 ║
╚════════════════════════════════════════╝
This ESP controls: FL motor
CAN IDs for others: 0x121, 0x123, 0x122

✓ CAN Driver Installed
✓ CAN Started @500kbps
✓ Ready to receive serial commands from ROS2
✓ CAN bus active for 3 remote motors
✓ Local motor controller active

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
📦 RX Serial Packet #1
   FL: 0.500 | RL: 0.500 | FR: 0.500 | RR: 0.500 rad/s
   → Local motor [FL]: 50.00 RPM
   ✓ CAN TX [0x123]: speed=50, dir=1 (0.500 rad/s)  ← RL
   ✓ CAN TX [0x121]: speed=50, dir=1 (0.500 rad/s)  ← FR
   ✓ CAN TX [0x122]: speed=50, dir=1 (0.500 rad/s)  ← RR
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
```

### ESP32 #2, #3, #4 (Motor Controllers)
```
╔════════════════════════════════════════╗
║  MOTOR CONTROLLER [RL] - v2.0          ║
║  Team Deimos IIT Mandi                 ║
╚════════════════════════════════════════╝
CAN ID: 0x123

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
✓ CAN RX [RL] #1
   ID: 0x123
   Speed: 50
   Direction: 1 (FWD)
   → Target RPM: 125.00
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
```

## Quick Test

```bash
# Terminal 1: Launch ROS2 hardware interface
cd /home/anish/test/mars_rover_ros2
source install/setup.bash
ros2 launch rover_hardware rover_hardware.launch.py

# Terminal 2: Send test command
ros2 topic pub /skid_steer_controller/cmd_vel_unstamped \
  geometry_msgs/msg/Twist "{linear: {x: 0.5}, angular: {z: 0.0}}" --once
```

**Expected**: 
- ROS2 logs TX command
- ESP32 #1 logs serial RX + local motor control + 3x CAN TX
- ESP32 #2, #3, #4 each log CAN RX for their respective IDs
- All 4 motors spin

## Files Summary

| File | Purpose |
|------|---------|
| `esp32_host_serial_can_bridge_with_motor.ino` | ESP32 #1 (Host + FL motor) |
| `esp32_motor_controller_template.ino` | ESP32 #2, #3, #4 (template, change CAN ID) |
| `rover_can_interface.cpp` | ROS2 hardware interface (already updated with debugging) |
| `ESP32_TESTING_GUIDE.md` | Detailed testing procedures |
| `ESP32_ARCHITECTURE.md` | This file |
