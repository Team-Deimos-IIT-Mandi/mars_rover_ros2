# Serial Port Assignments for Mars Rover Hardware

This document describes the serial port assignments for all hardware components on the Mars Rover.

## Port Assignments

| Device | Serial Port | Baud Rate | Description |
|--------|-------------|-----------|-------------|
| Motor Controller (ESP32) | `/dev/ttyUSB0` | 115200 | 4-wheel skid-steer motor control via serial |
| GPS Module | `/dev/ttyUSB1` | 9600 | NMEA GPS receiver |
| LiDAR Sensor | `/dev/ttyUSB2` | 115200 | RPLidar or YDLidar |

## Camera Assignments

| Camera | Device | Resolution | Frame Rate | Description |
|--------|--------|------------|------------|-------------|
| RGBD Camera (Front) | `/dev/video0` | 640x480 | 30 FPS | Front-facing depth camera |
| Camera 1 (Left) | `/dev/video1` | 640x480 | 30 FPS | Left side camera |
| Camera 2 (Right) | `/dev/video2` | 640x480 | 30 FPS | Right side camera |

## Notes

- All cameras are USB-connected (not CSI)
- Serial port assignments may vary depending on USB connection order
- Use `udev` rules to create persistent device names if needed
- IMU connection details depend on your specific IMU hardware (I2C or Serial)

## Creating Persistent Device Names (Optional)

To ensure consistent device names across reboots, create udev rules:

```bash
# Example udev rule for motor controller
# /etc/udev/rules.d/99-rover-serial.rules
SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", SYMLINK+="rover_motors"
```

Then use `/dev/rover_motors` instead of `/dev/ttyUSB0` in your configuration.
