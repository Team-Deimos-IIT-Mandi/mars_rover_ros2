# Jetson Nano Web GUI Setup - Quick Reference

## Network Architecture

```
┌─────────────────────────────────────────────┐
│         Your Network Setup                  │
│                                             │
│  ┌──────────────┐      ┌──────────────┐    │
│  │ Jetson Nano  │      │   Your PC    │    │
│  │ (Ubiquiti)   │◄────►│  (WiFi/LAN)  │    │
│  │ 192.168.x.x  │ ROS  │ 192.168.y.y  │    │
│  │              │Domain│              │    │
│  │ - ROS Bridge │      │ - RViz       │    │
│  │ - Web GUI    │      │ - Browser    │    │
│  │ - Cameras    │      │              │    │
│  └──────────────┘      └──────────────┘    │
│         │                      │            │
│         └──────────┬───────────┘            │
│                    │                        │
│            ┌───────▼────────┐               │
│            │  Mobile Phone  │               │
│            │  192.168.z.z   │               │
│            │  - Browser     │               │
│            └────────────────┘               │
└─────────────────────────────────────────────┘
```

## Installation on Jetson Nano

### Prerequisites
- Jetson Nano with Docker container running
- ROS 2 Humble installed in container
- Repository cloned to `~/mars_rover_ros2`

### One-Command Install

```bash
sudo ./install_jetson_webgui.sh
```

This installs:
- Node.js 22.x (LTS)
- ROS Bridge packages
- Web Video Server
- Web GUI npm dependencies
- Configures firewall

**Time:** ~5-10 minutes (with internet)

## Without Internet (Ubiquiti Module)

If Jetson has no internet via Ubiquiti:

### Option 1: Temporary Internet
1. Connect Jetson to WiFi/Ethernet temporarily
2. Run installation script
3. Disconnect and reconnect to Ubiquiti

### Option 2: Offline Installation
1. **On PC with internet:**
   ```bash
   # Download Node.js installer
   wget https://deb.nodesource.com/setup_22.x -O nodesource_setup.sh
   
   # Download npm packages
   cd my-app
   npm pack  # Creates tarball of dependencies
   ```

2. **Transfer to Jetson** (USB/SCP)

3. **On Jetson:**
   ```bash
   # Install Node.js from downloaded script
   sudo bash nodesource_setup.sh
   sudo apt-get install -y nodejs
   
   # Install npm packages from tarball
   cd ~/mars_rover_ros2/my-app
   npm install /path/to/package.tgz
   ```

## Running the Web GUI

### On Jetson Nano

**Terminal 1: ROS Bridge**
```bash
cd ~/mars_rover_ros2
./src/start_ros_bridge.sh
```

**Terminal 2: Web GUI**
```bash
cd ~/mars_rover_ros2/my-app
npm run dev
```

### Access from Network

**From your PC:**
```
http://<jetson-ip>:3000
```

**From mobile:**
```
http://<jetson-ip>:3000
```

**Find Jetson IP:**
```bash
hostname -I | awk '{print $1}'
```

## ROS Domain Setup (Cross-Network)

If Jetson and PC are on different subnets:

**On both Jetson and PC:**
```bash
export ROS_DOMAIN_ID=42  # Use same number
```

Add to `~/.bashrc` to make permanent:
```bash
echo "export ROS_DOMAIN_ID=42" >> ~/.bashrc
```

## Network Access Verification

### Test from PC

1. **Ping Jetson:**
   ```bash
   ping <jetson-ip>
   ```

2. **Test Web GUI:**
   ```bash
   curl http://<jetson-ip>:3000
   ```

3. **Test ROS Bridge:**
   ```bash
   curl http://<jetson-ip>:9090
   ```

4. **Test Camera Stream:**
   ```bash
   curl http://<jetson-ip>:8080
   ```

### Test ROS Topics

**On PC:**
```bash
export ROS_DOMAIN_ID=42
ros2 topic list  # Should see Jetson's topics
ros2 topic echo /camera_1/image_raw  # Should see data
```

## RViz on PC (Not Jetson)

**On your PC (not Jetson):**
```bash
export ROS_DOMAIN_ID=42
rviz2 -d ~/mars_rover_ros2/src/rover_description/rviz/nav2_config.rviz
```

RViz will display data from Jetson's ROS topics over the network!

## Troubleshooting

### Web GUI not accessible from network

1. **Check firewall on Jetson:**
   ```bash
   sudo ufw status
   sudo ufw allow 3000/tcp
   sudo ufw allow 9090/tcp
   sudo ufw allow 8080/tcp
   ```

2. **Verify services are listening on 0.0.0.0:**
   ```bash
   netstat -tuln | grep -E '(3000|9090|8080)'
   # Should show 0.0.0.0:PORT, not 127.0.0.1:PORT
   ```

3. **Check both devices on same network:**
   ```bash
   # On PC
   ip route | grep default
   
   # On Jetson
   ip route | grep default
   ```

### ROS topics not visible from PC

1. **Check ROS_DOMAIN_ID matches:**
   ```bash
   echo $ROS_DOMAIN_ID  # Should be same on both
   ```

2. **Check multicast is working:**
   ```bash
   ros2 multicast receive
   # Run on PC, should see Jetson's messages
   ```

3. **Disable firewall temporarily to test:**
   ```bash
   sudo ufw disable  # Test only
   sudo ufw enable   # Re-enable after
   ```

## Production Deployment

For permanent Jetson deployment:

1. **Auto-start on boot** (systemd services)
2. **Static IP** for Jetson
3. **Firewall rules** configured
4. **ROS_DOMAIN_ID** in bashrc

See `WEB_GUI_SETUP.md` for systemd service examples.

## Summary

✅ **Install script handles everything**
✅ **No VNC needed** (RViz runs on PC)
✅ **Network access works** (even with Ubiquiti)
✅ **ROS topics shared** via ROS_DOMAIN_ID
✅ **Mobile access** works from any device

The web GUI on Jetson is fully accessible from your network!
