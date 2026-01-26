# Network Access Configuration

## Accessing the Web GUI from Other Devices

The web GUI can be accessed from mobile phones, tablets, and other computers on the same WiFi network.

### Quick Setup

1. **Find your machine's IP address:**
   ```bash
   hostname -I | awk '{print $1}'
   ```
   Example output: `192.168.1.100`

2. **Start the services** (same as before):
   ```bash
   # Terminal 1: ROS Bridge
   ./src/start_ros_bridge.sh
   
   # Terminal 2: Web GUI
   cd my-app && npm run dev
   
   # Terminal 3: RViz VNC (optional)
   ./src/start_rviz_vnc.sh
   ```

3. **Access from any device on the network:**
   - On the same computer: `http://localhost:3000`
   - From mobile/other devices: `http://192.168.1.100:3000`
   
   Replace `192.168.1.100` with your actual IP address.

### Firewall Configuration

If you can't access from other devices, allow the ports through the firewall:

```bash
sudo ufw allow 3000/tcp   # Web GUI
sudo ufw allow 9090/tcp   # ROS Bridge
sudo ufw allow 8080/tcp   # Camera streams
sudo ufw allow 6080/tcp   # RViz VNC
```

### For Jetson Nano Deployment

When running on Jetson Nano connected to Ubiquiti network:

1. **Find Jetson's IP on the network:**
   ```bash
   # On Jetson
   hostname -I
   ```

2. **Start all services on Jetson**

3. **Access from any device on the same network:**
   ```
   http://<jetson-ip>:3000
   ```

### Dynamic Host Detection

The web GUI automatically detects the hostname and connects to ROS services on the same machine. This means:

- If you access via `http://192.168.1.100:3000`, it connects to `ws://192.168.1.100:9090` for ROS
- If you access via `http://localhost:3000`, it connects to `ws://localhost:9090`
- Camera streams and VNC also use the same hostname automatically

### Network Diagram

```
┌─────────────────────────────────────────┐
│         Ubiquiti WiFi Network           │
│                                         │
│  ┌──────────────┐    ┌──────────────┐  │
│  │ Jetson Nano  │    │ Mobile Phone │  │
│  │ 192.168.1.50 │    │ 192.168.1.20 │  │
│  │              │    │              │  │
│  │ - ROS Bridge │◄───┤ Browser:     │  │
│  │ - Web GUI    │    │ 192.168.1.50 │  │
│  │ - Cameras    │    │    :3000     │  │
│  └──────────────┘    └──────────────┘  │
│                                         │
│  ┌──────────────┐    ┌──────────────┐  │
│  │   Laptop     │    │   Tablet     │  │
│  │ 192.168.1.30 │    │ 192.168.1.40 │  │
│  └──────────────┘    └──────────────┘  │
│         │                    │          │
│         └────────────────────┘          │
│         Both can access Jetson GUI      │
└─────────────────────────────────────────┘
```

### Troubleshooting Network Access

**Can't connect from mobile:**
1. Check firewall (see above)
2. Verify both devices are on same network
3. Ping the Jetson/laptop from mobile:
   ```bash
   ping 192.168.1.50
   ```

**ROS connection fails:**
1. Check rosbridge is running: `netstat -tuln | grep 9090`
2. Verify firewall allows port 9090
3. Check browser console for WebSocket errors

**Cameras not loading:**
1. Verify web_video_server is running: `netstat -tuln | grep 8080`
2. Test camera stream directly: `http://<ip>:8080/stream?topic=/rgbd_camera/image&type=mjpeg`
3. Check firewall allows port 8080

**RViz VNC not showing:**
1. Verify VNC server is running: `netstat -tuln | grep 6080`
2. Test noVNC directly: `http://<ip>:6080/vnc.html`
3. Check firewall allows port 6080

### Production Deployment

For permanent deployment on Jetson Nano:

1. **Set static IP** for Jetson on your router
2. **Use systemd services** (see WEB_GUI_SETUP.md)
3. **Configure firewall** to allow required ports
4. **Optional**: Set up nginx reverse proxy for HTTPS

### Mobile App Considerations

The web GUI is mobile-responsive and works well on phones/tablets:
- Touch-friendly joystick controls
- Responsive camera layouts
- Optimized for portrait and landscape modes
- Works offline once loaded (PWA capable)

### Security Note

⚠️ **Important**: The web GUI has no authentication by default. On a trusted network (like your rover's local WiFi), this is fine. For internet-facing deployments, add authentication or use a VPN.
