#!/bin/bash
# Open firewall ports for Mars Rover Web GUI network access

echo "============================================"
echo "Opening Firewall Ports for Network Access"
echo "============================================"

echo "Opening port 3000 (Web GUI)..."
sudo ufw allow 3000/tcp

echo "Opening port 9090 (ROS Bridge)..."
sudo ufw allow 9090/tcp

echo "Opening port 8080 (Camera Streams)..."
sudo ufw allow 8080/tcp

echo "Opening port 6080 (RViz VNC)..."
sudo ufw allow 6080/tcp

echo ""
echo "============================================"
echo "Firewall Configuration Complete!"
echo "============================================"
echo ""
echo "Your laptop's IP address:"
hostname -I | awk '{print $1}'
echo ""
echo "Access from mobile/other devices:"
echo "  Web GUI: http://$(hostname -I | awk '{print $1}'):3000"
echo ""
echo "Testing ports..."
echo ""
netstat -tuln | grep -E '(3000|9090|8080|6080)' | awk '{print "  Port " $4 " - LISTENING"}'
echo ""
echo "If mobile still can't connect, try:"
echo "1. Make sure both devices are on same WiFi"
echo "2. Restart the web GUI (Ctrl+C and npm run dev)"
echo "3. Clear browser cache on mobile"
echo "============================================"
