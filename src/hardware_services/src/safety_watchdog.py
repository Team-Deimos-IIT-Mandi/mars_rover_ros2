import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import NavSatFix, BatteryState
from std_msgs.msg import Float32
import subprocess
import serial
import time
import json

# CONFIG
BASE_IP = "192.168.1.10" # Your Laptop IP
HELTEC_PORT = "/dev/ttyUSB0"
CHECK_INTERVAL = 0.5 # Seconds

class SafetyWatchdog(Node):
    def __init__(self):
        super().__init__('safety_watchdog')
        
        # 1. SAFETY: Publisher to override wheels
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel_teleop', 10) # High Priority Topic
        
        # 2. TELEMETRY: Subscribers
        self.create_subscription(NavSatFix, '/gps/fix', self.gps_callback, 10)
        self.create_subscription(BatteryState, '/battery_status', self.bat_callback, 10)
        self.create_subscription(Twist, '/cmd_vel', self.vel_callback, 10)
        
        # Data Storage
        self.telemetry = {
            "lat": 0.0, "lon": 0.0,
            "v": 0.0,   # Voltage
            "s": 0.0,   # Speed (linear x)
            "t": 45.0   # Temp (Dummy or read from sensor)
        }

        # 3. HARDWARE LINK
        try:
            self.lora_serial = serial.Serial(HELTEC_PORT, 115200, timeout=0.1)
            self.get_logger().info(f"Connected to LoRa on {HELTEC_PORT}")
        except:
            self.get_logger().error("LoRa Hardware NOT FOUND!")
            self.lora_serial = None

        # Main Loop Timer
        self.timer = self.create_timer(CHECK_INTERVAL, self.control_loop)

    def gps_callback(self, msg):
        self.telemetry["lat"] = round(msg.latitude, 6)
        self.telemetry["lon"] = round(msg.longitude, 6)

    def bat_callback(self, msg):
        self.telemetry["v"] = round(msg.voltage, 2)

    def vel_callback(self, msg):
        self.telemetry["s"] = round(msg.linear.x, 2)

    def check_connection(self):
        """Returns True if Base Station is reachable"""
        try:
            # Ping once, wait max 0.5s
            subprocess.check_call(
                ['ping', '-c', '1', '-W', '1', BASE_IP],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL
            )
            return True
        except subprocess.CalledProcessError:
            return False

    def control_loop(self):
        # --- A. CHECK UBIQUITI CONNECTION ---
        if self.check_connection():
            # Link is GOOD. Do nothing (allow normal driving)
            pass
        else:
            # Link is BAD. HARD STOP.
            self.get_logger().warn("CONNECTION LOST! HALTING ROVER.")
            stop_msg = Twist() # All zeros
            self.cmd_pub.publish(stop_msg)

        # --- B. SEND TELEMETRY TO LORA ---
        if self.lora_serial:
            try:
                # Create compact CSV: "lat,lon,volts,speed,temp"
                # Example: "28.123,76.456,12.4,0.5,42"
                packet = f"{self.telemetry['lat']},{self.telemetry['lon']},{self.telemetry['v']},{self.telemetry['s']},{self.telemetry['t']}\n"
                self.lora_serial.write(packet.encode('utf-8'))
            except Exception as e:
                self.get_logger().error(f"Serial Write Error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = SafetyWatchdog()
    rclpy.spin(node)
    rclpy.shutdown()