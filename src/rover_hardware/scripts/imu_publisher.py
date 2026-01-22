#!/usr/bin/env python3
"""
IMU Publisher Node for Jetson Nano with ICM-20948
Reads IMU data from I2C and publishes to /imu/data

Hardware Configuration:
- ICM-20948 9-DOF IMU (Accelerometer + Gyroscope + Magnetometer)
- Jetson Nano I2C Bus 1
- Pin 3: SDA (I2C Data)
- Pin 5: SCL (I2C Clock)

Setup Instructions:
1. Enable I2C permissions:
   sudo usermod -aG i2c $USER
   sudo chmod 666 /dev/i2c-1

2. Install dependencies:
   sudo apt-get install -y i2c-tools python3-smbus
   sudo pip3 install smbus2 adafruit-blinka adafruit-circuitpython-icm20x

3. Verify I2C connection:
   i2cdetect -y -r 1
   (Should show device at 0x68 or 0x69)

4. Test IMU:
   ros2 run rover_hardware imu_publisher_jetson
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField
from std_msgs.msg import Header
import math

# I2C Configuration for Jetson Nano
I2C_BUS = 1  # Jetson Nano I2C bus (Pin 3=SDA, Pin 5=SCL)
ICM20948_ADDR = 0x69  # Default ICM-20948 address (can also be 0x68)

# Try to import I2C libraries
try:
    import board
    import busio
    import adafruit_icm20x
    ADAFRUIT_AVAILABLE = True
except ImportError as e:
    print(f"WARNING: Adafruit libraries not available: {e}")
    print("Install with: sudo pip3 install adafruit-blinka adafruit-circuitpython-icm20x")
    ADAFRUIT_AVAILABLE = False

try:
    from smbus2 import SMBus
    SMBUS_AVAILABLE = True
except ImportError:
    print("WARNING: smbus2 not available")
    print("Install with: sudo pip3 install smbus2")
    SMBUS_AVAILABLE = False


class IMUPublisherJetson(Node):
    """ROS2 node to publish ICM-20948 IMU data on Jetson Nano"""
    
    def __init__(self):
        super().__init__('imu_publisher_jetson')
        
        # Parameters
        self.declare_parameter('publish_rate', 100.0)  # Hz
        self.declare_parameter('imu_frame', 'imu_link')
        self.declare_parameter('i2c_bus', I2C_BUS)
        self.declare_parameter('i2c_address', ICM20948_ADDR)
        self.declare_parameter('simulate', False)
        self.declare_parameter('publish_mag', True)  # Publish magnetometer data
        
        publish_rate = self.get_parameter('publish_rate').value
        self.imu_frame = self.get_parameter('imu_frame').value
        self.i2c_bus = self.get_parameter('i2c_bus').value
        self.i2c_address = self.get_parameter('i2c_address').value
        self.simulate = self.get_parameter('simulate').value
        self.publish_mag = self.get_parameter('publish_mag').value
        
        # Publishers
        self.imu_pub = self.create_publisher(Imu, '/imu/data', 10)
        
        if self.publish_mag:
            self.mag_pub = self.create_publisher(MagneticField, '/imu/mag', 10)
        
        # Timer
        self.timer = self.create_timer(1.0 / publish_rate, self.timer_callback)
        
        # Initialize IMU hardware
        self.imu = None
        self.use_adafruit = False
        
        if not self.simulate:
            if not self.init_imu():
                self.get_logger().warn('Failed to initialize IMU, running in simulation mode')
                self.simulate = True
        else:
            self.get_logger().warn('Running in SIMULATION mode')
        
        self.get_logger().info(f'IMU Publisher started at {publish_rate} Hz on Jetson Nano')
        self.get_logger().info(f'I2C Bus: {self.i2c_bus}, Address: 0x{self.i2c_address:02X}')
        self.get_logger().info(f'Publishing to /imu/data (frame: {self.imu_frame})')
    
    def init_imu(self):
        """Initialize ICM-20948 IMU hardware"""
        
        # Try Adafruit library first (recommended for ICM-20948)
        if ADAFRUIT_AVAILABLE:
            try:
                # Initialize I2C bus on Jetson Nano
                i2c = busio.I2C(board.SCL_1, board.SDA_1)  # Jetson Nano I2C bus 1
                
                # Initialize ICM-20948
                self.imu = adafruit_icm20x.ICM20948(i2c, address=self.i2c_address)
                
                # Configure sensor ranges
                self.imu.accelerometer_range = adafruit_icm20x.AccelRange.RANGE_2G
                self.imu.gyro_range = adafruit_icm20x.GyroRange.RANGE_250_DPS
                
                if self.publish_mag:
                    self.imu.magnetometer_data_rate = adafruit_icm20x.MagDataRate.RATE_100HZ
                
                self.use_adafruit = True
                self.get_logger().info('✓ ICM-20948 initialized successfully via Adafruit library')
                return True
                
            except ValueError as e:
                self.get_logger().error(f'ICM-20948 not found at address 0x{self.i2c_address:02X}')
                self.get_logger().info('Run "i2cdetect -y -r 1" to find the correct address')
                self.get_logger().error(f'Error: {e}')
                
            except Exception as e:
                self.get_logger().warn(f'Adafruit library init failed: {e}')
        
        # Fall back to direct SMBus access
        if SMBUS_AVAILABLE:
            try:
                self.bus = SMBus(self.i2c_bus)
                
                # Test connection by reading WHO_AM_I register
                who_am_i = self.bus.read_byte_data(self.i2c_address, 0x00)
                
                if who_am_i == 0xEA:  # ICM-20948 WHO_AM_I value
                    # Wake up device (clear sleep bit in PWR_MGMT_1)
                    self.bus.write_byte_data(self.i2c_address, 0x06, 0x01)
                    
                    # Configure accelerometer and gyroscope
                    # Set full scale ranges
                    self.bus.write_byte_data(self.i2c_address, 0x14, 0x00)  # ±2g
                    self.bus.write_byte_data(self.i2c_address, 0x01, 0x00)  # ±250°/s
                    
                    self.use_adafruit = False
                    self.get_logger().info(f'✓ ICM-20948 initialized via SMBus (WHO_AM_I=0x{who_am_i:02X})')
                    self.get_logger().warn('Magnetometer not available in SMBus mode')
                    return True
                else:
                    self.get_logger().error(f'Wrong WHO_AM_I: 0x{who_am_i:02X} (expected 0xEA for ICM-20948)')
                    self.get_logger().info('Check I2C wiring and address')
                    return False
                    
            except FileNotFoundError:
                self.get_logger().error(f'I2C device /dev/i2c-{self.i2c_bus} not found')
                self.get_logger().info('Enable I2C: sudo chmod 666 /dev/i2c-1')
                return False
                
            except PermissionError:
                self.get_logger().error(f'Permission denied for /dev/i2c-{self.i2c_bus}')
                self.get_logger().info('Fix permissions: sudo usermod -aG i2c $USER && sudo chmod 666 /dev/i2c-1')
                return False
                
            except Exception as e:
                self.get_logger().error(f'SMBus init failed: {e}')
                return False
        
        self.get_logger().error('No I2C libraries available')
        return False
    
    def read_word_2c(self, reg):
        """Read 16-bit signed word from ICM-20948 (two's complement)"""
        high = self.bus.read_byte_data(self.i2c_address, reg)
        low = self.bus.read_byte_data(self.i2c_address, reg + 1)
        value = (high << 8) | low
        if value >= 0x8000:
            value = -((65535 - value) + 1)
        return value
    
    def read_imu_smbus(self):
        """Read IMU data using direct SMBus access"""
        # ICM-20948 Register addresses
        ACCEL_XOUT_H = 0x2D
        GYRO_XOUT_H = 0x33
        
        # Read accelerometer (±2g range = 16384 LSB/g)
        accel_x = self.read_word_2c(ACCEL_XOUT_H) / 16384.0 * 9.81  # Convert to m/s²
        accel_y = self.read_word_2c(ACCEL_XOUT_H + 2) / 16384.0 * 9.81
        accel_z = self.read_word_2c(ACCEL_XOUT_H + 4) / 16384.0 * 9.81
        
        # Read gyroscope (±250°/s range = 131 LSB/(°/s))
        gyro_x = math.radians(self.read_word_2c(GYRO_XOUT_H) / 131.0)  # Convert to rad/s
        gyro_y = math.radians(self.read_word_2c(GYRO_XOUT_H + 2) / 131.0)
        gyro_z = math.radians(self.read_word_2c(GYRO_XOUT_H + 4) / 131.0)
        
        return {
            'accel': (accel_x, accel_y, accel_z),
            'gyro': (gyro_x, gyro_y, gyro_z),
            'mag': None  # Magnetometer requires bank switching - use Adafruit library
        }
    
    def timer_callback(self):
        """Read IMU and publish data"""
        msg = Imu()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.imu_frame
        
        if self.simulate:
            # Simulated data for testing
            msg.angular_velocity.x = 0.0
            msg.angular_velocity.y = 0.0
            msg.angular_velocity.z = 0.0
            msg.linear_acceleration.x = 0.0
            msg.linear_acceleration.y = 0.0
            msg.linear_acceleration.z = 9.81  # Gravity
            msg.orientation_covariance[0] = -1.0  # No orientation data
            
        else:
            try:
                if self.use_adafruit:
                    # Read using Adafruit library (recommended)
                    gyro = self.imu.gyro  # rad/s
                    accel = self.imu.acceleration  # m/s²
                    
                    msg.angular_velocity.x = gyro[0]
                    msg.angular_velocity.y = gyro[1]
                    msg.angular_velocity.z = gyro[2]
                    
                    msg.linear_acceleration.x = accel[0]
                    msg.linear_acceleration.y = accel[1]
                    msg.linear_acceleration.z = accel[2]
                    
                    # Publish magnetometer separately
                    if self.publish_mag:
                        try:
                            mag = self.imu.magnetic  # µT
                            mag_msg = MagneticField()
                            mag_msg.header = msg.header
                            mag_msg.magnetic_field.x = mag[0] * 1e-6  # Convert µT to Tesla
                            mag_msg.magnetic_field.y = mag[1] * 1e-6
                            mag_msg.magnetic_field.z = mag[2] * 1e-6
                            
                            # Set magnetometer covariance
                            mag_msg.magnetic_field_covariance = [
                                0.0001, 0.0, 0.0,
                                0.0, 0.0001, 0.0,
                                0.0, 0.0, 0.0001
                            ]
                            
                            self.mag_pub.publish(mag_msg)
                        except Exception as e:
                            self.get_logger().warn(f'Magnetometer read failed: {e}', throttle_duration_sec=5.0)
                    
                else:
                    # Read using direct SMBus
                    data = self.read_imu_smbus()
                    
                    gyro = data['gyro']
                    msg.angular_velocity.x = gyro[0]
                    msg.angular_velocity.y = gyro[1]
                    msg.angular_velocity.z = gyro[2]
                    
                    accel = data['accel']
                    msg.linear_acceleration.x = accel[0]
                    msg.linear_acceleration.y = accel[1]
                    msg.linear_acceleration.z = accel[2]
                
                # No orientation from raw IMU (use robot_localization for fusion)
                msg.orientation_covariance[0] = -1.0
                
            except Exception as e:
                self.get_logger().error(f'Error reading IMU: {e}', throttle_duration_sec=1.0)
                return
        
        # Set covariances (tune these based on your sensor characteristics)
        msg.angular_velocity_covariance = [
            0.02, 0.0, 0.0,
            0.0, 0.02, 0.0,
            0.0, 0.0, 0.02
        ]
        
        msg.linear_acceleration_covariance = [
            0.04, 0.0, 0.0,
            0.0, 0.04, 0.0,
            0.0, 0.0, 0.04
        ]
        
        self.imu_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = IMUPublisherJetson()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
