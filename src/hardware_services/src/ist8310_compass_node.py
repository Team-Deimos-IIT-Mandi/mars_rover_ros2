#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import MagneticField
import smbus2
import math
import time

class IST8310CompassNode(Node):
    # IST8310 Registers
    WAI_REG = 0x00
    STAT1_REG = 0x02
    DATAXL_REG = 0x03
    DATAXH_REG = 0x04
    DATAYL_REG = 0x05
    DATAYH_REG = 0x06
    DATAZL_REG = 0x07
    DATAZH_REG = 0x08
    STAT2_REG = 0x09
    CNTL1_REG = 0x0A
    CNTL2_REG = 0x0B
    CNTL3_REG = 0x0D
    
    # Control register values
    CNTL3_SRST = 0x01  # Software reset
    CNTL1_SINGLE = 0x01  # Single measurement mode
    
    def __init__(self):
        super().__init__('ist8310_compass_node')
        
        # Parameters
        self.declare_parameter('i2c_bus', 1)
        self.declare_parameter('i2c_address', 0x0C)
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('publish_rate', 50.0)
        self.declare_parameter('mag_offset_x', 0.0)
        self.declare_parameter('mag_offset_y', 0.0)
        self.declare_parameter('mag_offset_z', 0.0)
        self.declare_parameter('mag_scale_x', 1.0)
        self.declare_parameter('mag_scale_y', 1.0)
        self.declare_parameter('mag_scale_z', 1.0)
        self.declare_parameter('magnetic_declination_radians', 0.0)
        
        self.i2c_bus = self.get_parameter('i2c_bus').value
        self.i2c_address = self.get_parameter('i2c_address').value
        self.frame_id = self.get_parameter('frame_id').value
        self.publish_rate = self.get_parameter('publish_rate').value
        
        # Calibration parameters
        self.offset_x = self.get_parameter('mag_offset_x').value
        self.offset_y = self.get_parameter('mag_offset_y').value
        self.offset_z = self.get_parameter('mag_offset_z').value
        self.scale_x = self.get_parameter('mag_scale_x').value
        self.scale_y = self.get_parameter('mag_scale_y').value
        self.scale_z = self.get_parameter('mag_scale_z').value
        self.declination = self.get_parameter('magnetic_declination_radians').value
        
        # Initialize I2C
        try:
            self.bus = smbus2.SMBus(self.i2c_bus)
            self.get_logger().info(f'Opened I2C bus {self.i2c_bus}')
        except Exception as e:
            self.get_logger().error(f'Failed to open I2C bus: {e}')
            return
        
        # Initialize sensor
        if not self.init_sensor():
            self.get_logger().error('Failed to initialize IST8310')
            return
        
        # Publisher
        self.mag_pub = self.create_publisher(MagneticField, 'magnetic_field', 10)
        
        # Timer
        timer_period = 1.0 / self.publish_rate
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        self.get_logger().info('IST8310 compass node started')
    
    def init_sensor(self):
        """Initialize the IST8310 sensor"""
        try:
            # Software reset
            self.bus.write_byte_data(self.i2c_address, self.CNTL3_REG, self.CNTL3_SRST)
            time.sleep(0.01)
            
            # Check WHO_AM_I register
            who_am_i = self.bus.read_byte_data(self.i2c_address, self.WAI_REG)
            self.get_logger().info(f'IST8310 WHO_AM_I: 0x{who_am_i:02X} (expected 0x10)')
            
            if who_am_i != 0x10:
                self.get_logger().warn('Unexpected WHO_AM_I value, but continuing...')
            
            return True
            
        except Exception as e:
            self.get_logger().error(f'Sensor initialization error: {e}')
            return False
    
    def read_magnetometer(self):
        """Read magnetometer data"""
        try:
            # Trigger single measurement
            self.bus.write_byte_data(self.i2c_address, self.CNTL1_REG, self.CNTL1_SINGLE)
            
            # Wait for measurement to complete
            time.sleep(0.007)  # ~7ms measurement time
            
            # Check if data is ready
            stat1 = self.bus.read_byte_data(self.i2c_address, self.STAT1_REG)
            if not (stat1 & 0x01):
                return None
            
            # Read raw data (6 bytes)
            data = self.bus.read_i2c_block_data(self.i2c_address, self.DATAXL_REG, 6)
            
            # Convert to signed 16-bit values
            x = self.to_signed_16bit(data[1] << 8 | data[0])
            y = self.to_signed_16bit(data[3] << 8 | data[2])
            z = self.to_signed_16bit(data[5] << 8 | data[4])
            
            # IST8310 sensitivity: 0.3 µT/LSB = 3.0e-7 T/LSB
            sensitivity = 3.0e-7
            
            # Apply calibration
            x = (x * sensitivity - self.offset_x) * self.scale_x
            y = (y * sensitivity - self.offset_y) * self.scale_y
            z = (z * sensitivity - self.offset_z) * self.scale_z
            
            return (x, y, z)
            
        except Exception as e:
            self.get_logger().error(f'Error reading magnetometer: {e}')
            return None
    
    def to_signed_16bit(self, value):
        """Convert unsigned 16-bit to signed"""
        if value > 32767:
            return value - 65536
        return value
    
    def timer_callback(self):
        """Timer callback to publish magnetometer data"""
        mag_data = self.read_magnetometer()
        
        if mag_data is None:
            return
        
        # Create and publish message
        msg = MagneticField()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        
        msg.magnetic_field.x = mag_data[0]
        msg.magnetic_field.y = mag_data[1]
        msg.magnetic_field.z = mag_data[2]
        
        # Set covariance (adjust based on sensor specs)
        msg.magnetic_field_covariance = [
            0.01, 0.0, 0.0,
            0.0, 0.01, 0.0,
            0.0, 0.0, 0.01
        ]
        
        self.mag_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = IST8310CompassNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
