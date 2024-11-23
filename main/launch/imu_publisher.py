import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import smbus2
import math
import time

class MPU6050:
    def __init__(self, bus_number=1, address=0x68):
        self.bus = smbus2.SMBus(bus_number)
        self.address = address
        
        # MPU6050 initialization
        self.bus.write_byte_data(self.address, 0x6B, 0)  # Wake up the MPU6050

    def read_raw_data(self):
        # Read accelerometer and gyroscope data
        accel_x = self.bus.read_word_data(self.address, 0x3B)
        accel_y = self.bus.read_word_data(self.address, 0x3D)
        accel_z = self.bus.read_word_data(self.address, 0x3F)
        gyro_x = self.bus.read_word_data(self.address, 0x43)
        gyro_y = self.bus.read_word_data(self.address, 0x45)
        gyro_z = self.bus.read_word_data(self.address, 0x47)

        # Convert to signed 16-bit values
        accel_x = self.convert_to_signed(accel_x)
        accel_y = self.convert_to_signed(accel_y)
        accel_z = self.convert_to_signed(accel_z)
        gyro_x = self.convert_to_signed(gyro_x)
        gyro_y = self.convert_to_signed(gyro_y)
        gyro_z = self.convert_to_signed(gyro_z)

        return (accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z)

    @staticmethod
    def convert_to_signed(value):
        if value >= 32768:
            value -= 65536
        return value

class ImuPublisher(Node):
    def __init__(self):
        super().__init__('imu_publisher')
        self.imu_pub = self.create_publisher(Imu, 'imu_data', 10)
        self.mpu = MPU6050()

        self.timer = self.create_timer(0.1, self.publish_imu_data)  # Publish every 100 ms

    def publish_imu_data(self):
        accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z = self.mpu.read_raw_data()

        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = 'base_link'

        # Assuming no orientation data is available, set to zero
        imu_msg.orientation.x = 0.0
        imu_msg.orientation.y = 0.0
        imu_msg.orientation.z = 0.0
        imu_msg.orientation.w = 1.0  # No rotation

        # Set angular velocity
        imu_msg.angular_velocity.x = gyro_x * (math.pi / 180)  # Convert to radians
        imu_msg.angular_velocity.y = gyro_y * (math.pi / 180)
        imu_msg.angular_velocity.z = gyro_z * (math.pi / 180)

        # Set linear acceleration
        imu_msg.linear_acceleration.x = accel_x / 16384.0  # Convert to g
        imu_msg.linear_acceleration.y = accel_y / 16384.0
        imu_msg.linear_acceleration.z = accel_z / 16384.0

        self.imu_pub.publish(imu_msg)
        self.get_logger().info(f'Published IMU data: {imu_msg}')

def main(args=None):
    rclpy.init(args=args)
    imu_publisher = ImuPublisher()
    rclpy.spin(imu_publisher)
    imu_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
