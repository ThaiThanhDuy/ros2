import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import smbus
import math
import time

class MPU6050Node(Node):
    def __init__(self):
        super().__init__('mpu6050_node')
        self.publisher_ = self.create_publisher(Imu, 'imu/data', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)

        # Initialize I2C bus
        self.bus = smbus.SMBus(1)
        self.address = 0x68  # MPU6050 I2C address

        # Initialize MPU6050
        self.bus.write_byte_data(self.address, 0x6B, 0)  # Wake up the MPU6050

    def read_imu_data(self):
        # Read accelerometer data
        accel_x = self.read_word_2c(0x3B)
        accel_y = self.read_word_2c(0x3D)
        accel_z = self.read_word_2c(0x3F)

        # Read gyroscope data
        gyro_x = self.read_word_2c(0x43)
        gyro_y = self.read_word_2c(0x45)
        gyro_z = self.read_word_2c(0x47)

        return accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z

    def read_word_2c(self, addr):
        high = self.bus.read_byte_data(self.address, addr)
        low = self.bus.read_byte_data(self.address, addr + 1)
        value = (high << 8) + low
        if value >= 0x8000:
            return -((65535 - value) + 1)
        else:
            return value

    def timer_callback(self):
        accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z = self.read_imu_data()

        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = 'imu_link'  # Frame ID for the IMU

        # Fill in the IMU message
        imu_msg.linear_acceleration.x = accel_x
        imu_msg.linear_acceleration.y = accel_y
        imu_msg.linear_acceleration.z = accel_z
        imu_msg.angular_velocity.x = gyro_x
        imu_msg.angular_velocity.y = gyro_y
        imu_msg.angular_velocity.z = gyro_z

        self.publisher_.publish(imu_msg)

def main(args=None):
    rclpy.init(args=args)
    mpu6050_node = MPU6050Node()
    rclpy.spin(mpu6050_node)
    mpu6050_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
