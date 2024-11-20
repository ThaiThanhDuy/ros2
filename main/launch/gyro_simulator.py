import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import numpy as np
import time

class GyroSimulator(Node):
    def __init__(self):
        super().__init__('gyro_simulator')
        self.publisher_ = self.create_publisher(Imu, 'imu/data', 10)
        self.timer = self.create_timer(0.1, self.publish_imu_data)  # Publish at 10 Hz
        self.angle = 0.0  # Initial angle

    def publish_imu_data(self):
        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = 'imu_link'

        # Simulate gyroscope data (roll, pitch, yaw)
        self.angle += 0.1  # Simulate rotation
        gyro_x = np.sin(self.angle) * 100  # Simulated gyro x-axis
        gyro_y = np.cos(self.angle) * 100  # Simulated gyro y-axis
        gyro_z = 0.0  # No rotation around z-axis

        # Fill in the gyroscope data
        imu_msg.angular_velocity.x = gyro_x
        imu_msg.angular_velocity.y = gyro_y
        imu_msg.angular_velocity.z = gyro_z

        # Optionally, you can set linear acceleration data
        imu_msg.linear_acceleration.x = 0.0
        imu_msg.linear_acceleration.y = 0.0
        imu_msg.linear_acceleration.z = 9.81  # Gravity

        self.publisher_.publish(imu_msg)
        self.get_logger().info(f'Publishing IMU data: {gyro_x}, {gyro_y}, {gyro_z}')

def main(args=None):
    rclpy.init(args=args)
    gyro_simulator = GyroSimulator()
    rclpy.spin(gyro_simulator)
    gyro_simulator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
