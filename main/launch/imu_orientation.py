import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TransformStamped
import numpy as np
import time

class ImuOrientationSimulator(Node):
    def __init__(self):
        super().__init__('imu_orientation_simulator')
        self.imu_publisher = self.create_publisher(Imu, 'imu/data', 10)
        self.transform_publisher = self.create_publisher(TransformStamped, 'imu_transform', 10)
        self.timer = self.create_timer(0.1, self.publish_imu_data)  # Publish at 10 Hz

        # Initialize angles
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0

        # Time variables
        self.last_time = self.get_clock().now().nanoseconds / 1e9  # Convert to seconds

    def publish_imu_data(self):
        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = 'imu_link'

        # Simulate gyroscope data (roll, pitch, yaw)
        gyro_x = np.random.uniform(-1, 1)  # Simulated gyro x-axis (rad/s)
        gyro_y = np.random.uniform(-1, 1)  # Simulated gyro y-axis (rad/s)
        gyro_z = np.random.uniform(-1, 1)  # Simulated gyro z-axis (rad/s)

        # Fill in the gyroscope data
        imu_msg.angular_velocity.x = gyro_x
        imu_msg.angular_velocity.y = gyro_y
        imu_msg.angular_velocity.z = gyro_z

        # Calculate the time difference
        current_time = self.get_clock().now().nanoseconds / 1e9  # Convert to seconds
        dt = current_time - self.last_time
        self.last_time = current_time

        # Integrate gyroscope data to get roll, pitch, yaw
        self.roll += gyro_x * dt
        self.pitch += gyro_y * dt
        self.yaw += gyro_z * dt

        # Log the angles
        self.get_logger().info(f'Roll: {np.degrees(self.roll):.2f}, Pitch: {np.degrees(self.pitch):.2f}, Yaw: {np.degrees(self.yaw):.2f}')

        # Publish the IMU message
        self.imu_publisher.publish(imu_msg)

        # Create and publish the transform
        transform_msg = TransformStamped()
        transform_msg.header.stamp = self.get_clock().now().to_msg()
        transform_msg.header.frame_id = 'world'
        transform_msg.child_frame_id = 'imu_link'

        # Set the translation (position)
        transform_msg.transform.translation.x = 0.0
        transform_msg.transform.translation.y = 0.0
        transform_msg.transform.translation.z = 0.0

        # Set the rotation (orientation)
        q = self.euler_to_quaternion(self.roll, self.pitch, self.yaw)
        transform_msg.transform.rotation.x = q[0]
        transform_msg.transform.rotation.y = q[1]
        transform_msg.transform.rotation.z = q[2]
        transform_msg.transform.rotation.w = q[3]

        # Publish the transform
        self.transform_publisher.publish(transform_msg)

    def euler_to_quaternion(self, roll, pitch, yaw):
        """Convert Euler angles to quaternion."""
        qx = np.sin(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) - np.cos(roll / 2) * np.sin(pitch / 2) * np.sin(yaw / 2)
        qy = np.cos(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2) + np.sin(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2)
        qz = np.cos(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2) - np.sin(roll
