import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import smbus2
import math
import tf2_ros
import geometry_msgs.msg

class MPU6050:
    def __init__(self, bus_number=0, address=0x68):  # Changed bus_number to 0
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
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.mpu = MPU6050()  # Initialize MPU6050 with bus_number=0

        # Initialize orientation variables
        self.angle_x = 0.0
        self.angle_y = 0.0
        self.angle_z = 0.0
        self.alpha = 0.98  # Complementary filter constant

        self.timer = self.create_timer(0.1, self.publish_imu_data)  # Publish every 100 ms

    def publish_imu_data(self):
        accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z = self.mpu.read_raw_data()

        # Convert gyro rates from degrees/sec to radians/sec
        gyro_x_rad = gyro_x * (math.pi / 180)
        gyro_y_rad = gyro_y * (math.pi / 180)
        gyro_z_rad = gyro_z * (math.pi / 180)

        # Simple complementary filter to estimate orientation
        dt = 0.1  # Time interval (100 ms)
        self.angle_x = self.alpha * (self.angle_x + gyro_x_rad * dt) + (1 - self.alpha) * math.atan2(accel_y, accel_z)
        self.angle_y = self.alpha * (self.angle_y + gyro_y_rad * dt) + (1 - self.alpha) * math.atan2(-accel_x, accel_z)
        self.angle_z += gyro_z_rad * dt  # Assuming no drift correction on Z

        # Convert angles to quaternion
        qx = math.sin(self.angle_x / 2)
        qy = math.sin(self.angle_y / 2)
        qz = math.sin(self.angle_z / 2)  # Add this line to calculate qz
        qw = math.cos(self.angle_x / 2) * math.cos(self.angle_y / 2) * math.cos(self.angle_z / 2) + math.sin(self.angle_x / 2) * math.sin(self.angle_y / 2) * math.sin(self.angle_z / 2)

        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = 'base_link'

        # Set calculated orientation as quaternion
        imu_msg.orientation.x = qx
        imu_msg.orientation .y = qy
        imu_msg.orientation.z = qz  # Set the calculated qz
        imu_msg.orientation.w = qw

        # Set angular velocity
        imu_msg.angular_velocity.x = gyro_x_rad
        imu_msg.angular_velocity.y = gyro_y_rad
        imu_msg.angular_velocity.z = gyro_z_rad

        # Set linear acceleration
        imu_msg.linear_acceleration.x = accel_x / 16384.0  # Convert to g
        imu_msg.linear_acceleration.y = accel_y / 16384.0
        imu_msg.linear_acceleration.z = accel_z / 16384.0

        self.imu_pub.publish(imu_msg)
        self.get_logger().info(f'Published IMU data: {imu_msg}')

        # Publish the transform
        self.publish_transform(qx, qy, qz, qw)

    def publish_transform(self, qx, qy, qz, qw):
        t = geometry_msgs.msg.TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'base_link'
        t.child_frame_id = 'imu_link'
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw

        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    imu_publisher = ImuPublisher()
    rclpy.spin(imu_publisher)
    imu_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
