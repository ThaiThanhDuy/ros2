import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import smbus
import math
import tf2_ros
import geometry_msgs.msg

class MPU6050Node(Node):
    def __init__(self):
        super().__init__('mpu6050_node')
        self.publisher_ = self.create_publisher(Imu, 'imu/data', 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
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

        # Convert raw values to g and rad/s
        accel_x = accel_x / 16384.0  # Assuming the MPU6050 is set to ±2g
        accel_y = accel_y / 16384.0
        accel_z = accel_z / 16384.0

        gyro_x = gyro_x * (math.pi / 180)  # Convert degrees/sec to rad/sec
        gyro_y = gyro_y * (math.pi / 180)
        gyro_z = gyro_z * (math.pi / 180)

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
        imu_msg.linear_acceleration.x = float(accel_x)
        imu_msg.linear_acceleration.y = float(accel_y)
        imu_msg.linear_acceleration.z = float(accel_z)
        imu_msg.angular_velocity.x = float(gyro_x)
        imu_msg.angular_velocity.y = float(gyro_y)
        imu_msg.angular_velocity.z = float(gyro_z)

        self.publisher_.publish(imu_msg)

        # Calculate orientation from accelerometer data
        roll = math.atan2(accel_y, accel_z)
        pitch = math.atan2(-accel_x, math.sqrt(accel_y**2 + accel_z**2))
        yaw = 0.0  # You can set this to a fixed value or integrate gyroscope data for yaw

        # Convert to quaternion
        qx, qy, qz, qw = self.euler_to_quaternion(roll, pitch, yaw)

        # Publish transform
        self.publish_transform(qx, qy, qz, qw)

    def euler_to_quaternion(self, roll, pitch, yaw ):
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        qw = cr * cp * cy + sr * sp * sy
        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy

        return qx, qy, qz, qw

    def publish_transform(self, qx, qy, qz, qw):
        t = geometry_msgs.msg.TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'base_link'  # Base frame
        t.child_frame_id = 'imu_link'  # IMU frame

        # Set translation (you can set this to the actual position if needed)
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0

        # Set rotation
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw

        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    mpu6050_node = MPU6050Node()
    rclpy.spin(mpu6050_node)
    mpu6050_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
