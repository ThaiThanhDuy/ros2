import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import smbus
from time import sleep

class IMUNode(Node):
    def __init__(self):
        super().__init__('imu_node')
        self.publisher_ = self.create_publisher(Imu, 'imu/data', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.bus = smbus.SMBus(1)
        self.Device_Address = 0x68
        self.MPU_Init()

    def MPU_Init(self):
        # Wake up the MPU6050 (0x6B is the power management register)
        self.bus.write_byte_data(self.Device_Address, 0x6B, 0)

    def read_raw_data(self, addr):
        # Read raw 16-bit data from the MPU6050
        high = self.bus.read_byte_data(self.Device_Address, addr)
        low = self.bus.read_byte_data(self.Device_Address, addr + 1)
        value = (high << 8) | low
        return value if value < 32768 else value - 65536

    def timer_callback(self):
        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = 'imu_link'

        # Read raw gyro data
        gyro_x_raw = self.read_raw_data(0x43)  # Gyro X
        gyro_y_raw = self.read_raw_data(0x45)  # Gyro Y
        gyro_z_raw = self.read_raw_data(0x47)  # Gyro Z

        # Convert raw data to float (scale according to your needs)
        gyro_scale = 131.0  # Scale factor for MPU6050 (assuming ±250°/s range)
        imu_msg.angular_velocity.x = float(gyro_x_raw) / gyro_scale
        imu_msg.angular_velocity.y = float(gyro_y_raw) / gyro_scale
        imu_msg.angular_velocity.z = float(gyro_z_raw) / gyro_scale

        self.publisher_.publish(imu_msg)

def main(args=None):
    rclpy.init(args=args)
    imu_node = IMUNode()
    rclpy.spin(imu_node)
    imu_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
