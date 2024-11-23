import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Imu

class ImuPublisher(Node):
    def __init__(self):
        super().__init__('imu_publisher')  # Node name
        self.imu_pub = self.create_publisher(Imu, 'imu_data', 50)
        self.imu_sub = self.create_subscription(Float32MultiArray, 'imupub', self.imu_callback, 10)

    def imu_callback(self, msg):
        # Print the received data to the console
         self.get_logger().info(f'Received IMU data: {msg.data}')
        if len(msg.data) < 10:
            self.get_logger().warn('Received message with insufficient data. Expected at least 10 elements.')
            return

        current_time = self.get_clock().now().to_msg()

        imu = Imu()
        imu.header.stamp = current_time
        imu.header.frame_id = 'base_link'

        # Assuming the first 4 values are for orientation (x, y, z, w)
        imu.orientation.x = msg.data[0]
        imu.orientation.y = msg.data[1]
        imu.orientation.z = msg.data[2]
        imu.orientation.w = msg.data[3]

        # Next 3 values are for angular velocity (x, y, z)
        imu.angular_velocity.x = msg.data[4]
        imu.angular_velocity.y = msg.data[5]
        imu.angular_velocity.z = msg.data[6]

        # Last 3 values are for linear acceleration (x, y, z)
        imu.linear_acceleration.x = msg.data[7]
        imu.linear_acceleration.y = msg.data[8]
        imu.linear_acceleration.z = msg.data[9]

        self.imu_pub.publish(imu)

def main(args=None):
    rclpy.init(args=args)
    imu_publisher = ImuPublisher()
    rclpy.spin(imu_publisher)
    imu_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
