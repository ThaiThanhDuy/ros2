import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import tf2_ros
import geometry_msgs.msg

class ImuTfBroadcaster(Node):
    def __init__(self):
        super().__init__('imu_tf_broadcaster')
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.subscription = self.create_subscription(
            Imu,
            'imu_data',  # Change this to your IMU data topic
            self.listener_callback,
            10
        )

    def listener_callback(self, msg):
        # Create and publish the transform
        t = geometry_msgs.msg.TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'base_link'  # Change this to your base frame
        t.child_frame_id = 'imu_link'  # Change this to your IMU frame
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation.x = msg.orientation.x
        t.transform.rotation.y = msg.orientation.y
        t.transform.rotation.z = msg.orientation.z
        t.transform.rotation.w = msg.orientation.w

        self.tf_broadcaster.sendTransform(t)
        self.get_logger().info('Published TF for IMU')

def main(args=None):
    rclpy.init(args=args)
    imu_tf_broadcaster = ImuTfBroadcaster()
    rclpy.spin(imu_tf_broadcaster)
    imu_tf_broadcaster.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
