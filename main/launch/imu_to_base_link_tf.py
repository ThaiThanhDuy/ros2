import rclpy
from rclpy.node import Node
import tf2_ros
import geometry_msgs.msg
import math

class ImuToBaseLinkTF(Node):
    def __init__(self):
        super().__init__('imu_to_base_link_tf')
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.timer = self.create_timer(0.1, self.publish_transform)  # Publish at 10 Hz

    def publish_transform(self):
        # Create a transform message
        t = geometry_msgs.msg.TransformStamped()
        
        # Set the header
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'base_link'  # Parent frame
        t.child_frame_id = 'imu_link'     # Child frame
        
        # Set the translation (x, y, z)
        t.transform.translation.x = 0.0  # Adjust as necessary
        t.transform.translation.y = 0.0  # Adjust as necessary
        t.transform.translation.z = 0.0  # Adjust as necessary
        
        # Set the rotation (quaternion)
        # For example, if the IMU is aligned with the base link:
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0  # No rotation
        
        # Publish the transform
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = ImuToBaseLinkTF()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
