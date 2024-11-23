import rclpy
from rclpy.node import Node
from rclpy.time import Time
from geometry_msgs.msg import Quaternion, TransformStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Int16MultiArray
import tf2_ros
import tf_transformations
import math

class OdometryPublisher(Node):
    def __init__(self):
        super().__init__('odometry_publisher')
        
        self.odom_pub = self.create_publisher(Odometry, 'odom', 50)
        self.quadenc_sub = self.create_subscription(Int16MultiArray, 'quadenc', self.quadenc_callback, 50)

        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.qe1 = 0.0
        self.qe2 = 0.0
        self.dt = 0.0
        self.dx = 0.0
        self.dy = 0.0
        self.dth = 0.0
        self.dist = 0.0
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.vx = 0.0
        self.vy = 0.0
        self.vth = 0.0

        self.current_time = self.get_clock().now()
        self.last_time = self.get_clock().now()

    def quadenc_callback(self, msg):
        self.current_time = self.get_clock().now()
        
        self.qe1 = msg.data[0] * 0.00007124683339  # Convert counts to meters
        self.qe2 = msg.data[1] * 0.00007121120998  # Convert counts to meters

        self.dist = (self.qe1 + self.qe2) / 2.0
        self.dth = (self.qe1 - self.qe2) / 2.0
        self.dth /= 0.147701  # Convert to radians of rotation

        self.dx = self.dist * math.cos(self.th)
        self.dy = self.dist * math.sin(self.th)
        self.x += self.dx
        self.y += self.dy
        self.th += self.dth

        # Create quaternion from yaw
        odom_quat = tf_transformations.quaternion_from_euler(0, 0, self.th)

        # Calculate velocities
        self.dt = (self.current_time - self.last_time).nanoseconds * 1e-9  # Convert to seconds
        self.vx = self.dist / self.dt if self.dt > 0 else 0.0
        self.vy = 0.0
        self.vth = self.dth / self.dt if self.dt > 0 else 0.0

        # Create and publish odometry message
        odom = Odometry()
        odom.header.stamp = self.current_time
        odom.header.frame_id = "odom"
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = Quaternion(*odom_quat)

        odom.child_frame_id = "base_link"
        odom.twist.twist.linear.x = self.vx
        odom.twist.twist.linear.y = self.vy
        odom.twist.twist.angular.z = self.vth

        self.odom_pub.publish(odom)

        # Publish the transform
        t = TransformStamped()
        t.header.stamp = self.current_time
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation = Quaternion(*odom_quat)

        self.tf_broadcaster.sendTransform(t)

        self.last_time = self.current_time

def main(args=None):
    rclpy.init(args=args)
    odometry_publisher = OdometryPublisher()
    rclpy.spin(odometry_publisher)
    odometry_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
