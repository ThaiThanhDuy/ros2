import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Point

class Mpu6050AngularVelocityMarker(Node):
    def __init__(self):
        super().__init__('mpu6050_angular_velocity_marker')
        self.marker_publisher = self.create_publisher(Marker, '/mpu6050_marker', 10)
        self.imu_subscriber = self.create_subscription(Imu, '/imu/data', self.imu_callback, 10)
        self.timer = self.create_timer(0.1, self.publish_marker)

        self.angular_velocity = None

    def imu_callback(self, msg):
        # Store the angular velocity from the IMU message
        self.angular_velocity = msg.angular_velocity

    def publish_marker(self):
        if self.angular_velocity is None:
            return  # No data yet

        # Create a marker for the angular velocity vector
        marker = Marker()
        marker.header.frame_id = "imu_link"  # Change to your frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "mpu6050_angular_velocity"
        marker.id = 0
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD
        marker.scale.x = 0.02  # Line width

        # Define the origin point
        origin = Point(0.0, 0.0, 0.0)

        # Define the end point based on angular velocity
        end_point = Point(
            origin.x + self.angular_velocity.x,
            origin.y + self.angular_velocity.y,
            origin.z + self.angular_velocity.z
        )

        # Add points for the line
        marker.points.append(origin)
        marker.points.append(end_point)

        # Set the color of the line (e.g., red)
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0  # Alpha (transparency)

        # Publish the marker
        self.marker_publisher.publish(marker)

def main(args=None):
    rclpy.init(args=args)
    node = Mpu6050AngularVelocityMarker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
