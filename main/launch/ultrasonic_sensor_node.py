import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import RPi.GPIO as GPIO
import time
import tf2_ros
import geometry_msgs.msg

class UltrasonicSensorNode(Node):
    def __init__(self):
        super().__init__('ultrasonic_sensor_node')
        self.publisher_ = self.create_publisher(Float32, 'ultrasonic_distance', 10)
        
        # Initialize TF broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        GPIO.setmode(GPIO.BCM)
        self.TRIG = 18
        self.ECHO = 24
        GPIO.setup(self.TRIG, GPIO.OUT)
        GPIO.setup(self.ECHO, GPIO.IN)
        self.timer = self.create_timer(1.0, self.timer_callback)

    def measure_distance(self):
        GPIO.output(self.TRIG, True)
        time.sleep(0.00001)
        GPIO.output(self.TRIG, False)

        start_time = time.time()
        stop_time = time.time()

        while GPIO.input(self.ECHO) == 0:
            start_time = time.time()

        while GPIO.input(self.ECHO) == 1:
            stop_time = time.time()

        time_elapsed = stop_time - start_time
        distance = (time_elapsed * 34300) / 2
        return distance

    def timer_callback(self):
        distance = self.measure_distance()
        
        # Publish distance
        msg = Float32()
        msg.data = distance
        self.publisher_.publish(msg)
        self.get_logger().info(f'Distance: {distance:.2f} cm')

        # Create and publish the transform
        self.publish_transform(distance)

    def publish_transform(self, distance):
        # Create a transform message
        t = geometry_msgs.msg.TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'base_link'  # The frame you want to attach to
        t.child_frame_id = 'ultrasonic_sensor'  # The frame for the sensor
        t.transform.translation.x = distance / 100.0  # Convert cm to meters
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0  # No rotation

        # Send the transform
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = UltrasonicSensorNode()
    rclpy.spin(node)
    GPIO.cleanup()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
  
