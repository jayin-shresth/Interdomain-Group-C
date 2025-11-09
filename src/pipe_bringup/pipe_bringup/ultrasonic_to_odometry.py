#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, PoseWithCovariance, Quaternion
from std_msgs.msg import Header
import math

class UltrasonicToOdometry(Node):
    def __init__(self):
        super().__init__('ultrasonic_to_odometry')

        # Subscribe to ultrasonic sensors
        self.sub_fl = self.create_subscription(Range, '/sensors/front_left', self.fl_callback, 10)
        self.sub_fr = self.create_subscription(Range, '/sensors/front_right', self.fr_callback, 10)
        self.sub_l = self.create_subscription(Range, '/sensors/left', self.l_callback, 10)
        self.sub_r = self.create_subscription(Range, '/sensors/right', self.r_callback, 10)

        # Publisher for odometry
        self.odom_pub = self.create_publisher(Odometry, '/ultrasonic_odom', 10)

        # Latest distances
        self.dist_fl = None
        self.dist_fr = None
        self.dist_l = None
        self.dist_r = None

        # Timer for processing and publishing
        self.timer = self.create_timer(0.1, self.publish_odometry)

    def fl_callback(self, msg):
        self.dist_fl = msg.range

    def fr_callback(self, msg):
        self.dist_fr = msg.range

    def l_callback(self, msg):
        self.dist_l = msg.range

    def r_callback(self, msg):
        self.dist_r = msg.range

    def publish_odometry(self):
        # Only publish if all sensors data are available
        if None in (self.dist_fl, self.dist_fr, self.dist_l, self.dist_r):
            return

        # Example estimation: rough lateral position based on side sensors
        # and forward position estimated as average of front sensors
        x = (self.dist_fl + self.dist_fr) / 2.0
        y = (self.dist_l - self.dist_r) / 2.0  # Assuming center is zero
        # Assume orientation (theta) zero for simplicity
        theta = 0.0

        # Create Odometry message
        odom = Odometry()
        odom.header = Header()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"

        # Fill pose
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = 0.0

        # Convert theta to quaternion
        quat = Quaternion()
        quat.w = math.cos(theta / 2.0)
        quat.x = 0.0
        quat.y = 0.0
        quat.z = math.sin(theta / 2.0)
        pose.orientation = quat

        odom.pose.pose = pose

        # Optionally fill covariance here
        # For now, leave zeros (unknown covariance)

        # Publish odometry message
        self.odom_pub.publish(odom)

def main(args=None):
    rclpy.init(args=args)
    node = UltrasonicToOdometry()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

