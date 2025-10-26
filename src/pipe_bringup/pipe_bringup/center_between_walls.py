#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class CenteringController(Node):
    def __init__(self):
        super().__init__('centering_controller')
        time.sleep(1.0)  # wait gazebo
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 100)
        time.sleep(0.5)  # wait connect

        for i in range(5):  # test msgs
            cmd = Twist()
            cmd.linear.x = 0.3
            self.cmd_pub.publish(cmd)
            self.get_logger().info(f'Initial publish #{i+1}')
            time.sleep(0.1)

        self.timer = self.create_timer(0.2, self.control_loop)
        self.counter = 0
        self.get_logger().info('Controller ready')

    def control_loop(self):
        self.counter += 1
        cmd = Twist()
        cmd.linear.x = 0.3
        cmd.angular.z = 0.0
        self.cmd_pub.publish(cmd)
        if self.counter % 5 == 0:
            self.get_logger().info(f'Publishing #{self.counter}')

def main(args=None):
    rclpy.init(args=args)
    controller = CenteringController()
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

