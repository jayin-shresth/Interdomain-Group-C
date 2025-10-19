#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
from geometry_msgs.msg import Twist
import math

class CenteringController(Node):
    def __init__(self):
        super().__init__('centering_controller')
        
        self.left_sub = self.create_subscription(Range, '/pipe_bot/ultrasonic_left', self.left_callback, 10)
        self.right_sub = self.create_subscription(Range, '/pipe_bot/ultrasonic_right', self.right_callback, 10)
        self.front_sub = self.create_subscription(Range, '/pipe_bot/ultrasonic_front', self.front_callback, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.05, self.control_loop)
        
        self.left_distance = None
        self.right_distance = None
        self.front_distance = None
        
        self.target_center_tolerance = 0.05
        self.forward_speed = 0.2
        self.centering_gain = 0.5
        self.min_front_distance = 0.3
        
        self.get_logger().info('Centering Controller Started!')
        self.get_logger().info('Robot will center itself between walls and move forward')
    
    def left_callback(self, msg):
        self.left_distance = msg.range
    
    def right_callback(self, msg):
        self.right_distance = msg.range
    
    def front_callback(self, msg):
        self.front_distance = msg.range
    
    def control_loop(self):
        # Wait for sensors
        if self.left_distance is None or self.right_distance is None or self.front_distance is None:
            self.get_logger().info('Waiting for sensor data...', throttle_duration_sec=2.0)
            return
        
        cmd = Twist()
        centering_error = self.right_distance - self.left_distance
        
        self.get_logger().info(
            f'Left: {self.left_distance:.2f}m | Right: {self.right_distance:.2f}m | '
            f'Front: {self.front_distance:.2f}m | Error: {centering_error:.2f}m',
            throttle_duration_sec=1.0
        )
        
        # Obstacle check
        if self.front_distance < self.min_front_distance:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.get_logger().warn(f'Obstacle detected at {self.front_distance:.2f}m! Stopping.', throttle_duration_sec=1.0)
        else:
            cmd.linear.x = self.forward_speed
            cmd.angular.z = self.centering_gain * centering_error
            
            # Clamp angular
            max_angular_vel = 1.0
            cmd.angular.z = max(min(cmd.angular.z, max_angular_vel), -max_angular_vel)
            
            if abs(centering_error) < self.target_center_tolerance:
                self.get_logger().info('✓ Centered!', throttle_duration_sec=2.0)
            else:
                direction = "left" if centering_error < 0 else "right"
                self.get_logger().info(f'Centering... (adjusting {direction})', throttle_duration_sec=1.0)
        
        self.cmd_vel_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    centering_controller = CenteringController()
    try:
        rclpy.spin(centering_controller)
    except KeyboardInterrupt:
        pass
    centering_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

