#!/usr/bin/env python3
"""
Center Between Walls Node - Hardware Version
Reads 4 ultrasonic sensors and keeps robot centered in pipe
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range
import time

class CenteringController(Node):
    def __init__(self):
        super().__init__('centering_controller')
        
        # Parameters
        self.declare_parameter('stop_distance', 0.10)  # 10cm
        self.declare_parameter('side_distance', 0.08)   # 8cm
        self.declare_parameter('speed', 0.3)
        
        self.stop_distance = self.get_parameter('stop_distance').value
        self.side_distance = self.get_parameter('side_distance').value
        self.base_speed = self.get_parameter('speed').value
        
        # Publisher
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Subscribers - 4 ultrasonic sensors
        self.sub_front_left = self.create_subscription(
            Range, '/sensors/front_left', self.front_left_callback, 10)
        self.sub_front_right = self.create_subscription(
            Range, '/sensors/front_right', self.front_right_callback, 10)
        self.sub_left = self.create_subscription(
            Range, '/sensors/left', self.left_callback, 10)
        self.sub_right = self.create_subscription(
            Range, '/sensors/right', self.right_callback, 10)
        
        # Sensor data storage
        self.front_left_dist = 2.0   # Default: 200cm
        self.front_right_dist = 2.0
        self.left_dist = 2.0
        self.right_dist = 2.0
        
        # Control loop timer
        self.timer = self.create_timer(0.1, self.control_loop)  # 10Hz
        
        self.get_logger().info('=== Centering Controller Started ===')
        self.get_logger().info(f'Stop distance: {self.stop_distance}m')
        self.get_logger().info(f'Side distance: {self.side_distance}m')
        self.get_logger().info(f'Base speed: {self.base_speed}m/s')
    
    # Sensor callbacks
    def front_left_callback(self, msg):
        self.front_left_dist = msg.range
    
    def front_right_callback(self, msg):
        self.front_right_dist = msg.range
    
    def left_callback(self, msg):
        self.left_dist = msg.range
    
    def right_callback(self, msg):
        self.right_dist = msg.range
    
    def control_loop(self):
        """Main control logic - runs at 10Hz"""
        
        # Get minimum front distance
        min_front = min(self.front_left_dist, self.front_right_dist)
        
        # Create velocity command
        cmd = Twist()
        
        # Check for wall ahead
        if min_front < self.stop_distance:
            # STOP - wall detected
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.get_logger().warn(f'WALL AHEAD! Distance: {min_front:.2f}m - STOPPING')
        
        else:
            # Calculate centering error
            center_error = self.left_dist - self.right_dist
            
            # Decision logic
            if abs(center_error) > self.side_distance:
                # Need centering adjustment
                if center_error > 0:
                    # Too close to right wall - move left
                    cmd.linear.x = self.base_speed * 0.7
                    cmd.angular.z = 0.3  # Turn left
                    status = "ADJUSTING LEFT"
                else:
                    # Too close to left wall - move right
                    cmd.linear.x = self.base_speed * 0.7
                    cmd.angular.z = -0.3  # Turn right
                    status = "ADJUSTING RIGHT"
            else:
                # Well centered - move forward
                cmd.linear.x = self.base_speed
                cmd.angular.z = 0.0
                status = "CENTERED - FORWARD"
            
            # Log status
            self.get_logger().info(
                f'FL:{self.front_left_dist:.2f} FR:{self.front_right_dist:.2f} '
                f'L:{self.left_dist:.2f} R:{self.right_dist:.2f} | {status}'
            )
        
        # Publish command
        self.cmd_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    controller = CenteringController()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        # Stop robot before exit
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        controller.cmd_pub.publish(cmd)
        controller.get_logger().info('Shutting down - robot stopped')
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

