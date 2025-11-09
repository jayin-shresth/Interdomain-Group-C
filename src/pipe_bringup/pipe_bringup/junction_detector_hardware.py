#!/usr/bin/env python3
"""
Junction Detector Node - Updated for 4 Ultrasonic Sensors
Monitors 4 ultrasonic sensors and identifies junction types.
Works with /sensors/front_left, /sensors/front_right, /sensors/left, /sensors/right
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
from std_msgs.msg import String


class JunctionDetector(Node):
    """Detects junctions in pipe network based on ultrasonic sensor readings"""
    
    def __init__(self):
        super().__init__('junction_detector')
        
        # === Configuration Parameters ===
        self.declare_parameter('opening_threshold', 0.50)  # 50cm - wider opening = junction
        self.declare_parameter('cooldown', 3.0)  # 3 seconds cooldown
        
        self.OPENING_THRESHOLD = self.get_parameter('opening_threshold').value
        self.DETECTION_COOLDOWN = self.get_parameter('cooldown').value
        
        # === Sensor State ===
        self.front_left_dist = None
        self.front_right_dist = None
        self.left_dist = None
        self.right_dist = None
        self.last_detection_time = None
        
        # === ROS Setup ===
        self._setup_subscribers()
        self._setup_publishers()
        
        self._log_startup_info()
    
    def _setup_subscribers(self):
        """Initialize ultrasonic sensor subscriptions"""
        self.sub_fl = self.create_subscription(
            Range, '/sensors/front_left', self._on_front_left_update, 10)
        self.sub_fr = self.create_subscription(
            Range, '/sensors/front_right', self._on_front_right_update, 10)
        self.sub_l = self.create_subscription(
            Range, '/sensors/left', self._on_left_update, 10)
        self.sub_r = self.create_subscription(
            Range, '/sensors/right', self._on_right_update, 10)
    
    def _setup_publishers(self):
        """Initialize junction event publisher"""
        self.junction_pub = self.create_publisher(String, '/junction_detected', 10)
    
    def _log_startup_info(self):
        """Log node configuration on startup"""
        self.get_logger().info('🔍 Junction Detector Started (4 Ultrasonic Sensors)')
        self.get_logger().info(f'   Opening threshold: {self.OPENING_THRESHOLD}m')
        self.get_logger().info(f'   Cooldown period: {self.DETECTION_COOLDOWN}s')
    
    # === Sensor Callbacks ===
    
    def _on_front_left_update(self, msg):
        """Handle front-left ultrasonic sensor"""
        self.front_left_dist = msg.range
        self._check_for_junction()
    
    def _on_front_right_update(self, msg):
        """Handle front-right ultrasonic sensor"""
        self.front_right_dist = msg.range
        self._check_for_junction()
    
    def _on_left_update(self, msg):
        """Handle left ultrasonic sensor"""
        self.left_dist = msg.range
        self._check_for_junction()
    
    def _on_right_update(self, msg):
        """Handle right ultrasonic sensor"""
        self.right_dist = msg.range
        self._check_for_junction()
    
    # === Junction Detection Logic ===
    
    def _check_for_junction(self):
        """
        Main detection routine - checks if current sensor state matches a junction pattern.
        Only publishes if cooldown has expired.
        """
        # Wait for all sensors to have data
        if not self._all_sensors_ready():
            return
        
        # Check cooldown timer
        if not self._cooldown_expired():
            return
        
        # Classify the junction type
        junction_type = self._classify_junction()
        
        # Publish if we found a junction
        if junction_type:
            self._publish_junction(junction_type)
    
    def _all_sensors_ready(self):
        """Check if all four sensors have reported data"""
        return all([
            self.front_left_dist is not None,
            self.front_right_dist is not None,
            self.left_dist is not None,
            self.right_dist is not None
        ])
    
    def _cooldown_expired(self):
        """Check if enough time has passed since last detection"""
        if self.last_detection_time is None:
            return True
        
        elapsed = (self.get_clock().now() - self.last_detection_time).nanoseconds / 1e9
        return elapsed >= self.DETECTION_COOLDOWN
    
    def _classify_junction(self):
        """
        Analyze sensor readings to determine junction type.
        Returns junction type string or None if no junction detected.
        """
        # Average front sensors to get overall front distance
        front_dist = (self.front_left_dist + self.front_right_dist) / 2.0
        
        # Determine which directions are "open"
        front_open = self._is_path_open(front_dist)
        left_open = self._is_path_open(self.left_dist)
        right_open = self._is_path_open(self.right_dist)
        
        # Must have at least one side opening to be a junction
        if not (left_open or right_open):
            return None
        
        # Match pattern to junction type
        if left_open and right_open:
            if front_open:
                return 'CROSS'  # 4-way intersection
            else:
                return 'T_BOTH'  # T-junction (can go left or right, wall ahead)
        elif left_open and not right_open:
            return 'LEFT_TURN'  # Left opening only
        elif right_open and not left_open:
            return 'RIGHT_TURN'  # Right opening only
        else:
            return None  # No junction
    
    def _is_path_open(self, distance):
        """Determine if a path is open based on distance threshold"""
        return distance > self.OPENING_THRESHOLD
    
    def _publish_junction(self, junction_type):
        """Publish junction detection event and start cooldown"""
        # Create and publish message
        msg = String()
        msg.data = junction_type
        self.junction_pub.publish(msg)
        
        # Reset cooldown timer
        self.last_detection_time = self.get_clock().now()
        
        # Log detection
        self.get_logger().warn(f'🚦 JUNCTION DETECTED: {junction_type}')
        self.get_logger().info(
            f'📊 Distances - FL:{self.front_left_dist:.2f}m FR:{self.front_right_dist:.2f}m '
            f'L:{self.left_dist:.2f}m R:{self.right_dist:.2f}m'
        )


def main(args=None):
    """Main entry point"""
    rclpy.init(args=args)
    detector = JunctionDetector()
    
    try:
        rclpy.spin(detector)
    except KeyboardInterrupt:
        detector.get_logger().info('Junction detector shutting down...')
    finally:
        detector.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

