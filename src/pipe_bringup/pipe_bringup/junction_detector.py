#!/usr/bin/env python3
"""
Junction Detector Node
Monitors three distance sensors (front, left, right) and identifies junction types.
Publishes junction events with cooldown to prevent spam.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String


class JunctionDetector(Node):
    """Detects junctions in pipe network based on sensor readings"""
    
    def __init__(self):
        super().__init__('junction_detector')
        
        # === Configuration Parameters ===
        self.OPENING_THRESHOLD = 1.5  # Distance threshold to consider a path "open" (meters)
        self.DETECTION_COOLDOWN = 2.0  # Minimum time between detections (seconds)
        
        # === Sensor State ===
        self.front_distance = None
        self.left_distance = None
        self.right_distance = None
        self.last_detection_time = None
        
        # === ROS Setup ===
        self._setup_subscribers()
        self._setup_publishers()
        
        self._log_startup_info()
    
    def _setup_subscribers(self):
        """Initialize sensor topic subscriptions"""
        self.front_sub = self.create_subscription(
            LaserScan, 'front_scan', self._on_front_sensor_update, 10)
        self.left_sub = self.create_subscription(
            LaserScan, 'left_scan', self._on_left_sensor_update, 10)
        self.right_sub = self.create_subscription(
            LaserScan, 'right_scan', self._on_right_sensor_update, 10)
    
    def _setup_publishers(self):
        """Initialize junction event publisher"""
        self.junction_pub = self.create_publisher(String, 'junction_detected', 10)
    
    def _log_startup_info(self):
        """Log node configuration on startup"""
        self.get_logger().info('🔍 Junction Detector Started')
        self.get_logger().info(f'   Sensor topics: front_scan, left_scan, right_scan')
        self.get_logger().info(f'   Opening threshold: {self.OPENING_THRESHOLD}m')
        self.get_logger().info(f'   Cooldown period: {self.DETECTION_COOLDOWN}s')
    
    # === Sensor Callbacks ===
    
    def _on_front_sensor_update(self, msg):
        """Handle front sensor data"""
        if msg.ranges:
            self.front_distance = min(msg.ranges)
            self._check_for_junction()
    
    def _on_left_sensor_update(self, msg):
        """Handle left sensor data"""
        if msg.ranges:
            self.left_distance = min(msg.ranges)
            self._check_for_junction()
    
    def _on_right_sensor_update(self, msg):
        """Handle right sensor data"""
        if msg.ranges:
            self.right_distance = min(msg.ranges)
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
        """Check if all three sensors have reported data"""
        return all([
            self.front_distance is not None,
            self.left_distance is not None,
            self.right_distance is not None
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
        # Determine which directions are "open"
        front_open = self._is_path_open(self.front_distance)
        left_open = self._is_path_open(self.left_distance)
        right_open = self._is_path_open(self.right_distance)
        
        # Match pattern to junction type
        if front_open and left_open and right_open:
            return 'CROSS'
        elif front_open and left_open and not right_open:
            return 'T_LEFT'
        elif front_open and right_open and not left_open:
            return 'T_RIGHT'
        elif not front_open and left_open and not right_open:
            return 'LEFT_TURN'
        elif not front_open and right_open and not left_open:
            return 'RIGHT_TURN'
        else:
            return None  # No junction detected
    
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
        self.get_logger().info(f'🔍 JUNCTION: {junction_type}')
        self.get_logger().info(
            f'📊 Distances - F:{self.front_distance:.2f}m '
            f'L:{self.left_distance:.2f}m R:{self.right_distance:.2f}m'
        )
        self.get_logger().info(f'⏱️  Cooldown started ({self.DETECTION_COOLDOWN}s)')


def main(args=None):
    """Main entry point"""
    rclpy.init(args=args)
    detector = JunctionDetector()
    
    try:
        rclpy.spin(detector)
    except KeyboardInterrupt:
        pass
    finally:
        detector.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

