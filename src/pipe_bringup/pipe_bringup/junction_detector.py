#!/usr/bin/env python3
"""
Junction Detector Node - SIMULATION VERSION
Monitors three LaserScan sensors (front, left, right) and identifies junction types.
Improved with better detection logic and configurable parameters.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Twist


class JunctionDetectorSim(Node):
    """Detects junctions in pipe network using LaserScan sensors (Gazebo)"""
    
    def __init__(self):
        super().__init__('junction_detector')
        
        # === Configuration Parameters ===
        self.declare_parameter('opening_threshold', 1.5)  # Meters
        self.declare_parameter('cooldown', 2.0)  # Seconds
        self.declare_parameter('min_opening_angle', 30.0)  # Degrees
        self.declare_parameter('stop_at_junction', False)  # Auto-stop at junctions
        
        self.OPENING_THRESHOLD = self.get_parameter('opening_threshold').value
        self.DETECTION_COOLDOWN = self.get_parameter('cooldown').value
        self.MIN_OPENING_ANGLE = self.get_parameter('min_opening_angle').value
        self.STOP_AT_JUNCTION = self.get_parameter('stop_at_junction').value
        
        # === Sensor State ===
        self.front_distance = None
        self.left_distance = None
        self.right_distance = None
        self.last_detection_time = None
        self.in_junction = False
        
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
        """Initialize junction event publishers"""
        self.junction_pub = self.create_publisher(String, '/junction_detected', 10)
        self.junction_active_pub = self.create_publisher(Bool, '/junction_active', 10)
        
        if self.STOP_AT_JUNCTION:
            self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
    
    def _log_startup_info(self):
        """Log node configuration on startup"""
        self.get_logger().info('🔍 Junction Detector Started (SIMULATION)')
        self.get_logger().info(f'   Sensor topics: front_scan, left_scan, right_scan')
        self.get_logger().info(f'   Opening threshold: {self.OPENING_THRESHOLD}m')
        self.get_logger().info(f'   Min opening angle: {self.MIN_OPENING_ANGLE}°')
        self.get_logger().info(f'   Cooldown period: {self.DETECTION_COOLDOWN}s')
        self.get_logger().info(f'   Stop at junction: {self.STOP_AT_JUNCTION}')
    
    # === Sensor Callbacks ===
    
    def _on_front_sensor_update(self, msg):
        """Handle front sensor data with improved filtering"""
        if msg.ranges:
            # Filter out invalid readings and get average
            valid_ranges = [r for r in msg.ranges if msg.range_min < r < msg.range_max]
            if valid_ranges:
                self.front_distance = sum(valid_ranges) / len(valid_ranges)
                self._check_for_junction()
    
    def _on_left_sensor_update(self, msg):
        """Handle left sensor data with improved filtering"""
        if msg.ranges:
            valid_ranges = [r for r in msg.ranges if msg.range_min < r < msg.range_max]
            if valid_ranges:
                self.left_distance = sum(valid_ranges) / len(valid_ranges)
                self._check_for_junction()
    
    def _on_right_sensor_update(self, msg):
        """Handle right sensor data with improved filtering"""
        if msg.ranges:
            valid_ranges = [r for r in msg.ranges if msg.range_min < r < msg.range_max]
            if valid_ranges:
                self.right_distance = sum(valid_ranges) / len(valid_ranges)
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
        
        # Classify the junction type
        junction_type = self._classify_junction()
        
        # Update junction active state
        self._update_junction_state(junction_type)
        
        # Only publish new detection if cooldown expired
        if junction_type and self._cooldown_expired():
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
        
        Improved logic:
        - Requires significant opening (> threshold)
        - Filters out narrow gaps
        - Better pattern matching
        """
        # Determine which directions are "open"
        front_open = self._is_path_open(self.front_distance)
        left_open = self._is_path_open(self.left_distance)
        right_open = self._is_path_open(self.right_distance)
        
        # Log current state (debug)
        # self.get_logger().debug(
        #     f'F:{self.front_distance:.2f}({front_open}) '
        #     f'L:{self.left_distance:.2f}({left_open}) '
        #     f'R:{self.right_distance:.2f}({right_open})'
        # )
        
        # Must have at least one side opening to be a junction
        if not (left_open or right_open):
            return None
        
        # Match pattern to junction type
        if front_open and left_open and right_open:
            return 'CROSS'  # 4-way intersection
        elif front_open and left_open and not right_open:
            return 'T_LEFT'  # T-junction with left opening
        elif front_open and right_open and not left_open:
            return 'T_RIGHT'  # T-junction with right opening
        elif not front_open and left_open and right_open:
            return 'T_BOTH'  # Dead end with left and right openings
        elif not front_open and left_open and not right_open:
            return 'LEFT_TURN'  # Left turn only
        elif not front_open and right_open and not left_open:
            return 'RIGHT_TURN'  # Right turn only
        else:
            return None  # Corridor (no junction)
    
    def _is_path_open(self, distance):
        """Determine if a path is open based on distance threshold"""
        if distance is None:
            return False
        return distance > self.OPENING_THRESHOLD
    
    def _update_junction_state(self, junction_type):
        """Track whether robot is currently in a junction"""
        was_in_junction = self.in_junction
        self.in_junction = (junction_type is not None)
        
        # Publish junction active state
        msg = Bool()
        msg.data = self.in_junction
        self.junction_active_pub.publish(msg)
        
        # Log state changes
        if self.in_junction and not was_in_junction:
            self.get_logger().info('▶️  Entering junction area')
        elif not self.in_junction and was_in_junction:
            self.get_logger().info('◀️  Exiting junction area')
    
    def _publish_junction(self, junction_type):
        """Publish junction detection event and start cooldown"""
        # Create and publish junction type message
        msg = String()
        msg.data = junction_type
        self.junction_pub.publish(msg)
        
        # Reset cooldown timer
        self.last_detection_time = self.get_clock().now()
        
        # Log detection with emoji indicators
        emoji_map = {
            'LEFT_TURN': '⬅️',
            'RIGHT_TURN': '➡️',
            'T_LEFT': '⬆️⬅️',
            'T_RIGHT': '⬆️➡️',
            'T_BOTH': '⬅️➡️',
            'CROSS': '✖️'
        }
        emoji = emoji_map.get(junction_type, '🚦')
        
        self.get_logger().warn(f'{emoji} JUNCTION DETECTED: {junction_type}')
        self.get_logger().info(
            f'📊 Distances - F:{self.front_distance:.2f}m '
            f'L:{self.left_distance:.2f}m R:{self.right_distance:.2f}m'
        )
        
        # Optionally stop robot at junction
        if self.STOP_AT_JUNCTION:
            self._stop_robot()
    
    def _stop_robot(self):
        """Stop the robot when junction is detected"""
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.cmd_pub.publish(cmd)
        self.get_logger().info('🛑 Robot stopped at junction')


def main(args=None):
    """Main entry point"""
    rclpy.init(args=args)
    detector = JunctionDetectorSim()
    
    try:
        rclpy.spin(detector)
    except KeyboardInterrupt:
        detector.get_logger().info('Junction detector shutting down...')
    finally:
        detector.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

