#!/usr/bin/env python3
"""
GPS-Guided Navigator
Navigates robot to target coordinates using greedy algorithm.
Handles junction decisions, turning, and breadcrumb recording.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String
import json
import math


class GPSGuidedNavigator(Node):
    """Navigate to target using GPS coordinates and junction decisions"""
    
    # Navigation states
    STATE_MOVING = 'MOVING'
    STATE_STOPPING = 'STOPPING'
    STATE_TURNING = 'TURNING'
    STATE_EXITING = 'EXITING_JUNCTION'
    STATE_REACHED = 'TARGET_REACHED'
    
    def __init__(self):
        super().__init__('navigator')
        
        # === Configuration ===
        self.TARGET_POSITION = (8.0, -3.0)  # Goal coordinates (x, y)
        self.TARGET_THRESHOLD = 0.5  # Distance to consider target reached (meters)
        self.FORWARD_SPEED = 0.3  # Linear velocity (m/s)
        self.TURN_SPEED = 1.0  # Angular velocity (rad/s)
        self.TURN_TOLERANCE = math.radians(7)  # Acceptable turn error (radians)
        self.TURN_TIMEOUT = 7.0  # Max time for a turn (seconds)
        self.STOP_DURATION = 0.7  # How long to stop at junction (seconds)
        self.EXIT_DURATION = 3.5  # How long to drive to exit junction (seconds)
        self.JUNCTION_BLOCK_TIME = 7.0  # Ignore junctions for this long after handling one (seconds)
        
        # === State Variables ===
        self.current_state = self.STATE_MOVING
        self.current_position = (0.0, 0.0)
        self.current_heading = 0.0
        self.pending_direction = None
        self.target_heading = 0.0
        
        # === Timing ===
        self.stop_start_time = None
        self.turn_start_time = None
        self.exit_start_time = None
        self.junction_block_until = None
        
        # === Breadcrumb Trail ===
        self.breadcrumbs = []
        self.distance_since_last_junction = 0.0
        self.last_position = (0.0, 0.0)
        
        # === ROS Setup ===
        self._setup_subscribers()
        self._setup_publishers()
        self.create_timer(0.1, self._control_loop)  # 10 Hz
        
        self._log_startup_info()
    
    def _setup_subscribers(self):
        """Initialize topic subscriptions"""
        self.odom_sub = self.create_subscription(
            Odometry, 'odom', self._on_odometry_update, 10)
        self.junction_sub = self.create_subscription(
            String, 'junction_detected', self._on_junction_detected, 10)
    
    def _setup_publishers(self):
        """Initialize command publishers"""
        self.velocity_pub = self.create_publisher(Twist, 'cmd_vel', 10)
    
    def _log_startup_info(self):
        """Log configuration on startup"""
        self.get_logger().info('🚀 GPS Navigator Started')
        self.get_logger().info(f'   Target: {self.TARGET_POSITION}')
        self.get_logger().info(f'   Forward speed: {self.FORWARD_SPEED} m/s')
        self.get_logger().info(f'   Turn speed: {self.TURN_SPEED} rad/s')
    
    # === Odometry Callback ===
    
    def _on_odometry_update(self, msg):
        """Update robot's current pose from odometry"""
        # Extract position
        self.current_position = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y
        )
        
        # Extract heading from quaternion
        q = msg.pose.pose.orientation
        self.current_heading = self._quaternion_to_yaw(q)
        
        # Track distance traveled
        distance_moved = self._calculate_distance(
            self.current_position, self.last_position
        )
        self.distance_since_last_junction += distance_moved
        self.last_position = self.current_position
    
    def _quaternion_to_yaw(self, quaternion):
        """Convert quaternion to yaw angle (radians)"""
        siny_cosp = 2 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y)
        cosy_cosp = 1 - 2 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z)
        return math.atan2(siny_cosp, cosy_cosp)
    
    # === Junction Detection Callback ===
    
    def _on_junction_detected(self, msg):
        """Handle junction detection event"""
        # Check if we're blocked from processing junctions
        if not self._can_process_junction():
            return
        
        junction_type = msg.data
        self.get_logger().info(f'🔍 Junction Detected: {junction_type}')
        
        # Block further junctions for a while
        self._start_junction_block()
        
        # Decide which way to go
        self.pending_direction = self._choose_direction()
        
        # Record breadcrumb
        self._save_breadcrumb(self.pending_direction)
        
        # Reset distance tracker
        self.distance_since_last_junction = 0.0
        
        # Enter stopping state
        self.current_state = self.STATE_STOPPING
        self.stop_start_time = self.get_clock().now()
    
    def _can_process_junction(self):
        """Check if we're ready to handle a new junction"""
        # Only process if we're in normal driving mode
        if self.current_state != self.STATE_MOVING:
            return False
        
        # Check if block timer is active
        now = self.get_clock().now()
        if self.junction_block_until and now < self.junction_block_until:
            return False
        
        return True
    
    def _start_junction_block(self):
        """Start blocking junction detection for a period"""
        self.junction_block_until = (
            self.get_clock().now() + 
            rclpy.duration.Duration(seconds=self.JUNCTION_BLOCK_TIME)
        )
    
    def _choose_direction(self):
        """Decide which direction to turn based on target location"""
        # Calculate angle to target
        dx = self.TARGET_POSITION[0] - self.current_position[0]
        dy = self.TARGET_POSITION[1] - self.current_position[1]
        angle_to_target = math.atan2(dy, dx)
        
        # Calculate relative angle (target relative to current heading)
        relative_angle = self._normalize_angle(angle_to_target - self.current_heading)
        relative_degrees = math.degrees(relative_angle)
        
        self.get_logger().info(f'📊 Angle to target: {relative_degrees:.1f}°')
        
        # Decide direction based on angle
        if -45 < relative_degrees < 45:
            return 'STRAIGHT'
        elif relative_degrees >= 45:
            return 'LEFT'
        else:
            return 'RIGHT'
    
    def _save_breadcrumb(self, direction):
        """Record current junction for return journey"""
        breadcrumb = {
            'id': len(self.breadcrumbs),
            'position': list(self.current_position),
            'direction': direction,
            'distance': self.distance_since_last_junction,
            'heading': self.current_heading
        }
        self.breadcrumbs.append(breadcrumb)
        
        # Save to file
        try:
            with open('breadcrumbs.json', 'w') as f:
                json.dump(self.breadcrumbs, f, indent=2)
        except Exception as e:
            self.get_logger().error(f'Failed to save breadcrumbs: {e}')
        
        self.get_logger().info(f'💾 Breadcrumb #{len(self.breadcrumbs)} - {direction}')
    
    # === Main Control Loop ===
    
    def _control_loop(self):
        """Main state machine - runs at 10 Hz"""
        if self.current_state == self.STATE_MOVING:
            self._handle_moving_state()
        elif self.current_state == self.STATE_STOPPING:
            self._handle_stopping_state()
        elif self.current_state == self.STATE_TURNING:
            self._handle_turning_state()
        elif self.current_state == self.STATE_EXITING:
            self._handle_exiting_state()
        elif self.current_state == self.STATE_REACHED:
            self._handle_reached_state()
    
    def _handle_moving_state(self):
        """Normal forward driving"""
        # Check if we've reached the target
        distance_to_target = self._calculate_distance(
            self.current_position, self.TARGET_POSITION
        )
        
        if distance_to_target < self.TARGET_THRESHOLD:
            self.get_logger().info('🎯 TARGET REACHED!')
            self.current_state = self.STATE_REACHED
            self._stop_robot()
            return
        
        # Drive forward
        self._drive_forward()
    
    def _handle_stopping_state(self):
        """Brief stop at junction before turning"""
        self._stop_robot()
        
        elapsed = self._get_elapsed_time(self.stop_start_time)
        if elapsed >= self.STOP_DURATION:
            # Done stopping - decide next action
            if self.pending_direction == 'LEFT':
                self._start_turn(math.pi / 2)  # 90° left
            elif self.pending_direction == 'RIGHT':
                self._start_turn(-math.pi / 2)  # 90° right
            else:
                # Going straight - just exit junction
                self.get_logger().info('Going STRAIGHT')
                self.current_state = self.STATE_EXITING
                self.exit_start_time = self.get_clock().now()
    
    def _handle_turning_state(self):
        """Execute precise turn using odometry"""
        # Calculate how far we still need to turn
        angle_error = self._normalize_angle(
            self.target_heading - self.current_heading
        )
        
        # Check if turn is complete
        if abs(angle_error) < self.TURN_TOLERANCE:
            self._stop_robot()
            self.get_logger().info(
                f'✅ Turn complete! Heading: {math.degrees(self.current_heading):.1f}°'
            )
            self.current_state = self.STATE_EXITING
            self.exit_start_time = self.get_clock().now()
            return
        
        # Check for timeout
        elapsed = self._get_elapsed_time(self.turn_start_time)
        if elapsed > self.TURN_TIMEOUT:
            self.get_logger().warn('⚠️  Turn timeout!')
            self._stop_robot()
            self.current_state = self.STATE_EXITING
            self.exit_start_time = self.get_clock().now()
            return
        
        # Continue turning
        angular_velocity = self.TURN_SPEED if angle_error > 0 else -self.TURN_SPEED
        self._send_velocity(0.0, angular_velocity)
    
    def _handle_exiting_state(self):
        """Drive forward to clear junction area"""
        elapsed = self._get_elapsed_time(self.exit_start_time)
        
        if elapsed < self.EXIT_DURATION:
            self._drive_forward()
        else:
            self._stop_robot()
            self.current_state = self.STATE_MOVING
            self.get_logger().info('✅ Junction cleared - resuming navigation')
    
    def _handle_reached_state(self):
        """Target reached - stay stopped"""
        self._stop_robot()
    
    # === Turn Management ===
    
    def _start_turn(self, angle_delta):
        """Initiate a turn by specified angle"""
        self.target_heading = self._normalize_angle(
            self.current_heading + angle_delta
        )
        self.current_state = self.STATE_TURNING
        self.turn_start_time = self.get_clock().now()
        
        direction = 'LEFT' if angle_delta > 0 else 'RIGHT'
        self.get_logger().info(
            f'🔄 Turning {direction} to {math.degrees(self.target_heading):.1f}°'
        )
    
    # === Motion Commands ===
    
    def _drive_forward(self):
        """Send forward motion command"""
        self._send_velocity(self.FORWARD_SPEED, 0.0)
    
    def _stop_robot(self):
        """Send stop command"""
        self._send_velocity(0.0, 0.0)
    
    def _send_velocity(self, linear, angular):
        """Publish velocity command"""
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        self.velocity_pub.publish(msg)
    
    # === Utility Functions ===
    
    def _normalize_angle(self, angle):
        """Normalize angle to [-π, π]"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle
    
    def _calculate_distance(self, pos1, pos2):
        """Calculate Euclidean distance between two points"""
        return math.sqrt(
            (pos1[0] - pos2[0])**2 + 
            (pos1[1] - pos2[1])**2
        )
    
    def _get_elapsed_time(self, start_time):
        """Get elapsed time since start_time in seconds"""
        if start_time is None:
            return 0.0
        return (self.get_clock().now() - start_time).nanoseconds / 1e9


def main(args=None):
    """Main entry point"""
    rclpy.init(args=args)
    navigator = GPSGuidedNavigator()
    
    try:
        rclpy.spin(navigator)
    except KeyboardInterrupt:
        pass
    finally:
        navigator.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

