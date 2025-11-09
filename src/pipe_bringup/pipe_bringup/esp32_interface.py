#!/usr/bin/env python3
"""
ESP32 Hardware Interface Node
Bridges between ROS2 navigation system and ESP32 hardware.
- Reads sensor data from ESP32 via serial
- Publishes sensor data as ROS2 topics
- Subscribes to motor commands from centering controller
- Sends motor commands to ESP32 via serial
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import json
import serial
import threading
import time


class ESP32Interface(Node):
    """Interface between ROS2 and ESP32 hardware"""
    
    def __init__(self):
        super().__init__('esp32_interface')
        
        # === Configuration ===
        self.SERIAL_PORT = '/dev/ttyUSB0'
        self.BAUD_RATE = 115200
        self.READ_TIMEOUT = 1.0  # seconds
        
        # === Serial Connection ===
        try:
            self.ser = serial.Serial(
                port=self.SERIAL_PORT,
                baudrate=self.BAUD_RATE,
                timeout=self.READ_TIMEOUT
            )
            time.sleep(2)  # Wait for ESP32 to stabilize
            self.get_logger().info(f'✓ Connected to ESP32 on {self.SERIAL_PORT}')
        except Exception as e:
            self.get_logger().error(f'✗ Failed to connect to ESP32: {e}')
            raise
        
        # === ROS2 Publishers ===
        self.sensor_pub = self.create_publisher(String, 'esp32/sensor_data', 10)
        self.status_pub = self.create_publisher(String, 'esp32/status', 10)
        
        # === ROS2 Subscribers ===
        self.motor_cmd_sub = self.create_subscription(
            Twist, 'motor_commands', self._on_motor_command, 10
        )
        
        # === Serial Read Thread ===
        self.serial_thread = threading.Thread(target=self._serial_read_loop, daemon=True)
        self.serial_thread.start()
        
        self.get_logger().info('🚀 ESP32 Interface Node Started')
    
    def _on_motor_command(self, msg):
        """
        Receive motor commands from centering controller
        msg.linear.x = forward speed (-1.0 to +1.0)
        msg.angular.z = turn command (rotation)
        
        Convert to motor PWM values and send to ESP32
        """
        
        # Convert ROS Twist to motor speeds
        # For 3 motors on pipe robot, we'll do differential steering
        forward = msg.linear.x * 255  # Convert to PWM (0-255)
        turn = msg.angular.z * 255    # Convert to PWM
        
        # Simple differential steering (adjust for your wheel config)
        m1_speed = forward + turn
        m2_speed = forward - turn
        m3_speed = forward  # Center motor (or adjust based on layout)
        
        # Constrain to -255 to +255
        m1_speed = max(-255, min(255, int(m1_speed)))
        m2_speed = max(-255, min(255, int(m2_speed)))
        m3_speed = max(-255, min(255, int(m3_speed)))
        
        # Create JSON command
        cmd = {
            "msg_type": "motor",
            "m1": m1_speed,
            "m2": m2_speed,
            "m3": m3_speed
        }
        
        self._send_command(cmd)
    
    def _serial_read_loop(self):
        """Background thread: continuously read from serial"""
        while rclpy.ok():
            try:
                if self.ser.in_waiting > 0:
                    line = self.ser.readline().decode('utf-8').strip()
                    
                    if line:
                        # Parse JSON
                        try:
                            data = json.loads(line)
                            self._handle_esp32_message(data)
                        except json.JSONDecodeError:
                            self.get_logger().warn(f'Invalid JSON: {line}')
            except Exception as e:
                self.get_logger().error(f'Serial read error: {e}')
            
            time.sleep(0.01)  # 10ms delay to prevent CPU spinning
    
    def _handle_esp32_message(self, data):
        """Process incoming message from ESP32"""
        msg_type = data.get('msg_type', '')
        
        if msg_type == 'sensor':
            # Sensor data from ESP32
            sensor_msg = String()
            sensor_msg.data = json.dumps(data)
            self.sensor_pub.publish(sensor_msg)
            
            # Debug output
            self.get_logger().debug(
                f"📊 Sensors - F:{data['front']}cm "
                f"L:{data['left']}cm R:{data['right']}cm B:{data['back']}cm"
            )
        
        elif msg_type == 'pong':
            self.get_logger().debug('✓ Received pong from ESP32')
        
        elif msg_type == 'status':
            status_msg = String()
            status_msg.data = json.dumps(data)
            self.status_pub.publish(status_msg)
    
    def _send_command(self, command):
        """Send JSON command to ESP32"""
        try:
            json_str = json.dumps(command)
            self.ser.write((json_str + '\n').encode('utf-8'))
        except Exception as e:
            self.get_logger().error(f'Failed to send command: {e}')
    
    def destroy_node(self):
        """Cleanup on shutdown"""
        if self.ser and self.ser.is_open:
            self.ser.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    interface = ESP32Interface()
    
    try:
        rclpy.spin(interface)
    except KeyboardInterrupt:
        pass
    finally:
        interface.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

