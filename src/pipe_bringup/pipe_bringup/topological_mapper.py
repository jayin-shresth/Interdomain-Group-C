#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from nav_msgs.msg import Odometry
import json
import math

class TopologicalMapper(Node):
    def __init__(self):
        super().__init__('topological_mapper')
        
        # Subscriptions
        self.junction_sub = self.create_subscription(String, '/junction_detected', self.junction_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        
        # Map data
        self.graph = {"nodes": [], "edges": []}
        self.node_counter = 0
        self.last_node_id = None
        self.current_x = 0.0
        self.current_y = 0.0
        self.last_x = 0.0
        self.last_y = 0.0
        
        self.get_logger().info('Topological Mapper Started')
    
    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
    
    def junction_callback(self, msg):
        # Add new node
        junction_node = {
            "id": self.node_counter,
            "type": msg.data,
            "x": self.current_x,
            "y": self.current_y
        }
        self.graph["nodes"].append(junction_node)
        
        # Add edge
        if self.last_node_id is not None:
            distance = self.calculate_distance(self.last_x, self.last_y, self.current_x, self.current_y)
            edge = {
                "from": self.last_node_id,
                "to": self.node_counter,
                "distance": distance
            }
            self.graph["edges"].append(edge)
            self.get_logger().info(f'Edge: Node {self.last_node_id} → Node {self.node_counter} (dist: {distance:.2f}m)')
        
        self.get_logger().info(f'Added node {self.node_counter}: {msg.data} at ({self.current_x:.2f}, {self.current_y:.2f})')
        
        self.last_node_id = self.node_counter
        self.last_x = self.current_x
        self.last_y = self.current_y
        self.node_counter += 1
        
        self.save_map()
    
    def calculate_distance(self, x1, y1, x2, y2):
        dx = x2 - x1
        dy = y2 - y1
        return math.sqrt(dx*dx + dy*dy)
    
    def save_map(self):
        # Save to JSON
        try:
            with open('pipe_map.json', 'w') as f:
                json.dump(self.graph, f, indent=2)
            self.get_logger().info(f'Map saved: {len(self.graph["nodes"])} nodes, {len(self.graph["edges"])} edges')
        except Exception as e:
            self.get_logger().error(f'Failed to save map: {e}')

def main(args=None):
    rclpy.init(args=args)
    mapper = TopologicalMapper()
    try:
        rclpy.spin(mapper)
    except KeyboardInterrupt:
        pass
    finally:
        mapper.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

