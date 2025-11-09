#!/usr/bin/env python3
"""
Hardware launch file - Real robot with ESP32, localization, and navigation
"""

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_share = FindPackageShare('pipe_bringup').find('pipe_bringup')
    ekf_config = os.path.join(pkg_share, 'config', 'ekf_localization.yaml')
    
    return LaunchDescription([
        # ESP32 Interface Node
        Node(
            package='pipe_bringup',
            executable='esp32_interface',
            name='esp32_interface',
            output='screen',
            parameters=[{
                'serial_port': '/dev/ttyUSB0',
                'baud_rate': 115200
            }]
        ),
        
        # Ultrasonic to Odometry Converter
        Node(
            package='pipe_bringup',
            executable='ultrasonic_to_odometry',
            name='ultrasonic_to_odometry',
            output='screen'
        ),
        
        # EKF Localization Node
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_config]
        ),
        
        # Wall Centering Controller
        Node(
            package='pipe_bringup',
            executable='center_between_walls',
            name='center_between_walls',
            output='screen',
            parameters=[{
                'stop_distance': 0.10,
                'side_distance': 0.08,
                'speed': 0.3
            }]
        ),
        
        # Junction Detector
        Node(
            package='pipe_bringup',
            executable='junction_detector_hardware',
            name='junction_detector_hardware',
            output='screen'
        ),
    ])

