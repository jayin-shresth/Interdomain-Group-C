#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_pipe_bringup = FindPackageShare('pipe_bringup').find('pipe_bringup')
    
    urdf_file = os.path.join(pkg_pipe_bringup, 'urdf', 'pipe_bot.urdf.xacro')
    world_file = os.path.join(pkg_pipe_bringup, 'worlds', 'junction_test.world')
    ekf_config = os.path.join(pkg_pipe_bringup, 'config', 'ekf_localization.yaml')
    
    robot_description = Command(['xacro ', urdf_file])
    
    return LaunchDescription([
        ExecuteProcess(
            cmd=['gzserver', '--verbose', world_file, '-s', 'libgazebo_ros_factory.so'],
            output='screen'
        ),
        ExecuteProcess(
            cmd=['gzclient'],
            output='screen'
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
            name='robot_state_publisher',
        ),
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            output='screen',
            arguments=[
                '-entity', 'pipe_bot',
                '-topic', 'robot_description',
                '-x', '1.0',
                '-y', '0.0',
                '-z', '0.1',
                '-Y', '0.0'
            ]
        ),
        Node(
            package='pipe_bringup',
            executable='ultrasonic_to_odometry',
            output='screen',
            name='ultrasonic_to_odometry',
            parameters=[{'use_sim_time':True}]
        ),
        Node(
            package='robot_localization',
            executable='ekf_node',
            output='screen',
            name='ekf_filter_node',
            parameters=[ekf_config, {'use_sim_time': True}],
        ),
        Node(
            package='pipe_bringup',
            executable='junction_detector',
            output='screen',
            name='junction_detector',
            parameters=[{'use_sim_time': True}]
        ),
        TimerAction(
            period=3.0,
            actions=[
                Node(
                    package='pipe_bringup',
                    executable='center_between_walls',
                    output='screen',
                    name='centering_controller',
                    parameters=[{'use_sim_time': True,
                                'stop_distance': 0.5,
                                'side_distance': 0.3,
                                'speed': 0.3}]
                )
            ]
        ),
    ])

