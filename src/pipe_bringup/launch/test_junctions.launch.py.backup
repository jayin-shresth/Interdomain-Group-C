#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    
    # Package directories
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_pipe_bringup = get_package_share_directory('pipe_bringup')
    
    # Paths
    world_file = os.path.join(pkg_pipe_bringup, 'worlds', 'junction_test.world')
    xacro_file = os.path.join(pkg_pipe_bringup, 'urdf', 'pipe_bot.urdf.xacro')
    
    # Process xacro file to URDF using Command substitution
    robot_description_content = ParameterValue(
        Command(['xacro ', xacro_file]),
        value_type=str
    )
    
    # Gazebo launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': world_file}.items()
    )
    
    # Robot State Publisher - publishes robot transforms
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': True
        }],
        output='screen'
    )
    
    # Spawn robot in Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'pipe_bot',
            '-x', '0',
            '-y', '0.5',
            '-z', '0.1'
        ],
        output='screen'
    )
    
    # Junction Detector Node
    junction_detector = Node(
        package='pipe_bringup',
        executable='junction_detector',
        name='junction_detector',
        output='screen'
    )
    
    # GPS-Guided Navigator Node
    gps_navigator = Node(
        package='pipe_bringup',
        executable='gps_guided_navigator',
        name='navigator',
        output='screen'
    )
    
    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_entity,
        junction_detector,
        gps_navigator
    ])

