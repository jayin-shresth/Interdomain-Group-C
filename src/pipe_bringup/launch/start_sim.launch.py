import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_share = FindPackageShare('pipe_bringup').find('pipe_bringup')
    urdf_file = os.path.join(pkg_share, 'urdf', 'pipe_bot.urdf.xacro')
    world_file = os.path.join(pkg_share, 'worlds', 'test_corridor.world')
    robot_description = Command(['xacro ', urdf_file])

    gzserver = ExecuteProcess(
        cmd=['gzserver', '--verbose', world_file, '-s', 'libgazebo_ros_factory.so'],
        output='screen'
    )

    gzclient = ExecuteProcess(
        cmd=['gzclient'],
        output='screen'
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True
        }]
    )

    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'pipe_bot',
            '-topic', 'robot_description',
            '-x', '1.0',
            '-y', '0.0',
            '-z', '0.1',
            '-Y', '0.0'
        ],
        output='screen'
    )

    centering_controller = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='pipe_bringup',
                executable='center_between_walls',
                name='centering_controller',
                output='screen',
                parameters=[{'use_sim_time': True}]
            )
        ]
    )

    return LaunchDescription([
        gzserver,
        gzclient,
        robot_state_publisher,
        spawn_robot,
        centering_controller
    ])

