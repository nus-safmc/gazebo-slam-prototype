#!/usr/bin/env python3
"""
Launch file for the centralized swarm control system.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    num_drones_arg = DeclareLaunchArgument(
        'num_drones',
        default_value='8',
        description='Number of drones to control',
    )
    dashboard_arg = DeclareLaunchArgument(
        'dashboard',
        default_value='true',
        description='Launch the web dashboard on http://localhost:8080',
    )

    frontier_server = Node(
        package='swarm_control',
        executable='frontier_server',
        name='frontier_server',
        output='screen',
    )

    goal_allocator = Node(
        package='swarm_control',
        executable='goal_allocator',
        name='goal_allocator',
        output='screen',
    )

    mission_supervisor = Node(
        package='swarm_control',
        executable='mission_supervisor',
        name='mission_supervisor',
        parameters=[{
            'expected_drones': ['robot', 'robot2', 'robot3', 'robot4', 'robot5', 'robot6', 'robot7', 'robot8'],
        }],
        output='screen',
    )

    traffic_manager = Node(
        package='swarm_control',
        executable='traffic_manager',
        name='traffic_manager',
        output='screen',
    )

    dashboard = Node(
        package='swarm_control',
        executable='swarm_dashboard',
        name='swarm_dashboard',
        output='screen',
        condition=IfCondition(LaunchConfiguration('dashboard')),
    )

    # Per-robot drone executors
    drone_executors = []
    robot_names = ['robot', 'robot2', 'robot3', 'robot4', 'robot5', 'robot6', 'robot7', 'robot8']
    for robot_name in robot_names:
        namespace = '' if robot_name == 'robot' else robot_name
        drone_executors.append(Node(
            package='swarm_control',
            executable='drone_executor',
            name=f'{robot_name}_executor',
            namespace=namespace,
            parameters=[{'robot_namespace': namespace}],
            output='screen',
        ))

    return LaunchDescription([
        num_drones_arg,
        dashboard_arg,
        frontier_server,
        goal_allocator,
        mission_supervisor,
        traffic_manager,
        dashboard,
        *drone_executors,
    ])
