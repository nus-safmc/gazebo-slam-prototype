#!/usr/bin/env python3
"""
Launch file for the centralized swarm control system.

Accepts a dynamic robot count via ``num_drones`` (or ``robots`` comma list)
and spawns the appropriate number of DroneExecutor nodes alongside the
shared coordination services.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _parse_robots(context) -> list[str]:
    """Build the robot name list from launch arguments."""
    num_raw = str(LaunchConfiguration('num_drones').perform(context)).strip()
    robots_raw = str(LaunchConfiguration('robots').perform(context)).strip()

    if robots_raw:
        out: list[str] = []
        seen: set[str] = set()
        for part in robots_raw.split(','):
            name = part.strip()
            if not name or name in seen:
                continue
            seen.add(name)
            out.append(name)
        if out:
            return out

    try:
        n = int(num_raw)
    except ValueError:
        n = 4
    n = max(1, min(15, n))
    return ['robot'] + [f'robot{i}' for i in range(2, n + 1)]


def _build_nodes(context):
    robots = _parse_robots(context)

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
        parameters=[{'expected_drones': robots}],
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

    drone_executors = []
    for robot_name in robots:
        namespace = '' if robot_name == 'robot' else robot_name
        drone_executors.append(Node(
            package='swarm_control',
            executable='drone_executor',
            name=f'{robot_name}_executor',
            namespace=namespace,
            parameters=[{'robot_namespace': namespace}],
            output='screen',
        ))

    return [
        frontier_server,
        goal_allocator,
        mission_supervisor,
        traffic_manager,
        dashboard,
        *drone_executors,
    ]


def generate_launch_description():
    declare_num_drones = DeclareLaunchArgument(
        'num_drones',
        default_value='4',
        description='Number of drones to control (generates robot, robot2, ... robotN).',
    )
    declare_robots = DeclareLaunchArgument(
        'robots',
        default_value='',
        description='Comma-separated robot names. Overrides num_drones when set.',
    )
    declare_dashboard = DeclareLaunchArgument(
        'dashboard',
        default_value='true',
        description='Launch the web dashboard on http://localhost:8080.',
    )

    return LaunchDescription([
        declare_num_drones,
        declare_robots,
        declare_dashboard,
        OpaqueFunction(function=_build_nodes),
    ])
