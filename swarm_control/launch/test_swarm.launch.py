#!/usr/bin/env python3
"""
Test launch: full Gazebo swarm simulation + centralized swarm control.

Uses swarm_fast.launch.py for the infrastructure (Gazebo, bridges, TF, scan
mergers, map fuser, Nav2 stacks) with the old distributed explorer disabled,
then layers the centralized swarm_control nodes on top.
"""

import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
    Shutdown,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    declare_num_robots = DeclareLaunchArgument(
        'num_robots', default_value='5',
    )
    declare_dashboard = DeclareLaunchArgument(
        'dashboard', default_value='true',
    )

    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('tof_slam_sim'),
                'launch',
                'swarm_fast.launch.py',
            ])
        ]),
        launch_arguments={
            'num_robots': LaunchConfiguration('num_robots'),
            'default_spawn': 'true',
            'run_nav2': 'true',
            'run_explorer': 'false',
            'run_autopilot': 'false',
            'rviz': 'false',
            'health_ui': 'false',
            'publish_drone_markers': 'false',
        }.items(),
    )

    swarm_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('swarm_control'),
                'launch',
                'swarm_control.launch.py',
            ])
        ]),
        launch_arguments={
            'dashboard': LaunchConfiguration('dashboard'),
        }.items(),
    )

    delayed_swarm = TimerAction(period=8.0, actions=[swarm_launch])

    return LaunchDescription([
        declare_num_robots,
        declare_dashboard,
        sim_launch,
        delayed_swarm,
    ])
