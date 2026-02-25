#!/usr/bin/env python3
"""
Test Swarm Launch (PX4)
=======================

Convenience wrapper around ``px4_test_swarm.launch.py`` with sensible
defaults for quick local testing.  This is the recommended entry-point
for running the full PX4 swarm + centralized swarm_control stack.

For the legacy Gazebo-only (rex_quadcopter) setup, use the deprecated
``swarm_fast.launch.py`` directly from ``tof_slam_sim``.
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    declare_num_robots = DeclareLaunchArgument(
        'num_robots', default_value='4',
    )
    declare_dashboard = DeclareLaunchArgument(
        'dashboard', default_value='true',
    )
    declare_rviz = DeclareLaunchArgument(
        'rviz', default_value='false',
        description='Launch RViz to visualise the merged map.',
    )
    declare_nav_mode = DeclareLaunchArgument(
        'nav_mode', default_value='nav2',
        description='Navigation backend: "nav2" or "autopilot".',
    )

    px4_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('swarm_control'),
                'launch',
                'px4_test_swarm.launch.py',
            ])
        ]),
        launch_arguments={
            'num_robots': LaunchConfiguration('num_robots'),
            'dashboard': LaunchConfiguration('dashboard'),
            'rviz': LaunchConfiguration('rviz'),
            'nav_mode': LaunchConfiguration('nav_mode'),
        }.items(),
    )

    return LaunchDescription([
        declare_num_robots,
        declare_dashboard,
        declare_rviz,
        declare_nav_mode,
        px4_launch,
    ])
