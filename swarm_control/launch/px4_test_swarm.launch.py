#!/usr/bin/env python3
"""
PX4 Test Swarm Launch
=====================

Full end-to-end integration: PX4 SITL simulation infrastructure (Gazebo,
sensor bridges, TF, scan mergers, map fuser, Nav2 stacks) with the
centralized swarm_control layer on top.

The PX4 infrastructure is provided by ``px4_swarm_fast.launch.py`` from
``tof_slam_sim``.  After a short settling delay the swarm_control nodes
(FrontierServer, GoalAllocator, MissionSupervisor, DroneExecutors, etc.)
are brought up.

Navigation mode
---------------
* ``nav_mode:=nav2`` (default) -- DroneExecutors use Nav2 NavigateToPose.
  ``px4_swarm_fast`` is launched with ``run_autopilot:=false`` so Nav2
  drives ``cmd_vel``, which ``twist_to_px4_offboard`` translates into
  PX4 offboard setpoints.
* ``nav_mode:=autopilot`` -- The per-robot autopilot nodes handle
  exploration directly.  swarm_control still manages the mission
  lifecycle but does not send navigation goals.
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    declare_num_robots = DeclareLaunchArgument(
        'num_robots',
        default_value='4',
        description='Number of PX4 drones to spawn.',
    )
    declare_robots = DeclareLaunchArgument(
        'robots',
        default_value='',
        description='Comma-separated robot names (overrides num_robots when set).',
    )
    declare_default_spawn = DeclareLaunchArgument(
        'default_spawn',
        default_value='true',
        description='Skip spawn UI and use default spawn points.',
    )
    declare_world = DeclareLaunchArgument(
        'world',
        default_value='playfield_competition.sdf',
        description='Gazebo world file (under tof_slam_sim/worlds/).',
    )
    declare_dashboard = DeclareLaunchArgument(
        'dashboard',
        default_value='true',
        description='Launch the swarm_control web dashboard.',
    )
    declare_rviz = DeclareLaunchArgument(
        'rviz',
        default_value='false',
        description='Launch RViz for map visualisation.',
    )
    declare_nav_mode = DeclareLaunchArgument(
        'nav_mode',
        default_value='nav2',
        description='Navigation backend: "nav2" (default) or "autopilot".',
    )
    declare_gz_gui = DeclareLaunchArgument(
        'gz_gui',
        default_value='false',
        description='Start Gazebo GUI (gz sim -g).',
    )
    declare_target_alt = DeclareLaunchArgument(
        'target_alt_m',
        default_value='1.5',
        description='Target hover altitude for PX4 drones (metres).',
    )
    declare_swarm_delay = DeclareLaunchArgument(
        'swarm_delay_sec',
        default_value='30',
        description='Seconds to wait before launching swarm + Nav2 (reduces DDS storm).',
    )

    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('tof_slam_sim'),
                'launch',
                'px4_swarm_fast.launch.py',
            ])
        ]),
        launch_arguments={
            'num_robots': LaunchConfiguration('num_robots'),
            'robots': LaunchConfiguration('robots'),
            'default_spawn': LaunchConfiguration('default_spawn'),
            'world': LaunchConfiguration('world'),
            'gz_gui': LaunchConfiguration('gz_gui'),
            'rviz': LaunchConfiguration('rviz'),
            'nav_mode': LaunchConfiguration('nav_mode'),
            'nav2_launched_externally': PythonExpression([
                "'true' if '", LaunchConfiguration('nav_mode'), "' == 'nav2' else 'false'"
            ]),
            'health_ui': 'false',
            'publish_drone_markers': 'false',
            'target_alt_m': LaunchConfiguration('target_alt_m'),
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
            'num_drones': LaunchConfiguration('num_robots'),
            'robots': LaunchConfiguration('robots'),
            'dashboard': LaunchConfiguration('dashboard'),
        }.items(),
    )

    delayed_swarm = TimerAction(
        period=LaunchConfiguration('swarm_delay_sec'),
        actions=[swarm_launch],
    )

    # Nav2 stacks launched with swarm_delay_sec + staggered per-robot (avoids DDS storm).
    # Only when nav_mode==nav2; px4_swarm_fast skips Nav2 via nav2_launched_externally.
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('tof_slam_sim'),
                'launch',
                'nav2_multi_robot.launch.py',
            ])
        ]),
        launch_arguments={
            'num_robots': LaunchConfiguration('num_robots'),
            'robots': LaunchConfiguration('robots'),
            'use_sim_time': 'true',
        }.items(),
    )
    delayed_nav2 = TimerAction(
        period=LaunchConfiguration('swarm_delay_sec'),
        actions=[nav2_launch],
    )
    delayed_nav2_cond = GroupAction(
        condition=IfCondition(PythonExpression([
            "'", LaunchConfiguration('nav_mode'), "' == 'nav2'"
        ])),
        actions=[delayed_nav2],
    )

    return LaunchDescription([
        declare_num_robots,
        declare_robots,
        declare_default_spawn,
        declare_world,
        declare_dashboard,
        declare_rviz,
        declare_nav_mode,
        declare_gz_gui,
        declare_target_alt,
        declare_swarm_delay,
        sim_launch,
        delayed_swarm,
        delayed_nav2_cond,
    ])
