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
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    declare_num_robots = DeclareLaunchArgument(
        'num_robots',
        default_value='4',
        description='Number of PX4 drones to spawn.',
    )
    declare_world = DeclareLaunchArgument(
        'world',
        default_value='playfield_px4_sparse.sdf',
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

    nav_mode = LaunchConfiguration('nav_mode')

    # Determine infrastructure flags from nav_mode.
    run_autopilot = PythonExpression(["'", nav_mode, "' == 'autopilot'"])
    run_nav2 = PythonExpression(["'", nav_mode, "' == 'nav2'"])

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
            'world': LaunchConfiguration('world'),
            'default_spawn': 'true',
            'gz_gui': LaunchConfiguration('gz_gui'),
            'rviz': LaunchConfiguration('rviz'),
            'run_autopilot': run_autopilot,
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
            'dashboard': LaunchConfiguration('dashboard'),
        }.items(),
    )

    delayed_swarm = TimerAction(period=12.0, actions=[swarm_launch])

    return LaunchDescription([
        declare_num_robots,
        declare_world,
        declare_dashboard,
        declare_rviz,
        declare_nav_mode,
        declare_gz_gui,
        declare_target_alt,
        sim_launch,
        delayed_swarm,
    ])
