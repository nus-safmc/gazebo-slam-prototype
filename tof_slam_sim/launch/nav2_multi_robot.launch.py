#!/usr/bin/env python3
"""
Nav2 Multi-Robot Launch (Staggered)
===================================

Launches Nav2 stacks for multiple robots with STAGGERED delays to avoid DDS
saturation. Each robot's Nav2 stack starts 30 seconds after the previous one.

This launch file is designed to be included via TimerAction from px4_test_swarm
(e.g. TimerAction(60, nav2_multi_robot)) so that:
  1. Nav2 starts 60s after sim, when PX4 DDS discovery has settled
  2. Robot 1 Nav2 starts immediately when this launch runs
  3. Robot 2 Nav2 starts 30s later, robot 3 at 60s, etc.

This mitigates the "DDS storm" where multiple lifecycle managers compete for
service calls during PX4 topic discovery, causing planner_server/get_state
timeouts for robots >= 2.
"""

from __future__ import annotations

import os
import tempfile
import time

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction as _GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import PushRosNamespace


def _parse_robots(context) -> list[str]:
    num_raw = str(LaunchConfiguration('num_robots').perform(context)).strip()
    robots_raw = str(LaunchConfiguration('robots').perform(context)).strip()

    if num_raw:
        try:
            n = int(num_raw)
        except ValueError:
            n = 0
        n = max(1, min(15, n))
        out = ['robot']
        for i in range(2, n + 1):
            out.append(f'robot{i}')
        return out

    out: list[str] = []
    seen: set[str] = set()
    for part in robots_raw.split(','):
        name = part.strip()
        if not name or name in seen:
            continue
        seen.add(name)
        out.append(name)
    return out or ['robot']


def _write_nav2_params(*, base_path: str, robot: str) -> str:
    """Generate a per-robot Nav2 params YAML with correct TF frame names."""
    with open(base_path, 'r', encoding='utf-8') as f:
        text = f.read()

    if robot != 'robot':
        text = text.replace('robot/odom', f'{robot}/odom')
        text = text.replace('robot/base_footprint', f'{robot}/base_footprint')
        text = text.replace('robot/base_link', f'{robot}/base_link')

    stamp = int(time.time() * 1000)
    out_path = os.path.join(
        tempfile.gettempdir(), f'nav2_{robot}_{os.getpid()}_{stamp}.yaml'
    )
    with open(out_path, 'w', encoding='utf-8') as f:
        f.write(text)
    return out_path


def _build_nav2_staggered(context):
    robots = _parse_robots(context)
    use_sim_time = str(LaunchConfiguration('use_sim_time').perform(context)).strip().lower() in ('1', 'true', 'yes', 'on')
    nav2_params_path = str(LaunchConfiguration('nav2_params').perform(context)).strip()
    if not nav2_params_path or not os.path.isfile(nav2_params_path):
        nav2_params_path = os.path.join(
            get_package_share_directory('tof_slam_sim'), 'config', 'nav2_params_px4.yaml'
        )

    nav2_launch_path = os.path.join(
        get_package_share_directory('nav2_bringup'), 'launch', 'navigation_launch.py'
    )

    actions = []
    stagger_sec = float(str(LaunchConfiguration('nav2_stagger_sec').perform(context)).strip() or '30.0')

    for i, r in enumerate(robots):
        ns = '' if r == 'robot' else r
        robot_params = _write_nav2_params(base_path=nav2_params_path, robot=r)
        nav2_include = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_launch_path),
            launch_arguments={
                'namespace': ns,
                'use_sim_time': 'true' if use_sim_time else 'false',
                'params_file': robot_params,
                'autostart': 'true',
                'use_composition': 'False',
                'use_respawn': 'False',
                'log_level': 'info',
            }.items(),
        )
        if ns:
            nav2_action = _GroupAction(actions=[PushRosNamespace(ns), nav2_include])
        else:
            nav2_action = nav2_include

        # Stagger: robot 0 at 0s, robot 1 at stagger_sec, robot 2 at 2*stagger_sec, ...
        period = i * stagger_sec
        actions.append(TimerAction(period=period, actions=[nav2_action]))

    print(f'[nav2_multi_robot] Launching {len(robots)} Nav2 stacks with {stagger_sec}s stagger')
    return actions


def generate_launch_description() -> LaunchDescription:
    declare_num_robots = DeclareLaunchArgument(
        'num_robots',
        default_value='4',
        description='Number of robots (used if robots is empty).',
    )
    declare_robots = DeclareLaunchArgument(
        'robots',
        default_value='',
        description='Comma-separated robot names (overrides num_robots when set).',
    )
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time.',
    )
    declare_nav2_params = DeclareLaunchArgument(
        'nav2_params',
        default_value='',
        description='Base Nav2 params YAML (defaults to nav2_params_px4.yaml).',
    )
    declare_nav2_stagger_sec = DeclareLaunchArgument(
        'nav2_stagger_sec',
        default_value='30.0',
        description='Seconds between each robot Nav2 stack launch (avoids DDS storm).',
    )

    build_nav2 = OpaqueFunction(function=_build_nav2_staggered)

    return LaunchDescription([
        declare_num_robots,
        declare_robots,
        declare_use_sim_time,
        declare_nav2_params,
        declare_nav2_stagger_sec,
        build_nav2,
    ])
