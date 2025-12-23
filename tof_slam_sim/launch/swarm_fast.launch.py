from __future__ import annotations

import os
import tempfile
import time

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _scan_topics(prefix: str) -> list[str]:
    sensors = [
        'front',
        'front_right',
        'right',
        'back_right',
        'back',
        'back_left',
        'left',
        'front_left',
    ]
    return [f'{prefix}/scan/{name}'.replace('//', '/') for name in sensors]


def _truthy(value: object) -> bool:
    return str(value).strip().lower() in ('1', 'true', 'yes', 'on')


def _write_nav2_params(*, base_path: str, robot: str) -> str:
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


def _nav2_actions(context):
    robots = ['robot', 'robot2', 'robot3', 'robot4']

    use_sim_time_str = LaunchConfiguration('use_sim_time').perform(context)
    use_sim_time = _truthy(use_sim_time_str)
    run_nav2 = _truthy(LaunchConfiguration('run_nav2').perform(context))
    run_explorer = _truthy(LaunchConfiguration('run_explorer').perform(context))
    run_autopilot = _truthy(LaunchConfiguration('run_autopilot').perform(context))
    params_path = LaunchConfiguration('nav2_params').perform(context)

    if run_autopilot:
        # Use the repo's exploration autopilot (one per robot) instead of Nav2.
        lo = -18.4
        hi = 18.4
        mid = 0.0
        overlap = 0.6
        quadrants = {
            'robot': (lo, mid + overlap, lo, mid + overlap),          # SW
            'robot2': (lo, mid + overlap, mid - overlap, hi),         # NW
            'robot3': (mid - overlap, hi, mid - overlap, hi),         # NE
            'robot4': (mid - overlap, hi, lo, mid + overlap),         # SE
        }

        common_env = {
            'AP_MODE': 'explore',
            'AP_MAP_TOPIC': '/map',
            'AP_MAP_FRAME': 'robot/map',
            # Conservative defaults for swarm safety.
            'AP_RATE': '10.0',
            'AP_EXP_FORWARD': '0.45',
            'AP_EXP_STRAFE': '0.20',
            'AP_EXP_TURN': '0.55',
            'AP_EXP_CLEAR': '1.25',
            'AP_EXP_AVOID': '0.70',
            'AP_EXP_INFLATE_M': '0.45',
            'AP_EXP_BREADTH_FIRST': '1',
            'AP_EXP_ARENA_ENABLE': '1',
        }

        actions = []
        for r in robots:
            ns = '' if r == 'robot' else r
            min_x, max_x, min_y, max_y = quadrants[r]
            env = dict(common_env)
            env.update({
                'AP_TOPIC': 'cmd_vel',
                'AP_ODOM': 'odom',
                'AP_SCAN_TOPIC': 'scan_merged',
                'AP_ODOM_FRAME': f'{r}/odom',
                'AP_EXP_ARENA_MIN_X': str(min_x),
                'AP_EXP_ARENA_MAX_X': str(max_x),
                'AP_EXP_ARENA_MIN_Y': str(min_y),
                'AP_EXP_ARENA_MAX_Y': str(max_y),
            })
            actions.append(
                Node(
                    package='tof_slam_sim',
                    executable='auto_pilot',
                    name='auto_pilot',
                    namespace=ns,
                    output='screen',
                    parameters=[{'use_sim_time': use_sim_time}],
                    additional_env=env,
                )
            )
        return actions

    if not run_nav2 and not run_explorer:
        return []

    nav2_launch_path = os.path.join(
        get_package_share_directory('nav2_bringup'), 'launch', 'bringup_launch.py'
    )

    # Split arena into 4 overlapping quadrants so robots spread out instead of chasing the same frontier.
    # Keep a margin from the perimeter walls / keepout band so Nav2 doesn't end up with a
    # "start occupied" condition when a robot gets too close to the border.
    lo = -18.4
    hi = 18.4
    mid = 0.0
    overlap = 0.6
    quadrants = {
        'robot': (lo, mid + overlap, lo, mid + overlap),          # SW
        'robot2': (lo, mid + overlap, mid - overlap, hi),         # NW
        'robot3': (mid - overlap, hi, mid - overlap, hi),         # NE
        'robot4': (mid - overlap, hi, lo, mid + overlap),         # SE
    }

    actions = []
    for r in robots:
        ns = '' if r == 'robot' else r
        robot_params = _write_nav2_params(base_path=str(params_path), robot=r)

        if run_nav2:
            actions.append(
                GroupAction(
                    actions=[
                        IncludeLaunchDescription(
                            PythonLaunchDescriptionSource(nav2_launch_path),
                        )
                    ],
                    scoped=True,
                    launch_configurations={
                        'namespace': ns,
                        'use_namespace': 'true' if r != 'robot' else 'false',
                        'slam': 'False',
                        'map': '',
                        'use_localization': 'False',
                        'use_sim_time': 'true' if use_sim_time else 'false',
                        'params_file': robot_params,
                        'autostart': 'true',
                        'use_composition': 'True',
                        'use_respawn': 'False',
                        'log_level': 'info',
                    },
                ),
            )

        if run_explorer:
            min_x, max_x, min_y, max_y = quadrants[r]
            actions.append(
                Node(
                    package='tof_slam_sim',
                    executable='nav2_frontier_explorer',
                    name='nav2_frontier_explorer',
                    namespace=ns,
                    output='screen',
                    parameters=[{
                        'use_sim_time': use_sim_time,
                        'map_topic': '/map',
                        'goal_frame': 'robot/map',
                        'base_frame': f'{r}/base_footprint',
                        'arena_enabled': True,
                        'arena_min_x': float(min_x),
                        'arena_max_x': float(max_x),
                        'arena_min_y': float(min_y),
                        'arena_max_y': float(max_y),
                        'replan_period_sec': 2.0,
                        'goal_timeout_sec': 30.0,
                        'goal_offset_m': 1.6,
                        'min_frontier_cluster_size': 6,
                        'breadth_first': True,
                        'breadth_min_frontier_m_start': 1.8,
                        'breadth_min_frontier_m_end': 0.6,
                        'breadth_min_goal_dist_start': 2.2,
                        'breadth_min_goal_dist_end': 0.8,
                        # Bias goal selection toward open space so robots don't hug walls.
                        'clearance_weight': 10.0,
                        'clearance_min_start': 1.2,
                        'clearance_min_m': 0.9,
                        'clearance_penalty': 20.0,
                    }],
                )
            )

    return actions


def generate_launch_description() -> LaunchDescription:
    pkg = FindPackageShare('tof_slam_sim')
    use_sim_time = LaunchConfiguration('use_sim_time')
    world = LaunchConfiguration('world')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time',
    )
    declare_world = DeclareLaunchArgument(
        'world',
        default_value='playfield_swarm.sdf',
        description='World file (SDF) under tof_slam_sim/worlds.',
    )
    declare_run_nav2 = DeclareLaunchArgument(
        'run_nav2',
        default_value='true',
        description='Start 4 Nav2 stacks (one per robot).',
    )
    declare_run_explorer = DeclareLaunchArgument(
        'run_explorer',
        default_value='true',
        description='Start 4 frontier explorers (one per robot).',
    )
    declare_nav2_params = DeclareLaunchArgument(
        'nav2_params',
        default_value=PathJoinSubstitution([pkg, 'config', 'nav2_params_rex.yaml']),
        description='Nav2 params YAML template (robot2/3/4 are auto-rewritten).',
    )
    declare_rviz = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Launch RViz.',
    )

    rviz_enabled = IfCondition(LaunchConfiguration('rviz'))

    declare_stub_autopilot = DeclareLaunchArgument(
        'run_autopilot',
        default_value='false',
        description='Run 4 autopilot explorers (one per robot) instead of Nav2.',
    )

    robots = ['robot', 'robot2', 'robot3', 'robot4']

    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([pkg, 'launch', 'sim_with_bridge.launch.py'])
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'world': world,
            'robots': ','.join(robots),
            'run_autopilot': 'false',
            'run_logger': 'false',
            'bridge_world': 'playfield',
        }.items(),
    )

    # Publish required TF for all robots (static transforms + odom->base_footprint),
    # including namespaced `/robotN/tf` streams for Nav2 stacks.
    swarm_tf = Node(
        package='tof_slam_sim',
        executable='swarm_tf_broadcaster',
        name='swarm_tf_broadcaster',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robots': robots,
            'map_frame': 'robot/map',
            'publish_base_link_alias': True,
        }],
    )

    # Merge each robot's 8 ToF LaserScans into its own 360° scan topic.
    scan_mergers: list[Node] = []
    for r in robots:
        prefix = '' if r == 'robot' else f'/{r}'
        out_topic = '/scan_merged' if r == 'robot' else f'/{r}/scan_merged'
        scan_mergers.append(
            Node(
                package='tof_slam_sim',
                executable='scan_merger',
                name=f'{r}_scan_merger',
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'input_topics': _scan_topics(prefix),
                    'output_topic': out_topic,
                    'output_frame': f'{r}/base_footprint',
                    'publish_hz': 10.0,
                }],
                output='screen',
            )
        )

    # Fuse all robot scans into one global occupancy grid at /map (and /map_updates).
    fuser = Node(
        package='tof_slam_sim',
        executable='swarm_map_fuser',
        name='swarm_map_fuser',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'map_frame': 'robot/map',
            'map_topic': '/map',
            'update_topic': '/map_updates',
            'robots': robots,
            'scan_topics': [
                '/scan_merged',
                '/robot2/scan_merged',
                '/robot3/scan_merged',
                '/robot4/scan_merged',
            ],
            'resolution': 0.05,
            'min_x': -20.0,
            'max_x': 20.0,
            'min_y': -20.0,
            'max_y': 20.0,
            # Seed an occupied border so robots keep a safety buffer from the perimeter walls.
            # This is enforced in both global + local costmaps via the shared `/map` static layer.
            'seed_keepout': True,
            # Match the explorer arena bounds (±18.4m) so Nav2 won't route along the outer wall band.
            'keepout_margin_m': 1.6,
            'publish_period_sec': 0.5,
        }],
    )

    # Give Gazebo + bridge + TF a moment to come up so all Nav2 stacks can activate.
    nav2_and_exploration = TimerAction(
        period=5.0,
        actions=[OpaqueFunction(function=_nav2_actions)],
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', PathJoinSubstitution([pkg, 'config', 'slam.rviz'])],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=rviz_enabled,
    )

    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_world)
    ld.add_action(declare_run_nav2)
    ld.add_action(declare_run_explorer)
    ld.add_action(declare_nav2_params)
    ld.add_action(declare_rviz)
    ld.add_action(declare_stub_autopilot)
    ld.add_action(sim_launch)
    ld.add_action(swarm_tf)
    for n in scan_mergers:
        ld.add_action(n)
    ld.add_action(fuser)
    ld.add_action(nav2_and_exploration)
    ld.add_action(rviz)
    return ld
