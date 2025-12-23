from __future__ import annotations

import math
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
    SetLaunchConfiguration,
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


def _parse_robots(context) -> list[str]:
    raw = str(LaunchConfiguration('robots').perform(context)).strip()
    if not raw:
        return ['robot']
    robots = [r.strip() for r in raw.split(',') if r.strip()]
    # Ensure unique and stable order.
    out: list[str] = []
    seen: set[str] = set()
    for r in robots:
        if r in seen:
            continue
        seen.add(r)
        out.append(r)
    return out or ['robot']


def _scan_topic_list(robot: str) -> list[str]:
    prefix = '' if robot == 'robot' else f'/{robot}'
    return _scan_topics(prefix)


def _merged_scan_topic(robot: str) -> str:
    return '/scan_merged' if robot == 'robot' else f'/{robot}/scan_merged'


def _split_arena(
    *,
    robots: list[str],
    lo: float,
    hi: float,
    overlap: float,
) -> dict[str, tuple[float, float, float, float]]:
    n = max(1, len(robots))
    cols = int(math.ceil(math.sqrt(n)))
    rows = int(math.ceil(n / cols))
    span = hi - lo
    cell_w = span / cols
    cell_h = span / rows

    out: dict[str, tuple[float, float, float, float]] = {}
    for i, r in enumerate(robots):
        row = i // cols
        col = i % cols
        min_x = lo + col * cell_w
        max_x = lo + (col + 1) * cell_w
        min_y = lo + row * cell_h
        max_y = lo + (row + 1) * cell_h

        min_x = max(lo, min_x - overlap)
        max_x = min(hi, max_x + overlap)
        min_y = max(lo, min_y - overlap)
        max_y = min(hi, max_y + overlap)

        out[r] = (min_x, max_x, min_y, max_y)
    return out


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
    robots = _parse_robots(context)

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
        overlap = 0.6
        cells = _split_arena(robots=robots, lo=lo, hi=hi, overlap=overlap)

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
            min_x, max_x, min_y, max_y = cells.get(r, (lo, hi, lo, hi))
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
    overlap = 0.6
    cells = _split_arena(robots=robots, lo=lo, hi=hi, overlap=overlap)

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
            min_x, max_x, min_y, max_y = cells.get(r, (lo, hi, lo, hi))
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


def _maybe_select_spawns(context):
    if _truthy(LaunchConfiguration('default_spawn').perform(context)):
        return []

    world_arg = str(LaunchConfiguration('world').perform(context)).strip()
    if not world_arg:
        return []

    if os.path.isabs(world_arg) or '/' in world_arg:
        base_world = os.path.expanduser(world_arg)
    else:
        base_world = os.path.join(
            get_package_share_directory('tof_slam_sim'), 'worlds', world_arg
        )

    robots = _parse_robots(context)

    try:
        from tof_slam_sim.spawn_selector import select_spawn_points, write_world_with_robot_spawns
    except Exception:
        return []

    try:
        spawns = select_spawn_points(world_sdf_path=base_world, robots=robots)
    except Exception:
        spawns = None

    if not spawns:
        return []

    out_world = write_world_with_robot_spawns(
        base_world_sdf_path=base_world,
        robots=robots,
        spawns=spawns,
        z_m=0.05,
    )
    return [SetLaunchConfiguration('world', out_world)]


def _build_swarm(context):
    pkg_share = get_package_share_directory('tof_slam_sim')
    robots = _parse_robots(context)

    use_sim_time_str = LaunchConfiguration('use_sim_time').perform(context)
    use_sim_time = _truthy(use_sim_time_str)

    world_value = str(LaunchConfiguration('world').perform(context)).strip()
    if not world_value:
        world_value = 'playfield_swarm.sdf'

    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(pkg_share, 'launch', 'sim_with_bridge.launch.py')]),
        launch_arguments={
            'use_sim_time': 'true' if use_sim_time else 'false',
            'world': world_value,
            'robots': ','.join(robots),
            'run_autopilot': 'false',
            'run_logger': 'false',
            'bridge_world': 'playfield',
        }.items(),
    )

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

    scan_mergers: list[Node] = []
    for r in robots:
        out_topic = _merged_scan_topic(r)
        scan_mergers.append(
            Node(
                package='tof_slam_sim',
                executable='scan_merger',
                name=f'{r}_scan_merger',
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'input_topics': _scan_topic_list(r),
                    'output_topic': out_topic,
                    'output_frame': f'{r}/base_footprint',
                    'publish_hz': 10.0,
                }],
                output='screen',
            )
        )

    scan_topics = [_merged_scan_topic(r) for r in robots]
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
            'scan_topics': scan_topics,
            'resolution': 0.05,
            'min_x': -20.0,
            'max_x': 20.0,
            'min_y': -20.0,
            'max_y': 20.0,
            'seed_keepout': True,
            'keepout_margin_m': 1.6,
            'publish_period_sec': 0.5,
        }],
    )

    nav2_and_exploration = TimerAction(
        period=5.0,
        actions=[OpaqueFunction(function=_nav2_actions)],
    )

    rviz_enabled = IfCondition(LaunchConfiguration('rviz'))
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(pkg_share, 'config', 'slam.rviz')],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=rviz_enabled,
    )

    return [sim_launch, swarm_tf, *scan_mergers, fuser, nav2_and_exploration, rviz]


def generate_launch_description() -> LaunchDescription:
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
    declare_robots = DeclareLaunchArgument(
        'robots',
        default_value='robot,robot2,robot3,robot4',
        description='Comma-separated robot model names to use/spawn (e.g. "robot,robot2,robot3").',
    )
    declare_default_spawn = DeclareLaunchArgument(
        'default_spawn',
        default_value='false',
        description='If true, skip the spawn UI and use default world spawns.',
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
        default_value=PathJoinSubstitution([FindPackageShare('tof_slam_sim'), 'config', 'nav2_params_rex.yaml']),
        description='Nav2 params YAML template (robot2/3/4 are auto-rewritten).',
    )
    declare_rviz = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Launch RViz.',
    )

    declare_stub_autopilot = DeclareLaunchArgument(
        'run_autopilot',
        default_value='false',
        description='Run 4 autopilot explorers (one per robot) instead of Nav2.',
    )

    maybe_spawn_ui = OpaqueFunction(function=_maybe_select_spawns)
    build_swarm = OpaqueFunction(function=_build_swarm)

    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_world)
    ld.add_action(declare_robots)
    ld.add_action(declare_default_spawn)
    ld.add_action(declare_run_nav2)
    ld.add_action(declare_run_explorer)
    ld.add_action(declare_nav2_params)
    ld.add_action(declare_rviz)
    ld.add_action(declare_stub_autopilot)
    ld.add_action(maybe_spawn_ui)
    ld.add_action(build_swarm)
    return ld
