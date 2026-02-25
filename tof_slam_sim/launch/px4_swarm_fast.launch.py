from __future__ import annotations

import math
import os
import tempfile
import time
from typing import Optional

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch.actions import GroupAction as _GroupAction
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare


def _truthy(value: object) -> bool:
    return str(value).strip().lower() in ('1', 'true', 'yes', 'on')


def _guess_px4_dir() -> str:
    for key in ('PX4_AUTOPILOT_DIR', 'PX4_DIR'):
        value = os.environ.get(key)
        if value and os.path.isdir(value):
            return value

    try:
        pkg_share_real = os.path.realpath(get_package_share_directory('tof_slam_sim'))
        candidate = os.path.join(os.path.dirname(pkg_share_real), 'PX4-Autopilot')
        if os.path.isdir(candidate):
            return candidate
    except Exception:
        pass

    return os.path.join(os.getcwd(), 'PX4-Autopilot')


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


def _resolve_world_path(context) -> str:
    world_arg = str(LaunchConfiguration('world').perform(context)).strip()
    share_dir = get_package_share_directory('tof_slam_sim')
    default_world_path = os.path.join(share_dir, 'worlds', world_arg)
    if os.path.isabs(world_arg):
        return world_arg
    if world_arg and os.path.exists(world_arg):
        return world_arg
    return default_world_path


def _default_spawns(*, world_sdf_path: str, robots: list[str]) -> dict[str, tuple[float, float, float]]:
    try:
        from tof_slam_sim.spawn_selector import Bounds, default_spawn_spots, extract_view_and_spawn_bounds
    except Exception:
        # Last resort: spread around origin.
        out: dict[str, tuple[float, float, float]] = {}
        for i, r in enumerate(robots):
            out[r] = (float(i) * 1.5, -2.0, 0.0)
        return out

    _view, spawn_bounds = extract_view_and_spawn_bounds(world_sdf_path=world_sdf_path)

    # Compact cluster in the spawn zone to reduce bad initial map / waypoint issues.
    # Center of spawn zone with fixed 2 m spacing between drones.
    margin_m = 0.5
    min_x = float(spawn_bounds.min_x + margin_m)
    max_x = float(spawn_bounds.max_x - margin_m)
    min_y = float(spawn_bounds.min_y + margin_m)
    max_y = float(spawn_bounds.max_y - margin_m)
    center_x = 0.5 * (min_x + max_x)
    center_y = 0.5 * (min_y + max_y)
    spacing_m = 2.0

    if min_x < max_x and min_y < max_y:
        n = max(1, len(robots))
        cols = max(1, int(math.ceil(math.sqrt(n))))
        rows = max(1, int(math.ceil(n / cols)))
        start_x = center_x - 0.5 * spacing_m * (cols - 1)
        start_y = center_y - 0.5 * spacing_m * (rows - 1)
        out: dict[str, tuple[float, float, float]] = {}
        for idx, r in enumerate(robots):
            col = idx % cols
            row = idx // cols
            x = start_x + float(col) * spacing_m
            y = start_y + float(row) * spacing_m
            x = float(max(min_x, min(max_x, x)))
            y = float(max(min_y, min(max_y, y)))
            out[r] = (x, y, 0.0)
        return out

    # Fallback: pick spawn spots across the zone.
    center_x = 0.5 * (spawn_bounds.min_x + spawn_bounds.max_x)
    center_y = 0.5 * (spawn_bounds.min_y + spawn_bounds.max_y)
    spots = default_spawn_spots(
        bounds=Bounds(
            min_x=spawn_bounds.min_x,
            max_x=spawn_bounds.max_x,
            min_y=spawn_bounds.min_y,
            max_y=spawn_bounds.max_y,
        ),
        margin_m=margin_m,
    )
    spots.sort(key=lambda s: (s.x - center_x) ** 2 + (s.y - center_y) ** 2)
    out: dict[str, tuple[float, float, float]] = {}
    for i, r in enumerate(robots):
        spot = spots[i % len(spots)]
        out[r] = (spot.x, spot.y, spot.yaw)
    return out


def _select_spawns(*, world_sdf_path: str, robots: list[str], default_spawn: bool) -> dict[str, tuple[float, float, float]]:
    if default_spawn:
        return _default_spawns(world_sdf_path=world_sdf_path, robots=robots)

    try:
        from tof_slam_sim.spawn_selector import select_spawn_points
    except Exception:
        return _default_spawns(world_sdf_path=world_sdf_path, robots=robots)

    spawns = select_spawn_points(world_sdf_path=world_sdf_path, robots=robots, title="Spawn PX4 drones")
    if not spawns:
        return _default_spawns(world_sdf_path=world_sdf_path, robots=robots)
    return spawns


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


def _build_swarm(context):
    robots = _parse_robots(context)
    world_path = _resolve_world_path(context)

    use_sim_time = _truthy(LaunchConfiguration('use_sim_time').perform(context))
    nav_mode_val = str(LaunchConfiguration('nav_mode').perform(context)).strip().lower()
    gz_world_name = LaunchConfiguration('gz_world_name')
    gz_world_value = str(gz_world_name.perform(context)).strip() or 'default'
    px4_dir = str(LaunchConfiguration('px4_dir').perform(context)).strip() or _guess_px4_dir()
    headless_rendering = _truthy(LaunchConfiguration('headless_rendering').perform(context))

    default_spawn = _truthy(LaunchConfiguration('default_spawn').perform(context))
    spawns = _select_spawns(world_sdf_path=world_path, robots=robots, default_spawn=default_spawn)

    try:
        from tof_slam_sim.spawn_selector import extract_view_and_spawn_bounds
    except Exception:
        view_bounds = None
    else:
        view_bounds, _spawn_bounds = extract_view_and_spawn_bounds(world_sdf_path=world_path)

    # Conservative arena bounds (used by the exploration autopilot).
    arena_margin_m = 0.8
    if view_bounds is None:
        arena_min_x = -19.0
        arena_max_x = 19.0
        arena_min_y = -19.0
        arena_max_y = 19.0
    else:
        arena_min_x = float(view_bounds.min_x + arena_margin_m)
        arena_max_x = float(view_bounds.max_x - arena_margin_m)
        arena_min_y = float(view_bounds.min_y + arena_margin_m)
        arena_max_y = float(view_bounds.max_y - arena_margin_m)

    # Initial map bounds: small region around the spawn points (expands as robots explore).
    spawn_xs = [xy[0] for xy in spawns.values()]
    spawn_ys = [xy[1] for xy in spawns.values()]
    half_extent_m = 5.0
    min_x_val = float(min(spawn_xs) - half_extent_m)
    max_x_val = float(max(spawn_xs) + half_extent_m)
    min_y_val = float(min(spawn_ys) - half_extent_m)
    max_y_val = float(max(spawn_ys) + half_extent_m)

    odom_offsets = []
    for r in robots:
        x, y, _yaw = spawns.get(r, (0.0, 0.0, 0.0))
        odom_offsets.append(f'{r},{float(x):.6f},{float(y):.6f}')

    tof_share = FindPackageShare('tof_slam_sim')
    set_gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[
            PathJoinSubstitution([tof_share, 'models']),
            ':',
            PathJoinSubstitution([tof_share, 'worlds']),
            ':',
            os.path.join(px4_dir, 'Tools', 'simulation', 'gz', 'models'),
            ':',
            os.path.join(px4_dir, 'Tools', 'simulation', 'gz', 'worlds'),
        ],
    )

    # Ensure PX4 SITL sources our repo's `px4-rc.params` (searched via PATH).
    set_px4_rc_path = SetEnvironmentVariable(
        name='PATH',
        value=[
            PathJoinSubstitution([tof_share, 'scripts']),
            ':',
            EnvironmentVariable('PATH'),
        ],
    )

    gz_cmd = ['gz', 'sim', '--verbose=1']
    if headless_rendering:
        gz_cmd.append('--headless-rendering')
    gz_cmd.extend(['-r', '-s', world_path])
    gz_sim = ExecuteProcess(
        cmd=gz_cmd,
        output='screen',
        condition=IfCondition(LaunchConfiguration('run_gz')),
    )
    gz_gui = ExecuteProcess(
        cmd=['gz', 'sim', '-g'],
        output='screen',
        condition=IfCondition(LaunchConfiguration('gz_gui')),
    )

    micro_xrce_agent = ExecuteProcess(
        cmd=['MicroXRCEAgent', 'udp4', '-p', '8888', '-v', '2'],
        output='screen',
        condition=IfCondition(LaunchConfiguration('run_agent')),
    )

    clock_bridge = PythonExpression([
        '"/world/" + "',
        gz_world_name,
        '" + "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"',
    ])
    clock_remap = PythonExpression([
        '"/world/" + "',
        gz_world_name,
        '" + "/clock:=/clock"',
    ])

    # Build a single ros_gz_bridge process for clock + all ToF depth topics.
    bridge_topics: list[str] = [clock_bridge]
    bridge_remaps: list[str] = [clock_remap]

    for i, robot in enumerate(robots):
        model_name = f'x500_small_tof_{i}'
        if robot == 'robot':
            ros_depth_prefix = '/depth'
        else:
            ros_depth_prefix = f'/{robot}/depth'
        for k in range(1, 9):
            gz_topic = (
                f'/world/{gz_world_value}/model/{model_name}/link/tof_base_link/sensor/tof_{k}/depth_image'
            )
            bridge_topics.append(f'{gz_topic}@sensor_msgs/msg/Image[gz.msgs.Image')
            bridge_remaps.append(f'{gz_topic}:={ros_depth_prefix}/tof_{k}')

    bridge_cmd: list[str] = [
        'ros2',
        'run',
        'ros_gz_bridge',
        'parameter_bridge',
        *bridge_topics,
        '--ros-args',
        '-r',
        '__node:=ros_gz_bridge',
    ]
    for remap in bridge_remaps:
        bridge_cmd.extend(['-r', remap])

    bridge = ExecuteProcess(
        cmd=bridge_cmd,
        output='screen',
        condition=IfCondition(LaunchConfiguration('run_bridge')),
    )

    # PX4 instances (one per robot).
    px4_bin = os.path.join(px4_dir, 'build', 'px4_sitl_default', 'bin', 'px4')
    px4_etc = os.path.join(px4_dir, 'build', 'px4_sitl_default', 'etc')
    rootfs_base = os.path.join(px4_dir, 'build', 'px4_sitl_default', 'rootfs_swarm')

    px4_processes: list[ExecuteProcess] = []
    for i, robot in enumerate(robots):
        x, y, yaw = spawns.get(robot, (0.0, -2.0, 0.0))
        z = float(LaunchConfiguration('spawn_z_m').perform(context))
        os.makedirs(rootfs_base, exist_ok=True)
        rootfs = os.path.join(rootfs_base, f'instance_{i}')
        os.makedirs(rootfs, exist_ok=True)

        px4_processes.append(
            ExecuteProcess(
                cmd=[px4_bin, '-i', str(i), '-d', px4_etc],
                cwd=rootfs,
                output='screen',
                additional_env={
                    'PX4_SIM_MODEL': 'gz_x500_small_tof',
                    'PX4_SIMULATOR': 'gz',
                    # Force a namespace for all instances (including 0) to avoid topic collisions.
                    'PX4_UXRCE_DDS_NS': f'px4_{i}',
                    'PX4_GZ_MODEL_POSE': f'{x},{y},{z},0,0,{yaw}',
                },
                condition=IfCondition(LaunchConfiguration('run_px4')),
            )
        )

    start_px4 = TimerAction(
        period=float(LaunchConfiguration('px4_start_delay_sec').perform(context)),
        actions=px4_processes,
        condition=IfCondition(LaunchConfiguration('run_px4')),
    )

    # Per-robot ROS nodes.
    per_robot_nodes: list[Node] = []
    scan_topics: list[str] = []
    for i, robot in enumerate(robots):
        ns = '' if robot == 'robot' else robot
        px4_ns = f'px4_{i}'

        model_name = f'x500_small_tof_{i}'
        pose_topic = f'/model/{model_name}/pose'

        scan_topic = '/scan_merged' if robot == 'robot' else f'/{robot}/scan_merged'
        scan_topics.append(scan_topic)

        per_robot_nodes.append(
            Node(
                package='tof_slam_sim',
                executable='gz_pose_info_to_pose_stamped',
                name=f'{robot}_gz_pose',
                output='screen',
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'gz_world_name': gz_world_name,
                    'entity_name': model_name,
                    'pose_topic': pose_topic,
                    'pose_frame_id': 'world',
                    'publish_hz': 30.0,
                }],
            )
        )

        per_robot_nodes.append(
            Node(
                package='tof_slam_sim',
                executable='pose_to_px4_visual_odometry',
                name=f'{robot}_px4_visual_odometry',
                output='screen',
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'pose_topic': pose_topic,
                    'px4_time_topic': f'/{px4_ns}/fmu/out/vehicle_odometry',
                    'px4_status_topic': f'/{px4_ns}/fmu/out/vehicle_status',
                    'px4_visual_odometry_topic': f'/{px4_ns}/fmu/in/vehicle_visual_odometry',
                    'publish_rate_hz': 30.0,
                }],
                condition=IfCondition(LaunchConfiguration('use_visual_odometry')),
            )
        )

        per_robot_nodes.append(
            Node(
                package='tof_slam_sim',
                executable='px4_vehicle_odometry_to_odom',
                name=f'{robot}_px4_vehicle_odometry_to_odom',
                namespace=ns,
                output='screen',
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'vehicle_odometry_topic': f'/{px4_ns}/fmu/out/vehicle_odometry',
                    'odom_topic': 'odom',
                    'frame_id': f'{robot}/odom',
                    'child_frame_id': f'{robot}/base_footprint',
                    'yaw_only': True,
                    'use_message_z': False,
                    'z_override': 0.0,
                }],
            )
        )

        per_robot_nodes.append(
            Node(
                package='tof_slam_sim',
                executable='tof_to_scan.py',
                name=f'{robot}_tof_to_scan',
                namespace=ns,
                output='screen',
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'depth_topic_prefix': 'depth',
                    'output_topic': 'scan_merged',
                    'output_frame': f'{robot}/base_link',
                    'viz_output_topic': 'scan_merged_viz',
                    'publish_hz': 10.0,
                    'min_range_m': 0.35,
                    'max_range_m': 4.0,
                    'output_num_points': 360,
                    'output_fill_bins': True,
                    'roi_row_start': 3,
                    'roi_row_end': 5,
                    'column_reduce': 'median',
                    'min_valid_per_column': 1,
                    'far_clip_margin_m': 0.03,
                    'no_return_as_range_max': True,
                    'angular_filter_window': 7,
                    'angular_outlier_thresh_m': 0.7,
                    'temporal_window': 5,
                }],
            )
        )

        per_robot_nodes.append(
            Node(
                package='tof_slam_sim',
                executable='twist_to_px4_offboard',
                name=f'{robot}_twist_to_px4_offboard',
                namespace=ns,
                output='screen',
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'cmd_vel_topic': 'cmd_vel',
                    'vehicle_odometry_topic': f'/{px4_ns}/fmu/out/vehicle_odometry',
                    'vehicle_local_position_topic': f'/{px4_ns}/fmu/out/vehicle_local_position',
                    'publish_rate_hz': 20.0,
                    'deadman_timeout_sec': 0.75,
                    'target_alt_m': LaunchConfiguration('target_alt_m'),
                    'max_alt_m': LaunchConfiguration('max_alt_m'),
                    'max_alt_fraction': LaunchConfiguration('max_alt_fraction'),
                    'auto_arm': True,
                    'auto_offboard': True,
                    'warmup_setpoints': 25,
                    'command_period_sec': 2.0,
                    'px4_target_system': i + 1,
                    'px4_target_component': 1,
                    'px4_source_system': i + 1,
                    'px4_source_component': 1,
                }],
                remappings=[
                    ('/fmu/in/offboard_control_mode', f'/{px4_ns}/fmu/in/offboard_control_mode'),
                    ('/fmu/in/trajectory_setpoint', f'/{px4_ns}/fmu/in/trajectory_setpoint'),
                    ('/fmu/in/vehicle_command', f'/{px4_ns}/fmu/in/vehicle_command'),
                    ('/fmu/out/vehicle_status', f'/{px4_ns}/fmu/out/vehicle_status'),
                    ('/fmu/out/failsafe_flags', f'/{px4_ns}/fmu/out/failsafe_flags'),
                ],
            )
        )

        # Exploration autopilot (one per robot).
        # nav_mode takes priority: autopilot mode enables, nav2 mode disables.
        run_autopilot = (nav_mode_val == 'autopilot') if nav_mode_val in ('nav2', 'autopilot') \
            else _truthy(LaunchConfiguration('run_autopilot').perform(context))
        if run_autopilot:
            per_robot_nodes.append(
                Node(
                    package='tof_slam_sim',
                    executable='auto_pilot',
                    name=f'{robot}_auto_pilot',
                    namespace=ns,
                    output='screen',
                    parameters=[{'use_sim_time': use_sim_time}],
                    additional_env={
                        'AP_MODE': 'explore',
                        'AP_TOPIC': 'cmd_vel',
                        'AP_ODOM': 'odom',
                        'AP_SCAN_TOPIC': 'scan_merged',
                        'AP_MAP_TOPIC': '/map',
                        'AP_MAP_FRAME': 'robot/map',
                        'AP_ODOM_FRAME': f'{robot}/odom',
                        'AP_EXP_ARENA_ENABLE': '1',
                        'AP_EXP_ARENA_MIN_X': str(arena_min_x),
                        'AP_EXP_ARENA_MAX_X': str(arena_max_x),
                        'AP_EXP_ARENA_MIN_Y': str(arena_min_y),
                        'AP_EXP_ARENA_MAX_Y': str(arena_max_y),
                        # Conservative defaults for swarm safety.
                        'AP_RATE': '10.0',
                        'AP_EXP_FORWARD': '0.35',
                        'AP_EXP_STRAFE': '0.14',
                        'AP_EXP_TURN': '0.45',
                        'AP_EXP_CLEAR': '1.55',
                        'AP_EXP_AVOID': '0.85',
                        'AP_EXP_INFLATE_M': '0.55',
                        'AP_EXP_BREADTH_FIRST': '1',
                        # A bit more stuck recovery for indoor swarms.
                        'AP_STUCK_CHECK_SEC': '2.0',
                        'AP_STUCK_MIN_MOVE_M': '0.20',
                    },
                )
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
            'odom_offsets': odom_offsets,
        }],
    )

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
            'min_x': min_x_val,
            'max_x': max_x_val,
            'min_y': min_y_val,
            'max_y': max_y_val,
            'dynamic_bounds': True,
            'expand_margin_m': 2.0,
            'seed_keepout': False,
            'publish_period_sec': 0.5,
            'max_range_override': 4.05,
        }],
    )

    health = Node(
        package='tof_slam_sim',
        executable='drone_health_dashboard',
        name='drone_health_dashboard',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robots': robots,
            'map_frame': 'robot/map',
            'ui': LaunchConfiguration('health_ui'),
            'publish_markers': LaunchConfiguration('publish_drone_markers'),
            'marker_topic': '/swarm/drone_markers',
        }],
        condition=IfCondition(PythonExpression([
            "('", LaunchConfiguration('health_ui'), "' == 'true') or ('", LaunchConfiguration('publish_drone_markers'), "' == 'true')"
        ])),
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(get_package_share_directory('tof_slam_sim'), 'config', 'slam_px4.rviz')],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    # Per-robot Nav2 stacks (launched when run_nav2:=true).
    # Skip when nav2_launched_externally:=true (px4_test_swarm launches via TimerAction with stagger).
    # Uses navigation_launch.py (NOT bringup_launch.py) because we provide
    # our own TF / odometry and don't want AMCL localization.
    nav2_launched_externally = _truthy(LaunchConfiguration('nav2_launched_externally').perform(context))
    nav2_actions: list = []
    if nav2_launched_externally:
        run_nav2 = False  # Will be launched by px4_test_swarm
    elif nav_mode_val == 'nav2':
        run_nav2 = True
    elif nav_mode_val == 'autopilot':
        run_nav2 = False
    else:
        run_nav2 = _truthy(LaunchConfiguration('run_nav2').perform(context))
    print(f'[px4_swarm_fast] nav_mode={nav_mode_val!r}  run_nav2={run_nav2}  run_autopilot={run_autopilot}')
    if run_nav2:
        nav2_launch_path = os.path.join(
            get_package_share_directory('nav2_bringup'), 'launch', 'navigation_launch.py'
        )
        nav2_params_path = str(LaunchConfiguration('nav2_params').perform(context)).strip()
        if not nav2_params_path or not os.path.isfile(nav2_params_path):
            nav2_params_path = os.path.join(
                get_package_share_directory('tof_slam_sim'), 'config', 'nav2_params_px4.yaml'
            )

        for r in robots:
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
                nav2_actions.append(
                    _GroupAction(actions=[PushRosNamespace(ns), nav2_include])
                )
            else:
                nav2_actions.append(nav2_include)

    print(f'[px4_swarm_fast] Nav2: {len(nav2_actions)} stacks to launch directly')

    result = [
        set_gz_resource_path,
        set_px4_rc_path,
        gz_sim,
        gz_gui,
        micro_xrce_agent,
        bridge,
        start_px4,
        swarm_tf,
        fuser,
        health,
        *per_robot_nodes,
        rviz,
        *nav2_actions,
    ]
    return result


def generate_launch_description() -> LaunchDescription:
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time (Gazebo /clock).',
    )
    declare_world = DeclareLaunchArgument(
        'world',
        default_value='playfield_competition.sdf',
        description='World file under tof_slam_sim/worlds (PX4-safe defaults).',
    )
    declare_gz_world_name = DeclareLaunchArgument(
        'gz_world_name',
        default_value='playfield',
        description='Gazebo world name (used for /world/<name>/clock and pose topics).',
    )
    declare_gz_gui = DeclareLaunchArgument(
        'gz_gui',
        default_value='false',
        description='Start Gazebo GUI (gz sim -g).',
    )
    declare_headless_rendering = DeclareLaunchArgument(
        'headless_rendering',
        default_value='false',
        description='Start Gazebo server with --headless-rendering (useful when no display is available).',
    )
    declare_rviz = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Launch RViz.',
    )
    declare_num_robots = DeclareLaunchArgument(
        'num_robots',
        default_value='5',
        description='If set (>=1), auto-generate robot,robot2..robotN and ignore `robots`.',
    )
    declare_robots = DeclareLaunchArgument(
        'robots',
        default_value='robot,robot2,robot3,robot4,robot5',
        description='Comma-separated robot names (used for ROS namespaces and TF prefixes).',
    )
    declare_default_spawn = DeclareLaunchArgument(
        'default_spawn',
        default_value='false',
        description='If true, skip the spawn UI and use default spawn points.',
    )
    declare_run_gz = DeclareLaunchArgument(
        'run_gz',
        default_value='true',
        description='Start Gazebo (gz sim) server.',
    )
    declare_run_px4 = DeclareLaunchArgument(
        'run_px4',
        default_value='true',
        description='Start PX4 SITL instances (one per robot).',
    )
    declare_run_bridge = DeclareLaunchArgument(
        'run_bridge',
        default_value='true',
        description='Start ros_gz_bridge for clock + ToF depth topics.',
    )
    declare_run_agent = DeclareLaunchArgument(
        'run_agent',
        default_value='true',
        description='Start MicroXRCEAgent (PX4 uXRCE-DDS).',
    )
    declare_use_visual_odometry = DeclareLaunchArgument(
        'use_visual_odometry',
        default_value='true',
        description='Publish Gazebo pose as PX4 vehicle_visual_odometry (EKF2 external vision aiding).',
    )
    declare_px4_dir = DeclareLaunchArgument(
        'px4_dir',
        default_value=_guess_px4_dir(),
        description='Path to the PX4-Autopilot directory.',
    )
    declare_spawn_z = DeclareLaunchArgument(
        'spawn_z_m',
        default_value='0.4',
        description='Initial spawn height (meters) for each vehicle; 0.4 avoids ground clipping.',
    )
    declare_px4_delay = DeclareLaunchArgument(
        'px4_start_delay_sec',
        default_value='2.0',
        description='Delay (seconds) before starting PX4 instances (lets gz sim bring up /world/*/clock).',
    )
    declare_target_alt = DeclareLaunchArgument(
        'target_alt_m',
        default_value='1.0',
        description='Target altitude (meters, +up in Gazebo/ROS ENU) for PX4 offboard (clamped by max_alt_*).',
    )
    declare_max_alt_m = DeclareLaunchArgument(
        'max_alt_m',
        default_value='2.0',
        description='Environment max height in meters (100%). PX4 offboard target altitude is clamped to max_alt_fraction * max_alt_m.',
    )
    declare_max_alt_fraction = DeclareLaunchArgument(
        'max_alt_fraction',
        default_value='0.6',
        description='Clamp PX4 offboard target altitude to this fraction of max_alt_m (e.g. 0.6 = keep within lower 60%).',
    )
    declare_run_autopilot = DeclareLaunchArgument(
        'run_autopilot',
        default_value='true',
        description='Run per-robot exploration autopilots (cmd_vel -> PX4 offboard).',
    )
    declare_health_ui = DeclareLaunchArgument(
        'health_ui',
        default_value='true',
        description='Show runtime drone health UI (tkinter).',
    )
    declare_publish_markers = DeclareLaunchArgument(
        'publish_drone_markers',
        default_value='true',
        description='Publish drone pose markers for RViz (/swarm/drone_markers).',
    )
    declare_nav_mode = DeclareLaunchArgument(
        'nav_mode',
        default_value='',
        description='Navigation mode: "nav2" or "autopilot". When set, overrides run_nav2 / run_autopilot.',
    )
    declare_run_nav2 = DeclareLaunchArgument(
        'run_nav2',
        default_value='false',
        description='Launch a Nav2 navigation stack per robot (for swarm_control drone_executor).',
    )
    declare_nav2_params = DeclareLaunchArgument(
        'nav2_params',
        default_value='',
        description='Nav2 params YAML (defaults to tof_slam_sim/config/nav2_params_px4.yaml).',
    )
    declare_nav2_launched_externally = DeclareLaunchArgument(
        'nav2_launched_externally',
        default_value='false',
        description='When true, skip launching Nav2 (used when px4_test_swarm launches it via TimerAction).',
    )

    build_swarm = OpaqueFunction(function=_build_swarm)

    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_world)
    ld.add_action(declare_gz_world_name)
    ld.add_action(declare_gz_gui)
    ld.add_action(declare_headless_rendering)
    ld.add_action(declare_rviz)
    ld.add_action(declare_num_robots)
    ld.add_action(declare_robots)
    ld.add_action(declare_default_spawn)
    ld.add_action(declare_run_gz)
    ld.add_action(declare_run_px4)
    ld.add_action(declare_run_bridge)
    ld.add_action(declare_run_agent)
    ld.add_action(declare_use_visual_odometry)
    ld.add_action(declare_px4_dir)
    ld.add_action(declare_spawn_z)
    ld.add_action(declare_px4_delay)
    ld.add_action(declare_target_alt)
    ld.add_action(declare_max_alt_m)
    ld.add_action(declare_max_alt_fraction)
    ld.add_action(declare_run_autopilot)
    ld.add_action(declare_health_ui)
    ld.add_action(declare_publish_markers)
    ld.add_action(declare_nav_mode)
    ld.add_action(declare_run_nav2)
    ld.add_action(declare_nav2_params)
    ld.add_action(declare_nav2_launched_externally)
    ld.add_action(build_swarm)
    return ld
