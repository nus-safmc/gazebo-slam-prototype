import os
import xml.etree.ElementTree as ET

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.events import matches_action
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node, LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from launch_ros.substitutions import FindPackageShare
from lifecycle_msgs.msg import Transition


def _parse_pose_xy(pose_text: str) -> tuple[float, float] | None:
    parts = pose_text.strip().split()
    if len(parts) < 2:
        return None
    try:
        return float(parts[0]), float(parts[1])
    except ValueError:
        return None


def _parse_size_xyz(size_text: str) -> tuple[float, float, float] | None:
    parts = size_text.strip().split()
    if len(parts) < 3:
        return None
    try:
        return float(parts[0]), float(parts[1]), float(parts[2])
    except ValueError:
        return None


def _infer_arena_bounds(world_path: str, *, margin_m: float) -> tuple[float, float, float, float] | None:
    try:
        tree = ET.parse(world_path)
    except (ET.ParseError, OSError):
        return None

    root = tree.getroot()

    def _model(name: str):
        return root.find(f".//model[@name='{name}']")

    def _box_size(model):
        size_el = model.find(".//collision//geometry//box//size")
        if size_el is None:
            size_el = model.find(".//visual//geometry//box//size")
        if size_el is None or not size_el.text:
            return None
        return _parse_size_xyz(size_el.text)

    def _plane_size(model):
        size_el = model.find(".//collision//geometry//plane//size")
        if size_el is None:
            size_el = model.find(".//visual//geometry//plane//size")
        if size_el is not None and size_el.text:
            parts = size_el.text.strip().split()
            if len(parts) >= 2:
                try:
                    return float(parts[0]), float(parts[1])
                except ValueError:
                    pass

        box = _box_size(model)
        if box is not None:
            return box[0], box[1]
        return None

    # Prefer perimeter wall models so we can account for wall thickness.
    north = _model("perimeter_wall_north")
    south = _model("perimeter_wall_south")
    east = _model("perimeter_wall_east")
    west = _model("perimeter_wall_west")
    if all(m is not None for m in (north, south, east, west)):
        north_pose = _parse_pose_xy(north.findtext("pose", default=""))
        south_pose = _parse_pose_xy(south.findtext("pose", default=""))
        east_pose = _parse_pose_xy(east.findtext("pose", default=""))
        west_pose = _parse_pose_xy(west.findtext("pose", default=""))
        north_size = _box_size(north)
        south_size = _box_size(south)
        east_size = _box_size(east)
        west_size = _box_size(west)

        if all(v is not None for v in (north_pose, south_pose, east_pose, west_pose, north_size, south_size, east_size, west_size)):
            _, north_y = north_pose
            _, south_y = south_pose
            east_x, _ = east_pose
            west_x, _ = west_pose
            _, north_thick_y, _ = north_size
            _, south_thick_y, _ = south_size
            west_thick_x, _, _ = west_size
            east_thick_x, _, _ = east_size

            min_x = (west_x + 0.5 * west_thick_x) + margin_m
            max_x = (east_x - 0.5 * east_thick_x) - margin_m
            min_y = (south_y + 0.5 * south_thick_y) + margin_m
            max_y = (north_y - 0.5 * north_thick_y) - margin_m

            if min_x < max_x and min_y < max_y:
                return min_x, max_x, min_y, max_y

    # Fallback: use ground plane size if present (assumes centered at origin).
    ground = _model("ground_plane")
    if ground is None:
        return None
    plane = _plane_size(ground)
    if plane is None:
        return None
    size_x, size_y = plane
    half_x = 0.5 * size_x
    half_y = 0.5 * size_y
    min_x = (-half_x) + margin_m
    max_x = (half_x) - margin_m
    min_y = (-half_y) + margin_m
    max_y = (half_y) - margin_m
    if min_x < max_x and min_y < max_y:
        return min_x, max_x, min_y, max_y
    return None


def _infer_ground_plane_bounds(world_path: str) -> tuple[float, float, float, float] | None:
    try:
        tree = ET.parse(world_path)
    except (ET.ParseError, OSError):
        return None

    root = tree.getroot()
    ground = root.find(".//model[@name='ground_plane']")
    if ground is None:
        return None

    size_x: float | None = None
    size_y: float | None = None

    size_el = ground.find(".//collision//geometry//plane//size")
    if size_el is None:
        size_el = ground.find(".//visual//geometry//plane//size")
    if size_el is not None and size_el.text:
        parts = size_el.text.strip().split()
        if len(parts) >= 2:
            try:
                size_x = float(parts[0])
                size_y = float(parts[1])
            except ValueError:
                size_x = None
                size_y = None

    if size_x is None or size_y is None:
        box_el = ground.find(".//collision//geometry//box//size")
        if box_el is None:
            box_el = ground.find(".//visual//geometry//box//size")
        if box_el is None or not box_el.text:
            return None
        box = _parse_size_xyz(box_el.text)
        if box is None:
            return None
        size_x, size_y, _ = box

    half_x = 0.5 * size_x
    half_y = 0.5 * size_y
    return -half_x, half_x, -half_y, half_y


def generate_launch_description():
    pkg_share = FindPackageShare('tof_slam_sim')
    nav2_share = FindPackageShare('nav2_bringup')

    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('nav2_params')
    use_rviz = LaunchConfiguration('rviz')
    world = LaunchConfiguration('world')
    gz_world_name = LaunchConfiguration('gz_world_name')
    gz_gui = LaunchConfiguration('gz_gui')
    target_alt_m = LaunchConfiguration('target_alt_m')
    max_alt_m = LaunchConfiguration('max_alt_m')
    max_alt_fraction = LaunchConfiguration('max_alt_fraction')
    vehicle_odometry_topic = LaunchConfiguration('vehicle_odometry_topic')
    monitor = LaunchConfiguration('monitor')
    px4_model_pose = LaunchConfiguration('px4_model_pose')
    slam_config = LaunchConfiguration('slam_config')
    arena_enabled = LaunchConfiguration('arena_enabled')
    arena_min_x = LaunchConfiguration('arena_min_x')
    arena_max_x = LaunchConfiguration('arena_max_x')
    arena_min_y = LaunchConfiguration('arena_min_y')
    arena_max_y = LaunchConfiguration('arena_max_y')
    arena_margin_m = LaunchConfiguration('arena_margin_m')
    mapping_backend = LaunchConfiguration('mapping_backend')
    nav2_delay_sec = LaunchConfiguration('nav2_delay_sec')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time',
    )
    declare_nav2_params = DeclareLaunchArgument(
        'nav2_params',
        default_value=PathJoinSubstitution([pkg_share, 'config', 'nav2_params_rex.yaml']),
        description='Nav2 params YAML for controller/costmaps/behavior tree.',
    )
    declare_rviz = DeclareLaunchArgument(
        'rviz',
        default_value='false',
        description='Launch RViz for visualization.',
    )
    declare_world = DeclareLaunchArgument(
        'world',
        default_value='playfield_px4_sparse.sdf',
        description='World file under tof_slam_sim/worlds (PX4-safe defaults: playfield_px4_sparse.sdf).',
    )
    declare_gz_world_name = DeclareLaunchArgument(
        'gz_world_name',
        default_value='playfield',
        description='Gazebo world name (used for /world/<name>/clock).',
    )
    declare_gz_gui = DeclareLaunchArgument(
        'gz_gui',
        default_value='false',
        description='Start Gazebo GUI (gz sim -g).',
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
    declare_px4_model_pose = DeclareLaunchArgument(
        'px4_model_pose',
        default_value='0,-2,0.2,0,0,0',
        description='PX4_GZ_MODEL_POSE for spawning the vehicle (x,y,z,roll,pitch,yaw).',
    )
    declare_vehicle_odometry_topic = DeclareLaunchArgument(
        'vehicle_odometry_topic',
        default_value='/fmu/out/vehicle_odometry',
        description='PX4 VehicleOdometry topic (commonly /fmu/out/vehicle_odometry or /fmu/out/estimator_odometry).',
    )
    declare_monitor = DeclareLaunchArgument(
        'monitor',
        default_value='true',
        description='Run topic/TF health monitor (prints warnings every few seconds).',
    )
    declare_slam_config = DeclareLaunchArgument(
        'slam_config',
        default_value='slam_toolbox_px4_robust.yaml',
        description='SLAM Toolbox params file under tof_slam_sim/config (try slam_toolbox_px4_robust.yaml or slam_toolbox_px4_fast.yaml).',
    )
    declare_arena_enabled = DeclareLaunchArgument(
        'arena_enabled',
        default_value='true',
        description='Constrain exploration goals to within the playfield bounds (recommended).',
    )
    declare_arena_min_x = DeclareLaunchArgument(
        'arena_min_x',
        default_value='auto',
        description='Playfield min X in map frame. Use "auto" to infer from the SDF perimeter walls / ground plane.',
    )
    declare_arena_max_x = DeclareLaunchArgument(
        'arena_max_x',
        default_value='auto',
        description='Playfield max X in map frame. Use "auto" to infer from the SDF perimeter walls / ground plane.',
    )
    declare_arena_min_y = DeclareLaunchArgument(
        'arena_min_y',
        default_value='auto',
        description='Playfield min Y in map frame. Use "auto" to infer from the SDF perimeter walls / ground plane.',
    )
    declare_arena_max_y = DeclareLaunchArgument(
        'arena_max_y',
        default_value='auto',
        description='Playfield max Y in map frame. Use "auto" to infer from the SDF perimeter walls / ground plane.',
    )
    declare_arena_margin_m = DeclareLaunchArgument(
        'arena_margin_m',
        default_value='0.8',
        description='Inset the inferred arena bounds by this margin (meters) to keep goals away from perimeter walls.',
    )
    declare_mapping_backend = DeclareLaunchArgument(
        'mapping_backend',
        default_value='fuser',
        description='Mapping backend: "fuser" (raytrace /scan_merged into fixed bounds) or "slam_toolbox" (Karto SLAM).',
    )
    declare_nav2_delay_sec = DeclareLaunchArgument(
        'nav2_delay_sec',
        default_value='20.0',
        description='Delay (seconds) before starting Nav2 + explorer (wait for PX4 odom / TF).',
    )

    px4_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([pkg_share, 'launch', 'px4_sitl.launch.py'])
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'world': world,
            'gz_world_name': gz_world_name,
            'gz_gui': gz_gui,
            'px4_model_pose': px4_model_pose,
        }.items(),
    )

    px4_odom_to_odom = Node(
        package='tof_slam_sim',
        executable='px4_vehicle_odometry_to_odom',
        name='px4_vehicle_odometry_to_odom',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'vehicle_odometry_topic': vehicle_odometry_topic,
            'odom_topic': '/odom',
            'frame_id': 'robot/odom',
            'child_frame_id': 'robot/base_footprint',
            'yaw_only': True,
            'use_message_z': False,
            'z_override': 0.0,
        }],
    )

    odom_tf = Node(
        package='tof_slam_sim',
        executable='odom_tf_publisher',
        name='odom_tf_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'odom_topic': '/odom',
            'parent_frame': 'robot/odom',
            'child_frame': 'robot/base_footprint',
            'yaw_only': True,
            'use_message_z': False,
            'z_override': 0.0,
            'smoothing_alpha': 1.0,
            # Avoid publishing a bogus (0,0) transform before PX4 odometry is available.
            # Nav2 and mapping should wait for real odom instead of planning from the origin.
            'fallback_rate_hz': 0.0,
        }],
    )

    static_basefoot_to_baselink = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='robot_basefoot_to_baselink',
        arguments=[
            '--frame-id', 'robot/base_footprint',
            '--child-frame-id', 'robot/base_link',
            '--x', '0', '--y', '0', '--z', '0',
            '--roll', '0', '--pitch', '0', '--yaw', '0',
        ],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    mapping_is_slam = IfCondition(PythonExpression(["'", mapping_backend, "' == 'slam_toolbox'"]))
    mapping_is_fuser = IfCondition(PythonExpression(["'", mapping_backend, "' == 'fuser'"]))

    slam_toolbox_params = PathJoinSubstitution([pkg_share, 'config', slam_config])
    slam_toolbox = LifecycleNode(
        package='slam_toolbox',
        executable='sync_slam_toolbox_node',
        name='slam_toolbox',
        namespace='',
        output='screen',
        parameters=[
            slam_toolbox_params,
            {
                # Use the raw merged scan for SLAM. It uses `inf` for "no return" beams,
                # which slam_toolbox treats as free space up to `range_max` (without creating
                # a phantom ring obstacle at max range, which happens if we substitute inf).
                'scan_topic': '/scan_merged',
                'odom_frame': 'robot/odom',
                'map_frame': 'robot/map',
                'base_frame': 'robot/base_footprint',
            },
            {'use_sim_time': use_sim_time},
        ],
        remappings=[
            ('scan', '/scan_merged'),
            ('map', '/slam_map'),
            ('map_metadata', '/slam_map_metadata'),
            ('map_updates', '/slam_map_updates'),
        ],
        condition=mapping_is_slam,
    )

    slam_lifecycle = GroupAction(
        actions=[
            TimerAction(
                period=1.0,
                actions=[
                    EmitEvent(
                        event=ChangeState(
                            lifecycle_node_matcher=matches_action(slam_toolbox),
                            transition_id=Transition.TRANSITION_CONFIGURE,
                        )
                    )
                ],
            ),
            RegisterEventHandler(
                OnStateTransition(
                    target_lifecycle_node=slam_toolbox,
                    goal_state='inactive',
                    entities=[
                        EmitEvent(
                            event=ChangeState(
                                lifecycle_node_matcher=matches_action(slam_toolbox),
                                transition_id=Transition.TRANSITION_ACTIVATE,
                            )
                        )
                    ],
                )
            ),
        ],
        condition=mapping_is_slam,
    )

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([nav2_share, 'launch', 'navigation_launch.py'])
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'autostart': 'true',
            'params_file': params_file,
            'log_level': 'info',
        }.items(),
    )

    def _make_map_padder(context, *args, **kwargs):
        world_arg = world.perform(context)

        share_dir = FindPackageShare('tof_slam_sim').perform(context)
        default_world_path = os.path.join(share_dir, 'worlds', world_arg)
        if os.path.isabs(world_arg):
            world_path = world_arg
        elif os.path.exists(world_arg):
            world_path = world_arg
        else:
            world_path = default_world_path

        bounds = _infer_ground_plane_bounds(world_path)
        if bounds is None:
            min_x_val, max_x_val, min_y_val, max_y_val = (-20.0, 20.0, -20.0, 20.0)
        else:
            min_x_val, max_x_val, min_y_val, max_y_val = bounds

        padder = Node(
            package='tof_slam_sim',
            executable='arena_map_padder',
            name='arena_map_padder',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'arena_min_x': float(min_x_val),
                'arena_max_x': float(max_x_val),
                'arena_min_y': float(min_y_val),
                'arena_max_y': float(max_y_val),
                'resolution': 0.05,
                'input_map_topic': '/slam_map',
                'input_update_topic': '/slam_map_updates',
                'output_map_topic': '/map',
                'output_update_topic': '/map_updates',
                'frame_id': 'robot/map',
                'publish_period_sec': 1.0,
            }],
        )
        return [padder]

    def _make_fuser(context, *args, **kwargs):
        world_arg = world.perform(context)

        share_dir = FindPackageShare('tof_slam_sim').perform(context)
        default_world_path = os.path.join(share_dir, 'worlds', world_arg)
        if os.path.isabs(world_arg):
            world_path = world_arg
        elif os.path.exists(world_arg):
            world_path = world_arg
        else:
            world_path = default_world_path

        # For realism, start with a small local map around the spawn pose and let the
        # fuser expand as the vehicle explores (instead of publishing the full arena
        # bounds as unknown at t=0).
        pose_text = (px4_model_pose.perform(context) or '').strip()
        pose_parts = [p.strip() for p in pose_text.split(',') if p.strip()]
        try:
            spawn_x = float(pose_parts[0])
            spawn_y = float(pose_parts[1])
        except (IndexError, ValueError):
            spawn_x = 0.0
            spawn_y = -2.0

        # Keep this slightly larger than the ToF max range (4m) + expand_margin_m (2m)
        # so we don't resize immediately at startup.
        initial_half_extent_m = 7.0
        min_x_val = spawn_x - initial_half_extent_m
        max_x_val = spawn_x + initial_half_extent_m
        min_y_val = spawn_y - initial_half_extent_m
        max_y_val = spawn_y + initial_half_extent_m

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
                'robots': ['robot'],
                'scan_topics': ['/scan_merged'],
                'resolution': 0.05,
                'min_x': float(min_x_val),
                'max_x': float(max_x_val),
                'min_y': float(min_y_val),
                'max_y': float(max_y_val),
                # Grow the map as we explore (more realistic than publishing the full
                # arena bounds from the start).
                'dynamic_bounds': True,
                'expand_margin_m': 2.0,
                # Do NOT seed a keepout border in dynamic mode; it would create a fake wall
                # at the current map edge and reveals the arena bounds before mapping them.
                'seed_keepout': False,
                'publish_period_sec': 0.5,
                # Treat ToF far-clip / no-return beams as no-hit endpoints so we don't
                # create a phantom obstacle ring at max range.
                'max_range_override': 4.05,
            }],
        )
        return [fuser]

    def _make_explorer(context, *args, **kwargs):
        world_arg = world.perform(context)

        share_dir = FindPackageShare('tof_slam_sim').perform(context)
        default_world_path = os.path.join(share_dir, 'worlds', world_arg)
        if os.path.isabs(world_arg):
            world_path = world_arg
        elif os.path.exists(world_arg):
            world_path = world_arg
        else:
            world_path = default_world_path

        def _parse_auto(value: str) -> float | None:
            v = (value or '').strip().lower()
            if v in ('', 'auto'):
                return None
            try:
                return float(v)
            except ValueError:
                return None

        enabled_str = (arena_enabled.perform(context) or '').strip().lower()
        arena_enabled_val = enabled_str not in ('false', '0', 'no', 'off')

        try:
            margin_val = float(arena_margin_m.perform(context))
        except ValueError:
            margin_val = 0.8
        margin_val = max(0.0, min(5.0, margin_val))

        user_min_x = _parse_auto(arena_min_x.perform(context))
        user_max_x = _parse_auto(arena_max_x.perform(context))
        user_min_y = _parse_auto(arena_min_y.perform(context))
        user_max_y = _parse_auto(arena_max_y.perform(context))

        bounds = None
        if user_min_x is None and user_max_x is None and user_min_y is None and user_max_y is None:
            bounds = _infer_arena_bounds(world_path, margin_m=margin_val)

        if bounds is None:
            # Keep previous behavior as a fallback: match the 40×40 playfield walls at +/-20.
            min_x_val = user_min_x if user_min_x is not None else -19.0
            max_x_val = user_max_x if user_max_x is not None else 19.0
            min_y_val = user_min_y if user_min_y is not None else -19.0
            max_y_val = user_max_y if user_max_y is not None else 19.0
        else:
            min_x_val, max_x_val, min_y_val, max_y_val = bounds
            if user_min_x is not None:
                min_x_val = user_min_x
            if user_max_x is not None:
                max_x_val = user_max_x
            if user_min_y is not None:
                min_y_val = user_min_y
            if user_max_y is not None:
                max_y_val = user_max_y

        # Ensure sane ordering even if user overrides are swapped.
        if min_x_val > max_x_val:
            min_x_val, max_x_val = max_x_val, min_x_val
        if min_y_val > max_y_val:
            min_y_val, max_y_val = max_y_val, min_y_val

        explorer = Node(
            package='tof_slam_sim',
            executable='nav2_frontier_explorer',
            name='nav2_frontier_explorer',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'map_topic': '/map',
                'goal_frame': 'robot/map',
                'base_frame': 'robot/base_footprint',
                # Be conservative: keep more clearance around obstacles for the PX4 x500 model.
                'robot_radius_m': 0.30,
                'min_frontier_cluster_size': 6,
                # Keep this high: the PX4+Nav2 stack is intentionally slow/steady (see
                # nav2_params_rex.yaml max speeds). If this timeout is too small, goals a
                # few meters away will be canceled before the vehicle can reach them,
                # preventing exploration from expanding the map to the full playfield.
                'goal_timeout_sec': 200.0,
                'replan_period_sec': 2.5,
                # Pick goals further inside free space and penalize near-obstacle frontiers.
                'goal_offset_m': 1.0,
                'clearance_min_m': 0.9,
                'clearance_weight': 10.0,
                'clearance_penalty': 20.0,
                'arena_enabled': arena_enabled_val,
                'arena_min_x': float(min_x_val),
                'arena_max_x': float(max_x_val),
                'arena_min_y': float(min_y_val),
                'arena_max_y': float(max_y_val),
            }],
        )
        return [explorer]

    px4_offboard = Node(
        package='tof_slam_sim',
        executable='twist_to_px4_offboard',
        name='twist_to_px4_offboard',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'cmd_vel_topic': '/cmd_vel',
            'target_alt_m': target_alt_m,
            'max_alt_m': max_alt_m,
            'max_alt_fraction': max_alt_fraction,
            'vehicle_odometry_topic': vehicle_odometry_topic,
        }],
    )

    monitor_node = Node(
        package='tof_slam_sim',
        executable='topic_monitor',
        name='topic_monitor',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'report_period_sec': 5.0,
            'stale_seconds': 3.0,
            'topics': [
                '/clock:rosgraph_msgs/msg/Clock:best_effort',
                '/fmu/out/vehicle_odometry:px4_msgs/msg/VehicleOdometry:best_effort',
                '/fmu/out/vehicle_status:px4_msgs/msg/VehicleStatus:best_effort',
                '/fmu/in/offboard_control_mode:px4_msgs/msg/OffboardControlMode:best_effort',
                '/fmu/in/trajectory_setpoint:px4_msgs/msg/TrajectorySetpoint:best_effort',
                '/model/x500_small_tof_0/pose:geometry_msgs/msg/PoseStamped:best_effort',
                '/odom:nav_msgs/msg/Odometry:reliable',
                '/tf:tf2_msgs/msg/TFMessage:reliable',
                '/tf_static:tf2_msgs/msg/TFMessage:latched',
                '/scan_merged:sensor_msgs/msg/LaserScan:best_effort',
                '/scan_merged_viz:sensor_msgs/msg/LaserScan:best_effort',
                '/map:nav_msgs/msg/OccupancyGrid:reliable',
                '/map_updates:map_msgs/msg/OccupancyGridUpdate:reliable',
                '/cmd_vel:geometry_msgs/msg/Twist:reliable',
            ],
            'required_transforms': [
                'robot/map->robot/odom',
                'robot/odom->robot/base_footprint',
                'robot/map->robot/base_footprint',
            ],
        }],
        condition=IfCondition(monitor),
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', PathJoinSubstitution([pkg_share, 'config', 'slam_px4.rviz'])],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(use_rviz),
    )

    map_tf_fallback = Node(
        package='tof_slam_sim',
        executable='map_tf_fallback',
        name='map_tf_fallback',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'parent_frame': 'robot/map',
            'child_frame': 'robot/odom',
            'use_static': True,
        }],
        condition=mapping_is_fuser,
    )

    nav2_and_explorer = TimerAction(
        period=nav2_delay_sec,
        actions=[
            nav2_launch,
            OpaqueFunction(function=_make_explorer),
        ],
    )

    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_nav2_params)
    ld.add_action(declare_rviz)
    ld.add_action(declare_world)
    ld.add_action(declare_gz_world_name)
    ld.add_action(declare_gz_gui)
    ld.add_action(declare_target_alt)
    ld.add_action(declare_max_alt_m)
    ld.add_action(declare_max_alt_fraction)
    ld.add_action(declare_vehicle_odometry_topic)
    ld.add_action(declare_monitor)
    ld.add_action(declare_px4_model_pose)
    ld.add_action(declare_slam_config)
    ld.add_action(declare_arena_enabled)
    ld.add_action(declare_arena_min_x)
    ld.add_action(declare_arena_max_x)
    ld.add_action(declare_arena_min_y)
    ld.add_action(declare_arena_max_y)
    ld.add_action(declare_arena_margin_m)
    ld.add_action(declare_mapping_backend)
    ld.add_action(declare_nav2_delay_sec)
    ld.add_action(px4_launch)
    ld.add_action(px4_odom_to_odom)
    ld.add_action(odom_tf)
    ld.add_action(static_basefoot_to_baselink)
    ld.add_action(map_tf_fallback)
    ld.add_action(OpaqueFunction(function=_make_fuser, condition=mapping_is_fuser))
    ld.add_action(OpaqueFunction(function=_make_map_padder, condition=mapping_is_slam))
    ld.add_action(slam_toolbox)
    ld.add_action(slam_lifecycle)
    ld.add_action(nav2_and_explorer)
    ld.add_action(px4_offboard)
    ld.add_action(monitor_node)
    ld.add_action(rviz)
    return ld
