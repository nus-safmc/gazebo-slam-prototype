from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    IncludeLaunchDescription,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.events import matches_action
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from launch_ros.substitutions import FindPackageShare
from lifecycle_msgs.msg import Transition


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
    vehicle_odometry_topic = LaunchConfiguration('vehicle_odometry_topic')
    monitor = LaunchConfiguration('monitor')
    px4_model_pose = LaunchConfiguration('px4_model_pose')
    slam_config = LaunchConfiguration('slam_config')

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
        default_value='playfield_px4_small.sdf',
        description='World file under tof_slam_sim/worlds (PX4-safe defaults: playfield_px4_small.sdf).',
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
        description='Target altitude (meters, +up in Gazebo/ROS ENU) for PX4 offboard.',
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
                'scan_topic': '/scan_merged',
                'odom_frame': 'robot/odom',
                'map_frame': 'robot/map',
                'base_frame': 'robot/base_footprint',
            },
            {'use_sim_time': use_sim_time},
        ],
        remappings=[('scan', '/scan_merged')],
    )

    slam_configure = TimerAction(
        period=1.0,
        actions=[
            EmitEvent(
                event=ChangeState(
                    lifecycle_node_matcher=matches_action(slam_toolbox),
                    transition_id=Transition.TRANSITION_CONFIGURE,
                )
            )
        ],
    )
    slam_activate = RegisterEventHandler(
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
            'goal_timeout_sec': 35.0,
            'replan_period_sec': 2.5,
            # Pick goals further inside free space and penalize near-obstacle frontiers.
            'goal_offset_m': 1.0,
            'clearance_min_m': 0.9,
            'clearance_weight': 10.0,
            'clearance_penalty': 20.0,
            # Match `playfield_px4_small.sdf` interior bounds (walls at +/-3 with 0.4m thickness).
            'arena_min_x': -2.6,
            'arena_max_x': 2.6,
            'arena_min_y': -2.6,
            'arena_max_y': 2.6,
        }],
    )

    px4_offboard = Node(
        package='tof_slam_sim',
        executable='twist_to_px4_offboard',
        name='twist_to_px4_offboard',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'cmd_vel_topic': '/cmd_vel',
            'target_alt_m': target_alt_m,
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
                '/map:nav_msgs/msg/OccupancyGrid:reliable',
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

    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_nav2_params)
    ld.add_action(declare_rviz)
    ld.add_action(declare_world)
    ld.add_action(declare_gz_world_name)
    ld.add_action(declare_gz_gui)
    ld.add_action(declare_target_alt)
    ld.add_action(declare_vehicle_odometry_topic)
    ld.add_action(declare_monitor)
    ld.add_action(declare_px4_model_pose)
    ld.add_action(declare_slam_config)
    ld.add_action(px4_launch)
    ld.add_action(px4_odom_to_odom)
    ld.add_action(odom_tf)
    ld.add_action(static_basefoot_to_baselink)
    ld.add_action(slam_toolbox)
    ld.add_action(slam_configure)
    ld.add_action(slam_activate)
    ld.add_action(nav2_launch)
    ld.add_action(explorer)
    ld.add_action(px4_offboard)
    ld.add_action(monitor_node)
    ld.add_action(rviz)
    return ld
