from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    EmitEvent,
    RegisterEventHandler,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.events import matches_action
from launch_ros.actions import Node, LifecycleNode
from launch_ros.substitutions import FindPackageShare
from launch_ros.events.lifecycle import ChangeState
from launch_ros.event_handlers import OnStateTransition
from lifecycle_msgs.msg import Transition


def generate_launch_description():
    pkg_tof_slam_sim = FindPackageShare('tof_slam_sim')

    use_sim_time = LaunchConfiguration('use_sim_time')
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time',
    )
    vehicle_odometry_topic = LaunchConfiguration('vehicle_odometry_topic')
    declare_vehicle_odometry_topic = DeclareLaunchArgument(
        'vehicle_odometry_topic',
        default_value='/fmu/out/vehicle_odometry',
        description='PX4 VehicleOdometry topic (commonly /fmu/out/vehicle_odometry or /fmu/out/estimator_odometry).',
    )

    px4_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([pkg_tof_slam_sim, 'launch', 'px4_sitl.launch.py'])
        ]),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
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

    slam_toolbox_params = PathJoinSubstitution(
        [pkg_tof_slam_sim, 'config', 'slam_toolbox.yaml']
    )

    slam_params_override = {
        'scan_topic': '/scan_merged',
        'odom_frame': 'robot/odom',
        'map_frame': 'robot/map',
        'base_frame': 'robot/base_link',
    }

    slam_toolbox = LifecycleNode(
        package='slam_toolbox',
        executable='sync_slam_toolbox_node',
        name='slam_toolbox',
        namespace='',
        output='screen',
        parameters=[
            slam_toolbox_params,
            slam_params_override,
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

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', PathJoinSubstitution([pkg_tof_slam_sim, 'config', 'slam_px4.rviz'])],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_vehicle_odometry_topic)
    ld.add_action(px4_launch)
    ld.add_action(px4_odom_to_odom)
    ld.add_action(odom_tf)
    ld.add_action(slam_toolbox)
    ld.add_action(slam_configure)
    ld.add_action(slam_activate)
    ld.add_action(static_basefoot_to_baselink)
    ld.add_action(rviz)
    return ld
