from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_tof_slam_sim = FindPackageShare('tof_slam_sim')

    use_sim_time = LaunchConfiguration('use_sim_time')
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time',
    )

    set_home = SetEnvironmentVariable('HOME', '/home/rex')
    set_ros_log_dir = SetEnvironmentVariable('ROS_LOG_DIR', '/home/rex/.ros/log')
    set_stdout = SetEnvironmentVariable('RCUTILS_LOGGING_USE_STDOUT', '1')

    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([pkg_tof_slam_sim, 'launch', 'sim_with_bridge.launch.py'])
        ])
    )

    monitor_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([pkg_tof_slam_sim, 'launch', 'topic_monitor.launch.py'])
        ])
    )

    scan_merger = Node(
        package='tof_slam_sim',
        executable='scan_merger',
        name='scan_merger',
        parameters=[{
            'use_sim_time': use_sim_time,
            'output_frame': 'robot/base_footprint',
            'output_topic': '/scan_merged',
            'publish_hz': 10.0,
        }],
    )

    slam_toolbox = Node(
        package='slam_toolbox',
        executable='sync_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            PathJoinSubstitution([pkg_tof_slam_sim, 'config', 'slam_toolbox.yaml']),
            {'use_sim_time': use_sim_time},
        ],
        remappings=[('scan', '/scan_merged')],
    )

    static_world_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='world_to_odom_static',
        arguments=[
            '--frame-id', 'world',
            '--child-frame-id', 'robot/odom',
            '--x', '0', '--y', '0', '--z', '0',
            '--roll', '0', '--pitch', '0', '--yaw', '0',
        ],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    map_tf_fallback = Node(
        package='tof_slam_sim',
        executable='map_tf_fallback',
        name='map_tf_fallback',
        parameters=[{
            'use_sim_time': use_sim_time,
            'parent_frame': 'robot/map',
            'child_frame': 'robot/odom',
            'rate_hz': 10.0,
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

    alias_robot_base_link_to_plain = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='alias_robot_base_link_to_plain',
        arguments=[
            '--frame-id', 'robot/base_link',
            '--child-frame-id', 'base_link',
            '--x', '0', '--y', '0', '--z', '0',
            '--roll', '0', '--pitch', '0', '--yaw', '0',
        ],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', PathJoinSubstitution([pkg_tof_slam_sim, 'config', 'slam.rviz'])],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time)
    ld.add_action(set_home)
    ld.add_action(set_ros_log_dir)
    ld.add_action(set_stdout)
    ld.add_action(sim_launch)
    ld.add_action(monitor_launch)
    ld.add_action(scan_merger)
    ld.add_action(slam_toolbox)
    ld.add_action(static_world_to_odom)
    ld.add_action(map_tf_fallback)
    ld.add_action(static_basefoot_to_baselink)
    ld.add_action(alias_robot_base_link_to_plain)
    ld.add_action(rviz)
    return ld
