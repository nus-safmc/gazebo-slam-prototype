from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_tof_slam_sim = FindPackageShare('tof_slam_sim')

    # --- Launch args ---
    use_sim_time = LaunchConfiguration('use_sim_time')
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )

    # --- World + bridge ---
    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([pkg_tof_slam_sim, 'launch', 'sim_with_bridge.launch.py'])
        ])
    )

    # --- Topic monitor (unchanged) ---
    monitor_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([pkg_tof_slam_sim, 'launch', 'topic_monitor.launch.py'])
        ])
    )

    # --- Scan merger: publish with a TF-valid frame + safe stamps, to the topic RViz uses ---
    scan_merger = Node(
        package='tof_slam_sim',
        executable='scan_merger.py',
        name='scan_merger',
        parameters=[{
            'use_sim_time': use_sim_time,
            'output_frame': 'robot/base_footprint',
            'output_topic': '/scan_merged',   # match RViz LaserScan topic
            'stamp_policy': 'max'             # newest stamp avoids “too old” TF lookups
        }]
    )

    # --- SLAM Toolbox (unchanged) ---
    slam_toolbox = Node(
        package='slam_toolbox',
        executable='sync_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            PathJoinSubstitution([pkg_tof_slam_sim, 'config', 'slam_toolbox.yaml']),
            {'use_sim_time': use_sim_time}
        ]
    )

    # --- Seed TF so RViz can render immediately ---
    static_world_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='world_to_odom_static',
        arguments=[
            '--frame-id', 'world',
            '--child-frame-id', 'robot/odom',
            '--x','0','--y','0','--z','0','--roll','0','--pitch','0','--yaw','0',
        ],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # Publish identity map->odom until slam_toolbox owns it
    map_tf_fallback = Node(
        package='tof_slam_sim',
        executable='map_tf_fallback',
        name='map_tf_fallback',
        parameters=[{
            'use_sim_time': use_sim_time,
            'parent_frame': 'robot/map',
            'child_frame': 'robot/odom'
        }]
    )

    # Keep base_footprint -> base_link (zero transform)
    static_basefoot_to_baselink = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='robot_basefoot_to_baselink',
        arguments=[
            '--frame-id', 'robot/base_footprint',
            '--child-frame-id', 'robot/base_link',
            '--x','0','--y','0','--z','0','--roll','0','--pitch','0','--yaw','0',
        ],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # NEW: alias robot/base_link -> base_link so any un-prefixed scans still resolve
    static_alias_base_link = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='alias_robot_base_link_to_plain',
        arguments=[
            '--frame-id', 'robot/base_link',
            '--child-frame-id', 'base_link',
            '--x','0','--y','0','--z','0','--roll','0','--pitch','0','--yaw','0',
        ],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # --- RViz ---
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', PathJoinSubstitution([pkg_tof_slam_sim, 'config', 'slam.rviz'])]
    )

    # Assemble LD
    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time)
    ld.add_action(sim_launch)
    ld.add_action(monitor_launch)
    ld.add_action(scan_merger)
    ld.add_action(slam_toolbox)
    ld.add_action(static_world_to_odom)
    ld.add_action(map_tf_fallback)
    ld.add_action(static_basefoot_to_baselink)
    ld.add_action(static_alias_base_link)
    ld.add_action(rviz)
    return ld
