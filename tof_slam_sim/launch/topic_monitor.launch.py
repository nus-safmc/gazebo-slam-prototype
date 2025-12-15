from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time',
    )
    sensors = ['front', 'front_right', 'right', 'back_right', 'back', 'back_left', 'left', 'front_left']
    topics = [
        '/cmd_vel:geometry_msgs/msg/Twist:reliable',
        '/odom:nav_msgs/msg/Odometry:reliable',
        '/tf:tf2_msgs/msg/TFMessage:reliable',
        '/tf_static:tf2_msgs/msg/TFMessage:latched',
        '/map:nav_msgs/msg/OccupancyGrid:reliable+latched',
    ] + [f'/scan/{name}:sensor_msgs/msg/LaserScan:best_effort' for name in sensors]
    topics.append('/scan_merged:sensor_msgs/msg/LaserScan:best_effort')

    monitor_node = Node(
        package='tof_slam_sim',
        executable='topic_monitor',
        name='topic_monitor',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'topics': topics,
            'report_period_sec': 5.0,
            'stale_seconds': 5.0,
        }]
    )

    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time)
    ld.add_action(monitor_node)
    return ld
