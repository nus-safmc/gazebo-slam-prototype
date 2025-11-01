from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    sensors = ['front', 'front_right', 'right', 'back_right', 'back', 'back_left', 'left', 'front_left']
    topics = [
        '/cmd_vel:geometry_msgs/msg/Twist:reliable',
        '/odom:nav_msgs/msg/Odometry:reliable',
        '/tf:tf2_msgs/msg/TFMessage:reliable',
        '/tf_static:tf2_msgs/msg/TFMessage:latched',
    ] + [f'/scan/{name}:sensor_msgs/msg/LaserScan:best_effort' for name in sensors]

    monitor_node = Node(
        package='tof_slam_sim',
        executable='topic_monitor',
        name='topic_monitor',
        output='screen',
        parameters=[{
            'topics': topics,
            'report_period_sec': 5.0,
            'stale_seconds': 5.0,
        }]
    )

    ld = LaunchDescription()
    ld.add_action(monitor_node)
    return ld
