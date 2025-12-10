import os
from datetime import datetime
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, ThisLaunchFileDir, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    # Path to your existing sim launch file
    INCLUDE_SIM_LAUNCH = PathJoinSubstitution([
        ThisLaunchFileDir(),
        'YOUR_CURRENT_SIM.launch.py'  # <-- replace with your sim launch
    ])

    # Topics to log
    default_topics = [
        '/cmd_vel:geometry_msgs/msg/Twist',
        '/odom:nav_msgs/msg/Odometry',
        '/tf:tf2_msgs/msg/TFMessage',
        '/tf_static:tf2_msgs/msg/TFMessage'
    ]

    log_dir_arg = DeclareLaunchArgument('log_dir', default_value=os.path.expanduser('~/.ros/monitor_logs'))
    log_name_arg = DeclareLaunchArgument('log_name', default_value='topic_log')
    reliable_arg = DeclareLaunchArgument('reliable', default_value='false')
    queue_arg = DeclareLaunchArgument('queue_depth', default_value='50')
    bag_arg = DeclareLaunchArgument('record_bag', default_value='false')
    bag_out_arg = DeclareLaunchArgument('bag_out', default_value='bag_' + datetime.utcnow().strftime('%Y%m%d_%H%M%S'))

    # Python logger process
    logger_cmd = [
        'python3',
        PathJoinSubstitution([ThisLaunchFileDir(), '..', 'scripts', 'log_monitor.py']),
        '--log-dir', LaunchConfiguration('log_dir'),
        '--log-name', LaunchConfiguration('log_name'),
        '--queue-depth', LaunchConfiguration('queue_depth'),
    ]
    for t in default_topics:
        logger_cmd += ['--topics', t]

    logger_proc = ExecuteProcess(
        cmd=logger_cmd,
        output='screen'
    )

    # Optional rosbag2 recorder
    rosbag_proc = ExecuteProcess(
        cmd=['ros2', 'bag', 'record', '-o',
             LaunchConfiguration('bag_out')] + [t.split(':', 1)[0] for t in default_topics],
        condition=lambda context: LaunchConfiguration('record_bag').perform(context) == 'true',
        output='screen'
    )

    return LaunchDescription([
        log_dir_arg, log_name_arg, reliable_arg, queue_arg, bag_arg, bag_out_arg,
        IncludeLaunchDescription(PythonLaunchDescriptionSource(INCLUDE_SIM_LAUNCH)),
        logger_proc,
        rosbag_proc
    ])
