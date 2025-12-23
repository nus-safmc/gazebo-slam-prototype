from __future__ import annotations

import os
import platform
import sys
import tempfile
import time
from datetime import datetime

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _render_bridge_yaml(*, robots: list[str], world_name: str) -> str:
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

    def _entry(
        *,
        ros_topic: str,
        gz_topic: str,
        ros_type: str,
        gz_type: str,
        direction: str,
    ) -> str:
        return (
            f'- ros_topic_name: "{ros_topic}"\n'
            f'  gz_topic_name: "{gz_topic}"\n'
            f'  ros_type_name: "{ros_type}"\n'
            f'  gz_type_name: "{gz_type}"\n'
            f'  direction: {direction}\n\n'
        )

    out = ''
    out += _entry(
        ros_topic='/clock',
        gz_topic=f'/world/{world_name}/clock',
        ros_type='rosgraph_msgs/msg/Clock',
        gz_type='gz.msgs.Clock',
        direction='GZ_TO_ROS',
    )

    for robot in robots:
        robot = robot.strip()
        if not robot:
            continue

        # cmd_vel topic: robot uses /cmd_vel, others use dedicated per-robot gz topics.
        ros_cmd = '/cmd_vel' if robot == 'robot' else f'/{robot}/cmd_vel'
        gz_cmd = '/cmd_vel' if robot == 'robot' else f'/cmd_vel_{robot}'
        out += _entry(
            ros_topic=ros_cmd,
            gz_topic=gz_cmd,
            ros_type='geometry_msgs/msg/Twist',
            gz_type='gz.msgs.Twist',
            direction='ROS_TO_GZ',
        )

        ros_odom = '/odom' if robot == 'robot' else f'/{robot}/odom'
        out += _entry(
            ros_topic=ros_odom,
            gz_topic=f'/model/{robot}/odometry',
            ros_type='nav_msgs/msg/Odometry',
            gz_type='gz.msgs.Odometry',
            direction='GZ_TO_ROS',
        )

        for sensor in sensors:
            ros_scan = f'/scan/{sensor}' if robot == 'robot' else f'/{robot}/scan/{sensor}'
            out += _entry(
                ros_topic=ros_scan,
                gz_topic=f'/world/{world_name}/model/{robot}/link/base_link/sensor/tof_{sensor}/scan',
                ros_type='sensor_msgs/msg/LaserScan',
                gz_type='gz.msgs.LaserScan',
                direction='GZ_TO_ROS',
            )
    return out


def _make_bridge_node(context, *, use_sim_time):
    robots_raw = LaunchConfiguration('robots').perform(context)
    robots = [r.strip() for r in str(robots_raw).split(',') if r.strip()]
    world_name = LaunchConfiguration('bridge_world').perform(context)
    world_name = str(world_name).strip() or 'playfield'

    stamp = int(time.time() * 1000)
    bridge_path = os.path.join(
        tempfile.gettempdir(), f'tof_bridge_{os.getpid()}_{stamp}.yaml'
    )
    with open(bridge_path, 'w', encoding='utf-8') as f:
        f.write(_render_bridge_yaml(robots=robots, world_name=world_name))

    return [
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='bridge_all',
            output='screen',
            parameters=[{
                'config_file': bridge_path,
                'use_sim_time': use_sim_time,
            }],
        )
    ]


def generate_launch_description() -> LaunchDescription:
    pkg_share = FindPackageShare('tof_slam_sim')
    use_sim_time = LaunchConfiguration('use_sim_time')
    world = LaunchConfiguration('world')
    world_path = PathJoinSubstitution([pkg_share, 'worlds', world])
    log_monitor_path = PathJoinSubstitution([pkg_share, 'scripts', 'log_monitor.py'])

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

    log_dir_arg = DeclareLaunchArgument(
        'log_dir',
        default_value=os.path.expanduser('~/.ros/monitor_logs'),
        description='Directory for the topic health logs.',
    )
    log_name_arg = DeclareLaunchArgument(
        'log_name',
        default_value='topic_log',
        description='Base filename for topic health logs.',
    )
    qdepth_arg = DeclareLaunchArgument(
        'queue_depth',
        default_value='50',
        description='Queue depth used by the log monitor script.',
    )
    record_bag_arg = DeclareLaunchArgument(
        'record_bag',
        default_value='false',
        description='If true, record a rosbag alongside the monitor log.',
    )
    bag_out_arg = DeclareLaunchArgument(
        'bag_out',
        default_value='bag_' + datetime.utcnow().strftime('%Y%m%d_%H%M%S'),
        description='Bag output prefix when record_bag is true.',
    )
    run_autopilot_arg = DeclareLaunchArgument(
        'run_autopilot',
        default_value='true',
        description='Start the built-in cmd_vel autopilot node.',
    )
    autopilot_mode_arg = DeclareLaunchArgument(
        'autopilot_mode',
        default_value=EnvironmentVariable('AP_MODE', default_value='pattern'),
        description='Autopilot mode for tof_slam_sim/auto_pilot (pattern|explore).',
    )
    run_logger_arg = DeclareLaunchArgument(
        'run_logger',
        default_value='true',
        description='Start the topic log monitor process (can be expensive).',
    )
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time (Gazebo /clock).',
    )
    world_arg = DeclareLaunchArgument(
        'world',
        default_value='playfield_sparse.sdf',
        description='World file (SDF) under tof_slam_sim/worlds.',
    )
    robots_arg = DeclareLaunchArgument(
        'robots',
        default_value='robot',
        description='Comma-separated robot model names to bridge (e.g. "robot,robot2,robot3,robot4").',
    )
    bridge_world_arg = DeclareLaunchArgument(
        'bridge_world',
        default_value='playfield',
        description='Gazebo world name used in topic paths (must match <world name=\"...\">).',
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
        condition=IfCondition(LaunchConfiguration('run_autopilot')),
    )

    set_gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[
            PathJoinSubstitution([pkg_share, 'models']),
            ':',
            PathJoinSubstitution([pkg_share, 'worlds']),
        ],
    )
    set_rcl_logging_dir = SetEnvironmentVariable('RCL_LOGGING_DIR', '/tmp/ros_logs')
    set_ros_log_dir = SetEnvironmentVariable('ROS_LOG_DIR', '/tmp/ros_logs')
    make_log_dir = ExecuteProcess(
        cmd=['/usr/bin/env', 'bash', '-lc', 'mkdir -p /tmp/ros_logs'],
        output='screen',
    )

    is_macos = platform.system() == 'Darwin'
    gz_processes: list[ExecuteProcess] = []
    if is_macos:
        gz_processes.append(
            ExecuteProcess(
                cmd=['gz', 'sim', '-s', '-r', world_path],
                output='screen',
            )
        )
        gz_processes.append(
            ExecuteProcess(
                cmd=['gz', 'sim', '-g'],
                output='screen',
            )
        )
    else:
        gz_processes.append(
            ExecuteProcess(
                cmd=['gz', 'sim', '-r', world_path],
                output='screen',
            )
        )

    bridge_node = OpaqueFunction(
        function=_make_bridge_node,
        kwargs={'use_sim_time': use_sim_time},
    )

    autopilot_env = {
        'AP_MODE': LaunchConfiguration('autopilot_mode'),
        'AP_SCAN_TOPIC': os.environ.get('AP_SCAN_TOPIC', '/scan_merged'),
        'AP_LIN_Z': os.environ.get('AP_LIN_Z', '0.1'),
        'AP_LIN_Z_MAX': os.environ.get('AP_LIN_Z_MAX', '0.2'),
        'AP_LIN_X': os.environ.get('AP_LIN_X', '0.3'),
        'AP_LIN_Y': os.environ.get('AP_LIN_Y', '0.0'),
        'AP_ANG_Z': os.environ.get('AP_ANG_Z', '0.3'),
        'AP_ALT_TARGET': os.environ.get('AP_ALT_TARGET', '1.5'),
        'AP_ALT_KP': os.environ.get('AP_ALT_KP', '0.8'),
        'AP_ALT_DEADBAND': os.environ.get('AP_ALT_DEADBAND', '0.05'),
        'AP_RATE': os.environ.get('AP_RATE', '10.0'),
        'RCL_LOGGING_DIR': '/tmp/ros_logs',
        'ROS_LOG_DIR': '/tmp/ros_logs',
        'RCUTILS_LOGGING_USE_STDOUT': '1',
        'RMW_IMPLEMENTATION': os.environ.get('RMW_IMPLEMENTATION', 'rmw_cyclonedds_cpp'),
        'PYTHONFAULTHANDLER': os.environ.get('PYTHONFAULTHANDLER', '1'),
    }
    auto_pilot = Node(
        package='tof_slam_sim',
        executable='auto_pilot',
        name='auto_pilot',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        additional_env=autopilot_env,
        condition=IfCondition(LaunchConfiguration('run_autopilot')),
    )

    logger_cmd = [
        sys.executable,
        '-u',
        log_monitor_path,
        '--log-dir',
        LaunchConfiguration('log_dir'),
        '--log-name',
        LaunchConfiguration('log_name'),
        '--queue-depth',
        LaunchConfiguration('queue_depth'),
        '--topics',
        '/cmd_vel:geometry_msgs/msg/Twist',
        '--topics',
        '/odom:nav_msgs/msg/Odometry',
        '--topics',
        '/tf:tf2_msgs/msg/TFMessage',
        '--topics',
        '/tf_static:tf2_msgs/msg/TFMessage',
    ]
    for sensor in sensors:
        logger_cmd.extend(['--topics', f'/scan/{sensor}:sensor_msgs/msg/LaserScan'])

    logger_proc = ExecuteProcess(
        cmd=logger_cmd,
        output='screen',
        condition=IfCondition(LaunchConfiguration('run_logger')),
    )

    bag_topics = ['/cmd_vel', '/odom', '/tf', '/tf_static'] + [f'/scan/{s}' for s in sensors]
    rosbag_proc = ExecuteProcess(
        cmd=['ros2', 'bag', 'record', '-o', LaunchConfiguration('bag_out')] + bag_topics,
        condition=IfCondition(LaunchConfiguration('record_bag')),
        output='screen',
    )

    ld = LaunchDescription()
    ld.add_action(log_dir_arg)
    ld.add_action(log_name_arg)
    ld.add_action(qdepth_arg)
    ld.add_action(record_bag_arg)
    ld.add_action(bag_out_arg)
    ld.add_action(run_autopilot_arg)
    ld.add_action(autopilot_mode_arg)
    ld.add_action(run_logger_arg)
    ld.add_action(use_sim_time_arg)
    ld.add_action(world_arg)
    ld.add_action(robots_arg)
    ld.add_action(bridge_world_arg)

    ld.add_action(set_gz_resource_path)
    ld.add_action(set_rcl_logging_dir)
    ld.add_action(set_ros_log_dir)
    ld.add_action(make_log_dir)

    for action in gz_processes:
        ld.add_action(action)

    ld.add_action(bridge_node)
    ld.add_action(scan_merger)
    ld.add_action(auto_pilot)
    ld.add_action(logger_proc)
    ld.add_action(rosbag_proc)

    return ld
