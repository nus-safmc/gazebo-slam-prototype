from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable, DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import platform
import sys
import os
from datetime import datetime


def generate_launch_description():
    pkg = FindPackageShare('tof_slam_sim')

    # ===== EDIT THESE TO MATCH YOUR SETUP =====
    # Note: the instance name in the world overrides the internal SDF model name.
    # In worlds/playfield.sdf the quadcopter is included as <name>robot</name>,
    # so the effective Gazebo topics are /model/robot/*.
    MODEL = 'robot'                 # instance name from your world include
    WORLD = 'playfield'             # world name used in /world/<WORLD>/... topics
    DEPTH_STREAM = 'image'          # 'image' (default). Set to 'depth'/'depth_image' if your topics end that way.
    LOG_MONITOR_PATH = PathJoinSubstitution([pkg, 'scripts', 'log_monitor.py'])
    # =========================================

    # Common sensor names and helpers (used by both bridge + logger topic list)
    sensors = ['front','front_right','right','back_right','back','back_left','left','front_left']
    def gz_depth(s):
        # Depth sensors are nested models: /world/<WORLD>/model/<MODEL>/model/tof_<s>/...
        return f'/world/{WORLD}/model/{MODEL}/model/tof_{s}/link/sensor/depth_camera/{DEPTH_STREAM}'

    # Build the ROS topic names after remap for depth sensors
    depth_ros_topics = [f'/tof_{s}/depth' for s in sensors]

    # ====== Logging / Bag arguments ======
    log_dir_arg  = DeclareLaunchArgument('log_dir',     default_value=os.path.expanduser('~/.ros/monitor_logs'))
    log_name_arg = DeclareLaunchArgument('log_name',    default_value='topic_log')
    qdepth_arg   = DeclareLaunchArgument('queue_depth', default_value='50')
    reliable_arg = DeclareLaunchArgument('reliable',    default_value='false')  # parsed inside logger if you add --reliable
    record_bag   = DeclareLaunchArgument('record_bag',  default_value='false')
    bag_out_arg  = DeclareLaunchArgument('bag_out',     default_value='bag_' + datetime.utcnow().strftime('%Y%m%d_%H%M%S'))

    # Make our models + worlds discoverable by Gazebo
    set_gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[
            PathJoinSubstitution([pkg, 'models']),
            ':',
            PathJoinSubstitution([pkg, 'worlds'])
        ]
    )

    # Force ROS logging to a simple absolute path (avoids rcutils_expand_user issues)
    set_rcl_logging_dir = SetEnvironmentVariable(name='RCL_LOGGING_DIR', value='/tmp/ros_logs')
    set_ros_log_dir     = SetEnvironmentVariable(name='ROS_LOG_DIR',     value='/tmp/ros_logs')
    make_log_dir        = ExecuteProcess(cmd=['bash', '-lc', 'mkdir -p /tmp/ros_logs'], output='screen')

    is_macos  = platform.system() == 'Darwin'
    world_path = PathJoinSubstitution([pkg, 'worlds', f'{WORLD}.sdf'])

    actions = []
    if is_macos:
        actions.append(ExecuteProcess(cmd=['gz', 'sim', '-s', '-r', world_path], output='screen'))
        actions.append(ExecuteProcess(cmd=['gz', 'sim', '-g'], output='screen'))
    else:
        actions.append(ExecuteProcess(cmd=['gz', 'sim', '-r', world_path], output='screen'))

    # ---- Bridge: /cmd_vel <-> VelocityControl, plus odom/tf/clock ----
    bridge_cmd_odom = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='bridge_cmd_odom',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
            '/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V',
            '/clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock',
        ],
        output='screen',
    )

    # ---- Bridge: 8x ToF depth images ----
    depth_args = []
    for s in sensors:
        depth_args.append(f'{gz_depth(s)}@sensor_msgs/msg/Image@gz.msgs.Image')
    depth_args += ['--ros-args']
    for s in sensors:
        depth_args += ['-r', f'{gz_depth(s)}:=/tof_{s}/depth']

    bridge_all_depth = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='bridge_all_depth',
        arguments=depth_args,
        output='screen',
    )

    # ---- Autopilot: execute the bundled script so imports resolve consistently ----
    autopilot_env = {
        'AP_LIN_Z': os.environ.get('AP_LIN_Z', '0.6'),
        'AP_LIN_X': os.environ.get('AP_LIN_X', '0.8'),
        'AP_LIN_Y': os.environ.get('AP_LIN_Y', '0.0'),
        'AP_ANG_Z': os.environ.get('AP_ANG_Z', '0.2'),
        'AP_RATE':  os.environ.get('AP_RATE', '10.0'),
        'RCL_LOGGING_DIR': '/tmp/ros_logs',
        'ROS_LOG_DIR': '/tmp/ros_logs',
        'RCUTILS_LOGGING_USE_STDOUT': '1'
    }
    auto_pilot = ExecuteProcess(
        cmd=[
            sys.executable,
            '-u',
            PathJoinSubstitution([pkg, 'scripts', 'auto_pilot.py']),
        ],
        env=autopilot_env,
        output='screen',
    )

    # ===== Logger: run scripts/log_monitor.py with your topics =====
    # The logger expects topic:type pairs. We'll log:
    #   /cmd_vel, /odom, /tf, /tf_static, and all ToF depth topics.
    logger_cmd = [
        sys.executable, '-u',
        LOG_MONITOR_PATH,
        '--log-dir', LaunchConfiguration('log_dir'),
        '--log-name', LaunchConfiguration('log_name'),
        '--queue-depth', LaunchConfiguration('queue_depth'),
        # Add '--reliable' here if you want RELIABLE QoS by default:
        # '--reliable',
        '--topics', '/cmd_vel:geometry_msgs/msg/Twist',
        '--topics', '/odom:nav_msgs/msg/Odometry',
        '--topics', '/tf:tf2_msgs/msg/TFMessage',
        '--topics', '/tf_static:tf2_msgs/msg/TFMessage',
    ]
    for t in depth_ros_topics:
        logger_cmd += ['--topics', f'{t}:sensor_msgs/msg/Image']

    logger_proc = ExecuteProcess(
        cmd=logger_cmd,
        output='screen'
    )

    # ===== Optional rosbag2 recording of the same topics =====
    bag_topics = ['/cmd_vel', '/odom', '/tf', '/tf_static'] + depth_ros_topics
    rosbag_proc = ExecuteProcess(
        cmd=['ros2', 'bag', 'record', '-o', LaunchConfiguration('bag_out')] + bag_topics,
        condition=IfCondition(LaunchConfiguration('record_bag')),
        output='screen'
    )

    ld = LaunchDescription()
    # args for logging/bag
    ld.add_action(log_dir_arg)
    ld.add_action(log_name_arg)
    ld.add_action(qdepth_arg)
    ld.add_action(reliable_arg)
    ld.add_action(record_bag)
    ld.add_action(bag_out_arg)

    # env + dirs
    ld.add_action(set_gz_resource_path)
    ld.add_action(set_rcl_logging_dir)
    ld.add_action(set_ros_log_dir)
    ld.add_action(make_log_dir)

    # sim + bridges + autopilot
    for a in actions:
        ld.add_action(a)
    ld.add_action(bridge_cmd_odom)
    ld.add_action(bridge_all_depth)
    ld.add_action(auto_pilot)

    # logger + rosbag
    ld.add_action(logger_proc)
    ld.add_action(rosbag_proc)

    return ld
