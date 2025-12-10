from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
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
    LOG_MONITOR_PATH = PathJoinSubstitution([pkg, 'scripts', 'log_monitor.py'])
    # =========================================

    # Common sensor names and helpers (used by both bridge + logger topic list)
    sensors = ['front','front_right','right','back_right','back','back_left','left','front_left']
    def gz_scan(s):
        return f'/scan/{s}'

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

    # Initialize variables to avoid linter warnings
    gz_sim_server = None
    gz_sim_gui = None
    gz_sim = None

    if is_macos:
        actions.append(ExecuteProcess(cmd=['gz', 'sim', '-s', '-r', world_path], output='screen'))
    else:
        # On other platforms, use the combined -r flag
        gz_sim = ExecuteProcess(
            cmd=['gz', 'sim', '-r',
                 PathJoinSubstitution([pkg_tof_slam_sim, 'worlds', 'playfield.sdf'])],
            output='screen'
        )
    
    # Test all sensors with TimerAction to confirm CycloneDDS fix
    bridge_configs = []

    # All 8 sensors = 16 bridges total (8 × 2 topics each)
    sensor_names = ['front', 'front_right', 'right', 'back_right',
                   'back', 'back_left', 'left', 'front_left']

    for name in sensor_names:
        # Bridge camera image
        bridge_configs.append(Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name=f'bridge_image_{name}',
            parameters=[{
                'config_file': '',
                'gz_topic': f'/model/robot/model/tof_{name}/link/sensor/camera/image',
                'ros_topic': f'/tof_{name}/image',
                'gz_type': 'gz.msgs.Image',
                'ros_type': 'sensor_msgs/msg/Image',
                'lazy': True
            }]
        ))

    # ---- Bridge topics via YAML config ----
    bridge_config = PathJoinSubstitution([pkg, 'config', 'bridge.yaml'])
    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='bridge_all',
        parameters=[{'config_file': bridge_config}],
        output='screen',
    )

    # ---- Autopilot: execute the bundled script so imports resolve consistently ----
    autopilot_env = {
        'AP_LIN_Z': os.environ.get('AP_LIN_Z', '0.1'),
        'AP_LIN_Z_MAX': os.environ.get('AP_LIN_Z_MAX', '0.2'),
        'AP_LIN_X': os.environ.get('AP_LIN_X', '0.3'),
        'AP_LIN_Y': os.environ.get('AP_LIN_Y', '0.0'),
        'AP_ANG_Z': os.environ.get('AP_ANG_Z', '0.3'),
        'AP_ALT_TARGET': os.environ.get('AP_ALT_TARGET', '1.5'),
        'AP_ALT_KP': os.environ.get('AP_ALT_KP', '0.8'),
        'AP_ALT_DEADBAND': os.environ.get('AP_ALT_DEADBAND', '0.05'),
        'AP_RATE':  os.environ.get('AP_RATE', '10.0'),
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
        output='log',
        env=autopilot_env,
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
    for t in [f'/scan/{s}' for s in sensors]:
        logger_cmd += ['--topics', f'{t}:sensor_msgs/msg/LaserScan']

    logger_proc = ExecuteProcess(
        cmd=logger_cmd,
        output='screen'
    )

    # ===== Optional rosbag2 recording of the same topics =====
    bag_topics = ['/cmd_vel', '/odom', '/tf', '/tf_static'] + [f'/scan/{s}' for s in sensors]
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
    ld.add_action(bridge_node)
    ld.add_action(auto_pilot)

    # logger + rosbag
    ld.add_action(logger_proc)
    ld.add_action(rosbag_proc)

    # Add Gazebo
    if is_macos:
        # Add both server and GUI processes
        ld.add_action(gz_sim_server)
        ld.add_action(gz_sim_gui)
    else:
        ld.add_action(gz_sim)
    
    # Add bridge nodes with TimerAction delays for robustness
    for i, config in enumerate(bridge_configs):
        # Start each bridge with a small delay to prevent potential domain conflicts
        ld.add_action(TimerAction(period=i * 0.2, actions=[config]))

    # Add command and odometry bridges
    ld.add_action(cmd_vel_bridge)
    ld.add_action(odom_bridge)
    
    return ld
