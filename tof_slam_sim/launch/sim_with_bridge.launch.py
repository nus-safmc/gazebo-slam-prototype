from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import platform
import sys

def generate_launch_description():
    pkg = FindPackageShare('tof_slam_sim')

    # ===== EDIT THESE TO MATCH YOUR SETUP =====
    MODEL = 'tof_slam_quadcopter'   # exact <model name="..."> from your model.sdf
    WORLD = 'playfield'             # world name used in /world/<WORLD>/... topics
    DEPTH_STREAM = 'image'          # 'image' (default). Set to 'depth'/'depth_image' if your topics end that way.
    # =========================================

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
    set_rcl_logging_dir = SetEnvironmentVariable(
        name='RCL_LOGGING_DIR',
        value='/tmp/ros_logs'
    )
    # (optional, also set the deprecated var some packages still read)
    set_ros_log_dir = SetEnvironmentVariable(
        name='ROS_LOG_DIR',
        value='/tmp/ros_logs'
    )
    # Create the directory before nodes start (harmless if it exists)
    make_log_dir = ExecuteProcess(cmd=['bash', '-lc', 'mkdir -p /tmp/ros_logs'], output='screen')

    is_macos = platform.system() == 'Darwin'
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
            f'/model/{MODEL}/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            f'/model/{MODEL}/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry',
            '/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V',
            '/clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock',
            '--ros-args',
            # Your autopilot publishes /cmd_vel; remap it into Gazebo's namespaced topic:
            '-r', f'/model/{MODEL}/cmd_vel:=/cmd_vel',
            # Give odometry a nice ROS name:
            '-r', f'/model/{MODEL}/odometry:=/odom',
        ],
        output='screen',
    )

    # ---- Bridge: 8x ToF depth images ----
    sensors = ['front','front_right','right','back_right','back','back_left','left','front_left']
    def gz_depth(s):
        return f'/world/{WORLD}/model/{MODEL}/link/tof_{s}/sensor/depth_camera/{DEPTH_STREAM}'

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

    # ---- Autopilot: use THIS env's Python & harden logging env ----
    auto_pilot = ExecuteProcess(
        cmd=[
            sys.executable, '-u',
            PathJoinSubstitution([pkg, '../../lib/tof_slam_sim/auto_pilot.py']),
        ],
        # Add logging env here too, and keep Z=0 since gravity is off in your SDF
        env={
            'AP_LIN_Z': '0.0',
            'RCL_LOGGING_DIR': '/tmp/ros_logs',
            'ROS_LOG_DIR': '/tmp/ros_logs',
            'RCUTILS_LOGGING_USE_STDOUT': '1'  # also print to stdout
        },
        output='screen',
    )

    ld = LaunchDescription()
    ld.add_action(set_gz_resource_path)
    ld.add_action(set_rcl_logging_dir)
    ld.add_action(set_ros_log_dir)
    ld.add_action(make_log_dir)
    for a in actions:
        ld.add_action(a)
    ld.add_action(bridge_cmd_odom)
    ld.add_action(bridge_all_depth)
    ld.add_action(auto_pilot)
    return ld
