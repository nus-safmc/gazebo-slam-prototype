from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def _guess_px4_dir() -> str:
    for key in ('PX4_AUTOPILOT_DIR', 'PX4_DIR'):
        value = os.environ.get(key)
        if value and os.path.isdir(value):
            return value

    try:
        pkg_share_real = os.path.realpath(get_package_share_directory('tof_slam_sim'))
        candidate = os.path.join(os.path.dirname(pkg_share_real), 'PX4-Autopilot')
        if os.path.isdir(candidate):
            return candidate
    except Exception:
        pass

    return os.path.join(os.getcwd(), 'PX4-Autopilot')


def generate_launch_description() -> LaunchDescription:
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time (Gazebo /clock).',
    )
    px4_dir_arg = DeclareLaunchArgument(
        'px4_dir',
        default_value=_guess_px4_dir(),
        description='Path to the PX4-Autopilot directory.',
    )
    run_px4_arg = DeclareLaunchArgument(
        'run_px4',
        default_value='true',
        description='Start PX4 SITL (make px4_sitl gz_x500_small_tof).',
    )
    run_bridge_arg = DeclareLaunchArgument(
        'run_bridge',
        default_value='true',
        description='Start ros_gz_bridge for depth, pose, and clock topics.',
    )
    run_agent_arg = DeclareLaunchArgument(
        'run_agent',
        default_value='true',
        description='Start MicroXRCEAgent (PX4 uXRCE-DDS).',
    )

    px4_dir = LaunchConfiguration('px4_dir')
    set_gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[
            PathJoinSubstitution([px4_dir, 'Tools', 'simulation', 'gz', 'models']),
            ':',
            PathJoinSubstitution([px4_dir, 'Tools', 'simulation', 'gz', 'worlds']),
        ],
    )

    px4_sitl = ExecuteProcess(
        cmd=[
            'make',
            'px4_sitl',
            'gz_x500_small_tof',
            'CMAKE_ARGS=-DCONFIG_WARN_AS_ERROR=n',
        ],
        cwd=px4_dir,
        output='screen',
        condition=IfCondition(LaunchConfiguration('run_px4')),
    )

    bridge_tof = ExecuteProcess(
        cmd=[
            'ros2',
            'run',
            'ros_gz_bridge',
            'parameter_bridge',
            '/clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock',
            '/depth/tof_1@sensor_msgs/msg/Image@gz.msgs.Image',
            '/depth/tof_2@sensor_msgs/msg/Image@gz.msgs.Image',
            '/depth/tof_3@sensor_msgs/msg/Image@gz.msgs.Image',
            '/depth/tof_4@sensor_msgs/msg/Image@gz.msgs.Image',
            '/depth/tof_5@sensor_msgs/msg/Image@gz.msgs.Image',
            '/depth/tof_6@sensor_msgs/msg/Image@gz.msgs.Image',
            '/depth/tof_7@sensor_msgs/msg/Image@gz.msgs.Image',
            '/depth/tof_8@sensor_msgs/msg/Image@gz.msgs.Image',
            '/model/x500_small_tof_0/pose@geometry_msgs/msg/PoseStamped@gz.msgs.Pose',
        ],
        output='screen',
        condition=IfCondition(LaunchConfiguration('run_bridge')),
    )

    micro_xrce_agent = ExecuteProcess(
        cmd=['MicroXRCEAgent', 'udp4', '-p', '8888'],
        output='screen',
        condition=IfCondition(LaunchConfiguration('run_agent')),
    )

    tof_to_scan = Node(
        package='tof_slam_sim',
        executable='tof_to_scan.py',
        name='tof_to_scan',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'output_frame': 'robot/base_link',
        }],
    )

    ld = LaunchDescription()
    ld.add_action(use_sim_time_arg)
    ld.add_action(px4_dir_arg)
    ld.add_action(run_px4_arg)
    ld.add_action(run_bridge_arg)
    ld.add_action(run_agent_arg)
    ld.add_action(set_gz_resource_path)
    ld.add_action(px4_sitl)
    ld.add_action(bridge_tof)
    ld.add_action(micro_xrce_agent)
    ld.add_action(tof_to_scan)
    return ld
