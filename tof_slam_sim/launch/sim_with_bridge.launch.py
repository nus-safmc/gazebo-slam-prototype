from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import platform

def generate_launch_description():
    pkg_tof_slam_sim = FindPackageShare('tof_slam_sim')

    # Make our models + worlds discoverable by Gazebo
    set_gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[
            PathJoinSubstitution([pkg_tof_slam_sim, 'models']),
            ':',
            PathJoinSubstitution([pkg_tof_slam_sim, 'worlds'])
        ]
    )

    is_macos = platform.system() == 'Darwin'

    if is_macos:
        gz_sim_server = ExecuteProcess(
            cmd=['gz', 'sim', '-s', '-r',
                 PathJoinSubstitution([pkg_tof_slam_sim, 'worlds', 'playfield.sdf'])],
            output='screen'
        )
        gz_sim_gui = ExecuteProcess(cmd=['gz', 'sim', '-g'], output='screen')
        gz_sim = [gz_sim_server, gz_sim_gui]
    else:
        gz_sim = ExecuteProcess(
            cmd=['gz', 'sim', '-r',
                 PathJoinSubstitution([pkg_tof_slam_sim, 'worlds', 'playfield.sdf'])],
            output='screen'
        )

    # Per-ToF bridges
    bridge_configs = []
    sensor_names = ['front', 'front_right', 'right', 'back_right',
                    'back', 'back_left', 'left', 'front_left']

    for name in sensor_names:
        # Depth image
        bridge_configs.append(Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name=f'bridge_depth_{name}',
            parameters=[{
                'config_file': '',
                'gz_topic': f'/model/robot/model/tof_{name}/link/sensor/depth_camera/depth',
                'ros_topic': f'/tof_{name}/depth',
                'gz_type': 'gz.msgs.Image',
                'ros_type': 'sensor_msgs/msg/Image',
                'lazy': True
            }]
        ))

        # Camera info (may be empty in some models; harmless)
        bridge_configs.append(Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name=f'bridge_info_{name}',
            parameters=[{
                'config_file': '',
                'gz_topic': f'/model/robot/model/tof_{name}/link/sensor/depth_camera/camera_info',
                'ros_topic': f'/tof_{name}/camera_info',
                'gz_type': 'gz.msgs.CameraInfo',
                'ros_type': 'sensor_msgs/msg/CameraInfo',
                'lazy': True
            }]
        ))

    bridge_cmd_odom = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='bridge_cmd_odom',
        arguments=[
            # Twist: ROS /cmd_vel  <->  GZ /model/robot/cmd_vel
            '/model/robot/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            # Odometry: GZ /model/robot/odometry  ->  ROS /odom
            '/model/robot/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry',
            '--ros-args',
            '-r', '/model/robot/cmd_vel:=/cmd_vel',
            '-r', '/model/robot/odometry:=/odom',
        ],
        output='screen',
    )

    # Autopilot node — publishes /cmd_vel continuously
    auto_pilot = Node(
        package='tof_slam_sim',
        executable='auto_pilot.py',
        name='auto_pilot',
        output='screen',
        # Example env override (uncomment to tweak)
        # env={'AP_LIN_X': '1.0', 'AP_LIN_Z': '0.7', 'AP_ANG_Z': '0.2', 'AP_RATE': '15'}
    )

    ld = LaunchDescription()
    ld.add_action(set_gz_resource_path)

    if is_macos:
        ld.add_action(gz_sim[0])
        ld.add_action(gz_sim[1])
    else:
        ld.add_action(gz_sim)

    for config in bridge_configs:
        ld.add_action(config)

    ld.add_action(bridge_cmd_odom)
    ld.add_action(auto_pilot)

    return ld

