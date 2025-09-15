from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os
import platform

def generate_launch_description():
    pkg_tof_slam_sim = FindPackageShare('tof_slam_sim')

    # Check if we're on macOS
    is_macos = platform.system() == 'Darwin'

    if is_macos:
        # On macOS, launch server and GUI separately
        gz_sim_server = ExecuteProcess(
            cmd=['gz', 'sim', '-s', '-r',
                 PathJoinSubstitution([pkg_tof_slam_sim, 'worlds', 'playfield.sdf'])],
            output='screen'
        )

        gz_sim_gui = ExecuteProcess(
            cmd=['gz', 'sim', '-g'],
            output='screen'
        )

        gz_sim = [gz_sim_server, gz_sim_gui]
    else:
        # On other platforms, use the combined -r flag
        gz_sim = ExecuteProcess(
            cmd=['gz', 'sim', '-r',
                 PathJoinSubstitution([pkg_tof_slam_sim, 'worlds', 'playfield.sdf'])],
            output='screen'
        )
    
    # Bridge configurations for each ToF sensor
    bridge_configs = []
    sensor_names = ['front', 'front_right', 'right', 'back_right',
                   'back', 'back_left', 'left', 'front_left']
    
    for name in sensor_names:
        # Bridge depth image
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
        
        # Bridge camera info
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
    
    # Bridge robot commands
    cmd_vel_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='bridge_cmd_vel',
        parameters=[{
            'config_file': '',
            'gz_topic': '/model/robot/cmd_vel',
            'ros_topic': '/cmd_vel',
            'gz_type': 'gz.msgs.Twist',
            'ros_type': 'geometry_msgs/msg/Twist',
            'lazy': True
        }]
    )
    
    # Bridge odometry
    odom_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='bridge_odom',
        parameters=[{
            'config_file': '',
            'gz_topic': '/model/robot/odometry',
            'ros_topic': '/odom',
            'gz_type': 'gz.msgs.Odometry',
            'ros_type': 'nav_msgs/msg/Odometry',
            'lazy': True
        }]
    )
    
    # Create launch description
    ld = LaunchDescription()

    # Add Gazebo
    if is_macos:
        # Add both server and GUI processes
        ld.add_action(gz_sim[0])  # server
        ld.add_action(gz_sim[1])  # gui
    else:
        ld.add_action(gz_sim)
    
    # Add all bridge nodes
    for config in bridge_configs:
        ld.add_action(config)
    
    # Add command and odometry bridges
    ld.add_action(cmd_vel_bridge)
    ld.add_action(odom_bridge)
    
    return ld
