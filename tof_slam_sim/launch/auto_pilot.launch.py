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
    return LaunchDescription([
        declare_use_sim_time,
        Node(
            package='tof_slam_sim',
            executable='auto_pilot',
            name='auto_pilot',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            # Example env overrides without editing code:
            # env={'AP_LIN_X':'1.0','AP_LIN_Z':'0.7','AP_ANG_Z':'0.3','AP_RATE':'15'},
        )
    ])
