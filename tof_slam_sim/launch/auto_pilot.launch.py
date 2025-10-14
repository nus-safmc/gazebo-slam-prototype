from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tof_slam_sim',
            executable='auto_pilot',
            name='auto_pilot',
            output='screen',
            # Example env overrides without editing code:
            # env={'AP_LIN_X':'1.0','AP_LIN_Z':'0.7','AP_ANG_Z':'0.3','AP_RATE':'15'},
        )
    ])

