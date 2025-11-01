from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    pkg_tof_slam_sim = FindPackageShare('tof_slam_sim')
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    
    # Include simulation with bridge
    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([pkg_tof_slam_sim, 'launch', 'sim_with_bridge.launch.py'])
        ])
    )

    # Include topic monitor
    monitor_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([pkg_tof_slam_sim, 'launch', 'topic_monitor.launch.py'])
        ])
    )
    
    # Launch scan merger
    scan_merger = Node(
        package='tof_slam_sim',
        executable='scan_merger.py',
        name='scan_merger',
        parameters=[{'use_sim_time': use_sim_time,
                     'base_frame': 'robot/base_footprint'}]
    )
    
    # Launch SLAM Toolbox
    slam_toolbox = Node(
        package='slam_toolbox',
        executable='sync_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            PathJoinSubstitution([pkg_tof_slam_sim, 'config', 'slam_toolbox.yaml']),
            {'use_sim_time': use_sim_time}
        ]
    )

    # Launch RViz
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', PathJoinSubstitution([pkg_tof_slam_sim, 'config', 'slam.rviz'])]
    )
    
    # Create launch description
    ld = LaunchDescription()
    
    # Add declare arguments
    ld.add_action(declare_use_sim_time)
    
    # Add nodes
    ld.add_action(sim_launch)
    ld.add_action(monitor_launch)
    ld.add_action(scan_merger)
    ld.add_action(slam_toolbox)
    ld.add_action(rviz)
    
    return ld
