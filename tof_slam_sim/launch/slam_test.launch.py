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
    
    # Launch depth to scan converter
    depth_to_scan = Node(
        package='tof_slam_sim',
        executable='tof8x8_to_scan.py',
        name='tof8x8_to_scan',
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    # Launch scan merger
    scan_merger = Node(
        package='tof_slam_sim',
        executable='scan_merger.py',
        name='scan_merger',
        parameters=[{'use_sim_time': use_sim_time}]
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
    ld.add_action(depth_to_scan)
    ld.add_action(scan_merger)
    ld.add_action(slam_toolbox)
    ld.add_action(rviz)
    
    return ld
