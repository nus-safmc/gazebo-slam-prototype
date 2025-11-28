from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, SetEnvironmentVariable, TimerAction, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os.path
import platform

def generate_launch_description():
    ld = LaunchDescription()
    pkg_tof_slam_sim = FindPackageShare('tof_slam_sim')
    wd = os.path.dirname(str(pkg_tof_slam_sim))
    set_gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[
            PathJoinSubstitution([wd, 'PX4-Autopilot', 'Tools', 'simulation', 'gz', 'models']),
            ':',
            PathJoinSubstitution([wd, 'PX4-Autopilot', 'Tools', 'simulation', 'gz', 'worlds'])
        ]
    )
    ld.add_action(set_gz_resource_path)
    
    tof_to_scan = Node(
        package='tof_slam_sim',
        executable='tof_to_scan.py',
        name='tof_to_scan'
    )

    #Run processes in new gnome terminals
    #Might need to change this on other platforms depending on your terminal
    ld.add_action(ExecuteProcess(cmd=['gnome-terminal', '--', 'pixi', 'run', '-e', 'jazzy', 'run_px4'])) 
    ld.add_action(ExecuteProcess(cmd=['gnome-terminal', '--', 'pixi', 'run', '-e', 'jazzy', 'bridge_tof'])) 
    ld.add_action(ExecuteProcess(cmd=['gnome-terminal', '--', 'MicroXRCEAgent', 'udp4', '-p', '8888']))
    ld.add_action(tof_to_scan)
    
    return ld


