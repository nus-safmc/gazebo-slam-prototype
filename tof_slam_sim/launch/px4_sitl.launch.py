from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


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
    world_arg = DeclareLaunchArgument(
        'world',
        default_value='playfield_px4_sparse.sdf',
        description='World file under tof_slam_sim/worlds (PX4-safe defaults: playfield_px4_sparse.sdf).',
    )
    world_name_arg = DeclareLaunchArgument(
        'gz_world_name',
        default_value='playfield',
        description='Gazebo world name (used for /world/<name>/clock).',
    )
    gz_gui_arg = DeclareLaunchArgument(
        'gz_gui',
        default_value='false',
        description='Start Gazebo GUI (gz sim -g).',
    )
    use_visual_odometry_arg = DeclareLaunchArgument(
        'use_visual_odometry',
        default_value='true',
        description='Publish Gazebo pose as PX4 vehicle_visual_odometry (EKF2 external vision aiding).',
    )
    run_gz_arg = DeclareLaunchArgument(
        'run_gz',
        default_value='true',
        description='Start Gazebo (gz sim) server with PX4 default world.',
    )
    headless_rendering_arg = DeclareLaunchArgument(
        'headless_rendering',
        default_value='false',
        description='Start Gazebo server with --headless-rendering (useful when no display is available).',
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
    px4_model_pose_arg = DeclareLaunchArgument(
        'px4_model_pose',
        default_value='0,-2,0.2,0,0,0',
        description='PX4_GZ_MODEL_POSE for spawning the vehicle (x,y,z,roll,pitch,yaw).',
    )

    px4_dir = LaunchConfiguration('px4_dir')
    run_gz = LaunchConfiguration('run_gz')
    headless_rendering = LaunchConfiguration('headless_rendering')
    world = LaunchConfiguration('world')
    gz_world_name = LaunchConfiguration('gz_world_name')
    use_visual_odometry = LaunchConfiguration('use_visual_odometry')
    px4_model_pose = LaunchConfiguration('px4_model_pose')
    set_gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[
            PathJoinSubstitution([FindPackageShare('tof_slam_sim'), 'models']),
            ':',
            PathJoinSubstitution([FindPackageShare('tof_slam_sim'), 'worlds']),
            ':',
            PathJoinSubstitution([px4_dir, 'Tools', 'simulation', 'gz', 'models']),
            ':',
            PathJoinSubstitution([px4_dir, 'Tools', 'simulation', 'gz', 'worlds']),
        ],
    )

    # Ensure PX4 SITL sources our repo's `px4-rc.params` (searched via PATH).
    # This allows setting PX4 params (e.g., EKF2_OF_CTRL, NAV_DLL_ACT) without patching PX4.
    tof_share = FindPackageShare('tof_slam_sim')
    set_px4_rc_path = SetEnvironmentVariable(
        name='PATH',
        value=[
            PathJoinSubstitution([tof_share, 'scripts']),
            ':',
            EnvironmentVariable('PATH'),
        ],
    )
    set_px4_model_pose = SetEnvironmentVariable(
        name='PX4_GZ_MODEL_POSE',
        value=px4_model_pose,
    )

    gz_world_path = PathJoinSubstitution([FindPackageShare('tof_slam_sim'), 'worlds', world])
    gz_sim_headless = ExecuteProcess(
        cmd=[
            'gz',
            'sim',
            '--verbose=1',
            '--headless-rendering',
            '-r',
            '-s',
            gz_world_path,
        ],
        output='screen',
        condition=IfCondition(
            PythonExpression(["'", run_gz, "' == 'true' and '", headless_rendering, "' == 'true'"])
        ),
    )
    gz_sim = ExecuteProcess(
        cmd=[
            'gz',
            'sim',
            '--verbose=1',
            '-r',
            '-s',
            gz_world_path,
        ],
        output='screen',
        condition=IfCondition(
            PythonExpression(["'", run_gz, "' == 'true' and '", headless_rendering, "' != 'true'"])
        ),
    )

    gz_gui = ExecuteProcess(
        cmd=['gz', 'sim', '-g'],
        output='screen',
        condition=IfCondition(LaunchConfiguration('gz_gui')),
    )

    px4_sitl = ExecuteProcess(
        cmd=[
            'make',
            'px4_sitl',
            'gz_x500_small_tof',
        ],
        cwd=px4_dir,
        output='screen',
        additional_env={
            'CXXFLAGS': '-Wno-error=vla-cxx-extension -Wno-error=unused-but-set-variable -Wno-error=double-promotion',
        },
        condition=IfCondition(LaunchConfiguration('run_px4')),
    )

    clock_bridge = PythonExpression([
        '"/world/" + "',
        gz_world_name,
        '" + "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"',
    ])
    clock_remap = PythonExpression([
        '"/world/" + "',
        gz_world_name,
        '" + "/clock:=/clock"',
    ])

    bridge = ExecuteProcess(
        cmd=[
            'ros2',
            'run',
            'ros_gz_bridge',
            'parameter_bridge',
            clock_bridge,
            PythonExpression([
                '"/world/" + "',
                gz_world_name,
                '" + "/model/x500_small_tof_0/link/tof_base_link/sensor/tof_1/depth_image@sensor_msgs/msg/Image[gz.msgs.Image"',
            ]),
            PythonExpression([
                '"/world/" + "',
                gz_world_name,
                '" + "/model/x500_small_tof_0/link/tof_base_link/sensor/tof_2/depth_image@sensor_msgs/msg/Image[gz.msgs.Image"',
            ]),
            PythonExpression([
                '"/world/" + "',
                gz_world_name,
                '" + "/model/x500_small_tof_0/link/tof_base_link/sensor/tof_3/depth_image@sensor_msgs/msg/Image[gz.msgs.Image"',
            ]),
            PythonExpression([
                '"/world/" + "',
                gz_world_name,
                '" + "/model/x500_small_tof_0/link/tof_base_link/sensor/tof_4/depth_image@sensor_msgs/msg/Image[gz.msgs.Image"',
            ]),
            PythonExpression([
                '"/world/" + "',
                gz_world_name,
                '" + "/model/x500_small_tof_0/link/tof_base_link/sensor/tof_5/depth_image@sensor_msgs/msg/Image[gz.msgs.Image"',
            ]),
            PythonExpression([
                '"/world/" + "',
                gz_world_name,
                '" + "/model/x500_small_tof_0/link/tof_base_link/sensor/tof_6/depth_image@sensor_msgs/msg/Image[gz.msgs.Image"',
            ]),
            PythonExpression([
                '"/world/" + "',
                gz_world_name,
                '" + "/model/x500_small_tof_0/link/tof_base_link/sensor/tof_7/depth_image@sensor_msgs/msg/Image[gz.msgs.Image"',
            ]),
            PythonExpression([
                '"/world/" + "',
                gz_world_name,
                '" + "/model/x500_small_tof_0/link/tof_base_link/sensor/tof_8/depth_image@sensor_msgs/msg/Image[gz.msgs.Image"',
            ]),
            '--ros-args',
            '-r',
            '__node:=ros_gz_bridge',
            '-r',
            clock_remap,
            '-r',
            PythonExpression([
                '"/world/" + "',
                gz_world_name,
                '" + "/model/x500_small_tof_0/link/tof_base_link/sensor/tof_1/depth_image:=/depth/tof_1"',
            ]),
            '-r',
            PythonExpression([
                '"/world/" + "',
                gz_world_name,
                '" + "/model/x500_small_tof_0/link/tof_base_link/sensor/tof_2/depth_image:=/depth/tof_2"',
            ]),
            '-r',
            PythonExpression([
                '"/world/" + "',
                gz_world_name,
                '" + "/model/x500_small_tof_0/link/tof_base_link/sensor/tof_3/depth_image:=/depth/tof_3"',
            ]),
            '-r',
            PythonExpression([
                '"/world/" + "',
                gz_world_name,
                '" + "/model/x500_small_tof_0/link/tof_base_link/sensor/tof_4/depth_image:=/depth/tof_4"',
            ]),
            '-r',
            PythonExpression([
                '"/world/" + "',
                gz_world_name,
                '" + "/model/x500_small_tof_0/link/tof_base_link/sensor/tof_5/depth_image:=/depth/tof_5"',
            ]),
            '-r',
            PythonExpression([
                '"/world/" + "',
                gz_world_name,
                '" + "/model/x500_small_tof_0/link/tof_base_link/sensor/tof_6/depth_image:=/depth/tof_6"',
            ]),
            '-r',
            PythonExpression([
                '"/world/" + "',
                gz_world_name,
                '" + "/model/x500_small_tof_0/link/tof_base_link/sensor/tof_7/depth_image:=/depth/tof_7"',
            ]),
            '-r',
            PythonExpression([
                '"/world/" + "',
                gz_world_name,
                '" + "/model/x500_small_tof_0/link/tof_base_link/sensor/tof_8/depth_image:=/depth/tof_8"',
            ]),
        ],
        output='screen',
        condition=IfCondition(LaunchConfiguration('run_bridge')),
    )

    gz_pose = Node(
        package='tof_slam_sim',
        executable='gz_pose_info_to_pose_stamped',
        name='gz_pose_info_to_pose_stamped',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'gz_world_name': gz_world_name,
            'entity_name': 'x500_small_tof_0',
            'pose_topic': '/model/x500_small_tof_0/pose',
            'pose_frame_id': 'world',
            'publish_hz': 30.0,
        }],
    )

    micro_xrce_agent = ExecuteProcess(
        cmd=['MicroXRCEAgent', 'udp4', '-p', '8888', '-v', '2'],
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
            # Visualization helper for RViz: draws "no return" beams at range_max.
            'viz_output_topic': '/scan_merged_viz',
            'publish_hz': 10.0,
            # PX4's TOF-Ring model clips depth at 4m.
            # Filter very-near returns (drone body) but keep close obstacles for Nav2.
            'min_range_m': 0.35,
            'max_range_m': 4.0,
            # Publish a denser scan by expanding each ToF column (5.625°) into multiple
            # output rays. This significantly reduces the "spoke" artifacts in occupancy maps.
            'output_num_points': 360,
            'output_fill_bins': True,
            # Prefer the central rows (near 0° vertical) to avoid ground hits and
            # self-occlusion from aggressive flight attitudes.
            'roi_row_start': 3,
            'roi_row_end': 5,
            # Use a robust reduction across rows to avoid single-pixel self/ground hits.
            'column_reduce': 'median',
            # Require multiple valid samples per column to suppress depth speckle.
            'min_valid_per_column': 1,
            # Treat near-far-clip values as "no return" to avoid phantom perimeter walls.
            'far_clip_margin_m': 0.03,
            # Publish "no return" beams as a finite threshold so SLAM (Karto) can clear free
            # space in open areas. Internally, the LaserScan range_max is padded slightly above
            # the threshold so Karto doesn't ignore those beams (it drops readings >= maxRange).
            # We keep Nav2 obstacle marking bounded via obstacle_max_range (see nav2 params).
            'no_return_as_range_max': True,
            # Light angular-domain outlier suppression across the merged 360° scan.
            'angular_filter_window': 7,
            'angular_outlier_thresh_m': 0.7,
            # Short temporal median filter to reduce per-frame flicker.
            'temporal_window': 5,
        }],
    )

    pose_to_px4_vo = Node(
        package='tof_slam_sim',
        executable='pose_to_px4_visual_odometry',
        name='pose_to_px4_visual_odometry',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'pose_topic': '/model/x500_small_tof_0/pose',
            'px4_time_topic': '/fmu/out/vehicle_odometry',
            'px4_status_topic': '/fmu/out/vehicle_status',
            'px4_visual_odometry_topic': '/fmu/in/vehicle_visual_odometry',
            'publish_rate_hz': 30.0,
        }],
        condition=IfCondition(use_visual_odometry),
    )

    ld = LaunchDescription()
    ld.add_action(use_sim_time_arg)
    ld.add_action(world_arg)
    ld.add_action(world_name_arg)
    ld.add_action(gz_gui_arg)
    ld.add_action(use_visual_odometry_arg)
    ld.add_action(run_gz_arg)
    ld.add_action(headless_rendering_arg)
    ld.add_action(px4_dir_arg)
    ld.add_action(run_px4_arg)
    ld.add_action(run_bridge_arg)
    ld.add_action(run_agent_arg)
    ld.add_action(px4_model_pose_arg)
    ld.add_action(set_gz_resource_path)
    ld.add_action(set_px4_rc_path)
    ld.add_action(set_px4_model_pose)
    ld.add_action(gz_sim_headless)
    ld.add_action(gz_sim)
    ld.add_action(gz_gui)
    ld.add_action(px4_sitl)
    ld.add_action(bridge)
    ld.add_action(micro_xrce_agent)
    ld.add_action(tof_to_scan)
    ld.add_action(gz_pose)
    ld.add_action(pose_to_px4_vo)
    return ld
