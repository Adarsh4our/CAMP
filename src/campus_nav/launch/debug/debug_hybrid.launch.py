"""
Debug Hybrid Autonomy Launch — Complete Hybrid Driving Architecture Test

Architecture:
1. Normal Driving Mode:
   - planner_node (/campus/global_path)
   - lane_detector_node (/campus/raw_target_path -> /campus/target_path)
   - safety_monitor_node (/campus/safety_status: void / edge / obstacle)
   - path_follower_node (/campus/follower_enable: True -> Pure Pursuit on /cmd_vel)

2. Obstacle Mode (Dual-mode Switch):
   - When safety_monitor_node alerts an obstacle <= 6.0m:
   - campus_master_node pauses path_follower_node (/campus/follower_enable: False)
   - campus_master_node sends dynamic Nav2 goal 12m ahead past obstacle
   - Nav2 MPPI controller drives /cmd_vel using FAST-LIO2 point cloud (/cloud_registered)
   - When Nav2 finishes dodging obstacle, campus_master_node resumes path_follower_node!

Usage:
  ros2 launch campus_nav debug_hybrid.launch.py
"""

import os
import subprocess
import tempfile
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription, TimerAction, SetEnvironmentVariable
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from os.path import join


def generate_launch_description():
    subprocess.run(['pkill', '-9', '-f', 'gz sim'], capture_output=True)
    subprocess.run(['killall', '-9', 'gz-sim-server', 'gz-sim-gui'], capture_output=True)

    pkg_campus_nav   = get_package_share_directory('campus_nav')
    pkg_saye         = get_package_share_directory('saye_description')
    pkg_saye_bringup = get_package_share_directory('saye_bringup')
    pkg_ros_gz_sim   = get_package_share_directory('ros_gz_sim')

    saye_xacro       = os.path.join(pkg_saye, 'models', 'saye', 'model.xacro')
    bridge_config    = os.path.join(pkg_saye_bringup, 'config', 'ros_gz_bridge.yaml')
    campus_rviz      = os.path.join(pkg_campus_nav, 'rviz', 'campus_nav.rviz')
    nav2_params_file = os.path.join(pkg_campus_nav, 'config', 'nav2_params.yaml')
    fast_lio_yaml    = os.path.join(pkg_campus_nav, 'config', 'fast_lio_mid360.yaml')

    gz_resource_path = os.path.dirname(pkg_saye)
    os.environ['GZ_SIM_RESOURCE_PATH'] = gz_resource_path

    env_vars = [
        SetEnvironmentVariable('__NV_PRIME_RENDER_OFFLOAD', '1'),
        SetEnvironmentVariable('__GLX_VENDOR_LIBRARY_NAME', 'nvidia'),
        SetEnvironmentVariable('__VK_LAYER_NV_optimus', 'NVIDIA_only'),
        SetEnvironmentVariable('__GL_SYNC_TO_VBLANK', '0'),
        SetEnvironmentVariable('vblank_mode', '0'),
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', gz_resource_path),
    ]

    map_obj_abs = os.path.join(pkg_saye, 'worlds', 'map1.obj')
    sdf_template_path = os.path.join(pkg_saye, 'worlds', 'saye_world.sdf')
    with open(sdf_template_path, 'r') as f:
        sdf_content = f.read()
    sdf_content = sdf_content.replace(
        'model://saye_description/worlds/map1.obj',
        'file://' + map_obj_abs
    )
    tmp_sdf = tempfile.NamedTemporaryFile(mode='w', suffix='.sdf', delete=False)
    tmp_sdf.write(sdf_content)
    tmp_sdf.flush()
    tmp_sdf.close()

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': f'-r -v 1 {tmp_sdf.name}'
        }.items()
    )

    doc = xacro.process_file(saye_xacro)
    robot_description = doc.toxml()

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True
        }]
    )

    spawn_saye = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'saye',
            '-topic', 'robot_description',
            '-x', '-24.12',
            '-y', '385.98',
            '-z', '0.80',
            '-Y', '-1.56'
        ],
        output='screen'
    )

    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        parameters=[{'config_file': bridge_config}],
        output='screen'
    )

    odom_bridge_node = Node(
        package='campus_nav',
        executable='odom_bridge_node',
        name='odom_bridge_node',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    candidates_osm = [
        os.path.join(pkg_campus_nav, 'data', 'campus_with_junctions_and_stops.osm'),
        os.path.join(pkg_campus_nav, 'data', 'campus.osm'),
    ]
    default_osm = next((p for p in candidates_osm if os.path.exists(p)),
                       os.path.join(pkg_campus_nav, 'data', 'campus.osm'))
    stops_yaml = os.path.join(pkg_campus_nav, 'config', 'campus_junctions_and_stops.yaml')

    planner_node = Node(
        package='campus_nav',
        executable='planner_node',
        name='campus_planner_node',
        output='screen',
        parameters=[{
            'osm_file': default_osm,
            'stops_file': stops_yaml,
            'default_start_stop': 'LOC',
            'default_goal_stop': 'Chemistry Block',
            'nominal_speed': 2.22,
            'frame_id': 'map',
            'use_sim_time': True
        }]
    )

    lane_detector_node = Node(
        package='campus_nav',
        executable='lane_detector_node',
        name='lane_detector_node',
        output='screen',
        remappings=[
            ('/campus/raw_target_path', '/campus/global_path')
        ],
        parameters=[{
            'left_lane_offset': 1.2,
            'max_shift_clamp': 1.5,
            'control_rate_hz': 10.0,
            'use_sim_time': True
        }]
    )

    safety_monitor_node = Node(
        package='campus_nav',
        executable='safety_monitor_node',
        name='safety_monitor_node',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    path_follower_node = Node(
        package='campus_nav',
        executable='path_follower_node',
        name='path_follower_node',
        output='screen',
        parameters=[{
            'lookahead_distance': 4.0,
            'cruise_speed': 2.22,
            'max_speed': 2.22,
            'min_speed': 0.8,
            'goal_tolerance': 2.2,
            'control_rate_hz': 20.0,
            'use_sim_time': True
        }]
    )

    mission_controller_node = Node(
        package='campus_nav',
        executable='mission_controller_node',
        name='mission_controller_node',
        output='screen',
        parameters=[{'osm_file': default_osm, 'use_sim_time': True}]
    )

    # ── FAST-LIO2 SLAM ────────────────────────────────────────────────────────
    fast_lio_node = Node(
        package='fast_lio',
        executable='fastlio_mapping',
        name='fastlio_mapping',
        parameters=[
            fast_lio_yaml,
            {'use_sim_time': True}
        ],
        output='screen'
    )

    # ── Nav2 Controller (MPPI for obstacle evasion) ───────────────────────────
    nav2_controller_node = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[nav2_params_file],
        remappings=[('cmd_vel', '/cmd_vel'), ('/cmd_vel', '/cmd_vel')]
    )

    nav2_lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[
            {'use_sim_time': True},
            {'autostart': True},
            {'node_names': ['controller_server']}
        ]
    )

    # ── Campus Master FSM (Mode Switcher) ─────────────────────────────────────
    campus_master_node = Node(
        package='campus_nav',
        executable='campus_master_node',
        name='campus_master_node',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_debug_hybrid',
        arguments=['-d', campus_rviz],
        parameters=[{'use_sim_time': True}],
        output='screen',
        additional_env={
            '__NV_PRIME_RENDER_OFFLOAD': '1',
            '__GLX_VENDOR_LIBRARY_NAME': 'nvidia',
            '__GL_SYNC_TO_VBLANK': '0',
            'vblank_mode': '0'
        }
    )

    return LaunchDescription([
        *env_vars,

        gazebo,
        robot_state_publisher,
        spawn_saye,
        ros_gz_bridge,

        TimerAction(period=2.0, actions=[odom_bridge_node]),
        TimerAction(period=2.0, actions=[planner_node]),
        TimerAction(period=2.0, actions=[safety_monitor_node]),
        TimerAction(period=2.5, actions=[lane_detector_node]),
        TimerAction(period=2.5, actions=[path_follower_node]),
        TimerAction(period=3.0, actions=[fast_lio_node]),
        TimerAction(period=3.0, actions=[nav2_controller_node]),
        TimerAction(period=3.5, actions=[nav2_lifecycle_manager]),
        TimerAction(period=3.5, actions=[mission_controller_node]),
        TimerAction(period=4.0, actions=[campus_master_node]),
        TimerAction(period=4.0, actions=[rviz_node]),
    ])
