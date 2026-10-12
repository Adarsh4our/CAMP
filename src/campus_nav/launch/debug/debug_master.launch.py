"""
Debug Campus Master Launch — Isolated campus_master_node & full FSM integration test

Starts the entire operational autonomy stack with campus_master_node as the orchestrator:
  - Gazebo (campus world + buggy spawned)
  - ros_gz_bridge (forwards cmd_vel, cloud, odometry, clock)
  - robot_state_publisher
  - odom_bridge_node (pose & TF)
  - planner_node (PlanRoute Action Server)
  - safety_monitor_node (Safety interrupt watchdog)
  - mission_controller_node (RunMission Action Server for boarding dwell)
  - nav2_adapter_node (FollowRoute Action Server) + Nav2 controller_server
  - campus_master_node  ← THE ORCHESTRATOR UNDER TEST
  - RViz (full campus map visualization)

──────────────────────────────────────────────────────────────────────────
HOW TO TEST:
──────────────────────────────────────────────────────────────────────────

  Test 1 — Full 2-Leg Autonomous Mission:
    ros2 topic pub --once /campus/master_command std_msgs/msg/String \
      "{data: 'MISSION:SAB C|Chemistry Block'}"

  Test 2 — Single-Leg Mission from Current Position:
    ros2 topic pub --once /campus/master_command std_msgs/msg/String \
      "{data: 'GOAL:Chemistry Block'}"

  Test 3 — Emergency Stop Hazard Preemption:
    While buggy is moving, inject a pedestrian hazard in another terminal:
      ros2 run campus_nav test_obstacle_spawner --type human --distance 5.0
    Observe: Buggy halts instantly with zero velocity; FSM locks in SAFE_STOP.
    Clear hazard:
      ros2 run campus_nav test_obstacle_spawner --remove
    Observe: FSM recovers safely to IDLE.

  Test 4 — Operator Mission Abort:
    ros2 topic pub --once /campus/master_command std_msgs/msg/String \
      "{data: 'ABORT'}"

──────────────────────────────────────────────────────────────────────────
PASS CONDITIONS:
──────────────────────────────────────────────────────────────────────────

  ✅ Watch FSM status transitions on /campus/nav_status:
       IDLE ➔ PLANNING ➔ NAVIGATING ➔ BOARDING ➔ PLANNING ➔ NAVIGATING ➔ ARRIVED ➔ IDLE
  ✅ Buggy smoothly drives, stops for 5.0s boarding dwell, continues to destination.
  ✅ Safety interrupt triggers instant SAFE_STOP on obstacle detection.

Usage:
  ros2 launch campus_nav debug_master.launch.py
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
    tmp_sdf = tempfile.NamedTemporaryFile(
        mode='w', suffix='_saye_world_debug_master.sdf', delete=False)
    tmp_sdf.write(sdf_content)
    tmp_sdf.flush()
    tmp_sdf.close()
    campus_sdf = tmp_sdf.name

    candidates_osm = [
        os.path.join(pkg_campus_nav, 'data', 'campus_with_junctions_and_stops.osm'),
        os.path.join(pkg_campus_nav, 'data', 'campus.osm'),
    ]
    default_osm = next((p for p in candidates_osm if os.path.exists(p)),
                       os.path.join(pkg_campus_nav, 'data', 'campus.osm'))
    stops_yaml = os.path.join(pkg_campus_nav, 'config', 'campus_junctions_and_stops.yaml')

    robot_desc = xacro.process_file(saye_xacro).toxml()

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': '-r -v 4 ' + campus_sdf}.items()
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc, 'use_sim_time': True}]
    )

    spawn_saye = TimerAction(
        period=5.0,
        actions=[Node(
            package='ros_gz_sim',
            executable='create',
            name='spawn_saye',
            arguments=[
                '-string', robot_desc,
                '-name',   'saye',
                '-world',  'campus_world',
                '-allow_renaming', 'false',
                '-x', '15.02',
                '-y', '43.00',
                '-z', '0.80',
                '-Y', '3.14159'
            ],
            output='screen'
        )]
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
        parameters=[{'model_name': 'saye', 'use_sim_time': True}]
    )

    planner_node = Node(
        package='campus_nav',
        executable='planner_node',
        name='campus_planner_node',
        output='screen',
        parameters=[{
            'osm_file': default_osm,
            'stops_file': stops_yaml,
            'default_start_stop': 'SAB C',
            'default_goal_stop': 'Chemistry Block',
            'nominal_speed': 2.22,
            'frame_id': 'map',
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

    mission_controller_node = Node(
        package='campus_nav',
        executable='mission_controller_node',
        name='mission_controller_node',
        output='screen',
        parameters=[{'osm_file': default_osm, 'use_sim_time': True}]
    )

    nav2_adapter_node = Node(
        package='campus_nav',
        executable='nav2_adapter_node',
        name='nav2_adapter_node',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    nav2_controller_node = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[nav2_params_file],
        remappings=[('/cmd_vel', '/cmd_vel')]
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

    # ── THE ORCHESTRATOR UNDER TEST ────────────────────────────────────────────
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
        name='rviz2_debug_master',
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
        TimerAction(period=2.0, actions=[nav2_adapter_node]),
        TimerAction(period=2.0, actions=[nav2_controller_node]),
        TimerAction(period=2.5, actions=[nav2_lifecycle_manager]),
        TimerAction(period=2.5, actions=[mission_controller_node]),
        TimerAction(period=3.5, actions=[campus_master_node]),
        TimerAction(period=3.5, actions=[rviz_node]),
    ])
