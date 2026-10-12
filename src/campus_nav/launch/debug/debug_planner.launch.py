"""
Debug Planner Launch — Isolated planner_node test

Starts the minimum stack needed to test planner_node:
  - Gazebo (campus world + buggy spawned, but NOT moving)
  - ros_gz_bridge  (odom data flows in)
  - odom_bridge_node  (pose → planner frame)
  - robot_state_publisher
  - planner_node  ← THE NODE UNDER TEST
  - RViz (campus map view)

Does NOT start: campus_master_node, Nav2, safety_monitor, path_follower,
                mission_controller, lane_detector — buggy will NOT drive.

──────────────────────────────────────────────────────────────────────────
HOW TO TRIGGER A PLAN (3 methods, use whichever you prefer):
──────────────────────────────────────────────────────────────────────────

  Method 1 — Topic string (easiest):
    ros2 topic pub --once /campus/plan_route std_msgs/msg/String \
      "{data: 'SAB C|Chemistry Block'}"

  Method 2 — 2D Goal Pose in RViz:
    Click the "2D Goal Pose" button in RViz, then click somewhere on the
    campus map. The planner finds the nearest road node automatically.

  Method 3 — Action server (most verbose, shows feedback):
    ros2 action send_goal /campus/plan_route_action \
      saye_msgs/action/PlanRoute \
      "{mission: 'SAB C|Chemistry Block'}"

──────────────────────────────────────────────────────────────────────────
HOW TO SEE THE RESULT:
──────────────────────────────────────────────────────────────────────────

  Watch the path appear in RViz on /campus/global_path (green line).

  Or echo it in the terminal:
    ros2 topic echo /campus/global_path --field header

  See all waypoints:
    ros2 topic echo /campus/global_path

  List available stop names:
    ros2 param get /campus_planner_node stops_file
    (then read the yaml file to see valid stop names)

Usage:
  ros2 launch campus_nav debug_planner.launch.py
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
    # ── Kill any lingering Gazebo ──────────────────────────────────────────────
    subprocess.run(['pkill', '-9', '-f', 'gz sim'], capture_output=True)
    subprocess.run(['killall', '-9', 'gz-sim-server', 'gz-sim-gui'], capture_output=True)

    # ── Package paths ──────────────────────────────────────────────────────────
    pkg_campus_nav   = get_package_share_directory('campus_nav')
    pkg_saye         = get_package_share_directory('saye_description')
    pkg_saye_bringup = get_package_share_directory('saye_bringup')
    pkg_ros_gz_sim   = get_package_share_directory('ros_gz_sim')

    # ── File paths ─────────────────────────────────────────────────────────────
    saye_xacro    = os.path.join(pkg_saye, 'models', 'saye', 'model.xacro')
    bridge_config = os.path.join(pkg_saye_bringup, 'config', 'ros_gz_bridge.yaml')
    campus_rviz   = os.path.join(pkg_campus_nav, 'rviz', 'campus_nav.rviz')

    # ── GPU environment ────────────────────────────────────────────────────────
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

    # ── Patch SDF (same as full_simulation — model:// → file://) ──────────────
    map_obj_abs = os.path.join(pkg_saye, 'worlds', 'map1.obj')
    sdf_template_path = os.path.join(pkg_saye, 'worlds', 'saye_world.sdf')
    with open(sdf_template_path, 'r') as f:
        sdf_content = f.read()
    sdf_content = sdf_content.replace(
        'model://saye_description/worlds/map1.obj',
        'file://' + map_obj_abs
    )
    tmp_sdf = tempfile.NamedTemporaryFile(
        mode='w', suffix='_saye_world_debug.sdf', delete=False)
    tmp_sdf.write(sdf_content)
    tmp_sdf.flush()
    tmp_sdf.close()
    campus_sdf = tmp_sdf.name

    # ── OSM and stops config ───────────────────────────────────────────────────
    candidates_osm = [
        os.path.join(pkg_campus_nav, 'data', 'campus_with_junctions_and_stops.osm'),
        os.path.join(pkg_campus_nav, 'data', 'campus.osm'),
    ]
    default_osm = next((p for p in candidates_osm if os.path.exists(p)),
                       os.path.join(pkg_campus_nav, 'data', 'campus.osm'))
    stops_yaml = os.path.join(pkg_campus_nav, 'config', 'campus_junctions_and_stops.yaml')

    # ── Robot description ──────────────────────────────────────────────────────
    robot_desc = xacro.process_file(saye_xacro).toxml()

    # ── Nodes ──────────────────────────────────────────────────────────────────

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

    # Spawn the buggy so odom topics are published — but nothing will drive it
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

    # ── THE NODE UNDER TEST ────────────────────────────────────────────────────
    planner_node = Node(
        package='campus_nav',
        executable='planner_node',
        name='campus_planner_node',
        output='screen',
        respawn=True,
        respawn_delay=2.0,
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

    odom_bridge_node = Node(
        package='campus_nav',
        executable='odom_bridge_node',
        name='odom_bridge_node',
        output='screen',
        parameters=[{'model_name': 'saye', 'use_sim_time': True}]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_debug_planner',
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

        # Simulation base
        gazebo,
        robot_state_publisher,
        spawn_saye,
        ros_gz_bridge,

        # Planner stack (2s delay — Gazebo needs to be up first)
        TimerAction(period=2.0, actions=[odom_bridge_node]),
        TimerAction(period=2.0, actions=[planner_node]),

        # RViz last
        TimerAction(period=3.0, actions=[rviz_node]),
    ])
