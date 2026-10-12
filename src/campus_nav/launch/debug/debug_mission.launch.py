"""
Debug Mission Controller Launch — Isolated mission_controller_node test

Starts the minimum stack needed to test mission_controller_node:
  - Gazebo (campus world + buggy spawned)
  - ros_gz_bridge (teleport / odometry bridge)
  - robot_state_publisher
  - odom_bridge_node
  - mission_controller_node  ← THE NODE UNDER TEST

──────────────────────────────────────────────────────────────────────────
HOW TO TEST:
──────────────────────────────────────────────────────────────────────────

  Method 1 — RunMission Action Call (standard interface):
    ros2 action send_goal /campus/run_mission_action \
      saye_msgs/action/RunMission \
      "{start_stop: 'SAB C', goal_stop: 'Chemistry Block', dwell_seconds: 5.0}"

  Method 2 — Legacy Topic Input:
    ros2 topic pub --once /campus/mission std_msgs/msg/String \
      "{data: 'SAB C|Chemistry Block'}"

  Method 3 — Test cancellation:
    Send goal with dwell_seconds: 15.0 and hit Ctrl+C in terminal:
    verify clean cancel.

──────────────────────────────────────────────────────────────────────────
PASS CONDITIONS:
──────────────────────────────────────────────────────────────────────────

  ✅ Action Goal accepted immediately.
  ✅ Node logs: "[Boarding] Holding dwell for 5.0s at 'SAB C'...".
  ✅ After 5.0s, action returns:
       success: True
       message: "Boarding dwell completed successfully at SAB C."
       elapsed_seconds: ~5.0
  ✅ /campus/nav_status updates with boarding countdown.

Usage:
  ros2 launch campus_nav debug_mission.launch.py
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

    saye_xacro    = os.path.join(pkg_saye, 'models', 'saye', 'model.xacro')
    bridge_config = os.path.join(pkg_saye_bringup, 'config', 'ros_gz_bridge.yaml')

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
        mode='w', suffix='_saye_world_debug_mission.sdf', delete=False)
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
                '-x', '-24.12',
                '-y', '385.98',
                '-z', '0.80',
                '-Y', '-1.56'
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

    # ── THE NODE UNDER TEST ────────────────────────────────────────────────────
    mission_controller_node = Node(
        package='campus_nav',
        executable='mission_controller_node',
        name='mission_controller_node',
        output='screen',
        parameters=[{'osm_file': default_osm, 'use_sim_time': True}]
    )

    return LaunchDescription([
        *env_vars,

        gazebo,
        robot_state_publisher,
        spawn_saye,
        ros_gz_bridge,

        TimerAction(period=2.0, actions=[odom_bridge_node]),
        TimerAction(period=2.5, actions=[mission_controller_node]),
    ])
