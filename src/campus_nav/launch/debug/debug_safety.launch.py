"""
Debug Safety Monitor Launch — Isolated safety_monitor_node test

Starts the minimum stack needed to test safety_monitor_node:
  - Gazebo (campus world + buggy spawned)
  - ros_gz_bridge (forwards /cloud LiDAR PointCloud2 to ROS 2)
  - robot_state_publisher (static TF)
  - odom_bridge_node (odometry / TF)
  - safety_monitor_node  ← THE NODE UNDER TEST
  - RViz (Livox LiDAR PointCloud2 + safety markers)

──────────────────────────────────────────────────────────────────────────
HOW TO TEST:
──────────────────────────────────────────────────────────────────────────

  1. In a separate terminal, observe the baseline safety status:
       ros2 topic echo /campus/safety_status

     You will see:
       {"state": "CLEAR", "min_distance_m": 99.0, ...}

  2. Spawn an obstacle (human) 5 meters directly in front of the buggy:
       ros2 run campus_nav test_obstacle_spawner --type human --distance 5.0

  3. Spawn a dog:
       ros2 run campus_nav test_obstacle_spawner --type dog --distance 4.0

  4. Remove all test obstacles:
       ros2 run campus_nav test_obstacle_spawner --remove

──────────────────────────────────────────────────────────────────────────
PASS CONDITIONS:
──────────────────────────────────────────────────────────────────────────

  ✅ Obstacle spawned: /campus/safety_status state changes to "VOID_AHEAD"
     or "EMERGENCY_STOP" with obstacle_type="PEDESTRIAN" or "ANIMAL".
  ✅ Obstacle removed: /campus/safety_status reverts cleanly to "CLEAR".
  ✅ RViz displays the red threat bounding box on /campus/safety/threats.

Usage:
  ros2 launch campus_nav debug_safety.launch.py
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
    livox_rviz    = os.path.join(pkg_campus_nav, 'rviz', 'livox_mid360.rviz')

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
        mode='w', suffix='_saye_world_debug_safety.sdf', delete=False)
    tmp_sdf.write(sdf_content)
    tmp_sdf.flush()
    tmp_sdf.close()
    campus_sdf = tmp_sdf.name

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
    safety_monitor_node = Node(
        package='campus_nav',
        executable='safety_monitor_node',
        name='safety_monitor_node',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_debug_safety',
        arguments=['-d', livox_rviz],
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
        TimerAction(period=2.5, actions=[safety_monitor_node]),
        TimerAction(period=3.5, actions=[rviz_node]),
    ])
