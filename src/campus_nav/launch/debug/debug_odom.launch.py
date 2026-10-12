"""
Debug Odometry Launch — Isolated odom_bridge_node test

Starts the minimum stack needed to test odom_bridge_node:
  - Gazebo (campus world + buggy spawned)
  - ros_gz_bridge (forwards /model/saye/odometry_world to ROS 2)
  - robot_state_publisher (TF static)
  - odom_bridge_node  ← THE NODE UNDER TEST
  - wasd_teleop (Tkinter keyboard driving GUI)
  - RViz (visualize vehicle TF & pose markers)

──────────────────────────────────────────────────────────────────────────
HOW TO TEST:
──────────────────────────────────────────────────────────────────────────

  1. The WASD Teleop GUI window will pop up.
  2. Click into the WASD window to focus it.
  3. Press W / S to accelerate/brake, A / D to steer.
  4. Observe the buggy driving in Gazebo.

──────────────────────────────────────────────────────────────────────────
PASS CONDITIONS:
──────────────────────────────────────────────────────────────────────────

  ✅ Gazebo: Buggy drives forward/turns.
  ✅ RViz: The robot model and TF frame (nav_base_link) track the movement.
  ✅ Terminal (Echo Vehicle Pose in Planner coords):
       ros2 topic echo /campus/vehicle_pose
     (Values x, y should change smoothly as you drive).
  ✅ Check TF tree:
       ros2 run tf2_ros tf2_echo map nav_base_link

Usage:
  ros2 launch campus_nav debug_odom.launch.py
"""

import os
import subprocess
import tempfile
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription, TimerAction, SetEnvironmentVariable,
    DeclareLaunchArgument
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
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
    campus_rviz   = os.path.join(pkg_campus_nav, 'rviz', 'campus_nav.rviz')

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
        mode='w', suffix='_saye_world_debug_odom.sdf', delete=False)
    tmp_sdf.write(sdf_content)
    tmp_sdf.flush()
    tmp_sdf.close()
    campus_sdf = tmp_sdf.name

    robot_desc = xacro.process_file(saye_xacro).toxml()

    enable_teleop_arg = DeclareLaunchArgument(
        'teleop', default_value='true',
        description='Launch WASD keyboard teleop GUI'
    )

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

    # ── THE NODE UNDER TEST ────────────────────────────────────────────────────
    odom_bridge_node = Node(
        package='campus_nav',
        executable='odom_bridge_node',
        name='odom_bridge_node',
        output='screen',
        parameters=[{'model_name': 'saye', 'use_sim_time': True}]
    )

    # ── Interactive Teleop GUI ─────────────────────────────────────────────────
    teleop_node = Node(
        package='campus_nav',
        executable='wasd_teleop',
        name='wasd_teleop',
        output='screen',
        condition=IfCondition(LaunchConfiguration('teleop'))
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_debug_odom',
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

    # ── OSM and stops config for Road Markers in RViz ──────────────────────────
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
            'default_start_stop': 'SAB C',
            'default_goal_stop': 'Chemistry Block',
            'nominal_speed': 2.22,
            'frame_id': 'map',
            'use_sim_time': True
        }]
    )

    return LaunchDescription([
        *env_vars,
        enable_teleop_arg,

        gazebo,
        robot_state_publisher,
        spawn_saye,
        ros_gz_bridge,

        TimerAction(period=2.0, actions=[odom_bridge_node]),
        TimerAction(period=2.0, actions=[planner_node]),
        TimerAction(period=3.0, actions=[rviz_node]),
        TimerAction(period=4.0, actions=[teleop_node]),
    ])
