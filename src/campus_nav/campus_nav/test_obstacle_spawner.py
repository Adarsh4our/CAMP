"""
Test Obstacle Spawner for Gazebo Harmonic

Spawns test obstacles in front of the saye buggy or at specific coordinates:
  - human: Standing pedestrian model (height 1.7m, radius 0.25m)
  - dog: Small animal model (height 0.45m, length 0.6m)
  - speed_breaker: Realistic road bump (height 0.08m, width 3.6m, length 0.5m)

Usage:
  ros2 run campus_nav test_obstacle_spawner --type human --distance 6.0
  ros2 run campus_nav test_obstacle_spawner --type speed_breaker --distance 8.0
  ros2 run campus_nav test_obstacle_spawner --type dog --distance 5.0
  ros2 run campus_nav test_obstacle_spawner --remove
"""

import argparse
import subprocess
import sys


SDF_HUMAN = """<?xml version='1.0'?>
<sdf version='1.8'>
  <model name='test_obstacle_human'>
    <static>true</static>
    <link name='link'>
      <collision name='collision'>
        <geometry>
          <cylinder>
            <radius>0.25</radius>
            <length>1.7</length>
          </cylinder>
        </geometry>
      </collision>
      <visual name='visual'>
        <geometry>
          <cylinder>
            <radius>0.25</radius>
            <length>1.7</length>
          </cylinder>
        </geometry>
        <material>
          <ambient>0.8 0.1 0.1 1.0</ambient>
          <diffuse>0.9 0.1 0.1 1.0</diffuse>
        </material>
      </visual>
    </link>
  </model>
</sdf>"""

SDF_DOG = """<?xml version='1.0'?>
<sdf version='1.8'>
  <model name='test_obstacle_dog'>
    <static>true</static>
    <link name='link'>
      <collision name='collision'>
        <geometry>
          <box>
            <size>0.6 0.35 0.45</size>
          </box>
        </geometry>
      </collision>
      <visual name='visual'>
        <geometry>
          <box>
            <size>0.6 0.35 0.45</size>
          </box>
        </geometry>
        <material>
          <ambient>0.6 0.35 0.1 1.0</ambient>
          <diffuse>0.7 0.4 0.15 1.0</diffuse>
        </material>
      </visual>
    </link>
  </model>
</sdf>"""

SDF_BUMP = """<?xml version='1.0'?>
<sdf version='1.8'>
  <model name='test_obstacle_bump'>
    <static>true</static>
    <link name='link'>
      <collision name='collision'>
        <geometry>
          <box>
            <size>3.6 0.5 0.08</size>
          </box>
        </geometry>
      </collision>
      <visual name='visual'>
        <geometry>
          <box>
            <size>3.6 0.5 0.08</size>
          </box>
        </geometry>
        <material>
          <ambient>1.0 0.85 0.0 1.0</ambient>
          <diffuse>1.0 0.85 0.0 1.0</diffuse>
        </material>
      </visual>
    </link>
  </model>
</sdf>"""


import math
import time

def get_buggy_pose():
    """Query live buggy pose from ROS topics (/campus/vehicle_pose or /model/saye/odometry_world), falling back to Gazebo or LOC defaults."""
    # 1. Try ROS 2 topics (/campus/vehicle_pose or /model/saye/odometry_world)
    try:
        import rclpy
        from geometry_msgs.msg import PoseStamped
        from nav_msgs.msg import Odometry
        from campus_nav.coord_bridge import nav_to_gz, yaw_from_quaternion

        if not rclpy.ok():
            rclpy.init()
        node = rclpy.create_node('temp_spawner_listener')
        received = []

        def pose_cb(msg):
            p = msg.pose
            gz_x, gz_y = nav_to_gz(p.position.x, p.position.y)
            yaw = yaw_from_quaternion(p.orientation)
            received.append((gz_x, gz_y, p.position.z, yaw, 0.0, '/campus/vehicle_pose'))

        def odom_cb(msg):
            p = msg.pose.pose
            gz_x, gz_y = p.position.x, p.position.y
            v = msg.twist.twist.linear
            speed = math.hypot(v.x, v.y)
            raw_yaw = yaw_from_quaternion(p.orientation)
            # saye front is along -Y of base_link:
            driving_yaw = (raw_yaw - (math.pi / 2.0) + math.pi) % (2.0 * math.pi) - math.pi
            received.append((gz_x, gz_y, p.position.z, driving_yaw, speed, '/model/saye/odometry_world'))

        sub_odom = node.create_subscription(Odometry, '/model/saye/odometry_world', odom_cb, 10)
        sub_pose = node.create_subscription(PoseStamped, '/campus/vehicle_pose', pose_cb, 10)

        # Spin for up to 2.5 seconds to allow DDS endpoint discovery
        t_end = time.time() + 2.5
        while time.time() < t_end and not received:
            rclpy.spin_once(node, timeout_sec=0.05)

        node.destroy_node()
        if received:
            # Sort to prefer odometry (has velocity) if available
            received.sort(key=lambda item: 0 if 'odometry_world' in item[5] else 1)
            gz_x, gz_y, gz_z, yaw, speed, source = received[0]
            print(f"📍 Detected live buggy via {source} at Gazebo ({gz_x:.2f}, {gz_y:.2f}), speed={speed:.2f}m/s, driving_yaw={math.degrees(yaw):.1f}°")
            return (gz_x, gz_y, gz_z, yaw, speed)
    except Exception as e:
        print(f"⚠️ Exception querying ROS topics: {e}")

    # 2. Try Gazebo Harmonic directly via CLI
    try:
        import re
        res = subprocess.run(['gz', 'model', '-m', 'saye', '-p'], capture_output=True, text=True, timeout=1.5)
        m = re.search(r'\[\s*([-\d.]+)\s+([-\d.]+)\s+([-\d.]+)\s*\]\s*\n\s*\[\s*([-\d.]+)\s+([-\d.]+)\s+([-\d.]+)\s*\]', res.stdout)
        if m:
            gx, gy, gz = float(m.group(1)), float(m.group(2)), float(m.group(3))
            raw_yaw = float(m.group(6))
            driving_yaw = (raw_yaw - (math.pi / 2.0) + math.pi) % (2.0 * math.pi) - math.pi
            print(f"📍 Detected live buggy via Gazebo model service at ({gx:.2f}, {gy:.2f}), driving_yaw={math.degrees(driving_yaw):.1f}°")
            return (gx, gy, gz, driving_yaw, 0.0)
    except Exception:
        pass

    # 3. Default to LOC spawn coordinates facing West (heading along road: ~ -179.3°)
    print("⚠️ Using default LOC spawn coordinates for buggy (-24.12, 385.98, heading West).")
    return (-24.12, 385.98, 0.30, -3.1285, 0.0)


def spawn_obstacle(model_name: str, sdf_str: str, x: float, y: float, z: float, yaw: float = 0.0):
    """Call Gazebo Harmonic EntityFactory service."""
    # Flatten SDF to single line with escaped quotes
    sdf_escaped = sdf_str.replace('"', '\\"').replace('\n', ' ')
    req_str = (
        f'sdf: "{sdf_escaped}", '
        f'pose: {{ position: {{ x: {x:.2f}, y: {y:.2f}, z: {z:.2f} }} }}'
    )
    cmd = [
        'gz', 'service', '-s', '/world/campus_world/create',
        '--reqtype', 'gz.msgs.EntityFactory',
        '--reptype', 'gz.msgs.Boolean',
        '--timeout', '3000',
        '--req', req_str
    ]
    res = subprocess.run(cmd, capture_output=True, text=True)
    if "data: true" in res.stdout or res.returncode == 0:
        print(f"✅ Successfully spawned '{model_name}' at Gazebo ({x:.2f}, {y:.2f}, {z:.2f})")
    else:
        print(f"❌ Failed to spawn '{model_name}': {res.stderr or res.stdout}")


def remove_obstacle(model_name: str):
    """Remove entity from Gazebo."""
    req_str = f'name: "{model_name}", type: 2'
    cmd = [
        'gz', 'service', '-s', '/world/campus_world/remove',
        '--reqtype', 'gz.msgs.Entity',
        '--reptype', 'gz.msgs.Boolean',
        '--timeout', '2000',
        '--req', req_str
    ]
    res = subprocess.run(cmd, capture_output=True, text=True)
    if "data: true" in res.stdout or res.returncode == 0:
        print(f"🧹 Removed '{model_name}'")
    else:
        print(f"⚠️ Could not remove '{model_name}' (may not exist)")


def main():
    parser = argparse.ArgumentParser(description="Test Obstacle Spawner")
    parser.add_argument('--type', choices=['human', 'dog', 'speed_breaker'],
                        default='human', help="Obstacle type to spawn")
    parser.add_argument('--distance', type=float, default=7.0,
                        help="Distance ahead of buggy in meters (default 7.0m)")
    parser.add_argument('--x', type=float, default=None, help="Explicit Gazebo X")
    parser.add_argument('--y', type=float, default=None, help="Explicit Gazebo Y")
    parser.add_argument('--z', type=float, default=None, help="Explicit Gazebo Z")
    parser.add_argument('--remove', action='store_true',
                        help="Remove all test obstacles from simulation")
    args = parser.parse_args()

    models = ['test_obstacle_human', 'test_obstacle_dog', 'test_obstacle_bump']

    if args.remove:
        for m in models:
            remove_obstacle(m)
        return

    # Determine spawn location
    bx, by, bz, byaw, speed = get_buggy_pose()
    if args.x is not None and args.y is not None:
        target_x, target_y = args.x, args.y
        target_z = args.z if args.z is not None else 0.30
    else:
        # Account for ~1.2s Gazebo entity creation latency when vehicle is moving
        lead_dist = speed * 1.2 if speed > 0.3 else 0.0
        effective_dist = args.distance + lead_dist
        if lead_dist > 0.1:
            print(f"🏎️ Buggy moving at {speed:.2f} m/s — added {lead_dist:.2f}m lead compensation (total {effective_dist:.2f}m ahead)")
        target_x = bx + effective_dist * math.cos(byaw)
        target_y = by + effective_dist * math.sin(byaw)
        target_z = 0.30

    if args.type == 'human':
        spawn_obstacle('test_obstacle_human', SDF_HUMAN, target_x, target_y, target_z + 0.85)
    elif args.type == 'dog':
        spawn_obstacle('test_obstacle_dog', SDF_DOG, target_x, target_y, target_z + 0.22)
    elif args.type == 'speed_breaker':
        spawn_obstacle('test_obstacle_bump', SDF_BUMP, target_x, target_y, target_z + 0.04)


if __name__ == '__main__':
    main()
