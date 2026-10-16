# campus_nav — Autonomous Campus Road Navigation & Safety Stack

`campus_nav` is the core autonomy package for the `saye` electric buggy on ROS 2 Jazzy and Gazebo Harmonic. It provides OpenStreetMap (OSM) ingestion, A* routing, 3.8m route ribbons, camera-based path adjustment, Pure Pursuit driving, LiDAR safety classification, and mission orchestration. Nav2 MPPI temporarily takes over to pass an obstacle, then returns control to Pure Pursuit on the original route.

- [Command reference: build, launch, dispatch, and obstacle tests](command.md)
- [Full test commands and expected behavior](TESTING.md)
- [Obstacle-avoidance correction and validation results](test/OBSTACLE_AVOIDANCE_VALIDATION.md)

## Build, Launch, and Dispatch

From the workspace root, with the other workspace packages already built:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
colcon build --packages-select campus_nav --symlink-install
source install/setup.bash

export DISPLAY=:1
ros2 launch campus_nav full_simulation.launch.py
```

Wait about 20 seconds after the buggy appears for sensor initialization. In another terminal, source ROS and the workspace, then check Nav2 and dispatch through the master:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 lifecycle get /controller_server  # Expected: active

# Direct goal from the current position.
ros2 topic pub --once /campus/master_command std_msgs/msg/String \
  "{data: 'GOAL:Chemistry Block'}"

# Alternatively, pickup -> five-second boarding dwell -> drop-off.
ros2 topic pub --once /campus/master_command std_msgs/msg/String \
  "{data: 'MISSION:SAB C|Chemistry Block'}"
```

Choose one command per run. The old `/campus/mission` subscriber only logs instructions to use the master. `/campus/goal_stop` and RViz `/goal_pose` still request planner routes directly, but bypass the master's mission state transitions. Use `/campus/master_command` to test automatic Nav2 handoff.

---

## Architecture Overview

```
/campus/master_command
        |
        v
campus_master_node ---- PlanRoute action ----> planner_node <---- OSM map
        |                                          |
        |                                  /campus/global_path
        |                                          |
        |                                  lane_detector_node <---- RGB camera
        |                                          |
        |                                  /campus/target_path
        |                                          |
        |                                  path_follower_node ----> /cmd_vel
        |                                          ^
        +---- /campus/follower_enable --------------+
        |
        +---- FollowPath detour action ----> Nav2 MPPI controller ----> /cmd_vel
        ^                                       ^
        |                                       |
/campus/safety_status                     local costmap <---- /cloud
        |
safety_monitor_node <---- /cloud

Gazebo odometry ----> odom_bridge_node ----> /campus/vehicle_pose, /odom, TF
```

The full and hybrid launches remap the lane detector's `/campus/raw_target_path` input to `/campus/global_path`. The master calls Nav2 directly for obstacle avoidance; the adapter's FollowRoute interface is a separate entry point.

### Obstacle Avoidance Lifecycle

1. The master enters `NAVIGATING`; Pure Pursuit follows the camera-adjusted route.
2. The safety monitor reports obstacle type, distance, state, and speed limit.
3. During `NAVIGATING`, a non-speed-breaker obstacle reported as `SLOWDOWN_APPROACH` or `EMERGENCY_STOP` between 2.0m and 8.5m triggers avoidance, subject to the master's three-second trigger cooldown.
4. The master enters `OBSTACLE_AVOIDANCE`, publishes `follower_enable=false`, and sends a detour to Nav2's `follow_path` action. The existing detour shifts up to 1.35m left, passes the obstacle, and returns toward the route.
5. On successful Nav2 completion, the master re-enables Pure Pursuit and returns to `NAVIGATING` on the original route.

Speed breakers use `SLOWDOWN_BUMP` under Pure Pursuit. Confirm a successful pass using the vehicle motion and `Nav2 avoidance complete` log message. The master also resumes Pure Pursuit after rejected or aborted Nav2 goals, so returning to `NAVIGATING` alone does not prove success.

---

## Nodes & Modules

### 1. `planner_node`
* **File:** `campus_nav/planner_node.py`
* **Function:** Ingests `campus_with_junctions_and_stops.osm`, builds a network graph of roads, named campus stops, and intersection nodes, and computes optimal routes using A*.
* **Outputs:**
  * `/campus/global_path` (`nav_msgs/msg/Path`): Centerline waypoint trajectory.
  * `/campus/route_ribbon` (`visualization_msgs/msg/Marker`): Solid 3.8m wide road mesh overlay ("road carpet") marking the active navigation corridor.
  * `/campus/road_graph` & `/campus/stop_markers`: Interactive RViz markers for all 32 campus stops and road segments.

### 2. `path_follower_node`
* **File:** `campus_nav/path_follower_node.py`
* **Function:** Pure Pursuit controller tailored for Ackermann steering kinematics.
* **Features:**
  * Runs at 20 Hz with steering-dependent speed reduction and speed-dependent lookahead bounded by 1.5m and the configured lookahead distance (4.0m by default).
  * Normal driving in the validated workflow commanded 0.8m/s. The configured 2.22m/s cruise ceiling is not a measured operating speed for this workflow.
  * Safety arbitration: Reacts instantly to `/campus/safety_status` by halting on `EMERGENCY_STOP`, slowing to $0.8\,\text{m/s}$ on `SLOWDOWN_BUMP`, or injecting lateral steering bias on `EDGE_WARNING`.
  * Yields `/cmd_vel` while `/campus/follower_enable` is `false`, allowing Nav2 to drive the detour.

### 3. `safety_monitor_node`
* **File:** `campus_nav/safety_monitor_node.py`
* **Function:** 3D LiDAR obstacle detection, elevation profiling, and road drop-off supervision. The depth-cloud subscription exists, but its confirmation callback is currently a placeholder.
* **Features:**
  * **Dynamic Lookahead Corridor:** Begins 1.5m ahead and uses at least 6.5m of lookahead, increasing up to 8.5m with commanded speed.
  * **Geometric Elevation Profiling:**
    * Ground plane estimation ($<4\,\text{cm}$ relative height): Identified as normal drivable asphalt.
    * Speed breaker detection ($4\,\text{cm} - 16\,\text{cm}$): Triggers `SLOWDOWN_BUMP` ($0.8\,\text{m/s}$).
    * Confirmed obstacle returns 0.16–2.2m above ground: Reports `SLOWDOWN_APPROACH` beyond the 3.2m stop distance, and `EMERGENCY_STOP` inside it. The master can hand these obstacles to Nav2 while navigating.
  * **Flank Void & Cliff Guard:** Monitors lateral bands 0.8–2.2m from the vehicle center. Detects drops more than 0.40m below estimated ground, reporting `EDGE_WARNING` at the flanks or `VOID_AHEAD` in the forward corridor.
  * **Tactical 2D BEV Radar Feed:** Publishes `/campus/lidar_bev_image` (a $400 \times 400$ top-down radar image with $5\text{m}/10\text{m}/15\text{m}/20\text{m}$ distance rings, vehicle footprint, color-coded returns, and real-time HUD status banner).

### 4. `mission_controller_node`
* **File:** `campus_nav/mission_controller_node.py`
* **Function:** Provides `campus/run_mission_action` for the passenger boarding dwell phase. `campus_master_node` owns the mission sequence: navigate to pickup, request the five-second dwell, then navigate to drop-off.
* **Legacy interface:** `/campus/mission` only prints a migration instruction. Dispatch `MISSION:Pickup|Dropoff` or `GOAL:Destination` on `/campus/master_command`.

### 5. `odom_bridge_node`
* **File:** `campus_nav/odom_bridge_node.py`
* **Function:** Metric and TF frame bridge connecting Gazebo's world frame to the planner's map frame.
* **Calibration:** Translates between Gazebo world coords and planner map coords via calibrated offset ($X = 802.0\,\text{m}, Y = 679.7\,\text{m}$).
* **TF:** Broadcasts `map -> world`, `map -> odom` (identity), and `map -> nav_base_link`. The Gazebo TF bridge and robot state publisher provide the vehicle/sensor frame chain.
* **Nav2 feedback:** Publishes `/odom` with `map` as the pose frame and `nav_base_link` as the child frame. Navigation heading is CAD yaw minus 90 degrees; the fixed yaw rotation preserves angular Z velocity's sign. Vehicle pose is derived from Gazebo ground truth; FAST-LIO does not replace that source in this launch.
* **Void Catch Supervisor:** Automatically detects if the buggy drops below $Z = -0.5\,\text{m}$ (accidental cliff fall) and safely teleports it back to the nearest road node.

### 6. `orient_buggy` & `teleport_buggy`
* **File:** `campus_nav/teleport_buggy.py`
* **Function:** Instant repositioning, road lane alignment, and turnaround CLI tools.
* **Commands:**
  * `ros2 run campus_nav orient_buggy`: Automatically finds the nearest road segment and snaps the buggy's heading perfectly parallel to the road.
  * `ros2 run campus_nav orient_buggy --flip`: Performs an instant $180^\circ$ U-turn facing the opposite road direction.
  * `ros2 run campus_nav teleport_buggy --stop "<STOP_NAME>"`: Instantly teleports the buggy to any of the 32 campus stops, snapped parallel to the road.

### 7. `arrange_windows`
* **File:** `campus_nav/arrange_windows.py`
* **Function:** X11 window placement manager via `libX11` ctypes. Automatically detects and docks the Gazebo simulation window to the bottom-right quadrant ($960 \times 520$ at $X=960, Y=520$) during launch.

### 8. `test_obstacle_spawner`
* **File:** `campus_nav/test_obstacle_spawner.py`
* **Function:** Spawns physical obstacles in Gazebo for reactive testing:
  * `--type human`: Spawns a standing pedestrian in front of the vehicle.
  * `--type speed_breaker`: Spawns an $8\,\text{cm}$ elevated bump perpendicular across the road.
  * `--type dog`: Spawns a 0.45m-tall box representing a low animal obstacle.
  * `--remove`: Removes all test entities.
* These models are static obstacles spawned while the buggy is moving. The spawner adds lead distance when an odometry speed estimate is available.

### 9. `campus_master_node`
* **File:** `campus_nav/campus_master_node.py`
* **Function:** Operator command handling, planning/navigation/boarding state transitions, and switching between Pure Pursuit and Nav2 MPPI.
* **Outputs:** `/campus/follower_enable`, `/campus/avoidance_path`, and navigation status. Calls Nav2's `follow_path` action directly for detours.

### 10. `nav2_adapter_node`
* **File:** `campus_nav/nav2_adapter_node.py`
* **Function:** Exposes `campus/follow_route_action` and `/campus/nav2_target_path` for forwarding paths to Nav2. Supports fetching a route when an action supplies a destination and an empty path. The hybrid master's avoidance branch uses its own Nav2 action client.

---

## Topics Reference

| Topic | Type | Description |
|---|---|---|
| `/campus/master_command` | `std_msgs/String` | Hybrid workflow: `GOAL:Stop` or `MISSION:Pickup\|Dropoff` |
| `/campus/mission` | `std_msgs/String` | Legacy input; logs instructions to use the master |
| `/campus/goal_stop` | `std_msgs/String` | Planner-only goal input; bypasses master mission state transitions |
| `/campus/global_path` | `nav_msgs/Path` | Global path waypoints generated by A* planner |
| `/campus/target_path` | `nav_msgs/Path` | Lane-adjusted path consumed by Pure Pursuit |
| `/campus/speed_profile` | `std_msgs/Float32MultiArray` | Planner speed-profile output |
| `/campus/avoidance_path` | `nav_msgs/Path` | Master's local Nav2 detour for visualization |
| `/campus/follower_enable` | `std_msgs/Bool` | `false`: yield to Nav2; `true`: enable Pure Pursuit |
| `/campus/nav_status` | `std_msgs/String` | Mission/FSM and navigation status from multiple nodes |
| `/campus/pursuit_status` | `std_msgs/String` | Follower/adapter status |
| `/campus/route_ribbon` | `visualization_msgs/Marker` | 3.8m wide continuous road carpet mesh |
| `/campus/vehicle_pose` | `geometry_msgs/PoseStamped` | Ground-truth-derived vehicle pose in `map` frame |
| `/odom` | `nav_msgs/Odometry` | Bridge pose and velocity feedback for Nav2 |
| `/campus/vehicle_marker` | `visualization_msgs/MarkerArray` | 3D visual buggy model in RViz |
| `/campus/safety_status` | `std_msgs/String` | JSON safety arbitration state, speed limit, and threat info |
| `/campus/lidar_bev_image` | `sensor_msgs/Image` | Live 2D Bird's-Eye-View tactical radar stream |
| `/campus/safety_corridor` | `visualization_msgs/Marker` | Dynamic 3D lookahead corridor bounding box |
| `/campus/safety_threats` | `visualization_msgs/MarkerArray` | Clustered obstacle return points |
| `/cmd_vel` | `geometry_msgs/Twist` | Velocity and steering commands to Gazebo |
| `/cloud` | `sensor_msgs/PointCloud2` | 3D LiDAR point cloud (360 azimuth x 64 rings) |
| `/real_sense/image_raw` | `sensor_msgs/Image` | Forward RealSense RGB camera feed |
| `/local_costmap/costmap` | `nav_msgs/OccupancyGrid` | Nav2's local obstacle map |

| Action | Type | Use |
|---|---|---|
| `/campus/plan_route_action` | `saye_msgs/action/PlanRoute` | Route request/result |
| `/campus/run_mission_action` | `saye_msgs/action/RunMission` | Boarding dwell request/result |
| `/campus/follow_route_action` | `saye_msgs/action/FollowRoute` | Adapter interface to Nav2 |
| `/follow_path` | `nav2_msgs/action/FollowPath` | Nav2 controller action used for detours |

---

## Configuration Files

* `config/nav2_params.yaml`: MPPI Ackermann controller and local costmap used by the full and hybrid launches. The costmap uses `/cloud`, `map` as the global frame, and `nav_base_link` as the robot frame.
  * `obstacle_layer.raw_cloud.min_obstacle_height: 0.45`: Nav2 applies this after transforming points into `map`. It excludes the current campus road at Z=0.30m and its 0.08m speed breakers, while retaining dog and pedestrian returns.
  * `inflation_layer.inflation_radius: 1.2`: Exceeds the padded footprint's approximately 0.81m inscribed radius and supplies a clearance gradient.
  * `FollowPath.ObstaclesCritic.consider_footprint: true`: Uses the configured rectangular footprint for MPPI collision checks.
  * The height threshold is specific to this simulation's map elevation; see the [validation record](test/OBSTACLE_AVOIDANCE_VALIDATION.md).
* `config/fast_lio_mid360.yaml`: FAST-LIO sensor and mapping parameters.
* `config/campus_junctions_and_stops.yaml`: Defines coordinates, degree, and connections for all campus road junctions and the 32 named stops.
* `data/campus_with_junctions_and_stops.osm`: Preferred OSM map with named stops.
* `data/campus.osm`: Master OpenStreetMap XML vector map containing roads, walkways, and building footprints.
* `rviz/campus_nav.rviz`: Default RViz configuration with clean 3D campus map, RealSense camera feed panel, and 2D LiDAR tactical radar panel.
* `rviz/solo_lidar.rviz`: Dedicated point-cloud-only RViz configuration for raw 3D LiDAR inspection.
* `rviz/livox_mid360.rviz`: LiDAR display used by the full launch.

## Validation

The obstacle correction passed the package build, nine odometry regression cases, and headless Gazebo pedestrian, dog, and speed-breaker trials. On 2026-10-16, the user also confirmed that the supplied full-simulation test procedure ran correctly. See [TESTING.md](TESTING.md) for the commands and [the validation record](test/OBSTACLE_AVOIDANCE_VALIDATION.md) for scope and limitations.
