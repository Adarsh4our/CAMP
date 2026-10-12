# CAMP Autonomy Stack — Node-by-Node Debugging Guide

This guide enables isolated, unit-style debugging for each node in the campus navigation stack without needing to run the full end-to-end simulation.

Each test starts Gazebo Harmonic with the campus world and the saye buggy, along with only the minimal nodes required to verify that specific subsystem.

---

## 🛠️ General Environment Setup

Before running any test, ensure your shell environment is sourced:

```bash
source /opt/ros/jazzy/setup.bash
source ~/CAMP/install/setup.bash
```

> **Tip:** If Gazebo hangs or complains about an existing server, terminate background processes:
> ```bash
> pkill -9 -f "gz sim" && killall -9 gz-sim-server gz-sim-gui
> ```

---

## 📋 Quick Test Matrix

| Target Node | Launch Command | Primary Trigger / Interaction | Key Topic to Observe | Pass Condition |
|---|---|---|---|---|
| **1. `planner_node`** | `ros2 launch campus_nav debug_planner.launch.py` | Topic publish / RViz 2D Goal Pose / Action Goal | `/campus/global_path` | Green path ribbon appears in RViz; waypoints published |
| **2. `odom_bridge_node`** | `ros2 launch campus_nav debug_odom.launch.py` | Drive buggy using WASD popup window | `/campus/vehicle_pose` | Buggy moves in Gazebo & RViz; pose coordinates update smoothly |
| **3. `safety_monitor_node`** | `ros2 launch campus_nav debug_safety.launch.py` | Spawn test obstacle in front of buggy | `/campus/safety_status` | Status transitions `CLEAR` ➔ `VOID_AHEAD`/`EMERGENCY_STOP` |
| **4. `mission_controller_node`** | `ros2 launch campus_nav debug_mission.launch.py` | Send `RunMission` action goal | `/campus/run_mission_action` | Holds stationary dwell for 5.0s, returns `success: True` |
| **5. `nav2_adapter_node`** | `ros2 launch campus_nav debug_nav2.launch.py` | Plan route & send `FollowRoute` action goal | `/campus/pursuit_status` | Buggy drives along path; distance remaining decreases |
| **6. `campus_master_node`** | `ros2 launch campus_nav debug_master.launch.py` | Publish mission command to master sink | `/campus/nav_status` | FSM cycles: `IDLE ➔ PLANNING ➔ NAV ➔ BOARDING ➔ ARRIVED` |

---

## 1. `planner_node` (A* Path & Trajectory Planner)

Verifies OSM road graph parsing, A* waypoint search, and route publishing.

### Launch
```bash
ros2 launch campus_nav debug_planner.launch.py
```
*(Starts Gazebo, robot_state_publisher, ros_gz_bridge, odom_bridge_node, planner_node, and RViz)*

### Trigger the Test (Choose any method)

**Method A: Terminal Topic (Easiest)**
```bash
ros2 topic pub --once /campus/plan_route std_msgs/msg/String "{data: 'SAB C|Chemistry Block'}"
```

**Method B: RViz 2D Goal Pose**
Click the **2D Goal Pose** button in RViz header, then click any destination road on the campus map.

**Method C: PlanRoute Action Server**
```bash
ros2 action send_goal /campus/plan_route_action saye_msgs/action/PlanRoute "{mission: 'SAB C|Chemistry Block'}"
```

### Inspect Topics
```bash
# Verify path header
ros2 topic echo /campus/global_path --field header

# View path coordinate count
ros2 topic echo /campus/global_path --field poses
```

### Pass Criteria
1. Terminal prints route computation logs with estimated distance.
2. A green trajectory ribbon appears in RViz along campus roadways.
3. `/campus/global_path` publishes non-empty `nav_msgs/Path`.

---

## 2. `odom_bridge_node` (Odometry & TF Bridge)

Verifies coordinate transform from Gazebo world coordinates to planner frame, plus `map -> nav_base_link` TF broadcast (-90° heading alignment).

### Launch
```bash
ros2 launch campus_nav debug_odom.launch.py
```
*(Starts Gazebo, robot_state_publisher, ros_gz_bridge, odom_bridge_node, RViz, and opens a WASD Teleop GUI window)*

### Trigger the Test
1. Click into the **WASD Teleop** GUI window to gain keyboard focus.
2. Hold **W** to accelerate, use **A** and **D** to steer.
3. Observe the buggy moving in Gazebo.

### Inspect Topics & TF
```bash
# Terminal 2: Stream live vehicle pose in planner coordinates
ros2 topic echo /campus/vehicle_pose

# Terminal 3: Verify the TF broadcast from map to nav_base_link
ros2 run tf2_ros tf2_echo map nav_base_link
```

### Pass Criteria
1. As the buggy drives forward in Gazebo, coordinates on `/campus/vehicle_pose` update smoothly.
2. In RViz, the robot 3D model and `nav_base_link` frame track the buggy's position and orientation in real-time.
3. `tf2_echo map nav_base_link` reports continuous transforms at ~20-50 Hz without timeout warnings.

---

## 3. `safety_monitor_node` (LiDAR Obstacle & Corridor Watchdog)

Verifies point cloud corridor filtering, threat detection, and emergency stop signaling.

### Launch
```bash
ros2 launch campus_nav debug_safety.launch.py
```
*(Starts Gazebo, ros_gz_bridge LiDAR topic, odom_bridge_node, safety_monitor_node, and Livox Mid-360 RViz display)*

### Trigger the Test

**Step 1: Inspect baseline status (Clear)**
```bash
ros2 topic echo /campus/safety_status
```
*Expected: `{"state": "CLEAR", "min_distance_m": 99.0, ...}`*

**Step 2: Inject a pedestrian obstacle 5m ahead**
```bash
ros2 run campus_nav test_obstacle_spawner --type human --distance 5.0
```

**Step 3: Clear the obstacle**
```bash
ros2 run campus_nav test_obstacle_spawner --remove
```

### Pass Criteria
1. When the human is spawned, `/campus/safety_status` changes state to `"VOID_AHEAD"` or `"EMERGENCY_STOP"` with `"obstacle_type": "PEDESTRIAN"`.
2. In RViz, red threat markers appear on `/campus/safety/threats`.
3. When removed via `--remove`, safety status immediately recovers to `"CLEAR"`.

---

## 4. `mission_controller_node` (Boarding Dwell & Stop Management)

Verifies stop arrival verification, passenger boarding dwell timing, and cancellation handling.

### Launch
```bash
ros2 launch campus_nav debug_mission.launch.py
```
*(Starts Gazebo, ros_gz_bridge, odom_bridge_node, and mission_controller_node)*

### Trigger the Test

**Method A: Call RunMission Action Server**
```bash
ros2 action send_goal /campus/run_mission_action saye_msgs/action/RunMission \
  "{start_stop: 'SAB C', goal_stop: 'Chemistry Block', dwell_seconds: 5.0}"
```

**Method B: Test Action Cancellation**
Send a longer dwell and press `Ctrl+C`:
```bash
ros2 action send_goal /campus/run_mission_action saye_msgs/action/RunMission \
  "{start_stop: 'SAB C', goal_stop: 'Chemistry Block', dwell_seconds: 15.0}"
# Press Ctrl+C after 2 seconds
```
*(Note: If the terminal prints `Executor is already spinning` on Ctrl+C, this is a standard harmless ROS 2 CLI signal notice; the node cancels cleanly).*

### Inspect Topics
```bash
ros2 topic echo /campus/nav_status
```

### Pass Criteria
1. Action accepts goal and logs `[Boarding] Holding dwell for 5.0s at 'SAB C'...`.
2. After 5 seconds, action completes with:
   - `success: True`
   - `message: "Boarding dwell completed successfully at SAB C."`
   - `elapsed_seconds: ~5.0`
3. In cancellation test, the node logs `Boarding canceled by master!` and returns `success: False` cleanly.

---

## 5. `nav2_adapter_node` (Nav2 Regulated Pure Pursuit Controller)

Verifies bridge from campus routes to Nav2 controller server, lifecycle management, and path pursuit execution.

### Launch
```bash
ros2 launch campus_nav debug_nav2.launch.py
```
*(Starts Gazebo, odom_bridge, planner_node, nav2_adapter_node, controller_server, lifecycle_manager, and RViz)*

### Trigger the Test

**Step 1: Verify Nav2 Controller Server is Active**
```bash
ros2 lifecycle get /controller_server
# Expected output: active [3]
```

**Step 2: Command Nav2 Adapter to pursue route**
You can directly command a destination label (the adapter will auto-plan the route from the planner if path is omitted):
```bash
ros2 action send_goal /campus/follow_route_action saye_msgs/action/FollowRoute \
  "{destination_label: 'Chemistry Block'}"
```

*(Alternatively, publish to `/campus/plan_route` first, or pass a full `nav_msgs/Path`).*

### Inspect Pursuit Feedback
```bash
ros2 topic echo /campus/pursuit_status
```

### Pass Criteria
1. `/controller_server` reports `active [3]`.
2. `nav2_adapter_node` receives path and forwards to Nav2 `follow_path` action.
3. Buggy accelerates and steers along the road in Gazebo toward the destination.
4. `/campus/pursuit_status` streams decreasing `distance_remaining`.
5. Upon reaching tolerance, action returns `success: True`.

---

## 6. `campus_master_node` (Master FSM Orchestrator)

Verifies high-level mission state machine, action sequencing (`PlanRoute ➔ FollowRoute ➔ RunMission`), and emergency stop preemption.

### Launch
```bash
ros2 launch campus_nav debug_master.launch.py
```
*(Starts the entire integrated stack with campus_master_node orchestrating)*

### Trigger the Test

**Test A: Full 2-Leg Autonomous Mission**
```bash
ros2 topic pub --once /campus/master_command std_msgs/msg/String \
  "{data: 'MISSION:SAB C|Chemistry Block'}"
```

**Test B: Single-Leg Mission from Live Position**
```bash
ros2 topic pub --once /campus/master_command std_msgs/msg/String \
  "{data: 'GOAL:Chemistry Block'}"
```

**Test C: Live Emergency Hazard Preemption**
While the buggy is navigating, inject an obstacle:
```bash
ros2 run campus_nav test_obstacle_spawner --type human --distance 5.0
```
*Observe: Buggy halts instantly with zero velocity (`SAFE_STOP`).*

Clear the obstacle:
```bash
ros2 run campus_nav test_obstacle_spawner --remove
```
*Observe: Master node recovers safely back to `IDLE`.*

**Test D: Operator Abort**
```bash
ros2 topic pub --once /campus/master_command std_msgs/msg/String "{data: 'ABORT'}"
```

### Inspect FSM Telemetry
```bash
ros2 topic echo /campus/nav_status
```

### Pass Criteria
1. Master FSM transitions sequentially:
   `IDLE` ➔ `PLANNING` ➔ `NAVIGATING` ➔ `BOARDING` (5s dwell) ➔ `PLANNING` ➔ `NAVIGATING` ➔ `ARRIVED` ➔ `IDLE`.
2. Emergency hazard triggers immediate zero-velocity halt (`SAFE_STOP`).
3. Mission completion message logged with total trip duration.
