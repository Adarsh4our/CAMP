# 🧭 CAMP ROS 2 Node Registry & Interface Reference

This document provides a comprehensive inventory of all ROS 2 nodes in the **CAMP (Campus Autonomous Mobility Project)** stack, including the exact **Inputs (Subscribed Topics, Action Servers, Services)** and **Outputs (Published Topics, Action Clients, TF Broadcasts)** for every node.

---

## 🏗️ System Architecture

```mermaid
flowchart TD
    subgraph Operator["Operator Interface"]
        CLI["Operator CLI / UI<br/>(/campus/master_command)"]
    end

    subgraph MasterLayer["Orchestration Layer"]
        Master["CampusMasterNode<br/>(FSM Orchestrator)"]
        Audit["AuditLoggerNode<br/>(Flight Recorder)"]
    end

    subgraph PlanningLayer["Planning & Routing"]
        Planner["CampusPlannerNode<br/>(OSM Vector Graph & A*)"]
        Runner["MissionControllerNode<br/>(Boarding Dwell Runner)"]
    end

    subgraph Nav2Layer["Nav2 & Control Layer"]
        Adapter["Nav2AdapterNode<br/>(Action Bridge)"]
        Nav2Ctrl["controller_server<br/>(Regulated Pure Pursuit)"]
        Costmap["local_costmap<br/>(2D Obstacle Grid)"]
        Nav2Life["lifecycle_manager_navigation<br/>(Lifecycle Manager)"]
    end

    subgraph SafetyAndOdom["Safety & Transform Infrastructure"]
        Safety["SafetyMonitorNode<br/>(3D LiDAR & Void Guard)"]
        OdomBridge["OdomBridgeNode<br/>(Odometry & TF: nav_base_link)"]
        Lane["LaneDetectorNode<br/>(Camera IPM Vision)"]
    end

    subgraph SimLayer["Gazebo Harmonic Simulation"]
        Bridge["ros_gz_bridge<br/>(/clock, /cmd_vel, /odom, /cloud, /real_sense)"]
        SDF["saye Robot Model<br/>(Ackermann Dynamics)"]
    end

    CLI -->|Command| Master
    Safety -->|E-Stop Interrupt| Master
    Master -->|PlanRoute Action| Planner
    Master -->|RunMission Action| Runner
    Master -->|FollowRoute Action| Adapter
    Adapter -->|FollowPath Action| Nav2Ctrl
    Costmap --> Nav2Ctrl
    OdomBridge -.->|TF: nav_base_link| Costmap
    Nav2Ctrl -->|/cmd_vel| Bridge --> SDF
    Bridge -->|/cloud (LiDAR)| Costmap
    Bridge -->|/cloud| Safety
    Bridge -->|/model/saye/odometry_world| OdomBridge
    Master -.->|Telemetry| Audit
```

---

## 1. Active Autonomy & Orchestration Nodes (`campus_nav`)

### 1.1 `CampusMasterNode` (`campus_master_node`)
* **Source**: [`src/campus_nav/campus_nav/campus_master_node.py`](file:///home/adarsh4our/CAMP/src/campus_nav/campus_nav/campus_master_node.py)
* **Role**: Deterministic Finite State Machine (`IDLE` ➔ `PLANNING` ➔ `NAVIGATING` ➔ `BOARDING` ➔ `ARRIVED`) and sole operator interface.

| Category | Channel / Topic | Message Type | Description |
| :--- | :--- | :--- | :--- |
| **Input (Topic)** | `/campus/master_command` | `std_msgs/msg/String` | Operator commands (`MISSION:Start\|Goal`, `GOAL:Stop`, `ABORT`). |
| **Input (Topic)** | `/campus/safety_status` | `std_msgs/msg/String` (JSON) | High-priority safety feed; pre-empts FSM to `SAFE_STOP` on `EMERGENCY_STOP`. |
| **Output (Topic)** | `/campus/nav_status` | `std_msgs/msg/String` | Human-readable system state and mission progress. |
| **Output (Topic)** | `/cmd_vel` | `geometry_msgs/msg/Twist` | Emergency zero-velocity override on abort or hazard preemption. |
| **Output (Action Client)** | `/campus/plan_route_action` | `saye_msgs/action/PlanRoute` | Calls planner to compute trajectory between stops. |
| **Output (Action Client)** | `/campus/run_mission_action` | `saye_msgs/action/RunMission` | Calls mission runner for non-blocking boarding dwell delay. |
| **Output (Action Client)** | `/campus/follow_route_action`| `saye_msgs/action/FollowRoute` | Sends planned path to Nav2 adapter for execution. |

---

### 1.2 `CampusPlannerNode` (`planner_node`)
* **Source**: [`src/campus_nav/campus_nav/planner_node.py`](file:///home/adarsh4our/CAMP/src/campus_nav/campus_nav/planner_node.py)
* **Role**: Parses OpenStreetMap vector graph (`campus.osm`), calculates A* topological route, and generates smooth road ribbons.

| Category | Channel / Topic | Message Type | Description |
| :--- | :--- | :--- | :--- |
| **Input (Action Server)**| `/campus/plan_route_action` | `saye_msgs/action/PlanRoute` | Accepts `"Start\|Goal"` requests and returns smooth `nav_msgs/Path`. |
| **Input (Topic)** | `/campus/vehicle_pose` | `geometry_msgs/msg/PoseStamped` | Live vehicle pose in planner coordinate system. |
| **Input (Topic)** | `/odom` | `nav_msgs/msg/Odometry` | Wheel odometry for checkpoint progress tracking. |
| **Input (Topic - Legacy)**| `/goal_pose` | `geometry_msgs/msg/PoseStamped` | RViz 2D Goal Pose tool click for ad-hoc goal selection. |
| **Input (Topic - Legacy)**| `/campus/plan_route` | `std_msgs/msg/String` | Topic-based route plan trigger. |
| **Output (Topic)** | `/campus/global_path` | `nav_msgs/msg/Path` | Discrete planned waypoints in planner frame. |
| **Output (Topic)** | `/plan` | `nav_msgs/msg/Path` | Nav2 compatible global plan path. |
| **Output (Topic)** | `/campus/speed_profile` | `std_msgs/msg/Float32MultiArray`| Curvature-adapted maximum speed limits along waypoints. |
| **Output (Topic)** | `/campus/route_ribbon` | `visualization_msgs/msg/Marker` | 3.8m wide luminous electric-green road carpet for RViz. |
| **Output (Topic)** | `/campus/route_centerline`| `visualization_msgs/msg/Marker` | Pure white center dividing lane stripe marker. |
| **Output (Topic)** | `/campus/road_graph` | `visualization_msgs/msg/Marker` | Complete campus asphalt road network line list. |
| **Output (Topic)** | `/campus/stop_markers` | `visualization_msgs/msg/MarkerArray`| 32 cyan cylinder stops with text labels. |
| **Output (Topic)** | `/campus/junction_markers`| `visualization_msgs/msg/MarkerArray`| 92 orange junction sphere markers. |

---

### 1.3 `Nav2AdapterNode` (`nav2_adapter_node`)
* **Source**: [`src/campus_nav/campus_nav/nav2_adapter_node.py`](file:///home/adarsh4our/CAMP/src/campus_nav/campus_nav/nav2_adapter_node.py)
* **Role**: Translates Master's high-level route requests into native Nav2 `FollowPath` action invocations.

| Category | Channel / Topic | Message Type | Description |
| :--- | :--- | :--- | :--- |
| **Input (Action Server)**| `/campus/follow_route_action` | `saye_msgs/action/FollowRoute`| Receives target `Path` from `CampusMasterNode`. |
| **Input (Topic - Legacy)**| `/campus/nav2_target_path` | `nav_msgs/msg/Path` | Topic fallback for path injection. |
| **Output (Action Client)**| `/follow_path` | `nav2_msgs/action/FollowPath` | Native Nav2 action client to `controller_server`. |
| **Output (Topic)** | `/campus/pursuit_status` | `std_msgs/msg/String` | Pursuit status and remaining distance (`"ARRIVED"`, `"FAILED"`). |

---

### 1.4 `MissionControllerNode` (`mission_controller_node`)
* **Source**: [`src/campus_nav/campus_nav/mission_controller_node.py`](file:///home/adarsh4our/CAMP/src/campus_nav/campus_nav/mission_controller_node.py)
* **Role**: Handles passenger boarding dwell countdowns and Gazebo teleportation.

| Category | Channel / Topic | Message Type | Description |
| :--- | :--- | :--- | :--- |
| **Input (Action Server)**| `/campus/run_mission_action` | `saye_msgs/action/RunMission` | Receives dwell duration (default 5.0s) and runs non-blocking timer. |
| **Input (Topic)** | `/campus/vehicle_pose` | `geometry_msgs/msg/PoseStamped` | Live position for stop proximity validation. |
| **Output (Topic)** | `/campus/nav_status` | `std_msgs/msg/String` | Boarding countdown and leg readiness updates. |
| **Output (Service Client)**| `/world/<world>/set_pose` | `gz.msgs.Pose` (GZ CLI) | Teleports buggy to any campus stop via Gazebo service. |

---

### 1.5 `SafetyMonitorNode` (`safety_monitor_node`)
* **Source**: [`src/campus_nav/campus_nav/safety_monitor_node.py`](file:///home/adarsh4our/CAMP/src/campus_nav/campus_nav/safety_monitor_node.py)
* **Role**: Point cloud elevation filtering, speed breaker identification, and flank cliff guard.

| Category | Channel / Topic | Message Type | Description |
| :--- | :--- | :--- | :--- |
| **Input (Topic)** | `/cloud` | `sensor_msgs/msg/PointCloud2` | Roof-mounted Livox Mid-360 3D LiDAR point cloud. |
| **Input (Topic)** | `/real_sense/depth_point` | `sensor_msgs/msg/PointCloud2` | Forward RealSense depth camera point cloud. |
| **Input (Topic)** | `/cmd_vel` | `geometry_msgs/msg/Twist` | Monitors current vehicle commanded speed. |
| **Output (Topic)** | `/campus/safety_status` | `std_msgs/msg/String` (JSON) | Telemetry payload: `state` (`CLEAR`, `SLOWDOWN_BUMP`, `EMERGENCY_STOP`, `VOID_AHEAD`), `speed_limit`, `steering_bias`. |
| **Output (Topic)** | `/campus/safety_corridor` | `visualization_msgs/msg/Marker` | 3D dynamic bounding box (Green/Yellow/Red) showing active safety zone. |
| **Output (Topic)** | `/campus/safety_threats` | `visualization_msgs/msg/MarkerArray`| Red spheres on detected obstacle clusters. |
| **Output (Topic)** | `/campus/lidar_bev_image`| `sensor_msgs/msg/Image` | 2D Top-Down Bird's Eye View radar image with range rings. |

---

### 1.6 `OdomBridgeNode` (`odom_bridge_node`)
* **Source**: [`src/campus_nav/campus_nav/odom_bridge_node.py`](file:///home/adarsh4our/CAMP/src/campus_nav/campus_nav/odom_bridge_node.py)
* **Role**: Coordinate conversion (Gazebo ↔ Planner) and virtual coordinate frame broadcasting.

| Category | Channel / Topic | Message Type | Description |
| :--- | :--- | :--- | :--- |
| **Input (Topic)** | `/model/saye/odometry_world`| `nav_msgs/msg/Odometry` | Ground-truth odometry from Gazebo simulation physics. |
| **Output (TF Broadcast)**| `map` ➔ `world` | `geometry_msgs/msg/TransformStamped` | Static origin offset (X=802.0m, Y=679.7m). |
| **Output (TF Broadcast)**| `map` ➔ `nav_base_link` | `geometry_msgs/msg/TransformStamped` | Virtual frame rotated by -90° (aligns buggy heading to +X). |
| **Output (Topic)** | `/campus/vehicle_pose` | `geometry_msgs/msg/PoseStamped` | Current vehicle pose in planner map frame. |
| **Output (Topic)** | `/odom` | `nav_msgs/msg/Odometry` | Buggy odometry in ROS standard coordinate conventions. |
| **Output (Topic)** | `/campus/vehicle_marker` | `visualization_msgs/msg/MarkerArray`| High-contrast 3D buggy chassis model for RViz. |

---

### 1.7 `LaneDetectorNode` (`lane_detector`)
* **Source**: [`src/campus_nav/campus_nav/lane_detector_node.py`](file:///home/adarsh4our/CAMP/src/campus_nav/campus_nav/lane_detector_node.py)
* **Role**: Camera-based lane tracking and lateral offset calculation.

| Category | Channel / Topic | Message Type | Description |
| :--- | :--- | :--- | :--- |
| **Input (Topic)** | `/real_sense/image_raw` | `sensor_msgs/msg/Image` | Forward-facing RGB camera stream from buggy hood. |
| **Input (Topic)** | `/real_sense/camera_info`| `sensor_msgs/msg/CameraInfo` | Camera intrinsic parameters for inverse perspective mapping. |
| **Input (Topic)** | `/campus/vehicle_pose` | `geometry_msgs/msg/PoseStamped` | Vehicle pose for spatial map projection. |
| **Output (Topic)** | `/campus/vision_debug_image`| `sensor_msgs/msg/Image` | Processed camera feed with detected lane overlay. |
| **Output (Topic)** | `/campus/lane_detector_status`| `std_msgs/msg/String` | Lateral deviation metrics and lane confidence score. |

---

### 1.8 `AuditLoggerNode` (`audit_logger`)
* **Source**: [`src/campus_nav/campus_nav/audit_logger_node.py`](file:///home/adarsh4our/CAMP/src/campus_nav/campus_nav/audit_logger_node.py)
* **Role**: Flight recorder monitoring communication heartbeats and logging mission anomalies.

| Category | Channel / Topic | Message Type | Description |
| :--- | :--- | :--- | :--- |
| **Input (Topic)** | `/campus/safety_status` | `std_msgs/msg/String` | Monitors safety state transitions and hazard triggers. |
| **Input (Topic)** | `/campus/nav_status` | `std_msgs/msg/String` | Monitors master FSM state changes. |
| **Input (Topic)** | `/campus/global_path` | `nav_msgs/msg/Path` | Verifies route planning timestamps. |
| **Input (Topic)** | `/cloud` | `sensor_msgs/msg/PointCloud2` | Watchdog monitoring LiDAR sensor stream frequency. |
| **Output (File / Log)** | `log/audit_flight_record.log`| Local File Write | Persisted structured flight log with timestamped anomalies. |

---

## 2. Nav2 Navigation Stack Components

### 2.1 `controller_server` (`nav2_controller`)
* **Role**: Controls path following using **Regulated Pure Pursuit (RPP)**.

| Category | Channel / Topic | Message Type | Description |
| :--- | :--- | :--- | :--- |
| **Input (Action Server)**| `/follow_path` | `nav2_msgs/action/FollowPath` | Receives trajectory from `Nav2AdapterNode`. |
| **Input (Internal)** | `local_costmap` | `nav2_costmap_2d` | Cost queries checking for obstacle collisions along the lookahead arc. |
| **Input (TF)** | `map` ➔ `nav_base_link` | TF Tree | Transforms vehicle pose into path coordinate frame. |
| **Output (Topic)** | `/cmd_vel` | `geometry_msgs/msg/Twist` | Commanded linear velocity and steering angular velocity to Gazebo. |

---

### 2.2 `local_costmap` (`nav2_costmap_2d`)
* **Role**: Maintains a 10 Hz 15m $\times$ 15m 2D rolling grid around the vehicle.

| Category | Channel / Topic | Message Type | Description |
| :--- | :--- | :--- | :--- |
| **Input (Topic)** | `/cloud` | `sensor_msgs/msg/PointCloud2` | Livox Mid-360 point cloud marking obstacles between 0.1m and 2.2m elevation. |
| **Input (TF)** | `map` ➔ `nav_base_link` | TF Tree | Updates footprint bounds and rolling window origin. |
| **Output (Topic)** | `/local_costmap/costmap` | `nav_msgs/msg/OccupancyGrid` | 2D costmap grid for visualization and path cost checking. |

---

## 3. Simulation & Middleware Bridge Nodes

### 3.1 `ros_gz_bridge` (`parameter_bridge`)
* **Role**: Bridges DDS messages between Gazebo Harmonic simulation and ROS 2.

| Direction | ROS 2 Topic | Gazebo Transport Topic | Message Type |
| :--- | :--- | :--- | :--- |
| Gazebo ➔ ROS 2 | `/clock` | `/clock` | `rosgraph_msgs/msg/Clock` |
| ROS 2 ➔ Gazebo | `/cmd_vel` | `/cmd_vel` | `geometry_msgs/msg/Twist` |
| Gazebo ➔ ROS 2 | `/model/saye/odometry_world`| `/model/saye/odometry` | `nav_msgs/msg/Odometry` |
| Gazebo ➔ ROS 2 | `/cloud` | `/model/saye/lidar/points` | `sensor_msgs/msg/PointCloud2` |
| Gazebo ➔ ROS 2 | `/real_sense/image_raw` | `/model/saye/camera/image` | `sensor_msgs/msg/Image` |
| Gazebo ➔ ROS 2 | `/real_sense/camera_info`| `/model/saye/camera/camera_info`| `sensor_msgs/msg/CameraInfo` |

---

## 4. Diagnostics & Utility Nodes

| Executable | Source File | Inputs | Outputs | Description |
| :--- | :--- | :--- | :--- | :--- |
| **`wasd_teleop`** | [`wasd_teleop.py`](file:///home/adarsh4our/CAMP/src/campus_nav/campus_nav/wasd_teleop.py) | Keyboard keystrokes | `/cmd_vel` (`Twist`) | Manual keyboard teleoperation. |
| **`teleport_buggy`** | [`teleport_buggy.py`](file:///home/adarsh4our/CAMP/src/campus_nav/campus_nav/teleport_buggy.py) | CLI flags (`--stop`, `--rotate`, `--flip`) | Gazebo `/world/set_pose` service | Repositions buggy to any campus stop or snaps parallel to roads. |
| **`test_obstacle_spawner`**| [`test_obstacle_spawner.py`](file:///home/adarsh4our/CAMP/src/campus_nav/campus_nav/test_obstacle_spawner.py)| CLI commands | Gazebo `/world/create` service | Spawns test obstacles on the road to verify Nav2 avoidance. |
