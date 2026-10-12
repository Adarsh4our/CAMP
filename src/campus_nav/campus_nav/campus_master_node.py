"""
CampusMasterNode — Single Entry-Point Orchestrator

Single external interface and FSM orchestrator for the entire CAMP stack.

Publishes / Listens to:
  - Input: /campus/master_command (std_msgs/String)
      Examples:
        "MISSION:SAB C|Chemistry Block"
        "GOAL:Chemistry Block"
        "ABORT"
  - Safety: /campus/safety_status (std_msgs/String JSON)
      Forces SAFE_STOP if state == 'EMERGENCY_STOP'
  - Output: /campus/nav_status (std_msgs/String)
  - Output: /cmd_vel (std_msgs/Twist) — sends zero twist on emergency stop or abort

Action Clients:
  - campus/plan_route_action   → CampusPlannerNode (PlanRoute)
  - campus/follow_route_action  → Nav2AdapterNode   (FollowRoute)
  - campus/run_mission_action   → MissionControllerNode (RunMission - boarding dwell)

FSM States:
  IDLE → PLANNING → NAVIGATING → BOARDING → PLANNING(leg2) → NAVIGATING → ARRIVED → IDLE
  Any state → SAFE_STOP on LiDAR / Safety E-stop
"""

import asyncio
import json
import time
import threading
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Twist, PoseStamped
from nav2_msgs.action import FollowPath
from nav_msgs.msg import Path

from saye_msgs.action import PlanRoute, FollowRoute, RunMission


class CampusMasterNode(Node):

    # ── FSM States ─────────────────────────────────────────────────────────────
    STATE_IDLE        = 'IDLE'
    STATE_PLANNING    = 'PLANNING'
    STATE_NAV         = 'NAVIGATING'
    STATE_BOARDING    = 'BOARDING'
    STATE_ARRIVED     = 'ARRIVED'
    STATE_SAFESTOP    = 'SAFE_STOP'
    STATE_OBS_AVOID   = 'OBSTACLE_AVOIDANCE'   # Nav2 MPPI takes over

    def __init__(self):
        super().__init__('campus_master_node')
        self.cb = ReentrantCallbackGroup()

        # FSM tracking
        self._state = self.STATE_IDLE
        self._mission_start: str = ''
        self._mission_goal: str = ''
        self._mission_start_time: float = 0.0
        self._active_task = None
        self._lock = threading.Lock()

        # ── Action Clients ─────────────────────────────────────────────────────
        self._plan_client = ActionClient(
            self, PlanRoute, 'campus/plan_route_action', callback_group=self.cb)
        self._follow_client = ActionClient(
            self, FollowRoute, 'campus/follow_route_action', callback_group=self.cb)
        self._board_client = ActionClient(
            self, RunMission, 'campus/run_mission_action', callback_group=self.cb)

        # ── External Operator Command Subscriber ───────────────────────────────
        self.cmd_sub = self.create_subscription(
            String, '/campus/master_command', self._command_callback, 10,
            callback_group=self.cb)

        # ── High Priority Safety Interrupt Subscriber ──────────────────────────
        self.safety_sub = self.create_subscription(
            String, '/campus/safety_status', self._safety_callback, 10,
            callback_group=self.cb)

        # ── Telemetry and Control Publishers ───────────────────────────────────
        self._status_pub = self.create_publisher(String, '/campus/nav_status', 10)
        self._cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        # Pause/resume path_follower_node when switching to Nav2 obstacle avoidance
        self._follower_enable_pub = self.create_publisher(Bool, '/campus/follower_enable', 1)
        self._avoidance_path_pub = self.create_publisher(Path, '/campus/avoidance_path', 10)

        # ── Nav2 Action Client (MPPI obstacle avoidance mode) ──────────────────
        self._nav2_client = ActionClient(
            self, FollowPath, 'follow_path', callback_group=self.cb)
        self._current_path: Path = None

        # ── Vehicle Pose Subscriber (for Nav2 goal projection) ─────────────────
        self._vehicle_pose: PoseStamped = None
        self.create_subscription(
            PoseStamped, '/campus/vehicle_pose', self._pose_callback, 10,
            callback_group=self.cb)

        # ── Obstacle Avoidance Config ───────────────────────────────────────────
        # Distance (m) at which LiDAR obstacle triggers switch to Nav2 MPPI mode
        self.OBSTACLE_TRIGGER_DIST = 8.5
        self._last_obstacle_dist = 999.0
        self._nav2_goal_handle = None   # track active Nav2 goal for cancellation
        self._last_avoidance_trigger = 0.0  # timestamp of last Nav2 avoidance dispatch


        # ── Pure Pursuit Status Subscriber (Normal Mode Following) ─────────────
        self._abort_requested: bool = False
        self._pursuit_arrived: bool = False
        self.create_subscription(
            String, '/campus/pursuit_status', self._pursuit_status_callback, 10,
            callback_group=self.cb)

        self.get_logger().info(
            "CampusMasterNode online. Sole external sink listening on /campus/master_command.")

    # ── External Command Handler ───────────────────────────────────────────────
    def _command_callback(self, msg: String):
        raw = msg.data.strip()
        self.get_logger().info(f"[Master Command] Received: '{raw}'")

        if raw.upper() == 'ABORT':
            self._abort("Operator ABORT command received")
            return

        if raw.startswith('MISSION:'):
            parts = raw[len('MISSION:'):].split('|', 1)
            if len(parts) == 2:
                start, goal = parts[0].strip(), parts[1].strip()
                self._dispatch_mission_thread(start, goal)
            else:
                self.get_logger().error(f"Malformed MISSION command: '{raw}'. Expected 'MISSION:Start|Goal'")
            return

        if raw.startswith('GOAL:'):
            goal = raw[len('GOAL:'):].strip()
            self._dispatch_mission_thread('CURRENT', goal)
            return

        self.get_logger().warn(f"Unrecognized master command: '{raw}'")

    # ── Safety Interrupt Callback ──────────────────────────────────────────────
    def _safety_callback(self, msg: String):
        try:
            payload = json.loads(msg.data)
        except Exception:
            return

        safety_state = payload.get('state', 'CLEAR')
        obstacle_dist = float(payload.get('obstacle_dist', 999.0))
        obstacle_type = payload.get('obstacle_type', 'NONE')
        self._last_obstacle_dist = obstacle_dist

        with self._lock:
            current_state = self._state

        # ── Hard E-stop: cliff drop-off into void ──────────────────────────────
        if safety_state == 'VOID_AHEAD':
            self.get_logger().error(f"🚨 [Master Safety] {safety_state}! Forcing SAFE_STOP.")
            self._cancel_nav2_avoidance()
            self._set_state(self.STATE_SAFESTOP)
            self._stop_vehicle()
            self._publish_status("EMERGENCY SAFE STOP: Cliff/Void ahead!")
            return

        # ── Dynamic Obstacle — switch to Nav2 MPPI ─────────────────────────────
        # Only switch if bypassable (not a speed bump), has swerve clearance, and cooldown passed
        if (safety_state in ('SLOWDOWN_APPROACH', 'EMERGENCY_STOP')
                and 2.0 <= obstacle_dist <= self.OBSTACLE_TRIGGER_DIST
                and obstacle_type != 'SPEED_BREAKER'
                and current_state == self.STATE_NAV):
            now_t = time.time()
            if now_t - self._last_avoidance_trigger > 3.0:  # 3s cooldown between triggers
                self._last_avoidance_trigger = now_t
                self.get_logger().warn(
                    f"⚠️ Obstacle at {obstacle_dist:.1f}m — switching to Nav2 MPPI OBSTACLE_AVOIDANCE.")
                self._enter_obstacle_avoidance()
            return

        # ── Imminent collision (< 0.8m) even while avoiding ───────────────────
        if safety_state == 'EMERGENCY_STOP' and obstacle_dist < 0.8:
            if current_state != self.STATE_SAFESTOP:
                self.get_logger().error("🚨 Imminent physical collision (<0.8m)! Forcing SAFE_STOP.")
                self._cancel_nav2_avoidance()
                self._set_state(self.STATE_SAFESTOP)
                self._stop_vehicle()
                self._publish_status("EMERGENCY SAFE STOP: Imminent collision (<0.8m)")
            return

        # ── Hazard cleared while in SAFE_STOP ──────────────────────────────────
        if safety_state == 'CLEAR':
            if current_state == self.STATE_SAFESTOP:
                self.get_logger().info("🛡️ [Master Safety] Hazard cleared. Reverting to IDLE.")
                self._set_state(self.STATE_IDLE)
                self._publish_status("Safety cleared. Master in IDLE, ready for next command.")

    def _pose_callback(self, msg: PoseStamped):
        """Track vehicle pose for projecting Nav2 avoidance goal ahead."""
        self._vehicle_pose = msg

    def _enter_obstacle_avoidance(self):
        """Pause Pure Pursuit, construct lateral detour path, send Nav2 MPPI to steer around obstacle."""
        import math
        self._set_state(self.STATE_OBS_AVOID)
        # Pause Pure Pursuit immediately so Nav2 has exclusive /cmd_vel
        self._follower_enable_pub.publish(Bool(data=False))
        self._publish_status("OBSTACLE_AVOIDANCE: Nav2 MPPI active — routing around obstacle.")

        if self._vehicle_pose is None:
            self.get_logger().warn("No vehicle pose available — cannot pass path to Nav2.")
            self._resume_normal_driving()
            return

        from campus_nav.coord_bridge import yaw_from_quaternion, quaternion_from_yaw
        yaw = yaw_from_quaternion(self._vehicle_pose.pose.orientation)
        nav_x = self._vehicle_pose.pose.position.x
        nav_y = self._vehicle_pose.pose.position.y

        local_path = Path()
        local_path.header.frame_id = 'map'
        local_path.header.stamp = self.get_clock().now().to_msg()

        # Start waypoint at current vehicle pose
        curr_p = PoseStamped()
        curr_p.header = local_path.header
        curr_p.pose = self._vehicle_pose.pose
        local_path.poses.append(curr_p)

        # Build candidate base waypoints
        raw_pts = []
        if getattr(self, '_current_path', None) and self._current_path.poses:
            vx = nav_x
            vy = nav_y
            closest_idx = min(range(len(self._current_path.poses)),
                              key=lambda i: math.hypot(self._current_path.poses[i].pose.position.x - vx,
                                                      self._current_path.poses[i].pose.position.y - vy))
            # Slice forward waypoints (~25m ahead) past the obstacle
            end_idx = min(len(self._current_path.poses), closest_idx + 45)
            for i in range(closest_idx, end_idx):
                p = self._current_path.poses[i].pose.position
                raw_pts.append((p.x, p.y, p.z))
        else:
            # Fallback: project waypoints 25m ahead along current heading
            for step in range(1, 45):
                dist = step * 0.5
                rx = nav_x + dist * math.cos(yaw)
                ry = nav_y + dist * math.sin(yaw)
                raw_pts.append((rx, ry, self._vehicle_pose.pose.position.z))

        # Generate smooth lateral detour shifting +1.35m to the left (open lane)
        # S-curve smoothstep profile avoids sharp turns for Ackermann kinematics
        L_max = 1.35
        d_obs = max(4.0, min(8.5, self._last_obstacle_dist))
        s1 = 3.5             # ramp up lateral shift
        s2 = d_obs + 4.0     # hold lateral shift past obstacle
        s3 = s2 + 5.0        # smooth return to centerline

        detour_coords = []
        s_accum = 0.0
        last_x, last_y = nav_x, nav_y

        for i, (px, py, pz) in enumerate(raw_pts):
            ds = math.hypot(px - last_x, py - last_y)
            s_accum += ds
            last_x, last_y = px, py

            # Segment tangent and left normal
            if i + 1 < len(raw_pts):
                tx = raw_pts[i + 1][0] - px
                ty = raw_pts[i + 1][1] - py
                t_len = math.hypot(tx, ty)
                if t_len > 1e-3:
                    tx /= t_len
                    ty /= t_len
                else:
                    tx, ty = math.cos(yaw), math.sin(yaw)
            else:
                tx, ty = math.cos(yaw), math.sin(yaw)
            nx, ny = -ty, tx  # 90 deg counter-clockwise = left flank

            # Smoothstep lateral offset
            if s_accum <= s1:
                u = max(0.0, s_accum / s1)
                w = 3.0 * (u ** 2) - 2.0 * (u ** 3)
                offset = L_max * w
            elif s_accum <= s2:
                offset = L_max
            elif s_accum <= s3:
                u = max(0.0, min(1.0, (s_accum - s2) / (s3 - s2)))
                w = 1.0 - (3.0 * (u ** 2) - 2.0 * (u ** 3))
                offset = L_max * w
            else:
                offset = 0.0

            detour_x = px + offset * nx
            detour_y = py + offset * ny
            detour_coords.append((detour_x, detour_y, pz))

        # Build PoseStamped list with aligned yaw orientations
        for j, (dx, dy, dz) in enumerate(detour_coords):
            if j + 1 < len(detour_coords):
                seg_yaw = math.atan2(detour_coords[j + 1][1] - dy, detour_coords[j + 1][0] - dx)
            elif j > 0:
                seg_yaw = math.atan2(dy - detour_coords[j - 1][1], dx - detour_coords[j - 1][0])
            else:
                seg_yaw = yaw

            qx, qy, qz, qw = quaternion_from_yaw(seg_yaw)
            p = PoseStamped()
            p.header = local_path.header
            p.pose.position.x = dx
            p.pose.position.y = dy
            p.pose.position.z = dz
            p.pose.orientation.x = qx
            p.pose.orientation.y = qy
            p.pose.orientation.z = qz
            p.pose.orientation.w = qw
            local_path.poses.append(p)

        # Publish avoidance path for RViz visualization
        self._avoidance_path_pub.publish(local_path)
        self.get_logger().info(
            f"🚀 Dispatched {len(local_path.poses)}-waypoint lateral detour (+{L_max}m left) to Nav2 MPPI.")

        goal_msg = FollowPath.Goal()
        goal_msg.path = local_path
        goal_msg.controller_id = 'FollowPath'
        goal_msg.goal_checker_id = 'goal_checker'

        # Dispatch in a background thread so wait_for_server doesn't block the ROS executor
        def _send():
            if not self._nav2_client.wait_for_server(timeout_sec=3.0):
                self.get_logger().warn("Nav2 controller_server not available — resuming Pure Pursuit.")
                self._resume_normal_driving()
                return
            self._nav2_client.send_goal_async(
                goal_msg,
                feedback_callback=None
            ).add_done_callback(self._nav2_goal_response_callback)

        threading.Thread(target=_send, daemon=True).start()



    def _nav2_goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle or not goal_handle.accepted:
            self.get_logger().warn("Nav2 rejected avoidance path — resuming Pure Pursuit.")
            self._resume_normal_driving()
            return
        self._nav2_goal_handle = goal_handle
        goal_handle.get_result_async().add_done_callback(self._nav2_result_callback)

    def _nav2_result_callback(self, future):
        """Called when Nav2 finishes or aborts the avoidance maneuver."""
        try:
            status = future.result().status
        except Exception:
            status = -1

        # action_msgs.msg.GoalStatus: STATUS_SUCCEEDED = 4, STATUS_ABORTED = 6, STATUS_CANCELED = 5
        if status == 4:
            self.get_logger().info("✅ Nav2 avoidance complete — resuming Pure Pursuit.")
        else:
            self.get_logger().warn(
                f"⚠️ Nav2 avoidance ended with status {status} — resuming Pure Pursuit.")
        # Always hand back to Pure Pursuit; do NOT SAFE_STOP here.
        # path_follower will itself stop for any imminent collisions via safety_monitor.
        self._resume_normal_driving()

    def _cancel_nav2_avoidance(self):
        if self._nav2_goal_handle is not None:
            try:
                self._nav2_goal_handle.cancel_goal_async()
            except Exception:
                pass
            self._nav2_goal_handle = None

    def _resume_normal_driving(self):
        """Hand control back to path_follower_node (Pure Pursuit)."""
        self._cancel_nav2_avoidance()
        if self._state == self.STATE_OBS_AVOID:
            self._set_state(self.STATE_NAV)
        self._follower_enable_pub.publish(Bool(data=True))
        self._publish_status("Resumed NORMAL_DRIVING — camera/LiDAR mode active.")



    # ── State Management ───────────────────────────────────────────────────────
    def _set_state(self, new_state: str):
        with self._lock:
            old_state = self._state
            self._state = new_state
        self.get_logger().info(f"[FSM] {old_state} ➔ {new_state}")
        self._publish_status(f"Master FSM: {new_state}")

    def _stop_vehicle(self):
        """Immediately publish zero velocities to /cmd_vel."""
        zero_twist = Twist()
        self._cmd_pub.publish(zero_twist)

    def _pursuit_status_callback(self, msg: String):
        text = msg.data.strip()
        if "ARRIVED" in text.upper():
            self._pursuit_arrived = True

    def _abort(self, reason: str):
        self.get_logger().warn(f"[Master] Aborting mission: {reason}")
        with self._lock:
            self._abort_requested = True
        self._stop_vehicle()
        self._set_state(self.STATE_IDLE)
        self._publish_status(f"Mission aborted: {reason}")

    # ── Mission Execution Runner (Worker Thread) ──────────────────────────────
    def _dispatch_mission_thread(self, start: str, goal: str):
        with self._lock:
            if self._state not in (self.STATE_IDLE, self.STATE_ARRIVED):
                self.get_logger().warn(
                    f"Ignored mission request ('{start}' ➔ '{goal}'). Busy in state '{self._state}'")
                return
            self._abort_requested = False

        worker = threading.Thread(
            target=self._run_mission_sequence,
            args=(start, goal),
            daemon=True
        )
        worker.start()

    def _run_mission_sequence(self, start: str, goal: str):
        self._mission_start = start
        self._mission_goal = goal
        self._mission_start_time = time.time()
        self.get_logger().info(f"▶️ Starting Mission: '{start}' ➔ '{goal}'")

        is_direct_goal = start.upper() in ('CURRENT', 'CURRENT_LOCATION', 'LIVE', '')

        # ── Leg 1 (Pick-up stop) if needed ─────────────────────────────────────
        if not is_direct_goal:
            self._set_state(self.STATE_PLANNING)
            self._publish_status(f"Leg 1: Planning route from Current Pose to '{start}'...")
            path = self._call_plan_action(f"CURRENT|{start}")
            if not path or not path.poses:
                self._publish_status(f"❌ Failed to plan route to pick-up '{start}'")
                self._set_state(self.STATE_IDLE)
                return

            if self._is_stopped():
                return

            self._set_state(self.STATE_NAV)
            self._publish_status(f"Leg 1: Navigating to pick-up stop '{start}'...")
            arrived = self._call_follow_action(path, destination_label=start)
            if not arrived:
                self._publish_status(f"❌ Could not reach pick-up stop '{start}'")
                self._set_state(self.STATE_IDLE)
                return

            if self._is_stopped():
                return

            # ── Boarding Phase ────────────────────────────────────────────────────
            self._set_state(self.STATE_BOARDING)
            self._publish_status(f"Buggy at '{start}'. Boarding passengers (5s dwell)...")
            boarded = self._call_boarding_action(start, goal, dwell_seconds=5.0)
            if not boarded:
                self.get_logger().warn("Boarding phase reported incomplete or canceled.")

            if self._is_stopped():
                return

        # ── Destination Leg ───────────────────────────────────────────────────
        self._set_state(self.STATE_PLANNING)
        self._publish_status(f"Planning route to destination '{goal}'...")
        path = self._call_plan_action(f"CURRENT|{goal}")
        if not path or not path.poses:
            self._publish_status(f"❌ Failed to plan route to final destination '{goal}'")
            self._set_state(self.STATE_IDLE)
            return

        if self._is_stopped():
            return

        self._set_state(self.STATE_NAV)
        self._publish_status(f"Navigating to final destination '{goal}'...")
        arrived = self._call_follow_action(path, destination_label=goal)

        # ── Mission Completion ────────────────────────────────────────────────
        self._set_state(self.STATE_ARRIVED)
        elapsed = time.time() - self._mission_start_time
        m, s = divmod(int(elapsed), 60)
        if arrived:
            completion_msg = f"🏁 Mission Complete ✅ | '{start}' ➔ '{goal}' | {m}m {s}s"
        else:
            completion_msg = f"⚠️ Mission Ended (Navigation incomplete) | '{start}' ➔ '{goal}' | {m}m {s}s"

        self._publish_status(completion_msg)
        self.get_logger().info(completion_msg)

        # Transition cleanly back to IDLE after brief pause
        time.sleep(2.0)
        with self._lock:
            if self._state == self.STATE_ARRIVED:
                self._state = self.STATE_IDLE
        self._publish_status("Master IDLE. Ready for next mission.")

    def _is_stopped(self) -> bool:
        with self._lock:
            return self._abort_requested or self._state == self.STATE_SAFESTOP

    # ── Action Invocations ─────────────────────────────────────────────────────
    def _call_plan_action(self, mission_str: str):
        if not self._plan_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("PlanRoute Action Server (/campus/plan_route_action) not available!")
            return None

        goal = PlanRoute.Goal()
        goal.mission = mission_str
        future = self._plan_client.send_goal_async(goal)

        while not future.done():
            if self._is_stopped():
                return None
            time.sleep(0.05)

        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error(f"PlanRoute goal rejected for '{mission_str}'")
            return None

        result_future = goal_handle.get_result_async()
        while not result_future.done():
            if self._is_stopped():
                goal_handle.cancel_goal_async()
                return None
            time.sleep(0.05)

        action_result = result_future.result()
        if action_result.result.success:
            return action_result.result.path
        else:
            self.get_logger().error(f"PlanRoute failed: {action_result.result.message}")
            return None

    def _call_follow_action(self, path, destination_label: str) -> bool:
        import math
        self._current_path = path
        self._pursuit_arrived = False
        # Enable Pure Pursuit for normal camera + LiDAR driving
        self._follower_enable_pub.publish(Bool(data=True))

        if not path or not path.poses:
            return False

        goal_pose = path.poses[-1]
        gx = goal_pose.pose.position.x
        gy = goal_pose.pose.position.y

        self.get_logger().info(
            f"Tracking route to '{destination_label}' via Camera + LiDAR Pure Pursuit...")

        deadline = time.time() + 300.0  # 5 min max
        last_status_time = 0.0

        while time.time() < deadline:
            if self._is_stopped():
                self._stop_vehicle()
                return False

            if self._pursuit_arrived:
                self.get_logger().info(
                    f"✅ Arrived at '{destination_label}' (path follower reached goal)!")
                return True

            if self._vehicle_pose is not None:
                vx = self._vehicle_pose.pose.position.x
                vy = self._vehicle_pose.pose.position.y
                dist = math.hypot(gx - vx, gy - vy)
                if dist < 2.5:
                    self.get_logger().info(
                        f"✅ Arrived at '{destination_label}' ({dist:.1f}m to goal)!")
                    return True

                now = time.time()
                if now - last_status_time > 2.0:
                    last_status_time = now
                    self._publish_status(
                        f"[Tracking] Heading to '{destination_label}' — {dist:.1f}m remaining")

            time.sleep(0.1)

        self.get_logger().warn(f"Navigation to '{destination_label}' timed out after 300s.")
        return False

    def _follow_feedback_callback(self, feedback_msg):
        fb = feedback_msg.feedback
        self._publish_status(f"[Tracking] {fb.pursuit_status} ({fb.distance_remaining:.1f}m remaining)")

    def _call_boarding_action(self, start: str, goal: str, dwell_seconds: float = 5.0) -> bool:
        if not self._board_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().warn("RunMission Action Server not found, falling back to local sleep...")
            time.sleep(dwell_seconds)
            return True

        goal_msg = RunMission.Goal()
        goal_msg.start_stop = start
        goal_msg.goal_stop = goal
        goal_msg.dwell_seconds = float(dwell_seconds)
        future = self._board_client.send_goal_async(goal_msg)

        while not future.done():
            if self._is_stopped():
                return False
            time.sleep(0.05)

        goal_handle = future.result()
        if not goal_handle.accepted:
            return False

        result_future = goal_handle.get_result_async()
        while not result_future.done():
            if self._is_stopped():
                goal_handle.cancel_goal_async()
                return False
            time.sleep(0.05)

        action_result = result_future.result()
        return action_result.result.success

    # ── Helpers ────────────────────────────────────────────────────────────────
    def _publish_status(self, text: str):
        msg = String()
        msg.data = text
        self._status_pub.publish(msg)
        self.get_logger().info(f"[status] {text}")


def main(args=None):
    rclpy.init(args=args)
    node = CampusMasterNode()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
