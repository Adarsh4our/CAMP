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
from std_msgs.msg import String
from geometry_msgs.msg import Twist

from saye_msgs.action import PlanRoute, FollowRoute, RunMission


class CampusMasterNode(Node):

    # ── FSM States ─────────────────────────────────────────────────────────────
    STATE_IDLE     = 'IDLE'
    STATE_PLANNING = 'PLANNING'
    STATE_NAV      = 'NAVIGATING'
    STATE_BOARDING = 'BOARDING'
    STATE_ARRIVED  = 'ARRIVED'
    STATE_SAFESTOP = 'SAFE_STOP'

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
        with self._lock:
            current_state = self._state

        if safety_state in ('EMERGENCY_STOP', 'VOID_AHEAD'):
            if current_state != self.STATE_SAFESTOP:
                self.get_logger().error(f"🚨 [Master Safety Interrupt] {safety_state}! Forcing SAFE_STOP.")
                self._set_state(self.STATE_SAFESTOP)
                self._stop_vehicle()
                self._publish_status(f"EMERGENCY SAFE STOP: Hazard detected ({payload.get('obstacle_type', 'UNKNOWN')})")
        elif safety_state == 'CLEAR':
            if current_state == self.STATE_SAFESTOP:
                self.get_logger().info("🛡️ [Master Safety] Hazard cleared. Reverting to IDLE.")
                self._set_state(self.STATE_IDLE)
                self._publish_status("Safety cleared. Master in IDLE, ready for next command.")

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

    def _abort(self, reason: str):
        self.get_logger().warn(f"[Master] Aborting mission: {reason}")
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

        # ── Leg 1 (Pick-up stop) if needed ─────────────────────────────────────
        if start.upper() not in ('CURRENT', 'CURRENT_LOCATION', 'LIVE', ''):
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

        # ── Leg 2 (Destination stop) ──────────────────────────────────────────
        self._set_state(self.STATE_PLANNING)
        self._publish_status(f"Leg 2: Planning route to destination '{goal}'...")
        path = self._call_plan_action(f"CURRENT|{goal}")
        if not path or not path.poses:
            self._publish_status(f"❌ Failed to plan route to final destination '{goal}'")
            self._set_state(self.STATE_IDLE)
            return

        if self._is_stopped():
            return

        self._set_state(self.STATE_NAV)
        self._publish_status(f"Leg 2: Navigating to final destination '{goal}'...")
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
            return self._state in (self.STATE_IDLE, self.STATE_SAFESTOP)

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
        if not self._follow_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("FollowRoute Action Server (/campus/follow_route_action) not available!")
            return False

        goal = FollowRoute.Goal()
        goal.path = path
        goal.destination_label = destination_label
        future = self._follow_client.send_goal_async(
            goal,
            feedback_callback=self._follow_feedback_callback
        )

        while not future.done():
            if self._is_stopped():
                return False
            time.sleep(0.05)

        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error(f"FollowRoute goal rejected for '{destination_label}'")
            return False

        result_future = goal_handle.get_result_async()
        while not result_future.done():
            if self._is_stopped():
                self.get_logger().warn("Canceling active FollowRoute goal due to stop/abort...")
                goal_handle.cancel_goal_async()
                return False
            time.sleep(0.05)

        action_result = result_future.result()
        return action_result.result.success

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
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
