"""
Mission Controller Node (Refactored for Master Orchestration)

Exposes:
  - RunMission Action Server on 'campus/run_mission_action'
    Handles the boarding dwell timer phase and stop validation.
  - Retains Gazebo teleport helper and /campus/nav_status reporting.
  - Backward compatible: If a message is published directly to /campus/mission,
    it redirects or logs a prompt to use /campus/master_command.
"""

import math
import subprocess
import time
import os
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup

from nav_msgs.msg import Path
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from saye_msgs.action import RunMission

from campus_nav.coord_bridge import nav_to_gz, gz_to_nav
from campus_nav.osm_loader import CampusMap

WORLD_NAME = 'campus_world'
MODEL_NAME = 'saye'


class MissionControllerNode(Node):

    def __init__(self):
        super().__init__('mission_controller_node')
        self.cb_group = ReentrantCallbackGroup()

        # Load map to look up stop road node coordinates
        candidates_osm = []
        try:
            from ament_index_python.packages import get_package_share_directory
            pkg = get_package_share_directory('campus_nav')
            candidates_osm.extend([
                os.path.join(pkg, 'data', 'campus_with_junctions_and_stops.osm'),
                os.path.join(pkg, 'data', 'campus.osm')
            ])
        except Exception:
            pass
        curr_dir = os.path.dirname(os.path.abspath(__file__))
        repo_root = os.path.abspath(os.path.join(curr_dir, '..', '..', '..'))
        candidates_osm.extend([
            os.path.join(curr_dir, '..', 'data', 'campus_with_junctions_and_stops.osm'),
            os.path.join(curr_dir, '..', 'data', 'campus.osm'),
            os.path.join(repo_root, 'campus_with_junctions_and_stops.osm'),
            os.path.join(repo_root, 'campus.osm')
        ])
        default_osm = next((p for p in candidates_osm if os.path.exists(p)), '')
        self.declare_parameter('osm_file', default_osm)
        osm_file = self.get_parameter('osm_file').get_parameter_value().string_value

        self.campus = CampusMap(osm_file)
        self.get_logger().info(
            f"MissionController loaded map: {len(self.campus.stops)} stops, "
            f"{len(self.campus.main_nodes)} road nodes")

        # State
        self.current_pose_nav = None
        self._pose_last_updated: float = 0.0

        # Status publisher (for AuditLogger & HUD)
        self.status_pub = self.create_publisher(String, '/campus/nav_status', 10)

        # Vehicle pose subscriber to keep track of location
        self.pose_sub = self.create_subscription(
            PoseStamped, '/campus/vehicle_pose', self.vehicle_pose_callback, 10,
            callback_group=self.cb_group)

        # Action Server: RunMission for boarding & dwell timing
        self._run_mission_server = ActionServer(
            self,
            RunMission,
            'campus/run_mission_action',
            execute_callback=self._execute_run_mission,
            goal_callback=lambda g: GoalResponse.ACCEPT,
            cancel_callback=lambda g: CancelResponse.ACCEPT,
            callback_group=self.cb_group
        )

        # Legacy /campus/mission topic subscriber (redirects to master)
        self.legacy_mission_sub = self.create_subscription(
            String, '/campus/mission', self.legacy_mission_callback, 10,
            callback_group=self.cb_group)

        self.get_logger().info(
            "MissionController ready. Action Server active on /campus/run_mission_action.")

    def vehicle_pose_callback(self, msg: PoseStamped):
        """Track buggy's live position in planner coords."""
        self.current_pose_nav = (msg.pose.position.x, msg.pose.position.y)
        self._pose_last_updated = time.time()

    def legacy_mission_callback(self, msg: String):
        raw = msg.data.strip()
        self.get_logger().warn(
            f"Received legacy topic mission: '{raw}'. Forwarding instruction: "
            f"Please publish to /campus/master_command with format 'MISSION:{raw}'")

    # ── Action Server Implementation ──────────────────────────────────────────
    def _execute_run_mission(self, goal_handle):
        """
        Called by CampusMasterNode to handle the passenger boarding phase.
        Pauses for dwell_seconds without blocking other node callbacks.
        """
        start_stop = goal_handle.request.start_stop
        goal_stop = goal_handle.request.goal_stop
        dwell_secs = goal_handle.request.dwell_seconds or 5.0
        t0 = time.time()

        self.get_logger().info(
            f"[RunMission Action] Boarding phase at '{start_stop}' for {dwell_secs:.1f}s...")

        feedback = RunMission.Feedback()
        feedback.leg = 'BOARDING'
        feedback.status_text = (
            f"Arrived at '{start_stop}'. Boarding passengers... ({dwell_secs:.0f}s dwell)"
        )
        goal_handle.publish_feedback(feedback)
        self._publish_status(feedback.status_text)

        # Dwell wait in small chunks so cancel requests are honored
        while time.time() - t0 < dwell_secs:
            if goal_handle.is_cancel_requested:
                self.get_logger().warn("Boarding canceled by master!")
                goal_handle.canceled()
                result = RunMission.Result()
                result.success = False
                result.message = "Boarding canceled."
                result.elapsed_seconds = float(time.time() - t0)
                return result
            time.sleep(0.1)

        result = RunMission.Result()
        result.success = True
        result.message = f"Boarding complete at '{start_stop}'. Ready for Leg 2 to '{goal_stop}'."
        result.elapsed_seconds = float(time.time() - t0)
        goal_handle.succeed()

        self.get_logger().info(f"✅ {result.message}")
        self._publish_status(result.message)
        return result

    # ── Gazebo Teleport Utility ───────────────────────────────────────────────
    def _teleport_model(self, model_name: str, gz_x: float, gz_y: float,
                        gz_z: float = 1.2, yaw: float = 0.0) -> bool:
        """
        Teleport a Gazebo model to (gz_x, gz_y, gz_z) with given yaw.
        Uses `gz service` CLI to call /world/<world>/set_pose.
        """
        sin_h = math.sin(yaw / 2.0)
        cos_h = math.cos(yaw / 2.0)

        req = (
            f'name: "{model_name}", '
            f'position: {{x: {gz_x:.4f}, y: {gz_y:.4f}, z: {gz_z:.4f}}}, '
            f'orientation: {{x: 0.0, y: 0.0, z: {sin_h:.6f}, w: {cos_h:.6f}}}'
        )

        cmd = [
            'gz', 'service',
            '-s', f'/world/{WORLD_NAME}/set_pose',
            '--reqtype', 'gz.msgs.Pose',
            '--reptype', 'gz.msgs.Boolean',
            '--timeout', '3000',
            '--req', req
        ]

        self.get_logger().info(f"Teleporting {model_name} to GZ ({gz_x:.1f}, {gz_y:.1f})")
        try:
            result = subprocess.run(cmd, capture_output=True, text=True, timeout=5.0)
            if result.returncode == 0:
                self.get_logger().info("  Teleport successful.")
                return True
            else:
                self.get_logger().warn(
                    f"  Teleport returned non-zero: {result.stderr.strip()}")
                return False
        except subprocess.TimeoutExpired:
            self.get_logger().warn("  Teleport timed out (5s)")
            return False
        except FileNotFoundError:
            self.get_logger().warn(
                "  'gz' CLI not found. Is Gazebo Harmonic installed and sourced?")
            return False

    def _publish_status(self, text: str):
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)
        self.get_logger().info(f"[status] {text}")


def main(args=None):
    rclpy.init(args=args)
    node = MissionControllerNode()
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
