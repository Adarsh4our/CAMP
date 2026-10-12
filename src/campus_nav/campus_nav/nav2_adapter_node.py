import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from nav2_msgs.action import FollowPath
from rclpy.action import ActionClient, ActionServer, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from std_msgs.msg import String
from saye_msgs.action import FollowRoute, PlanRoute
import time


class Nav2AdapterNode(Node):
    def __init__(self):
        super().__init__('nav2_adapter_node')
        self.cb_group = ReentrantCallbackGroup()

        # Legacy topic subscription to preserve backward compatibility
        self.path_sub = self.create_subscription(
            Path, '/campus/nav2_target_path', self.path_callback, 10,
            callback_group=self.cb_group)

        # Status publisher (read by AuditLogger & operator HUD)
        self.status_pub = self.create_publisher(String, '/campus/pursuit_status', 10)

        # Action client to talk to Nav2's controller_server
        self.action_client = ActionClient(self, FollowPath, 'follow_path', callback_group=self.cb_group)

        # Action client to request routes on-demand from planner_node
        self._plan_client = ActionClient(self, PlanRoute, 'campus/plan_route_action', callback_group=self.cb_group)

        # Action server to receive follow route goals from CampusMasterNode
        self._follow_route_server = ActionServer(
            self,
            FollowRoute,
            'campus/follow_route_action',
            execute_callback=self._execute_follow_route,
            goal_callback=lambda g: GoalResponse.ACCEPT,
            cancel_callback=lambda g: CancelResponse.ACCEPT,
            callback_group=self.cb_group
        )
        self.get_logger().info("Nav2 Adapter Node ready with Action Server on /campus/follow_route_action.")

    def _auto_fetch_plan(self, dest_label: str):
        """Call planner_node's PlanRoute action to compute a route dynamically."""
        if not self._plan_client.wait_for_server(timeout_sec=4.0):
            self.get_logger().error("PlanRoute action server unavailable!")
            return None
        goal_msg = PlanRoute.Goal()
        goal_msg.mission = f"CURRENT|{dest_label}"
        future = self._plan_client.send_goal_async(goal_msg)
        while not future.done():
            time.sleep(0.05)
        gh = future.result()
        if not gh or not gh.accepted:
            return None
        res_future = gh.get_result_async()
        while not res_future.done():
            time.sleep(0.05)
        res = res_future.result()
        return res.result.path if res and res.result.success else None

    # ── Action Server Implementation ──────────────────────────────────────────
    def _execute_follow_route(self, goal_handle):
        """
        Receives a nav_msgs/Path from CampusMasterNode,
        forwards it to Nav2 controller_server via FollowPath Action,
        relays distance feedback, and returns ARRIVED or FAILED.
        """
        path = goal_handle.request.path
        dest_label = goal_handle.request.destination_label
        n_pts = len(path.poses)
        self.get_logger().info(f"[FollowRoute Action] Received request for '{dest_label}' ({n_pts} points)")

        # Auto-fetch route from planner if an empty path was passed with a destination label
        if n_pts == 0:
            if dest_label:
                self.get_logger().info(
                    f"[FollowRoute Action] Path has 0 poses! Auto-querying planner for '{dest_label}'...")
                path = self._auto_fetch_plan(dest_label)
                if not path or len(path.poses) == 0:
                    result = FollowRoute.Result()
                    result.success = False
                    result.message = f"Failed to plan route for '{dest_label}'."
                    self.get_logger().error(result.message)
                    goal_handle.abort()
                    return result
                n_pts = len(path.poses)
                self.get_logger().info(f"[FollowRoute Action] Obtained route with {n_pts} waypoints.")
            else:
                result = FollowRoute.Result()
                result.success = False
                result.message = "Path is empty and no destination_label was provided."
                self.get_logger().error(result.message)
                goal_handle.abort()
                return result

        if not self.action_client.wait_for_server(timeout_sec=5.0):
            result = FollowRoute.Result()
            result.success = False
            result.message = "Nav2 controller_server action server not available!"
            self.get_logger().error(result.message)
            goal_handle.abort()
            return result

        goal_msg = FollowPath.Goal()
        goal_msg.path = path
        goal_msg.controller_id = 'FollowPath'

        # Send goal to Nav2 synchronously with future waiting in this thread
        send_future = self.action_client.send_goal_async(
            goal_msg,
            feedback_callback=lambda fb: self._relay_action_feedback(goal_handle, fb)
        )

        while not send_future.done():
            time.sleep(0.05)

        nav2_goal_handle = send_future.result()
        if not nav2_goal_handle.accepted:
            result = FollowRoute.Result()
            result.success = False
            result.message = "Nav2 rejected the path!"
            self.get_logger().error(result.message)
            goal_handle.abort()
            return result

        self.get_logger().info("Nav2 accepted the path! Tracking trajectory...")
        result_future = nav2_goal_handle.get_result_async()

        while not result_future.done():
            if goal_handle.is_cancel_requested:
                self.get_logger().warn("Cancel requested by master. Canceling Nav2 goal...")
                nav2_goal_handle.cancel_goal_async()
                goal_handle.canceled()
                result = FollowRoute.Result()
                result.success = False
                result.message = "CANCELED"
                return result
            time.sleep(0.05)

        nav2_result = result_future.result()
        status_code = nav2_result.status

        result = FollowRoute.Result()
        # In rclpy.action.GoalStatus, 4 is SUCCEEDED, 5 is CANCELED, 6 is ABORTED
        if status_code == 4:
            self.get_logger().info("Nav2 completed path successfully!")
            result.success = True
            result.message = "ARRIVED"
            goal_handle.succeed()

            status_msg = String()
            status_msg.data = "ARRIVED"
            self.status_pub.publish(status_msg)
        else:
            self.get_logger().error(f"Nav2 failed to reach destination! Status code: {status_code}")
            result.success = False
            result.message = f"FAILED (Nav2 status {status_code})"
            goal_handle.abort()

        return result

    def _relay_action_feedback(self, goal_handle, nav2_feedback):
        """Forward Nav2 FollowPath feedback to Master."""
        fb = FollowRoute.Feedback()
        fb.distance_remaining = float(getattr(nav2_feedback.feedback, 'distance_to_goal', 0.0))
        fb.pursuit_status = f"[Nav2] Driving... {fb.distance_remaining:.1f}m left"
        goal_handle.publish_feedback(fb)

        # Also publish to legacy topic for HUD
        status_msg = String()
        status_msg.data = fb.pursuit_status
        self.status_pub.publish(status_msg)

    # ── Legacy Topic Fallback ──────────────────────────────────────────────────
    def path_callback(self, msg: Path):
        self.get_logger().info(f"Received path on legacy topic with {len(msg.poses)} waypoints! Sending to Nav2...")

        if not self.action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Nav2 controller_server action server not available!")
            return

        goal_msg = FollowPath.Goal()
        goal_msg.path = msg
        goal_msg.controller_id = 'FollowPath'

        self.send_goal_future = self.action_client.send_goal_async(goal_msg)
        self.send_goal_future.add_done_callback(self.goal_response_callback)

        status_msg = String()
        status_msg.data = "[Nav2] Following: 0% | Driving..."
        self.status_pub.publish(status_msg)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Nav2 rejected the path!")
            return
        self.get_logger().info("Nav2 accepted the path! Driving...")
        self.result_future = goal_handle.get_result_async()
        self.result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        status = future.result().status
        if status == 4:
            self.get_logger().info("Nav2 reached the destination successfully!")
            status_msg = String()
            status_msg.data = "ARRIVED"
            self.status_pub.publish(status_msg)
        else:
            self.get_logger().error(f"Nav2 failed to reach destination! Status code: {status}")


def main(args=None):
    rclpy.init(args=args)
    node = Nav2AdapterNode()
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
