"""Regression coverage for the velocity feedback used by Nav2 MPPI.

Run after sourcing ROS and the workspace:
    python3 -m pytest src/campus_nav/test/test_odom_bridge.py
"""

import math
from types import SimpleNamespace
from unittest.mock import Mock

import pytest
from builtin_interfaces.msg import Time
from geometry_msgs.msg import Quaternion, Twist

from campus_nav.coord_bridge import gz_to_nav, yaw_from_quaternion
from campus_nav.odom_bridge_node import OdomBridgeNode


@pytest.mark.parametrize('initial_yaw', [-3.13, 0.0, 1.6])
@pytest.mark.parametrize('yaw_rate', [-0.3, 0.0, 0.3])
def test_velocity_matches_pose_change_in_navigation_frame(initial_yaw, yaw_rate):
    """Left/right turn feedback must agree with the published pose derivative.

    Use the real pose conversion with recording publishers. No ROS graph,
    Gazebo process, or vehicle command is needed for this regression.
    """
    clock = Mock()
    clock.now.return_value.to_msg.return_value = Time()
    bridge = SimpleNamespace(
        get_clock=lambda: clock,
        vehicle_pose_pub=Mock(),
        odom_pub=Mock(),
        tf_broadcaster=Mock(),
        _publish_vehicle_markers=Mock(),
    )
    twist = Twist()
    twist.linear.y = -0.8  # The vehicle drives along its CAD frame's -Y axis.
    twist.angular.z = yaw_rate
    dt = 0.1

    for yaw in (initial_yaw, initial_yaw + yaw_rate * dt):
        orientation = Quaternion(z=math.sin(yaw / 2), w=math.cos(yaw / 2))
        OdomBridgeNode._process_pose(bridge, 10.0, 20.0, 0.3, orientation, twist)

    poses = [call.args[0] for call in bridge.vehicle_pose_pub.publish.call_args_list]
    heading_change = (
        yaw_from_quaternion(poses[1].pose.orientation)
        - yaw_from_quaternion(poses[0].pose.orientation)
    )
    measured_yaw_rate = math.atan2(math.sin(heading_change), math.cos(heading_change)) / dt
    odom = bridge.odom_pub.publish.call_args.args[0]

    assert odom.twist.twist.angular.z == pytest.approx(measured_yaw_rate)
    assert odom.twist.twist.linear.x == pytest.approx(0.8)
    assert odom.header.frame_id == 'map'
    assert odom.child_frame_id == 'nav_base_link'
    assert (odom.pose.pose.position.x, odom.pose.pose.position.y) == gz_to_nav(10.0, 20.0)
    assert odom.pose.pose.orientation == poses[-1].pose.orientation
