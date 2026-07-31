#!/usr/bin/env python3

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient
from rclpy.node import Node
from visualization_msgs.msg import Marker

from qbot_rl_interfaces.action import PolicyNavigate


class NavigateActionClient(Node):
    def __init__(self) -> None:
        super().__init__("navigate_action_client")
        self.goal_pose_topic = str(self.declare_parameter("goal_pose_topic", "/goal_pose").value)
        self.action_name = str(self.declare_parameter("action_name", "policy_navigate").value)
        self.timeout_sec = float(self.declare_parameter("timeout_sec", 60.0).value)
        self.success_radius_m = float(self.declare_parameter("success_radius_m", 0.25).value)
        self.goal_marker_topic = str(self.declare_parameter("goal_marker_topic", "policy_goal_marker").value)

        if self.timeout_sec <= 0.0:
            raise RuntimeError("Parameter 'timeout_sec' must be > 0.")
        if self.success_radius_m <= 0.0:
            raise RuntimeError("Parameter 'success_radius_m' must be > 0.")

        self._client = ActionClient(self, PolicyNavigate, self.action_name)
        self._goal_marker_pub = self.create_publisher(Marker, self.goal_marker_topic, 10)
        self.create_subscription(PoseStamped, self.goal_pose_topic, self._goal_pose_cb, 10)

        self.get_logger().info(
            "navigate_action_client started. "
            f"goal_pose_topic={self.goal_pose_topic}, action_name={self.action_name}, "
            f"timeout_sec={self.timeout_sec:.2f}, success_radius_m={self.success_radius_m:.3f}, "
            f"goal_marker_topic={self.goal_marker_topic}"
        )

    def _goal_pose_cb(self, msg: PoseStamped) -> None:
        goal = PolicyNavigate.Goal()
        goal.timeout_sec = float(self.timeout_sec)
        goal.success_radius_m = float(self.success_radius_m)

        self._publish_goal_marker_add(msg)

        self.get_logger().info(
            f"Goal trigger received. Sending action goal: timeout={goal.timeout_sec:.2f}, "
            f"success_radius={goal.success_radius_m:.3f}"
        )

        send_future = self._client.send_goal_async(goal, feedback_callback=self._feedback_cb)
        send_future.add_done_callback(self._goal_response_cb)

    def _publish_goal_marker_add(self, goal_pose_msg: PoseStamped) -> None:
        marker = Marker()
        marker.header = goal_pose_msg.header
        marker.ns = "policy_navigate_goal"
        marker.id = 0
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD
        marker.pose = goal_pose_msg.pose
        diameter = 2.0 * float(self.success_radius_m)
        marker.scale.x = diameter
        marker.scale.y = diameter
        marker.scale.z = 0.03
        marker.color.a = 1.0
        marker.color.r = 0.2
        marker.color.g = 0.85
        marker.color.b = 0.25
        self._goal_marker_pub.publish(marker)

    def _publish_goal_marker_delete(self) -> None:
        marker = Marker()
        marker.header.frame_id = "odom"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "policy_navigate_goal"
        marker.id = 0
        marker.action = Marker.DELETE
        self._goal_marker_pub.publish(marker)

    def _goal_response_cb(self, future) -> None:
        try:
            goal_handle = future.result()
        except Exception as exc:
            self.get_logger().error(f"Failed to send action goal: {exc}")
            return

        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().warn("Action goal rejected by server.")
            return

        self.get_logger().info("Action goal accepted by server.")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._result_cb)

    def _feedback_cb(self, feedback_msg) -> None:
        fb = feedback_msg.feedback
        self.get_logger().info(
            f"feedback: t={fb.running_time_sec:.2f}s, dist={fb.distance_to_goal:.3f}, "
            f"heading={fb.heading_error:.3f}"
        )

    def _result_cb(self, future) -> None:
        try:
            result_msg = future.result()
        except Exception as exc:
            self.get_logger().error(f"Failed to get action result: {exc}")
            return

        if result_msg is None:
            self.get_logger().error("Action result is empty.")
            return

        result = result_msg.result
        status_text = self._status_to_str(result_msg.status)
        self.get_logger().info(
            f"Result: status={status_text}, succeeded={result.succeeded}, "
            f"message='{result.message}', final_distance={result.final_distance:.3f}"
        )

        message_lower = (result.message or "").lower()
        is_preempted = "preempt" in message_lower
        is_timeout = "time" in message_lower and "out" in message_lower
        should_delete_marker = (
            result_msg.status in (GoalStatus.STATUS_SUCCEEDED, GoalStatus.STATUS_CANCELED)
            or is_timeout
        ) and not is_preempted

        if should_delete_marker:
            self._publish_goal_marker_delete()

    @staticmethod
    def _status_to_str(status: int) -> str:
        if status == GoalStatus.STATUS_UNKNOWN:
            return "UNKNOWN"
        if status == GoalStatus.STATUS_ACCEPTED:
            return "ACCEPTED"
        if status == GoalStatus.STATUS_EXECUTING:
            return "EXECUTING"
        if status == GoalStatus.STATUS_CANCELING:
            return "CANCELING"
        if status == GoalStatus.STATUS_SUCCEEDED:
            return "SUCCEEDED"
        if status == GoalStatus.STATUS_CANCELED:
            return "CANCELED"
        if status == GoalStatus.STATUS_ABORTED:
            return "ABORTED"
        return f"STATUS_{status}"


def main() -> None:
    rclpy.init()
    node = NavigateActionClient()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
