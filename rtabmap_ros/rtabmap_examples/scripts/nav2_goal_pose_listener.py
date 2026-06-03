#!/usr/bin/env python3
"""
nav2_goal_pose_listener.py

Listens for PoseStamped goals on /goal_pose, looks up the robot's current
position via TF, calls Nav2's ComputePathToPose action and publishes the
resulting path on /visual_plan.

Key design:
  - Fully async action client — never blocks the spin loop.
  - On rejection or failure: logs an error and keeps listening.  No sys.exit.
  - Ignores a new goal while a request is still in-flight.
"""
from __future__ import annotations

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, DurabilityPolicy

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from nav2_msgs.action import ComputePathToPose
import tf2_ros


class Nav2GoalPoseListener(Node):

    def __init__(self):
        super().__init__('nav2_goal_pose_listener')

        # ── Parameters ────────────────────────────────────────────────────
        self.declare_parameter('goal_topic',      '/goal_pose')
        self.declare_parameter('compute_service', '/compute_path_to_pose')
        self.declare_parameter('output_topic',    '/visual_plan')
        self.declare_parameter('source_frame',    'map')
        self.declare_parameter('robot_frame',     'oak-d-base-frame')

        goal_topic      = self.get_parameter('goal_topic').value
        action_name     = self.get_parameter('compute_service').value
        output_topic    = self.get_parameter('output_topic').value
        self.source_frame = self.get_parameter('source_frame').value
        self.robot_frame  = self.get_parameter('robot_frame').value

        # ── TF ────────────────────────────────────────────────────────────
        self.tf_buffer   = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ── Action client ─────────────────────────────────────────────────
        self._action_client = ActionClient(self, ComputePathToPose, action_name)
        self._busy = False   # True while a request is in-flight

        # ── Path publisher (transient-local so RViz latches last plan) ────
        qos = QoSProfile(depth=1)
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self._path_pub = self.create_publisher(Path, output_topic, qos)

        # ── Goal subscriber ───────────────────────────────────────────────
        self._sub = self.create_subscription(
            PoseStamped, goal_topic, self._goal_cb, 10)

        self.get_logger().info(
            f"Listening for goals on '{goal_topic}'  "
            f"action='{action_name}'  "
            f"output='{output_topic}'"
        )

    # ── Incoming goal ──────────────────────────────────────────────────────

    def _goal_cb(self, msg: PoseStamped) -> None:
        if self._busy:
            self.get_logger().warn("Still computing previous path — ignoring new goal")
            return

        # Transform goal into map frame if needed
        goal_in_map = msg
        try:
            if msg.header.frame_id not in ('', self.source_frame):
                goal_in_map = self.tf_buffer.transform(
                    msg, self.source_frame, timeout=Duration(seconds=1.0))
        except Exception as e:
            self.get_logger().warn(
                f"Could not transform goal into {self.source_frame}: {e} — using original")

        # Look up current robot pose
        try:
            trans = self.tf_buffer.lookup_transform(
                self.source_frame, self.robot_frame,
                rclpy.time.Time(), timeout=Duration(seconds=1.0))
        except Exception as e:
            self.get_logger().error(
                f"TF lookup {self.robot_frame}→{self.source_frame} failed: {e}")
            return

        start = PoseStamped()
        start.header.frame_id = self.source_frame
        start.header.stamp    = self.get_clock().now().to_msg()
        start.pose.position.x = float(trans.transform.translation.x)
        start.pose.position.y = float(trans.transform.translation.y)
        start.pose.position.z = float(trans.transform.translation.z)
        start.pose.orientation = trans.transform.rotation

        goal_ps = PoseStamped()
        goal_ps.header.frame_id = self.source_frame
        goal_ps.header.stamp    = self.get_clock().now().to_msg()
        goal_ps.pose.position.x = float(goal_in_map.pose.position.x)
        goal_ps.pose.position.y = float(goal_in_map.pose.position.y)
        goal_ps.pose.orientation.w = 1.0

        sx, sy = start.pose.position.x, start.pose.position.y
        gx, gy = goal_ps.pose.position.x, goal_ps.pose.position.y
        self.get_logger().info(
            f"Received goal  start=({sx:.3f}, {sy:.3f})  goal=({gx:.3f}, {gy:.3f})  "
            f"frame='{self.source_frame}'"
        )

        if not self._action_client.wait_for_server(timeout_sec=3.0):
            self.get_logger().error("ComputePathToPose action server not available — skipping")
            return

        goal_msg = ComputePathToPose.Goal()
        goal_msg.start     = start
        goal_msg.goal      = goal_ps
        goal_msg.use_start = True   # use our TF-derived start, not planner's own localization

        self._busy = True
        future = self._action_client.send_goal_async(goal_msg)
        future.add_done_callback(self._on_goal_response)

    # ── Async callbacks ────────────────────────────────────────────────────

    def _on_goal_response(self, future):
        try:
            handle = future.result()
        except Exception as e:
            self.get_logger().error(f"send_goal failed: {e}")
            self._busy = False
            return

        if not handle.accepted:
            self.get_logger().error(
                "Goal REJECTED by planner.  Common causes: costmap not yet built, "
                "start/goal in occupied/unknown cell, planner lifecycle not ACTIVE, "
                "or inflation_radius < robot inscribed radius."
            )
            self._busy = False
            return

        self.get_logger().info("Goal accepted — waiting for path …")
        result_future = handle.get_result_async()
        result_future.add_done_callback(self._on_result)

    def _on_result(self, future):
        self._busy = False
        try:
            result = future.result().result
        except Exception as e:
            self.get_logger().error(f"get_result failed: {e}")
            return

        plan: Path = getattr(result, 'path', None)
        if plan is None or len(plan.poses) == 0:
            self.get_logger().warn("Planner returned empty path — target unreachable")
            return

        self._path_pub.publish(plan)
        self.get_logger().info(
            f"Path published on /visual_plan  ({len(plan.poses)} poses)"
        )


def main(args=None):
    import os
    os.environ.setdefault(
        "RCUTILS_CONSOLE_OUTPUT_FORMAT",
        "[{severity}] [{name}]: {message}",
    )
    rclpy.init(args=args)
    node = Nav2GoalPoseListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
