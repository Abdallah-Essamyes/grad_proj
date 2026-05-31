#!/usr/bin/env python3
"""Listen for goal poses and ask Nav2 to compute a path to them.

Subscribes to a PoseStamped topic (default: /goal_pose). When a goal is
received the node looks up the robot's current pose (via TF), transforms the
goal to the same frame if needed, calls the Nav2 `compute_path_to_pose`
service and publishes the returned `nav_msgs/Path` on `/computed_path`.
"""
from __future__ import annotations

import sys
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
import tf2_ros

import subprocess
import shlex
# reuse helper functions from existing script
from compute_nav2_plan import *
from visualization_msgs.msg import Marker
from builtin_interfaces.msg import Duration as MsgDuration


class Nav2GoalPoseListener(Node):
    def __init__(self):
        super().__init__('nav2_goal_pose_listener')

        self.declare_parameter('goal_topic', '/goal_pose')
        self.declare_parameter('compute_service', '/compute_path_to_pose')
        self.declare_parameter('output_topic', '/visual_plan')
        self.declare_parameter('source_frame', 'map')
        self.declare_parameter('robot_frame', 'oak-d-base-frame')
        self.declare_parameter('tolerance', 0.3)

        goal_topic = self.get_parameter('goal_topic').get_parameter_value().string_value
        compute_service = self.get_parameter('compute_service').get_parameter_value().string_value
        output_topic = self.get_parameter('output_topic').get_parameter_value().string_value
        self.source_frame = self.get_parameter('source_frame').get_parameter_value().string_value
        self.robot_frame = self.get_parameter('robot_frame').get_parameter_value().string_value
        self.tolerance = float(self.get_parameter('tolerance').get_parameter_value().double_value)

        # store output topic for use when delegating to compute_nav2_plan
        self.output_topic = output_topic

        self.get_logger().info(f"Listening for goals on '{goal_topic}' and calling '{compute_service}'")

        self.sub = self.create_subscription(PoseStamped, goal_topic, self._goal_cb, 10)
        self.pub = self.create_publisher(Path, output_topic, 10)
        # Marker publisher for start (green) and goal (red)
        self.marker_pub = self.create_publisher(Marker, 'goal_markers', 10)

        # action name (used when calling compute_nav2_plan.py)
        self.action_name = compute_service

        # TF buffer & listener for getting current robot pose and transforming goals
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Map YAML optional (not required when using live TF for start pose)
        self.declare_parameter('map_yaml', '/home/ggsya/ros_ws/src/rtabmap_ros/rtabmap_examples/scripts/nav2_white_map.yaml')
        self.map_yaml = self.get_parameter('map_yaml').get_parameter_value().string_value

        # Ensure pub uses transient local so RViz can latch last plan
        # (QoS left default here; compute_nav2_plan uses TRANSIENT_LOCAL when publishing)

    # helper functions are reused from compute_nav2_plan.py

    def _goal_cb(self, msg: PoseStamped) -> None:
        self.get_logger().info(f"Received goal: frame={msg.header.frame_id} stamp={msg.header.stamp.sec}.{msg.header.stamp.nanosec}")

        # Transform goal into source_frame (map) if needed
        goal_in_map = msg
        try:
            if msg.header.frame_id != self.source_frame and msg.header.frame_id != '':
                goal_in_map = self.tf_buffer.transform(msg, self.source_frame, timeout=Duration(seconds=1.0))
        except Exception as e:
            self.get_logger().warning(f'Could not transform goal into {self.source_frame}: {e} — using original goal pose')

        # Get current robot pose by looking up transform from robot_frame to source_frame
        start = PoseStamped()
        try:
            now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform(self.source_frame, self.robot_frame, now, timeout=Duration(seconds=1.0))
            start.header.frame_id = self.source_frame
            start.header.stamp = self.get_clock().now().to_msg()
            start.pose.position.x = float(trans.transform.translation.x)
            start.pose.position.y = float(trans.transform.translation.y)
            start.pose.position.z = float(trans.transform.translation.z)
            start.pose.orientation = trans.transform.rotation
        except Exception as e:
            self.get_logger().error(f'Could not lookup transform from {self.robot_frame} to {self.source_frame}: {e}')
            return

        goal_ps = PoseStamped()
        goal_ps.header.frame_id = self.source_frame
        goal_ps.header.stamp = self.get_clock().now().to_msg()
        goal_ps.pose.position.x = float(goal_in_map.pose.position.x)
        goal_ps.pose.position.y = float(goal_in_map.pose.position.y)
        goal_ps.pose.orientation = goal_in_map.pose.orientation

        # Publish markers: green for start, red for goal
        try:
            start_marker = Marker()
            start_marker.header.frame_id = self.source_frame
            start_marker.header.stamp = self.get_clock().now().to_msg()
            start_marker.ns = 'nav2_goals'
            start_marker.id = 0
            start_marker.type = Marker.SPHERE
            start_marker.action = Marker.ADD
            start_marker.pose = start.pose
            start_marker.pose.position.z = 0.0 
            start_marker.scale.x = start_marker.scale.y = start_marker.scale.z = 0.2
            start_marker.color.r = 0.0
            start_marker.color.g = 1.0
            start_marker.color.b = 0.0
            start_marker.color.a = 1.0
            start_marker.lifetime = MsgDuration()

            goal_marker = Marker()
            goal_marker.header.frame_id = self.source_frame
            goal_marker.header.stamp = self.get_clock().now().to_msg()
            goal_marker.ns = 'nav2_goals'
            goal_marker.id = 1
            goal_marker.type = Marker.SPHERE
            goal_marker.action = Marker.ADD
            goal_marker.pose = goal_ps.pose
            goal_marker.pose.position.z = 0.0
            goal_marker.scale.x = goal_marker.scale.y = goal_marker.scale.z = 0.2
            goal_marker.color.r = 1.0
            goal_marker.color.g = 0.0
            goal_marker.color.b = 0.0
            goal_marker.color.a = 1.0
            goal_marker.lifetime = MsgDuration()

            self.marker_pub.publish(start_marker)
            self.marker_pub.publish(goal_marker)
        except Exception as e:
            self.get_logger().warning(f'Failed to publish markers: {e}')

        # Log start and goal positions in meters
        try:
            sx = start.pose.position.x
            sy = start.pose.position.y
            gx = goal_ps.pose.position.x
            gy = goal_ps.pose.position.y
            self.get_logger().info(f"Start (m): x={sx:.3f} y={sy:.3f}")
            self.get_logger().info(f"Goal  (m): x={gx:.3f} y={gy:.3f}")
        except Exception as e:
            self.get_logger().warning(f'Failed to log start/goal positions: {e}')

        # Delegate to the Pose-based planner in compute_nav2_plan
        try:
            create_nav2_path_from_Pose(start, goal_ps, action=self.action_name, topic=self.output_topic, frame=self.source_frame, tolerance=self.tolerance)
        except Exception as e:
            self.get_logger().error(f'Failed to call create_nav2_path_from_Pose: {e}')







def main(args=None):
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
