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


class Nav2GoalPoseListener(Node):
    def __init__(self):
        super().__init__('nav2_goal_pose_listener')

        self.declare_parameter('goal_topic', '/goal_pose')
        self.declare_parameter('compute_service', '/compute_path_to_pose')
        self.declare_parameter('output_topic', '/computed_path')
        self.declare_parameter('source_frame', 'map')
        self.declare_parameter('robot_frame', 'base_link')
        self.declare_parameter('tolerance', 0.5)

        goal_topic = self.get_parameter('goal_topic').get_parameter_value().string_value
        compute_service = self.get_parameter('compute_service').get_parameter_value().string_value
        output_topic = self.get_parameter('output_topic').get_parameter_value().string_value
        self.source_frame = self.get_parameter('source_frame').get_parameter_value().string_value
        self.robot_frame = self.get_parameter('robot_frame').get_parameter_value().string_value
        self.tolerance = float(self.get_parameter('tolerance').get_parameter_value().double_value)

        self.get_logger().info(f"Listening for goals on '{goal_topic}' and calling '{compute_service}'")

        self.sub = self.create_subscription(PoseStamped, goal_topic, self._goal_cb, 10)
        self.pub = self.create_publisher(Path, output_topic, 10)

        # action name (used when calling compute_nav2_plan.py)
        self.action_name = compute_service

        # TF buffer & listener for getting current robot pose and transforming goals
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Map YAML and fixed start pixel configuration
        self.declare_parameter('map_yaml', '/home/ggsya/ros_ws/src/rtabmap_ros/rtabmap_examples/scripts/nav2_white_map.yaml')
        self.declare_parameter('start_px', [10, 10])
        self.map_yaml = self.get_parameter('map_yaml').get_parameter_value().string_value
        self.start_px = tuple(self.get_parameter('start_px').get_parameter_value().integer_array_value)

        try:
            self._map_image, self._map_resolution, self._map_origin = read_map_yaml(self.map_yaml)
            self._map_width, self._map_height = read_pgm_size(self._map_image)
        except Exception as e:
            self.get_logger().error(f'Failed to read map yaml/pgm: {e}')
            # fall back to defaults
            self._map_image = None
            self._map_resolution = 0.05
            self._map_origin = (0.0, 0.0, 0.0)
            self._map_width = 0
            self._map_height = 0

        self._start_m = pixel_to_map(self.start_px[0], self.start_px[1], self._map_height, self._map_resolution, self._map_origin)
        self.get_logger().info(f'Fixed start px={self.start_px} -> meters={self._start_m}')

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

        # Build start and goal PoseStamped for action
        start = PoseStamped()
        start.header.frame_id = self.source_frame
        start.header.stamp = self.get_clock().now().to_msg()
        start.pose.position.x = float(self._start_m[0])
        start.pose.position.y = float(self._start_m[1])
        start.pose.orientation.w = 1.0

        goal_ps = PoseStamped()
        goal_ps.header.frame_id = self.source_frame
        goal_ps.header.stamp = self.get_clock().now().to_msg()
        goal_ps.pose.position.x = float(goal_in_map.pose.position.x)
        goal_ps.pose.position.y = float(goal_in_map.pose.position.y)
        goal_ps.pose.orientation = goal_in_map.pose.orientation

        # Convert goal (meters in map frame) to pixel coordinates for compute_nav2_plan
        try:
            map_image, map_res, map_origin = read_map_yaml(self.map_yaml)
            map_w, map_h = read_pgm_size(map_image)
        except Exception as e:
            self.get_logger().error(f'Failed to read map yaml for pixel conversion: {e}')
            return

        ox, oy, _ = map_origin
        gx = goal_ps.pose.position.x
        gy = goal_ps.pose.position.y
        mx = int((gx - ox) / map_res)
        my = 255 - int((gy - oy) / map_res)

        # create_nav2_path_from_pixels(
        # yaml_path = self.map_yaml,
        # start_px = (10, 10),
        # goal_px = (mx, my),
        # action = self.action_name,
        # topic = "/visual_plan",
        # frame = "map",
        # tolerance = 0.2,
        # )





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
