#!/usr/bin/env python3
"""
Publish a simple `nav_msgs/Path` from pixel coordinates on a Nav2-style map.

Usage:
  publish_nav2_path.py --yaml /path/to/map.yaml --start 10 10 --goal 100 150

This node reads the YAML to get `resolution` and `origin`, reads the PGM to
discover image height, converts pixel coordinates to map meters and publishes
the path once on `/visual_path` (TRANSIENT_LOCAL QoS so RViz can latch it).
"""
import argparse
import os
import yaml
import sys

def read_map_yaml(yaml_path):
    with open(yaml_path, "r") as f:
        data = yaml.safe_load(f)
    image = data.get("image")
    resolution = float(data.get("resolution", 0.05))
    origin = data.get("origin", [0.0, 0.0, 0.0])
    # Resolve image relative to YAML file
    if not os.path.isabs(image):
        image = os.path.join(os.path.dirname(yaml_path), image)
    return image, resolution, origin


def read_pgm_size(pgm_path):
    # Read header tokens until we have magic, width, height, maxval
    with open(pgm_path, "rb") as f:
        data = f.read(1024)
    parts = []
    for token in data.split():
        parts.append(token)
        if len(parts) >= 4:
            break
    if len(parts) < 4:
        raise RuntimeError("Can't parse PGM header: %s" % pgm_path)
    # parts[0] = b'P5'
    width = int(parts[1])
    height = int(parts[2])
    return width, height


def pixel_to_map(px, py, height, resolution, origin):
    ox, oy, oyaw = origin
    # pixel origin top-left; map origin assumed at YAML origin; convert
    x = ox + (px + 0.5) * resolution
    y = oy + (height - 1 - py + 0.5) * resolution
    return x, y


def build_path_msg(start_xy, goal_xy, frame_id, stamp_msg):
    from nav_msgs.msg import Path
    from geometry_msgs.msg import PoseStamped

    path = Path()
    path.header.stamp = stamp_msg
    path.header.frame_id = frame_id

    for xy in (start_xy, goal_xy):
        ps = PoseStamped()
        ps.header.stamp = stamp_msg
        ps.header.frame_id = frame_id
        ps.pose.position.x = float(xy[0])
        ps.pose.position.y = float(xy[1])
        ps.pose.position.z = 0.0
        ps.pose.orientation.w = 1.0
        path.poses.append(ps)
    return path


def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument("--yaml", required=True, help="path to the map YAML")
    p.add_argument("--start", nargs=2, type=int, default=[10, 10], help="start pixel x y")
    p.add_argument("--goal", nargs=2, type=int, default=[100, 150], help="goal pixel x y")
    p.add_argument("--topic", default="/visual_path", help="topic to publish Path")
    p.add_argument("--frame", default="map", help="frame_id for the path")
    return p.parse_args()


def main():
    args = parse_args()

    try:
        image, resolution, origin = read_map_yaml(args.yaml)
        width, height = read_pgm_size(image)
    except Exception as e:
        print("Failed to read map files:", e, file=sys.stderr)
        sys.exit(1)

    start_px = (int(args.start[0]), int(args.start[1]))
    goal_px = (int(args.goal[0]), int(args.goal[1]))

    start_xy = pixel_to_map(start_px[0], start_px[1], height, resolution, origin)
    goal_xy = pixel_to_map(goal_px[0], goal_px[1], height, resolution, origin)

    # Lazy import rclpy to avoid requiring it during static analysis
    try:
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import QoSProfile, DurabilityPolicy
    except Exception as e:
        print("rclpy not available; print computed coordinates instead:\n")
        print("start (m):", start_xy)
        print("goal  (m):", goal_xy)
        print("image size (w,h):", width, height)
        sys.exit(0)

    class PathPublisher(Node):
        def __init__(self):
            super().__init__("visual_path_publisher")
            qos = QoSProfile(depth=1)
            qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
            self.pub = self.create_publisher(
                "nav_msgs/msg/Path" if False else type(build_path_msg((0,0),(0,0),args.frame,self.get_clock().now().to_msg())),
                args.topic,
                qos)

    # Note: create_publisher requires a message type class; import it now
    from nav_msgs.msg import Path as PathMsg

    rclpy.init()
    node = Node("visual_path_publisher")
    qos = QoSProfile(depth=1)
    qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
    pub = node.create_publisher(PathMsg, args.topic, qos)

    stamp = node.get_clock().now().to_msg()
    path_msg = build_path_msg(start_xy, goal_xy, args.frame, stamp)

    pub.publish(path_msg)
    node.get_logger().info(f"Published path on {args.topic} — start={start_xy} goal={goal_xy}")

    # Give DDS time to send the message
    rclpy.spin_once(node, timeout_sec=0.5)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
