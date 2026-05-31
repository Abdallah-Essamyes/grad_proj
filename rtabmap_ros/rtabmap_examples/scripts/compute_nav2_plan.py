#!/usr/bin/env python3
"""
Call Nav2's `ComputePathToPose` action to compute a global plan and publish it.

Usage:
  compute_nav2_plan.py --yaml map.yaml --start_px 0 0 --goal_px 255 255

The script converts pixel coordinates to map meters using the YAML `resolution`
and `origin`, calls the action (default `/planner_server/compute_path_to_pose`) and
publishes the returned `nav_msgs/Path` on `/visual_plan` for RViz.
"""
import argparse
import os
import sys


def read_map_yaml(yaml_path):
    import yaml
    with open(yaml_path, "r") as f:
        data = yaml.safe_load(f)
    image = data.get("image")
    resolution = float(data.get("resolution", 0.05))
    origin = data.get("origin", [0.0, 0.0, 0.0])
    if not os.path.isabs(image):
        image = os.path.join(os.path.dirname(yaml_path), image)
    return image, resolution, origin


def read_pgm_size(pgm_path):
    with open(pgm_path, "rb") as f:
        data = f.read(1024)
    parts = []
    for token in data.split():
        parts.append(token)
        if len(parts) >= 4:
            break
    if len(parts) < 4:
        raise RuntimeError(f"Can't parse PGM header: {pgm_path}")
    width = int(parts[1])
    height = int(parts[2])
    return width, height


def pixel_to_map(px, py, height, resolution, origin):
    ox, oy, oyaw = origin
    x = ox + (px + 0.5) * resolution
    y = oy + (height - 1 - py + 0.5) * resolution
    return x, y


def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument("--yaml", required=True)
    p.add_argument("--start-px", nargs=2, type=int, default=[0, 0])
    p.add_argument("--goal-px", nargs=2, type=int, default=[255, 255])
    p.add_argument("--action", default="/compute_path_to_pose")
    p.add_argument("--topic", default="/visual_plan")
    p.add_argument("--frame", default="map")
    p.add_argument("--tolerance", type=float, default=0.1, help="planner tolerance in meters")
    return p.parse_args()


def create_nav2_path_from_Pose(start_pose, goal_pose, action="/compute_path_to_pose",
                               topic="/visual_plan", frame="map", tolerance=0.1,
                               node_name="compute_nav2_plan_action_client"):
    """Send a ComputePathToPose action using already-constructed PoseStamped messages.

    This function performs the action-client interaction and publishes the returned
    nav_msgs/Path on `topic`.
    """
    try:
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import QoSProfile, DurabilityPolicy
        from nav_msgs.msg import Path
        from nav2_msgs.action import ComputePathToPose
        from rclpy.action import ActionClient
    except Exception:
        # rclpy not available: print poses and exit gracefully
        print("rclpy or nav2_msgs not available; start/goal poses:\n")
        try:
            print("start (m):", (start_pose.pose.position.x, start_pose.pose.position.y))
            print("goal  (m):", (goal_pose.pose.position.x, goal_pose.pose.position.y))
        except Exception:
            print("Provided start/goal are not PoseStamped objects")
        sys.exit(0)

    if not rclpy.ok():
        rclpy.init()

    node = Node(node_name)

    action_client = ActionClient(node, ComputePathToPose, action)
    if not action_client.wait_for_server(timeout_sec=5.0):
        node.get_logger().error(f"Action server {action} not available")
        rclpy.shutdown()
        sys.exit(1)

    goal_msg = ComputePathToPose.Goal()

    # Ensure headers/frame and stamps
    if not getattr(start_pose.header, 'frame_id', None):
        start_pose.header.frame_id = frame
    start_pose.header.stamp = node.get_clock().now().to_msg()
    if not getattr(goal_pose.header, 'frame_id', None):
        goal_pose.header.frame_id = frame
    goal_pose.header.stamp = node.get_clock().now().to_msg()

    goal_msg.start = start_pose
    goal_msg.goal = goal_pose

    # Print a literal `ros2 action send_goal` command that represents this goal
    try:
        import json
        payload_dict = {
            'start': {
                'header': {'frame_id': start_pose.header.frame_id},
                'pose': {
                    'position': {'x': float(start_pose.pose.position.x), 'y': float(start_pose.pose.position.y), 'z': 0.0},
                    'orientation': {'w': float(getattr(start_pose.pose.orientation, 'w', 1.0))}
                }
            },
            'goal': {
                'header': {'frame_id': goal_pose.header.frame_id},
                'pose': {
                    'position': {'x': float(goal_pose.pose.position.x), 'y': float(goal_pose.pose.position.y), 'z': 0.0},
                    'orientation': {'w': float(getattr(goal_pose.pose.orientation, 'w', 1.0))}
                }
            },
            'tolerance': float(tolerance)
        }
        yaml_payload = json.dumps(payload_dict)
    except Exception:
        # fallback to a safely formatted string
        yaml_payload = (
            "{{start: {{header: {{frame_id: '{frame}'}}, pose: {{position: {{x: {sx:.6f}, y: {sy:.6f}, z: 0.0}}, orientation: {{w: {sw:.6f}}}}}}}, "
            "goal: {{header: {{frame_id: '{frame}'}}, pose: {{position: {{x: {gx:.6f}, y: {gy:.6f}, z: 0.0}}, orientation: {{w: {gw:.6f}}}}}}}, tolerance: {tol:.6f}}}"
        ).format(frame=start_pose.header.frame_id,
                 sx=start_pose.pose.position.x, sy=start_pose.pose.position.y, sw=getattr(start_pose.pose.orientation, 'w', 1.0),
                 gx=goal_pose.pose.position.x, gy=goal_pose.pose.position.y, gw=getattr(goal_pose.pose.orientation, 'w', 1.0),
                 tol=float(tolerance))

    ros_cmd = f'ros2 action send_goal {action} nav2_msgs/action/ComputePathToPose "{yaml_payload}"'
    print("=== Literal command to send to Nav2 ===")
    print(ros_cmd)

    node.get_logger().info(f"Sending ComputePathToPose goal to {action} (tolerance={tolerance})")
    send_goal_future = action_client.send_goal_async(goal_msg)
    rclpy.spin_until_future_complete(node, send_goal_future, timeout_sec=10.0)
    if not send_goal_future.done():
        node.get_logger().error("Timed out waiting for goal response")
        rclpy.shutdown()
        sys.exit(1)

    goal_handle = send_goal_future.result()
    if not goal_handle.accepted:
        node.get_logger().error("Goal was rejected by action server")
        rclpy.shutdown()
        sys.exit(1)

    result_future = goal_handle.get_result_async()
    rclpy.spin_until_future_complete(node, result_future, timeout_sec=10.0)
    if not result_future.done():
        node.get_logger().error("Timed out waiting for result")
        rclpy.shutdown()
        sys.exit(1)

    result = result_future.result().result
    plan = getattr(result, "path", None)
    if plan is None:
        node.get_logger().error("Action result did not contain a path")
        rclpy.shutdown()
        sys.exit(1)

    if len(getattr(plan, 'poses', [])) == 0:
        print("target is unreachable")
        node.get_logger().warning("Target is unreachable: planner returned empty path")
        return

    qos = QoSProfile(depth=1)
    qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
    pub = node.create_publisher(Path, topic, qos)
    pub.publish(plan)
    node.get_logger().info(f"Published plan on {topic} with {len(plan.poses)} poses")


def create_nav2_path_from_pixels(yaml_path, start_px, goal_px, action="/planner_server/compute_path_to_pose",
                                 topic="/visual_plan", frame="map", tolerance=0.1):
    """Convert start/goal pixel coords to PoseStamped and call Pose-based planner.

    This function re-uses the map YAML/PGM reading and attempts to prefer live /map
    metadata when available.
    """
    try:
        image, res, origin = read_map_yaml(yaml_path)
        width, height = read_pgm_size(image)
    except Exception as e:
        print("Failed to read map yaml/pgm:", e, file=sys.stderr)
        sys.exit(1)

    # If a live /map topic exists, prefer its metadata (width/height/resolution/origin)
    def try_read_map_topic(timeout=2.0):
        try:
            import rclpy
            from rclpy.node import Node
            from nav_msgs.msg import OccupancyGrid
            import time
            rclpy.init()

        except Exception:
            return None

        node = Node("tmp_map_reader")
        holder = {}

        def cb(msg: OccupancyGrid):
            holder['msg'] = msg

        sub = node.create_subscription(OccupancyGrid, '/map', cb, 1)
        deadline = time.time() + timeout
        while time.time() < deadline and 'msg' not in holder:
            rclpy.spin_once(node, timeout_sec=0.1)

        msg = holder.get('msg')
        try:
            node.destroy_node()
        except Exception:
            pass
        rclpy.shutdown()
        if msg is None:
            return None
        info = msg.info
        return (info.width, info.height, info.resolution, (info.origin.position.x, info.origin.position.y, 0.0))

    live = try_read_map_topic()
    if live is not None:
        lw, lh, lres, lorigin = live
        print(f"Using live /map metadata: width={lw} height={lh} res={lres} origin={lorigin}")
        width, height, res, origin = lw, lh, lres, lorigin

    # validate and clamp pixel inputs to map bounds
    def clamp_pixel(px, py, w, h):
        cx = max(0, min(px, w - 1))
        cy = max(0, min(py, h - 1))
        return cx, cy

    start_px = (int(start_px[0]), int(start_px[1]))
    goal_px = (int(goal_px[0]), int(goal_px[1]))
    if not (0 <= start_px[0] < width and 0 <= start_px[1] < height):
        print(f"Warning: start pixel {start_px} outside map bounds (0..{width-1},0..{height-1}), clamping")
    if not (0 <= goal_px[0] < width and 0 <= goal_px[1] < height):
        print(f"Warning: goal pixel {goal_px} outside map bounds (0..{width-1},0..{height-1}), clamping")

    start_px = clamp_pixel(start_px[0], start_px[1], width, height)
    goal_px = clamp_pixel(goal_px[0], goal_px[1], width, height)

    start_m = pixel_to_map(start_px[0], start_px[1], height, res, origin)
    goal_m = pixel_to_map(goal_px[0], goal_px[1], height, res, origin)

    print(f"Start pixel -> meters: {start_px} -> {start_m}")
    print(f"Goal  pixel -> meters: {goal_px} -> {goal_m}")

    # compute map indices (world->map) used by planner and clamp if out-of-bounds
    def world_to_map_idx(world_xy, origin, resolution):
        ox, oy, _ = origin
        mx = int((world_xy[0] - ox) / resolution)
        my = int((world_xy[1] - oy) / resolution)
        return mx, my

    start_idx = world_to_map_idx(start_m, origin, res)
    goal_idx = world_to_map_idx(goal_m, origin, res)
    print(f"Start map idx: {start_idx}   Goal map idx: {goal_idx}   map size: ({width},{height})")

    def clamp_world_to_map_with_tol(world_xy, origin, resolution, w, h, tol):
        ox, oy, _ = origin
        mx = int((world_xy[0] - ox) / resolution)
        my = int((world_xy[1] - oy) / resolution)
        tol_cells = int(max(0, round(tol / resolution)))
        cmx = max(tol_cells, min(mx, w - 1 - tol_cells))
        cmy = max(tol_cells, min(my, h - 1 - tol_cells))
        wx = ox + (cmx + 0.5) * resolution
        wy = oy + (cmy + 0.5) * resolution
        return (wx, wy), (cmx, cmy)

    tol = float(tolerance)
    if not (0 <= start_idx[0] < width and 0 <= start_idx[1] < height):
        print("Clamping start to map bounds")
        start_m, start_idx = clamp_world_to_map_with_tol(start_m, origin, res, width, height, tol)
        print(f"Clamped start meters: {start_m} idx: {start_idx}")
    else:
        start_m, start_idx = clamp_world_to_map_with_tol(start_m, origin, res, width, height, tol)
    if not (0 <= goal_idx[0] < width and 0 <= goal_idx[1] < height):
        print("Clamping goal to map bounds")
        goal_m, goal_idx = clamp_world_to_map_with_tol(goal_m, origin, res, width, height, tol)
        print(f"Clamped goal meters: {goal_m} idx: {goal_idx}")
    else:
        goal_m, goal_idx = clamp_world_to_map_with_tol(goal_m, origin, res, width, height, tol)

    # Build PoseStamped messages and delegate to Pose-based function
    try:
        from geometry_msgs.msg import PoseStamped
    except Exception:
        print("geometry_msgs not available; computed coordinates:\n")
        print("start (m):", start_m)
        print("goal  (m):", goal_m)
        sys.exit(0)

    start = PoseStamped()
    start.header.frame_id = frame
    start.pose.position.x = float(start_m[0])
    start.pose.position.y = float(start_m[1])
    start.pose.orientation.w = 1.0

    goal = PoseStamped()
    goal.header.frame_id = frame
    goal.pose.position.x = float(goal_m[0])
    goal.pose.position.y = float(goal_m[1])
    goal.pose.orientation.w = 1.0

    return create_nav2_path_from_Pose(start, goal, action=action, topic=topic, frame=frame, tolerance=tolerance)


if __name__ == "__main__":
    create_nav2_path_from_pixels(
        yaml_path = "/home/ggsya/ros_ws/src/rtabmap_ros/rtabmap_examples/scripts/nav2_white_map.yaml",
        start_px = (10, 10),
        goal_px = (100, 50),
        action = "/compute_path_to_pose",
        topic = "/visual_plan",
        frame = "map",
        tolerance = 0.2,
    )
