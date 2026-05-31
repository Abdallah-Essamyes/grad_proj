#!/usr/bin/env python3
import math
import sys
import rclpy
from rclpy.node import Node
from nav2_msgs.srv import ComputePathToPose
from geometry_msgs.msg import PoseStamped


def yaw_from_quat(q):
    # simple yaw extraction for planar motion
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def angle_diff(a, b):
    d = a - b
    while d > math.pi:
        d -= 2*math.pi
    while d < -math.pi:
        d += 2*math.pi
    return d


class PlannerClient(Node):
    def __init__(self):
        super().__init__('nav2_path_to_steps')
        self.cli = self.create_client(ComputePathToPose, '/planner_server/compute_path_to_pose')
        if not self.cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('Planner service not available: /planner_server/compute_path_to_pose')
            raise RuntimeError('planner service not available')

    def build_pose(self, frame, x, y, yaw_deg=0.0):
        p = PoseStamped()
        p.header.frame_id = frame
        p.pose.position.x = float(x)
        p.pose.position.y = float(y)
        p.pose.position.z = 0.0
        yaw = math.radians(float(yaw_deg))
        p.pose.orientation.z = math.sin(yaw/2.0)
        p.pose.orientation.w = math.cos(yaw/2.0)
        return p

    def call_planner(self, start, goal, tol=0.5):
        req = ComputePathToPose.Request()
        req.start = start
        req.goal = goal
        req.tolerance = float(tol)
        fut = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut)
        if fut.result() is None:
            self.get_logger().error('Planner call failed')
            return None
        return fut.result().path


def path_to_steps(path, start_yaw=0.0):
    steps = []
    if not path.poses:
        return steps
    cur_x = path.poses[0].pose.position.x
    cur_y = path.poses[0].pose.position.y
    cur_yaw = start_yaw
    for ps in path.poses[1:]:
        nx = ps.pose.position.x
        ny = ps.pose.position.y
        dx = nx - cur_x
        dy = ny - cur_y
        dist = math.hypot(dx, dy)
        desired = math.atan2(dy, dx)
        yaw_change = angle_diff(desired, cur_yaw)
        # rotate first if needed
        if abs(math.degrees(yaw_change)) > 1.0:
            steps.append({'type': 'rotate', 'angle_deg': math.degrees(yaw_change)})
            cur_yaw = desired
        if dist > 0.005:
            steps.append({'type': 'forward', 'distance_m': dist})
            cur_x = nx
            cur_y = ny
    return steps


def main(args=None):
    rclpy.init(args=args)
    node = PlannerClient()

    # CLI args: start_x start_y goal_x goal_y [start_yaw_deg]
    if len(sys.argv) < 5:
        print('Usage: nav2_path_to_steps.py start_x start_y goal_x goal_y [start_yaw_deg]')
        return
    start_x, start_y, goal_x, goal_y = map(float, sys.argv[1:5])
    start_yaw = float(sys.argv[5]) if len(sys.argv) > 5 else 0.0

    start = node.build_pose('map', start_x, start_y, start_yaw)
    goal = node.build_pose('map', goal_x, goal_y, 0.0)

    path = node.call_planner(start, goal, tol=0.5)
    if path is None:
        print('No path returned')
        return

    print('\nReceived path with %d poses' % len(path.poses))
    for i, p in enumerate(path.poses[:10]):
        print('  pose %d: x=%.3f y=%.3f' % (i, p.pose.position.x, p.pose.position.y))
    if len(path.poses) > 10:
        print('  ... (%d poses total)' % len(path.poses))

    steps = path_to_steps(path, math.radians(start_yaw))
    print('\nConverted steps (rotate in deg, forward in cm):')
    for s in steps:
        if s['type'] == 'rotate':
            print('  rotate: %.2f deg' % s['angle_deg'])
        else:
            print('  forward: %.1f cm' % (s['distance_m'] * 100.0))

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
