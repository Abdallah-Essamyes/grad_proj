#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped
import sys
import math
import heapq
from rclpy.qos import QoSProfile, DurabilityPolicy

class AStarPlanner(Node):
    def __init__(self):
        super().__init__('map_astar_planner')
        self.map = None
        qos = QoSProfile(depth=1)
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.sub = self.create_subscription(OccupancyGrid, '/map', self.map_cb, qos)

    def map_cb(self, msg: OccupancyGrid):
        self.map = msg

    def wait_for_map(self, timeout=5.0):
        t0 = self.get_clock().now().nanoseconds/1e9
        while self.map is None:
            rclpy.spin_once(self, timeout_sec=0.1)
            if (self.get_clock().now().nanoseconds/1e9 - t0) > timeout:
                return False
        return True

    def world_to_map(self, x, y):
        origin = self.map.info.origin.position
        res = self.map.info.resolution
        mx = int((x - origin.x) / res)
        my = int((y - origin.y) / res)
        if mx < 0 or my < 0 or mx >= self.map.info.width or my >= self.map.info.height:
            return None
        return mx, my

    def map_to_world(self, mx, my):
        origin = self.map.info.origin.position
        res = self.map.info.resolution
        x = origin.x + (mx + 0.5) * res
        y = origin.y + (my + 0.5) * res
        return x, y

    def is_free(self, mx, my):
        idx = my * self.map.info.width + mx
        val = self.map.data[idx]
        # treat unknown (-1) as traversable for planning by default
        return val == 0 or val == -1

    def astar(self, start, goal):
        w = self.map.info.width
        h = self.map.info.height
        start_idx = start[1]*w + start[0]
        goal_idx = goal[1]*w + goal[0]
        open_set = []
        heapq.heappush(open_set, (0, start))
        came_from = {}
        gscore = {start: 0}
        def heur(p):
            return math.hypot(p[0]-goal[0], p[1]-goal[1])
        dirs = [(1,0),(-1,0),(0,1),(0,-1),(1,1),(1,-1),(-1,1),(-1,-1)]
        while open_set:
            _, current = heapq.heappop(open_set)
            if current == goal:
                path = []
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                path.append(start)
                path.reverse()
                return path
            for d in dirs:
                nx = current[0]+d[0]
                ny = current[1]+d[1]
                if nx<0 or ny<0 or nx>=w or ny>=h: continue
                if not self.is_free(nx, ny): continue
                tentative = gscore[current] + math.hypot(d[0], d[1])
                neighbor = (nx, ny)
                if tentative < gscore.get(neighbor, 1e9):
                    came_from[neighbor] = current
                    gscore[neighbor] = tentative
                    heapq.heappush(open_set, (tentative + heur(neighbor), neighbor))
        return None

    def compute_and_print(self, sx, sy, gx, gy, start_yaw_deg=0.0):
        if not self.wait_for_map(10.0):
            print('No /map received')
            return
        s = self.world_to_map(sx, sy)
        g = self.world_to_map(gx, gy)
        if s is None or g is None:
            print('Start/goal out of map bounds')
            return
        if not self.is_free(s[0], s[1]) or not self.is_free(g[0], g[1]):
            print('Start or goal inside obstacle')
            return
        path = self.astar(s,g)
        if path is None:
            print('No path found')
            return
        # build nav_msgs/Path
        p = Path()
        p.header.frame_id = 'map'
        p.header.stamp = self.get_clock().now().to_msg()
        for (mx,my) in path:
            x,y = self.map_to_world(mx,my)
            ps = PoseStamped()
            ps.header.frame_id = 'map'
            ps.pose.position.x = x
            ps.pose.position.y = y
            ps.pose.position.z = 0.0
            ps.pose.orientation.w = 1.0
            p.poses.append(ps)
        print(f'Received path with {len(p.poses)} poses')
        # print first 10
        for i,ps in enumerate(p.poses[:10]):
            print(f' pose {i}: x={ps.pose.position.x:.3f} y={ps.pose.position.y:.3f}')
        if len(p.poses)>10:
            print(f' ... ({len(p.poses)} poses total)')
        # convert to steps
        cur_yaw = math.radians(start_yaw_deg)
        cur_x = sx
        cur_y = sy
        steps = []
        def angdiff(a,b):
            d = a-b
            while d>math.pi: d-=2*math.pi
            while d<-math.pi: d+=2*math.pi
            return d
        for ps in p.poses[1:]:
            nx = ps.pose.position.x
            ny = ps.pose.position.y
            dx = nx - cur_x
            dy = ny - cur_y
            dist = math.hypot(dx,dy)
            desired = math.atan2(dy,dx)
            yaw_change = angdiff(desired, cur_yaw)
            if abs(math.degrees(yaw_change))>1.0:
                steps.append({'type':'rotate','angle_deg':math.degrees(yaw_change)})
                cur_yaw = desired
            if dist>0.005:
                steps.append({'type':'forward','distance_cm':dist*100.0})
                cur_x = nx
                cur_y = ny
        print('\nConverted steps:')
        for s in steps:
            if s['type']=='rotate':
                print(f" rotate {s['angle_deg']:.2f} deg")
            else:
                print(f" forward {s['distance_cm']:.1f} cm")


def main(argv=None):
    rclpy.init()
    node = AStarPlanner()
    if len(sys.argv) < 5:
        print('Usage: map_astar_planner.py start_x start_y goal_x goal_y [start_yaw_deg]')
        return
    sx,sy,gx,gy = map(float, sys.argv[1:5])
    syaw = float(sys.argv[5]) if len(sys.argv)>5 else 0.0
    node.compute_and_print(sx,sy,gx,gy,syaw)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
