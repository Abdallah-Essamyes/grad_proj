#!/usr/bin/env python3

import argparse
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from nav2_msgs.srv import GetCostmap
from nav_msgs.msg import OccupancyGrid

publisher_topic = '/global_costmap/costmap'
service_name = '/global_costmap/get_costmap'


def publish_costmap_once(node, pub, client):
    if not client.wait_for_service(timeout_sec=2.0):
        node.get_logger().warning('GetCostmap service not available')
        return False

    req = GetCostmap.Request()
    future = client.call_async(req)
    rclpy.spin_until_future_complete(node, future, timeout_sec=2.0)
    if not future.done():
        node.get_logger().warning('GetCostmap call timed out')
        return False

    res = future.result()
    if not res or not getattr(res, 'map', None):
        node.get_logger().warning('Empty costmap response')
        return False

    cm = res.map
    og = OccupancyGrid()
    try:
        og.header = cm.header
        md = cm.metadata
        og.info.width = int(getattr(md, 'size_x', 0))
        og.info.height = int(getattr(md, 'size_y', 0))
        og.info.resolution = float(getattr(md, 'resolution', 0.05))
        if hasattr(md, 'origin') and md.origin:
            og.info.origin = md.origin

        data = []
        for v in cm.data:
            try:
                vi = int(v)
            except Exception:
                vi = 0
            if vi < 0:
                data.append(-1)
            else:
                val = int((vi / 255.0) * 100)
                if val < 0:
                    val = 0
                if val > 100:
                    val = 100
                data.append(val)
        og.data = data

        pub.publish(og)
        #node.get_logger().info('Published costmap width=%d height=%d to %s' % (og.info.width, og.info.height, publisher_topic))
        return True
    except Exception as e:
        node.get_logger().error('Failed to convert/publish costmap: %s' % e)
        return False


def main(args=None):
    parser = argparse.ArgumentParser()
    parser.add_argument('--frequency', type=float, default=1.0)

    parsed, unknown = parser.parse_known_args()
    freq = parsed.frequency
    if freq <= 0:
        print('frequency must be > 0')
        return

    period = 1.0 / float(freq)

    rclpy.init(args=args)
    node = Node('get_costmap_client')

    qos = QoSProfile(depth=10)
    qos.reliability = ReliabilityPolicy.RELIABLE
    qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
    pub = node.create_publisher(OccupancyGrid, publisher_topic, qos)

    client = node.create_client(GetCostmap, service_name)

    try:
        while rclpy.ok():
            start = time.time()
            publish_costmap_once(node, pub, client)
            # keep loop frequency over entire read+publish cycle
            elapsed = time.time() - start
            to_sleep = period - elapsed
            if to_sleep > 0:
                # allow ROS callbacks while sleeping
                rclpy.spin_once(node, timeout_sec=min(0.1, to_sleep))
                time.sleep(max(0.0, to_sleep - 0.1))
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
