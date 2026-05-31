#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav2_msgs.srv import GetCostmap


def main():
    rclpy.init()
    node = Node('call_get_costmap')
    client = node.create_client(GetCostmap, '/global_costmap/get_costmap')
    if not client.wait_for_service(timeout_sec=5.0):
        print('service /global_costmap/get_costmap not available')
        return
    req = GetCostmap.Request()
    future = client.call_async(req)
    rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
    if future.done():
        res = future.result()
        print('Service response type:', type(res))
        # Dump available attributes for inspection
        try:
            attrs = [a for a in dir(res) if not a.startswith('_')]
            print('Response attrs:', attrs)
        except Exception as e:
            print('Failed to list response attrs:', e)
        # Try common fields
        if hasattr(res, 'map') and res.map:
            m = res.map
            print('res.map type:', type(m))
            try:
                m_attrs = [a for a in dir(m) if not a.startswith('_')]
                print('res.map attrs sample:', m_attrs[:40])
            except Exception as e:
                print('Failed listing map attrs:', e)
            if hasattr(m, 'metadata') and m.metadata:
                md = m.metadata
                print('metadata attrs:', [a for a in dir(md) if not a.startswith('_')][:40])
                # try common metadata fields
                try:
                    sx = getattr(md, 'size_x', None)
                    sy = getattr(md, 'size_y', None)
                    reso = getattr(md, 'resolution', None)
                    print('metadata size_x,size_y,resolution:', sx, sy, reso)
                except Exception as e:
                    print('Failed to read metadata fields:', e)
        if hasattr(res, 'costmap') and res.costmap:
            cm = res.costmap
            print('Got nav2 Costmap object, attributes:', [a for a in dir(cm) if not a.startswith('_')][:40])
    else:
        print('Service call timed out')
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
