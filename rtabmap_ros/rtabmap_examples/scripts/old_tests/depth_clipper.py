#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import numpy as np
import sys

class DepthClipper(Node):
    def __init__(self):
        super().__init__('depth_clipper')
        self.declare_parameter('cutoff_mm', 500)
        self.declare_parameter('input_topic', 'stereo/depth')
        self.declare_parameter('output_topic', 'stereo/depth_clipped')
        cutoff_mm = self.get_parameter('cutoff_mm').get_parameter_value().integer_value
        self.cutoff_mm = int(cutoff_mm)
        self.cutoff_m = self.cutoff_mm / 1000.0
        self.input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        self.output_topic = self.get_parameter('output_topic').get_parameter_value().string_value
        self.get_logger().info(f"DepthClipper: cutoff_mm={self.cutoff_mm}, input_topic={self.input_topic}, output_topic={self.output_topic}")

        self.pub = self.create_publisher(Image, self.output_topic, 10)
        self.sub = self.create_subscription(Image, self.input_topic, self.callback, 10)

    def callback(self, msg: Image):
        try:
            encoding = msg.encoding
            height = msg.height
            width = msg.width
            # Detect encoding and dtype
            if '32F' in encoding or '32FC1' in encoding or encoding.lower().startswith('32f'):
                arr = np.frombuffer(msg.data, dtype=np.float32).reshape((height, width))
                # assume units are meters
                mask = arr > self.cutoff_m
                if mask.any():
                    arr = arr.copy()
                    arr[mask] = 0.0
                    msg.data = arr.tobytes()
                    msg.step = arr.strides[0]
                    self.pub.publish(msg)
                else:
                    self.pub.publish(msg)
            elif '16U' in encoding or '16UC1' in encoding or encoding.lower().startswith('16u'):
                arr = np.frombuffer(msg.data, dtype=np.uint16).reshape((height, width))
                # assume units are millimeters
                mask = arr > self.cutoff_mm
                if mask.any():
                    arr = arr.copy()
                    arr[mask] = 0
                    msg.data = arr.tobytes()
                    msg.step = arr.strides[0]
                    self.pub.publish(msg)
                else:
                    self.pub.publish(msg)
            else:
                # Unknown encoding: try to treat as float32 fallback
                try:
                    arr = np.frombuffer(msg.data, dtype=np.float32).reshape((height, width))
                    mask = arr > self.cutoff_m
                    if mask.any():
                        arr = arr.copy()
                        arr[mask] = 0.0
                        msg.data = arr.tobytes()
                        msg.step = arr.strides[0]
                        self.pub.publish(msg)
                    else:
                        self.pub.publish(msg)
                except Exception:
                    self.get_logger().warn(f"Unsupported encoding: {encoding}; forwarding unchanged")
                    self.pub.publish(msg)
        except Exception as e:
            self.get_logger().error(f"Error in depth clipper callback: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = DepthClipper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
