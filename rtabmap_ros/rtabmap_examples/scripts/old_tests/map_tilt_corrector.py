#!/usr/bin/env python3
"""
Map Tilt Corrector — real-time visual roll/pitch correction

Publishes a TF  world → map  with a controllable roll/pitch rotation.
Set RViz2 fixed frame to "world" and the entire map (point cloud, occupancy
grid, trajectory) rotates instantly when you nudge the values.

Use this to FIND the right correction angle visually, then copy the final
values into imu_bias_corrector init_roll_deg / init_pitch_deg and restart.

─────────────────────────────────────────────────────────────────
INTERFACE — same style as imu_bias_corrector (incremental deltas):
─────────────────────────────────────────────────────────────────

  Nudge roll right by 1°:
    ros2 topic pub --once /map_tilt/adjust geometry_msgs/msg/Vector3 "{x: 1.0, y: 0.0, z: 0.0}"

  Nudge pitch forward by 2°:
    ros2 topic pub --once /map_tilt/adjust geometry_msgs/msg/Vector3 "{x: 0.0, y: 2.0, z: 0.0}"

  Check current values:
    ros2 topic echo /map_tilt/current

  Reset to zero:
    ros2 topic pub --once /map_tilt/adjust geometry_msgs/msg/Vector3 "{x: 0.0, y: 0.0, z: -999.0}"

  x = delta_roll_deg   (positive = tilt world right relative to map)
  y = delta_pitch_deg  (positive = tilt world forward relative to map)
  z = -999             = reset

─────────────────────────────────────────────────────────────────
RViz2 setup:  Global Options → Fixed Frame → set to  world
─────────────────────────────────────────────────────────────────
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from geometry_msgs.msg import Vector3, TransformStamped
from tf2_ros import TransformBroadcaster

RESET_SENTINEL = -999.0


def _quat_from_roll_pitch(roll_rad: float, pitch_rad: float):
    """Quaternion (x,y,z,w) for roll + pitch, yaw=0."""
    cr, sr = math.cos(roll_rad * 0.5), math.sin(roll_rad * 0.5)
    cp, sp = math.cos(pitch_rad * 0.5), math.sin(pitch_rad * 0.5)
    return sr*cp, cr*sp, -sr*sp, cr*cp


class MapTiltCorrector(Node):
    """
    Publishes world → map TF with adjustable roll/pitch for real-time
    visual correction of map tilt in RViz2. No data is modified — purely
    a view correction for finding the right bias values to hardcode.
    """

    def __init__(self):
        super().__init__('map_tilt_corrector')

        self.declare_parameter('init_roll_deg', 0.0)
        self.declare_parameter('init_pitch_deg', 0.0)
        self.declare_parameter('publish_rate_hz', 50.0)

        init_roll  = self.get_parameter('init_roll_deg').value
        init_pitch = self.get_parameter('init_pitch_deg').value
        rate_hz    = self.get_parameter('publish_rate_hz').value

        self._roll_rad  = math.radians(init_roll)
        self._pitch_rad = math.radians(init_pitch)

        latch_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self._tf_broadcaster = TransformBroadcaster(self)
        self._sub_adj = self.create_subscription(
            Vector3, '/map_tilt/adjust', self._adjust_cb, 10)
        self._pub_cur = self.create_publisher(Vector3, '/map_tilt/current', latch_qos)
        self._timer = self.create_timer(1.0 / rate_hz, self._publish_tf)

        self._publish_current()
        self.get_logger().info(
            f'Map Tilt Corrector started. Initial: roll={init_roll:+.2f}°, pitch={init_pitch:+.2f}°. '
            f'Set RViz2 fixed frame to "world". '
            f'Adjust: ros2 topic pub --once /map_tilt/adjust geometry_msgs/msg/Vector3 '
            f'"{{x: DELTA_ROLL, y: DELTA_PITCH, z: 0.0}}"'
        )

    def _adjust_cb(self, msg: Vector3):
        if msg.z == RESET_SENTINEL:
            self._roll_rad  = 0.0
            self._pitch_rad = 0.0
            self.get_logger().info('Map tilt RESET to 0°, 0°')
        else:
            self._roll_rad  += math.radians(msg.x)
            self._pitch_rad += math.radians(msg.y)
            self.get_logger().info(
                f'Map tilt adjusted by ({msg.x:+.2f}°, {msg.y:+.2f}°) → '
                f'total: roll={math.degrees(self._roll_rad):+.3f}°, '
                f'pitch={math.degrees(self._pitch_rad):+.3f}°'
            )
        self._publish_current()

    def _publish_current(self):
        v = Vector3()
        v.x = math.degrees(self._roll_rad)
        v.y = math.degrees(self._pitch_rad)
        self._pub_cur.publish(v)

    def _publish_tf(self):
        qx, qy, qz, qw = _quat_from_roll_pitch(self._roll_rad, self._pitch_rad)

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id  = 'map'
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        self._tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = MapTiltCorrector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
