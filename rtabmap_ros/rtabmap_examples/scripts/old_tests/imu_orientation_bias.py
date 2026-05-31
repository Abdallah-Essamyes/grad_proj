#!/usr/bin/env python3
"""
IMU Orientation Bias — real-time roll/pitch bias AFTER Madgwick

Subscribes to /imu/madgwick_out, applies a controllable roll/pitch rotation
directly to the orientation quaternion, and re-publishes to /imu/data.

WHY THIS WORKS:
  RTAB-Map's Optimizer/GravitySigma uses the orientation quaternion to constrain
  every pose in the graph toward the "gravity direction" the IMU reports.
  As keyframes accumulate the optimizer re-runs and rotates the ENTIRE point
  cloud (and occupancy grid) toward the orientation we feed it here.

  Operating post-Madgwick means changes are IMMEDIATE — no Madgwick convergence
  delay.  The optimizer responds within one detection cycle (~1–2 s at default
  rates) and you can watch the room rotate live.

INTERFACE:
  Nudge roll  +2°:  ros2 topic pub --once /imu_bias/adjust geometry_msgs/msg/Vector3 "{x: 2.0, y: 0.0, z: 0.0}"
  Nudge pitch +1°:  ros2 topic pub --once /imu_bias/adjust geometry_msgs/msg/Vector3 "{x: 0.0, y: 1.0, z: 0.0}"
  Check current:    ros2 topic echo /imu_bias/current
  Reset to zero:    ros2 topic pub --once /imu_bias/adjust geometry_msgs/msg/Vector3 "{x: 0.0, y: 0.0, z: -999.0}"

  x = delta_roll_deg    y = delta_pitch_deg    z = -999 → reset

  If a nudge tilts the wrong way, negate the sign — rotation direction depends
  on how your camera is mounted relative to ENU world frame.

PARAMETERS:
  init_roll_deg   (default 0.0) — applied before first IMU message
  init_pitch_deg  (default 0.0) — applied before first IMU message

TUNING WORKFLOW:
  1. Launch with init values = 0. Run the robot, watch the map stabilize.
  2. If the occupancy grid is still tilted, nudge via /imu_bias/adjust.
  3. ros2 topic echo /imu_bias/current — note accepted values.
  4. Copy those values to init_roll_deg / init_pitch_deg in launch file.
  5. Restart — map builds correctly from frame 1.
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3

RESET_SENTINEL = -999.0


def _quat_from_roll_pitch(roll_rad: float, pitch_rad: float):
    """Returns (qx, qy, qz, qw) for roll + pitch, yaw = 0."""
    cr, sr = math.cos(roll_rad * 0.5), math.sin(roll_rad * 0.5)
    cp, sp = math.cos(pitch_rad * 0.5), math.sin(pitch_rad * 0.5)
    return sr * cp, cr * sp, -sr * sp, cr * cp   # qx, qy, qz, qw


def _quat_mul(a, b):
    """Hamilton product of (qx,qy,qz,qw) quaternions a × b."""
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw*bx + ax*bw + ay*bz - az*by,
        aw*by - ax*bz + ay*bw + az*bx,
        aw*bz + ax*by - ay*bx + az*bw,
        aw*bw - ax*bx - ay*by - az*bz,
    )


class ImuOrientationBias(Node):

    def __init__(self):
        super().__init__('imu_orientation_bias')

        self.declare_parameter('init_roll_deg', 0.0)
        self.declare_parameter('init_pitch_deg', 0.0)

        self._roll_rad  = math.radians(self.get_parameter('init_roll_deg').value)
        self._pitch_rad = math.radians(self.get_parameter('init_pitch_deg').value)
        self._bq        = _quat_from_roll_pitch(self._roll_rad, self._pitch_rad)  # cached

        imu_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=200,
        )
        latch_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self._pub   = self.create_publisher(Imu,     '/imu/data',         imu_qos)
        self._cpub  = self.create_publisher(Vector3, '/imu_bias/current', latch_qos)
        self._sub   = self.create_subscription(Imu,     '/imu/madgwick_out', self._imu_cb,  imu_qos)
        self._adj   = self.create_subscription(Vector3, '/imu_bias/adjust',  self._adj_cb,  10)

        self._publish_current()
        self.get_logger().info(
            f'IMU orientation bias ready  '
            f'roll={math.degrees(self._roll_rad):.2f}°  '
            f'pitch={math.degrees(self._pitch_rad):.2f}°'
        )

    # ────────────────────── helpers ──────────────────────

    def _publish_current(self):
        msg = Vector3()
        msg.x = math.degrees(self._roll_rad)
        msg.y = math.degrees(self._pitch_rad)
        msg.z = 0.0
        self._cpub.publish(msg)

    def _refresh_bias(self):
        self._bq = _quat_from_roll_pitch(self._roll_rad, self._pitch_rad)

    # ────────────────────── callbacks ────────────────────

    def _adj_cb(self, msg: Vector3):
        if msg.z == RESET_SENTINEL:
            self._roll_rad  = 0.0
            self._pitch_rad = 0.0
            self.get_logger().info('Bias reset to zero')
        else:
            self._roll_rad  += math.radians(msg.x)
            self._pitch_rad += math.radians(msg.y)
            self.get_logger().info(
                f'Bias  roll={math.degrees(self._roll_rad):.2f}°  '
                f'pitch={math.degrees(self._pitch_rad):.2f}°'
            )
        self._refresh_bias()
        self._publish_current()

    def _imu_cb(self, msg: Imu):
        # Fast path: identity bias → forward unchanged
        bx, by, bz, bw = self._bq
        if bx == 0.0 and by == 0.0 and bz == 0.0 and bw == 1.0:
            self._pub.publish(msg)
            return

        # Pre-multiply: q_out = q_bias × q_madgwick
        # This shifts the world/gravity frame RTAB-Map's optimizer converges toward.
        ox, oy, oz, ow = msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w
        rx, ry, rz, rw = _quat_mul((bx, by, bz, bw), (ox, oy, oz, ow))

        out = Imu()
        out.header = msg.header
        out.orientation.x = rx
        out.orientation.y = ry
        out.orientation.z = rz
        out.orientation.w = rw
        out.orientation_covariance = msg.orientation_covariance
        # accel/gyro stay in sensor frame — only the orientation (world→sensor mapping) changes
        out.linear_acceleration              = msg.linear_acceleration
        out.linear_acceleration_covariance   = msg.linear_acceleration_covariance
        out.angular_velocity                 = msg.angular_velocity
        out.angular_velocity_covariance      = msg.angular_velocity_covariance
        self._pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = ImuOrientationBias()
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
