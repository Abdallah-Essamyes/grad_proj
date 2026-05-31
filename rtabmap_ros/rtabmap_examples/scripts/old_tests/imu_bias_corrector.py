#!/usr/bin/env python3
"""
IMU Roll/Pitch Bias Corrector

Applies a user-adjustable roll/pitch rotation bias to all IMU messages.
Used to manually correct residual tilt errors that remain after gravity leveling
(e.g. floor not perfectly level, persistent roll/pitch drift in the map).

Pipeline:
  /imu/leveled → THIS NODE → /imu/corrected → madgwick → /imu/data → RTAB-Map

─────────────────────────────────────────────────────────────────
USAGE — adjust bias INCREMENTALLY (numbers ADD to current bias):
─────────────────────────────────────────────────────────────────

  Nudge roll by +1.5° and pitch by -2°:
    ros2 topic pub --once /imu_bias/adjust geometry_msgs/msg/Vector3 "{x: 1.5, y: -2.0, z: 0.0}"

  x = delta_roll_deg   (positive = tilt right)
  y = delta_pitch_deg  (positive = tilt forward)
  z = ignored

  See current accumulated bias at any time:
    ros2 topic echo /imu_bias/current
    (x = total_roll_deg, y = total_pitch_deg)

  Reset bias to zero:
    ros2 topic pub --once /imu_bias/adjust geometry_msgs/msg/Vector3 "{x: 0.0, y: 0.0, z: -999.0}"
    (z = -999 is the reset sentinel)
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3

RESET_SENTINEL = -999.0


def _quat_from_roll_pitch(roll_rad: float, pitch_rad: float):
    """Build quaternion (x,y,z,w) from roll + pitch, yaw=0 (intrinsic XY)."""
    cr, sr = math.cos(roll_rad * 0.5), math.sin(roll_rad * 0.5)
    cp, sp = math.cos(pitch_rad * 0.5), math.sin(pitch_rad * 0.5)
    return (
        sr * cp,   # qx
        cr * sp,   # qy
        -sr * sp,  # qz
        cr * cp,   # qw
    )


def _rotate_vector_by_quat(qx, qy, qz, qw, vx, vy, vz):
    """Rotate vector v by unit quaternion q: v' = q * v * conj(q)."""
    t0 = qw*vx + qy*vz - qz*vy
    t1 = qw*vy - qx*vz + qz*vx
    t2 = qw*vz + qx*vy - qy*vx
    t3 = -qx*vx - qy*vy - qz*vz
    rx = t0*qw + t3*(-qx) + t1*(-qz) - t2*(-qy)
    ry = t1*qw + t3*(-qy) + t2*(-qx) - t0*(-qz)
    rz = t2*qw + t3*(-qz) + t0*(-qy) - t1*(-qx)
    return rx, ry, rz


def _quat_multiply(aq, bq):
    """Hamilton product aq * bq, each (x,y,z,w)."""
    ax, ay, az, aw = aq
    bx, by, bz, bw = bq
    return (
        aw*bx + ax*bw + ay*bz - az*by,
        aw*by - ax*bz + ay*bw + az*bx,
        aw*bz + ax*by - ay*bx + az*bw,
        aw*bw - ax*bx - ay*by - az*bz,
    )


def _rotate_covariance(cov, qx, qy, qz, qw):
    """Transform 3×3 covariance (9-element row-major) by rotation q: C' = R*C*R^T."""
    x2, y2, z2 = qx*qx, qy*qy, qz*qz
    xy, xz, yz = qx*qy, qx*qz, qy*qz
    wx, wy, wz = qw*qx, qw*qy, qw*qz
    R = [
        1-2*(y2+z2), 2*(xy-wz),   2*(xz+wy),
        2*(xy+wz),   1-2*(x2+z2), 2*(yz-wx),
        2*(xz-wy),   2*(yz+wx),   1-2*(x2+y2),
    ]
    C = list(cov)
    RC = [0.0]*9
    for i in range(3):
        for j in range(3):
            for k in range(3):
                RC[i*3+j] += R[i*3+k] * C[k*3+j]
    RCRt = [0.0]*9
    for i in range(3):
        for j in range(3):
            for k in range(3):
                RCRt[i*3+j] += RC[i*3+k] * R[j*3+k]
    return RCRt


class IMUBiasCorrector(Node):
    """
    Applies an accumulating roll/pitch bias correction to /imu/leveled.

    Subscribes:
      /imu/leveled          — gravity-leveled IMU from imu_gravity_leveler
      /imu_bias/adjust      — geometry_msgs/Vector3: x=delta_roll_deg, y=delta_pitch_deg
                              send z=-999 to reset bias to zero
    Publishes:
      /imu/corrected        — bias-corrected IMU (feed into madgwick)
      /imu_bias/current     — geometry_msgs/Vector3: current accumulated bias in degrees
    """

    def __init__(self):
        super().__init__('imu_bias_corrector')

        self.declare_parameter('init_roll_deg', 0.0)
        self.declare_parameter('init_pitch_deg', 0.0)

        init_roll = self.get_parameter('init_roll_deg').value
        init_pitch = self.get_parameter('init_pitch_deg').value

        self._roll_rad = math.radians(init_roll)
        self._pitch_rad = math.radians(init_pitch)
        self._corr_q = _quat_from_roll_pitch(self._roll_rad, self._pitch_rad)

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

        self._sub_imu = self.create_subscription(Imu, '/imu/leveled', self._imu_cb, imu_qos)
        self._sub_adj = self.create_subscription(Vector3, '/imu_bias/adjust', self._adjust_cb, 10)
        self._pub_imu = self.create_publisher(Imu, '/imu/corrected', imu_qos)
        self._pub_cur = self.create_publisher(Vector3, '/imu_bias/current', latch_qos)

        self._publish_current()
        self.get_logger().info(
            f'IMU Bias Corrector started. Initial bias: roll={init_roll:+.3f}°, pitch={init_pitch:+.3f}°. '
            f'Adjust with: ros2 topic pub --once /imu_bias/adjust '
            f'geometry_msgs/msg/Vector3 "{{x: DELTA_ROLL_DEG, y: DELTA_PITCH_DEG, z: 0.0}}"'
        )

    def _adjust_cb(self, msg: Vector3):
        if msg.z == RESET_SENTINEL:
            self._roll_rad = 0.0
            self._pitch_rad = 0.0
            self.get_logger().info('Bias RESET to 0°, 0°')
        else:
            self._roll_rad += math.radians(msg.x)
            self._pitch_rad += math.radians(msg.y)
            self.get_logger().info(
                f'Bias adjusted by ({msg.x:+.2f}°, {msg.y:+.2f}°) → '
                f'total: ({math.degrees(self._roll_rad):+.3f}°, '
                f'{math.degrees(self._pitch_rad):+.3f}°)'
            )
        self._corr_q = _quat_from_roll_pitch(self._roll_rad, self._pitch_rad)
        self._publish_current()

    def _publish_current(self):
        v = Vector3()
        v.x = math.degrees(self._roll_rad)
        v.y = math.degrees(self._pitch_rad)
        v.z = 0.0
        self._pub_cur.publish(v)

    def _imu_cb(self, msg: Imu):
        qx, qy, qz, qw = self._corr_q

        # Identity bias — pass through with no allocation
        if qw >= 0.9999999:
            self._pub_imu.publish(msg)
            return

        out = Imu()
        out.header = msg.header

        lx, ly, lz = _rotate_vector_by_quat(
            qx, qy, qz, qw,
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z,
        )
        out.linear_acceleration.x = lx
        out.linear_acceleration.y = ly
        out.linear_acceleration.z = lz

        wx, wy, wz = _rotate_vector_by_quat(
            qx, qy, qz, qw,
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z,
        )
        out.angular_velocity.x = wx
        out.angular_velocity.y = wy
        out.angular_velocity.z = wz

        out.linear_acceleration_covariance = _rotate_covariance(
            msg.linear_acceleration_covariance, qx, qy, qz, qw)
        out.angular_velocity_covariance = _rotate_covariance(
            msg.angular_velocity_covariance, qx, qy, qz, qw)

        if (msg.orientation.w != 0.0 or msg.orientation.x != 0.0 or
                msg.orientation.y != 0.0 or msg.orientation.z != 0.0):
            rx, ry, rz, rw = _quat_multiply(
                (qx, qy, qz, qw),
                (msg.orientation.x, msg.orientation.y,
                 msg.orientation.z, msg.orientation.w),
            )
            out.orientation.x = rx
            out.orientation.y = ry
            out.orientation.z = rz
            out.orientation.w = rw
        else:
            out.orientation = msg.orientation

        out.orientation_covariance = _rotate_covariance(
            msg.orientation_covariance, qx, qy, qz, qw)

        self._pub_imu.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = IMUBiasCorrector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
