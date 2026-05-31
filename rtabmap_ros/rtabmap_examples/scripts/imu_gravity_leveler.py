#!/usr/bin/env python3
"""
IMU Gravity Leveler Node

At startup, collects the first N accelerometer readings while the sensor is
static, averages them to find the true gravity vector (which includes floor
tilt + IMU mounting bias), then applies a static corrective rotation to ALL
subsequent IMU messages so that Madgwick sees perfect Z-up gravity.

This eliminates:
  - Physical floor tilt relative to Earth's Z axis
  - IMU mounting offset (residual after axis transformer)
  - Static accelerometer bias

Pipeline:
  /imu (oak-d-base-frame, axis-corrected) → THIS NODE → /imu/leveled
  /imu/leveled → madgwick → /imu/data → RTAB-Map

The corrective rotation is computed once from the averaged gravity vector and
then applied as a constant quaternion rotation — zero per-message allocation.
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Imu


def _normalize(x, y, z):
    n = math.sqrt(x*x + y*y + z*z)
    if n < 1e-10:
        return 0.0, 0.0, 1.0
    return x/n, y/n, z/n


def _rotation_quat_from_vec_to_vec(ax, ay, az, bx, by, bz):
    """
    Quaternion (qx, qy, qz, qw) that rotates unit vector a → unit vector b.
    Uses half-angle formula: q = normalize(a+b, cross(a,b)).
    """
    # half-vector
    hx, hy, hz = ax + bx, ay + by, az + bz
    hn = math.sqrt(hx*hx + hy*hy + hz*hz)
    if hn < 1e-10:
        # 180-degree rotation — pick any perpendicular axis
        if abs(ax) < 0.9:
            px, py, pz = 1.0, 0.0, 0.0
        else:
            px, py, pz = 0.0, 1.0, 0.0
        # cross(a, p)
        cx = ay*pz - az*py
        cy = az*px - ax*pz
        cz = ax*py - ay*px
        cn = math.sqrt(cx*cx + cy*cy + cz*cz)
        return cx/cn, cy/cn, cz/cn, 0.0
    hx /= hn; hy /= hn; hz /= hn
    # cross(a, h) = sin(angle/2) * rotation_axis * sin(angle/2) => but half-vector gives:
    # qw = dot(a, h), q_xyz = cross(a, h)
    qw = ax*hx + ay*hy + az*hz
    qx = ay*hz - az*hy
    qy = az*hx - ax*hz
    qz = ax*hy - ay*hx
    # already normalized because a and h are unit vectors
    return qx, qy, qz, qw


def _quat_multiply(aq, bq):
    """Hamilton product: aq * bq, each (x, y, z, w)."""
    ax, ay, az, aw = aq
    bx, by, bz, bw = bq
    return (
        aw*bx + ax*bw + ay*bz - az*by,
        aw*by - ax*bz + ay*bw + az*bx,
        aw*bz + ax*by - ay*bx + az*bw,
        aw*bw - ax*bx - ay*by - az*bz,
    )


def _rotate_vector_by_quat(qx, qy, qz, qw, vx, vy, vz):
    """Rotate vector v by quaternion q: v' = q * v * q^-1."""
    # q * v (treating v as pure quaternion)
    t0 = qw*vx + qy*vz - qz*vy
    t1 = qw*vy - qx*vz + qz*vx
    t2 = qw*vz + qx*vy - qy*vx
    t3 = -qx*vx - qy*vy - qz*vz
    # result * q^-1 (conjugate since unit quat)
    rx = t0*qw + t3*(-qx) + t1*(-qz) - t2*(-qy)
    ry = t1*qw + t3*(-qy) + t2*(-qx) - t0*(-qz)
    rz = t2*qw + t3*(-qz) + t0*(-qy) - t1*(-qx)
    return rx, ry, rz


def _rotate_covariance(cov, qx, qy, qz, qw):
    """
    Transform 3x3 covariance (9-element row-major list) by rotation q:
      C_out = R * C * R^T
    where R is the rotation matrix equivalent of quaternion q.
    """
    # Build R from quaternion
    x2, y2, z2 = qx*qx, qy*qy, qz*qz
    xy, xz, yz = qx*qy, qx*qz, qy*qz
    wx, wy, wz = qw*qx, qw*qy, qw*qz
    R = [
        1-2*(y2+z2),  2*(xy-wz),   2*(xz+wy),
        2*(xy+wz),    1-2*(x2+z2), 2*(yz-wx),
        2*(xz-wy),    2*(yz+wx),   1-2*(x2+y2),
    ]
    # C as flat row-major
    C = list(cov)
    # RC = R * C  (3x3 @ 3x3)
    RC = [0.0]*9
    for i in range(3):
        for j in range(3):
            for k in range(3):
                RC[i*3+j] += R[i*3+k] * C[k*3+j]
    # RCRt = RC * R^T
    RCRt = [0.0]*9
    for i in range(3):
        for j in range(3):
            for k in range(3):
                RCRt[i*3+j] += RC[i*3+k] * R[j*3+k]  # R^T[k,j] = R[j,k]
    return RCRt


class IMUGravityLeveler(Node):
    """
    Collects N IMU samples at startup, computes static gravity correction,
    then continuously publishes gravity-leveled IMU data.

    Subscribes:  /imu  (sensor_msgs/Imu, already axis-corrected to oak-d-base-frame)
    Publishes:   /imu/leveled  (sensor_msgs/Imu, gravity-leveled)

    Parameters:
      num_calibration_samples (int, default 50): Samples averaged for calibration.
      Expected gravity magnitude (m/s^2) is used only for logging.
    """

    def __init__(self):
        super().__init__('imu_gravity_leveler')

        self.declare_parameter('num_calibration_samples', 50)
        self._n_samples = self.get_parameter('num_calibration_samples').value

        # Gravity correction quaternion (set after calibration)
        self._corr_qx = 0.0
        self._corr_qy = 0.0
        self._corr_qz = 0.0
        self._corr_qw = 1.0  # identity until calibrated
        self._calibrated = False

        # Accumulate accel readings
        self._accel_sum = [0.0, 0.0, 0.0]
        self._count = 0

        # BEST_EFFORT matches the camera driver's publisher QoS.
        # At 400-500 Hz, individual dropped samples are irrelevant; RELIABLE would
        # cause "message lost" warnings because DDS cannot retransmit from a
        # BEST_EFFORT publisher.
        imu_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=50,
        )

        self._sub = self.create_subscription(Imu, '/imu', self._callback, imu_qos)
        self._pub = self.create_publisher(Imu, '/imu/leveled', imu_qos)

        self.get_logger().info(
            f'IMU Gravity Leveler started — collecting {self._n_samples} samples for calibration...'
        )

    def _calibrate(self, ax, ay, az):
        """Called for each sample during calibration window."""
        self._accel_sum[0] += ax
        self._accel_sum[1] += ay
        self._accel_sum[2] += az
        self._count += 1

        if self._count < self._n_samples:
            return  # still collecting

        # Average measured gravity vector
        gx = self._accel_sum[0] / self._n_samples
        gy = self._accel_sum[1] / self._n_samples
        gz = self._accel_sum[2] / self._n_samples
        gmag = math.sqrt(gx*gx + gy*gy + gz*gz)

        # Normalize measured gravity direction
        nx, ny, nz = _normalize(gx, gy, gz)

        # Target: perfect Z-up [0, 0, 1]
        self._corr_qx, self._corr_qy, self._corr_qz, self._corr_qw = \
            _rotation_quat_from_vec_to_vec(nx, ny, nz, 0.0, 0.0, 1.0)

        self._calibrated = True

        tilt_deg = math.degrees(math.acos(max(-1.0, min(1.0, nz))))
        self.get_logger().info(
            f'Gravity leveling calibrated: measured gravity = ({gx:.4f}, {gy:.4f}, {gz:.4f}) '
            f'mag={gmag:.4f} m/s², tilt from Z-up = {tilt_deg:.3f}°. '
            f'Correction quat = ({self._corr_qx:.5f}, {self._corr_qy:.5f}, '
            f'{self._corr_qz:.5f}, {self._corr_qw:.5f})'
        )

    def _callback(self, msg: Imu):
        if not self._calibrated:
            self._calibrate(
                msg.linear_acceleration.x,
                msg.linear_acceleration.y,
                msg.linear_acceleration.z,
            )
            # DO NOT publish during calibration. Publishing uncorrected data here would
            # let Madgwick initialize from the physical camera orientation and RTAB-Map's
            # wait_imu_to_init would fire before leveling is applied, capturing the wrong
            # orientation as the initial pose. Instead, hold everything until the correction
            # quaternion is ready. RTAB-Map buffers image frames silently while no IMU flows.
            return

        out = Imu()
        out.header = msg.header

        # Rotate linear acceleration
        lx, ly, lz = _rotate_vector_by_quat(
            self._corr_qx, self._corr_qy, self._corr_qz, self._corr_qw,
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z,
        )
        out.linear_acceleration.x = lx
        out.linear_acceleration.y = ly
        out.linear_acceleration.z = lz

        # Rotate angular velocity
        wx, wy, wz = _rotate_vector_by_quat(
            self._corr_qx, self._corr_qy, self._corr_qz, self._corr_qw,
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z,
        )
        out.angular_velocity.x = wx
        out.angular_velocity.y = wy
        out.angular_velocity.z = wz

        # Rotate covariances
        out.linear_acceleration_covariance = _rotate_covariance(
            msg.linear_acceleration_covariance,
            self._corr_qx, self._corr_qy, self._corr_qz, self._corr_qw,
        )
        out.angular_velocity_covariance = _rotate_covariance(
            msg.angular_velocity_covariance,
            self._corr_qx, self._corr_qy, self._corr_qz, self._corr_qw,
        )

        # Orientation: compose correction with existing orientation (if set)
        # correction_quat * original_orientation
        if (msg.orientation.w != 0.0 or msg.orientation.x != 0.0 or
                msg.orientation.y != 0.0 or msg.orientation.z != 0.0):
            rx, ry, rz, rw = _quat_multiply(
                (self._corr_qx, self._corr_qy, self._corr_qz, self._corr_qw),
                (msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w),
            )
            out.orientation.x = rx
            out.orientation.y = ry
            out.orientation.z = rz
            out.orientation.w = rw
        else:
            out.orientation = msg.orientation

        out.orientation_covariance = _rotate_covariance(
            msg.orientation_covariance,
            self._corr_qx, self._corr_qy, self._corr_qz, self._corr_qw,
        )

        self._pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = IMUGravityLeveler()
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
