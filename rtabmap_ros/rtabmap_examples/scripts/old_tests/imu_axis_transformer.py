#!/usr/bin/env python3
"""
IMU Axis Transformer Node

Transforms IMU data from OAK-D IMU sensor frame to ROS REP-103 convention
(FLU: Forward-Left-Up) in the robot base frame.

The OAK-D BNO086 IMU is mounted with a 90-degree pitch relative to the base
frame (as defined in the depthai URDF). Its raw data uses the IMU sensor's own
coordinate system. This node converts that to the base frame convention so that
RTAB-Map's gravity alignment works correctly with wait_imu_to_init=True.

Transformation (from URDF TF inverse: oak_imu_frame → oak-d-base-frame):
- ros_x = optical_z (forward)
- ros_y = -optical_x (left, negative of right)
- ros_z = -optical_y (up, negative of down)

Rotation matrix R:
| 0   0   1 |
|-1   0   0 |
| 0  -1   0 |

For covariance: C_ros = R × C_optical × R^T

IMPORTANT: This node is required for OAK-D + RTAB-Map integration.
Without it, the IMU data remains in the sensor's optical frame and SLAM
cannot correctly determine world orientation (Z-axis alignment).

Performance: Uses inline math (no numpy) to handle 400-500 Hz IMU data
with minimal latency. QoS depth set to 200 to prevent message drops.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Imu


class IMUAxisTransformer(Node):
    """
    ROS2 node that transforms IMU data between coordinate frames.
    
    Subscribes to /imu (raw OAK-D IMU) and publishes to /imu/transformed.
    Optimized for high-frequency (400-500 Hz) processing without numpy overhead.
    """

    def __init__(self, enable_transform=None):
        """
        Initialize the IMU Axis Transformer node.
        
        Args:
            enable_transform: Optional bool to override parameter (for testing)
        """
        super().__init__('imu_axis_transformer')
        
        # Declare parameters
        self.declare_parameter('enable_imu_transform', True)
        self.declare_parameter('output_frame', 'oak-d-base-frame')
        
        # Get parameter value (use override if provided for testing)
        if enable_transform is not None:
            self.enable_transform = enable_transform
        else:
            self.enable_transform = self.get_parameter('enable_imu_transform').value
        
        self.output_frame = self.get_parameter('output_frame').value
        
        # High-throughput QoS for 400-500 Hz IMU data
        # Use large depth to prevent message drops under load
        imu_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=200
        )
        
        # Create subscriber with high-throughput QoS
        self.subscription = self.create_subscription(
            Imu,
            '/imu',
            self.imu_callback,
            imu_qos
        )
        
        # Create publisher with high-throughput QoS
        self.publisher = self.create_publisher(
            Imu,
            '/imu/transformed',
            imu_qos
        )
        
        # Log startup status
        if self.enable_transform:
            self.get_logger().info(f'IMU Axis Transformer started - Transformation ENABLED (output_frame: {self.output_frame})')
            self.get_logger().info('Optimized inline math for 400-500 Hz IMU (QoS depth=200)')
        else:
            self.get_logger().info('IMU Axis Transformer started - Transformation DISABLED (pass-through mode)')

    def transform_vector(self, x, y, z):
        """
        Transform a 3D vector from optical frame to ROS REP-103 frame.
        
        Uses inline math instead of numpy for zero-allocation performance.
        Rotation: ros_x = z, ros_y = -x, ros_z = -y
        
        Args:
            x: X component in optical/IMU sensor frame
            y: Y component in optical/IMU sensor frame
            z: Z component in optical/IMU sensor frame
            
        Returns:
            tuple: (ros_x, ros_y, ros_z) in ROS REP-103 base frame
        """
        return z, -x, -y

    def transform_covariance(self, covariance):
        """
        Transform a covariance matrix from optical frame to ROS REP-103 frame.
        
        Uses pre-computed index mapping for C_ros = R × C_optical × R^T
        with no numpy allocation. The rotation matrix R is orthogonal so
        the transformation is a permutation with sign changes.
        
        For R = [[0,0,1],[-1,0,0],[0,-1,0]]:
        C_out[0,0]=C[2,2], C_out[0,1]=-C[2,0], C_out[0,2]=-C[2,1]
        C_out[1,0]=-C[0,2], C_out[1,1]=C[0,0], C_out[1,2]=C[0,1]
        C_out[2,0]=-C[1,2], C_out[2,1]=C[1,0], C_out[2,2]=C[1,1]
        
        Args:
            covariance: 9-element array representing 3x3 covariance matrix (row-major)
            
        Returns:
            list: Transformed 9-element covariance matrix
        """
        c = covariance
        # Check for all-zero covariance (unknown in ROS convention)
        if c[0] == 0.0 and c[4] == 0.0 and c[8] == 0.0:
            return [0.0] * 9
        
        # Inline R × C × R^T using pre-computed index mapping
        # Input indices: c[0]=C00, c[1]=C01, c[2]=C02,
        #                c[3]=C10, c[4]=C11, c[5]=C12,
        #                c[6]=C20, c[7]=C21, c[8]=C22
        return [
             c[8], -c[6], -c[7],   # row 0
            -c[2],  c[0],  c[1],   # row 1
            -c[5],  c[3],  c[4]    # row 2
        ]

    def imu_callback(self, msg):
        """
        Callback for IMU messages. Optimized for 400-500 Hz throughput.
        
        Transforms IMU acceleration and angular velocity vectors using inline
        math (no numpy arrays allocated per callback).
        
        Args:
            msg: sensor_msgs/Imu message
        """
        if not self.enable_transform:
            self.publisher.publish(msg)
            return
        
        # Create output message
        output_msg = Imu()
        
        # Copy and update header
        output_msg.header = msg.header
        output_msg.header.frame_id = self.output_frame
        
        # Transform linear acceleration (inline, no numpy)
        # ros_x = z, ros_y = -x, ros_z = -y
        a = msg.linear_acceleration
        output_msg.linear_acceleration.x = a.z
        output_msg.linear_acceleration.y = -a.x
        output_msg.linear_acceleration.z = -a.y
        
        # Transform angular velocity (inline, no numpy)
        g = msg.angular_velocity
        output_msg.angular_velocity.x = g.z
        output_msg.angular_velocity.y = -g.x
        output_msg.angular_velocity.z = -g.y
        
        # Transform covariance matrices (inline index mapping)
        output_msg.linear_acceleration_covariance = self.transform_covariance(
            msg.linear_acceleration_covariance
        )
        output_msg.angular_velocity_covariance = self.transform_covariance(
            msg.angular_velocity_covariance
        )
        
        # Copy orientation unchanged (computed downstream by imu_filter_madgwick)
        output_msg.orientation = msg.orientation
        output_msg.orientation_covariance = list(msg.orientation_covariance)
        
        # Publish transformed message
        self.publisher.publish(output_msg)


def main(args=None):
    """Main entry point for the node."""
    rclpy.init(args=args)
    
    node = IMUAxisTransformer()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
