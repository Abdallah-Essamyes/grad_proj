#!/usr/bin/env python3
"""
Integration test for IMU Axis Transformer Node

This test verifies the node can be launched and transforms IMU data correctly
in a real ROS2 environment.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import time


class IMUTestPublisher(Node):
    """Test node that publishes IMU data and verifies transformation"""

    def __init__(self):
        super().__init__('imu_test_publisher')
        
        # Publisher to /imu
        self.imu_pub = self.create_publisher(Imu, '/imu', 10)
        
        # Subscriber to /imu/transformed
        self.transformed_sub = self.create_subscription(
            Imu,
            '/imu/transformed',
            self.transformed_callback,
            10
        )
        
        self.received_transformed = False
        self.transformed_msg = None
        
        # Timer to publish test data
        self.timer = self.create_timer(0.1, self.publish_test_imu)
        self.publish_count = 0
        
        self.get_logger().info('IMU Test Publisher started')

    def publish_test_imu(self):
        """Publish test IMU data in optical frame"""
        if self.publish_count >= 10:
            return
            
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'oak_imu_frame'
        
        # Optical frame: X=right, Y=down, Z=forward
        msg.linear_acceleration.x = 1.0  # right
        msg.linear_acceleration.y = 2.0  # down
        msg.linear_acceleration.z = 3.0  # forward
        
        msg.angular_velocity.x = 0.1  # right
        msg.angular_velocity.y = 0.2  # down
        msg.angular_velocity.z = 0.3  # forward
        
        # Simple diagonal covariance
        msg.linear_acceleration_covariance = [
            1.0, 0.0, 0.0,
            0.0, 4.0, 0.0,
            0.0, 0.0, 9.0
        ]
        msg.angular_velocity_covariance = [
            0.01, 0.0, 0.0,
            0.0, 0.04, 0.0,
            0.0, 0.0, 0.09
        ]
        
        self.imu_pub.publish(msg)
        self.publish_count += 1
        
        if self.publish_count == 1:
            self.get_logger().info('Published test IMU data (optical frame)')

    def transformed_callback(self, msg):
        """Callback for transformed IMU data"""
        if not self.received_transformed:
            self.received_transformed = True
            self.transformed_msg = msg
            
            self.get_logger().info('Received transformed IMU data!')
            self.get_logger().info(f'  Frame ID: {msg.header.frame_id}')
            self.get_logger().info(f'  Linear acceleration (ROS REP-103):')
            self.get_logger().info(f'    X (forward): {msg.linear_acceleration.x:.3f}')
            self.get_logger().info(f'    Y (left):    {msg.linear_acceleration.y:.3f}')
            self.get_logger().info(f'    Z (up):      {msg.linear_acceleration.z:.3f}')
            self.get_logger().info(f'  Angular velocity (ROS REP-103):')
            self.get_logger().info(f'    X (forward): {msg.angular_velocity.x:.3f}')
            self.get_logger().info(f'    Y (left):    {msg.angular_velocity.y:.3f}')
            self.get_logger().info(f'    Z (up):      {msg.angular_velocity.z:.3f}')
            
            # Verify transformation
            expected_accel = (3.0, -1.0, -2.0)  # ROS REP-103 from optical (1, 2, 3)
            actual_accel = (
                msg.linear_acceleration.x,
                msg.linear_acceleration.y,
                msg.linear_acceleration.z
            )
            
            tolerance = 0.001
            if all(abs(a - e) < tolerance for a, e in zip(actual_accel, expected_accel)):
                self.get_logger().info('✓ Transformation CORRECT!')
            else:
                self.get_logger().error('✗ Transformation INCORRECT!')
                self.get_logger().error(f'  Expected: {expected_accel}')
                self.get_logger().error(f'  Got:      {actual_accel}')


def main(args=None):
    """Run integration test"""
    rclpy.init(args=args)
    
    node = IMUTestPublisher()
    
    print("\n" + "="*70)
    print("IMU Axis Transformer - Integration Test")
    print("="*70)
    print("\nThis test will:")
    print("1. Publish IMU data in optical frame (RDF: Right-Down-Forward)")
    print("2. Verify the transformer converts it to ROS REP-103 (FLU: Forward-Left-Up)")
    print("\nMake sure the IMU transformer node is running:")
    print("  ros2 run rtabmap_examples imu_axis_transformer.py")
    print("\nPress Ctrl+C to stop the test")
    print("="*70 + "\n")
    
    try:
        # Spin for 5 seconds
        timeout = 5.0
        start_time = time.time()
        while time.time() - start_time < timeout:
            rclpy.spin_once(node, timeout_sec=0.1)
            if node.received_transformed:
                break
        
        if not node.received_transformed:
            node.get_logger().error('Did not receive transformed message!')
            node.get_logger().error('Make sure the transformer node is running.')
        
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
