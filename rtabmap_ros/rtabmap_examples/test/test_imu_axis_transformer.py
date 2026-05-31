#!/usr/bin/env python3
"""
Unit tests for IMU Axis Transformer Node

Tests the transformation of IMU data from optical frame (RDF: Right-Down-Forward)
to ROS REP-103 convention (FLU: Forward-Left-Up).

Test cases:
1. test_transform_optical_to_rep103 - Verify axis transformation math
2. test_covariance_rotation - Verify covariance matrix transformation
3. test_frame_id_update - Verify frame_id is updated correctly
4. test_enable_disable_parameter - Verify pass-through when disabled
"""

import unittest
import numpy as np
from sensor_msgs.msg import Imu
from std_msgs.msg import Header
import rclpy
from rclpy.node import Node
import sys
import os

# Add scripts directory to path for importing the transformer
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'scripts'))


class TestIMUAxisTransformer(unittest.TestCase):
    """Test suite for IMU axis transformation"""

    @classmethod
    def setUpClass(cls):
        """Initialize ROS2"""
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        """Shutdown ROS2"""
        rclpy.shutdown()

    def setUp(self):
        """Set up test fixtures"""
        self.test_node = Node('test_node')

    def tearDown(self):
        """Clean up test resources"""
        self.test_node.destroy_node()

    def test_transform_optical_to_rep103(self):
        """Test 1: Verify axis transformation math"""
        try:
            from imu_axis_transformer import IMUAxisTransformer
        except ImportError:
            self.fail("Cannot import IMUAxisTransformer - implementation not found")

        # Create test IMU message in optical frame (RDF)
        # optical_x = 1.0 (right), optical_y = 2.0 (down), optical_z = 3.0 (forward)
        optical_imu = Imu()
        optical_imu.linear_acceleration.x = 1.0  # right
        optical_imu.linear_acceleration.y = 2.0  # down
        optical_imu.linear_acceleration.z = 3.0  # forward
        
        optical_imu.angular_velocity.x = 4.0  # right
        optical_imu.angular_velocity.y = 5.0  # down
        optical_imu.angular_velocity.z = 6.0  # forward

        # Expected transformation to ROS REP-103 (FLU)
        # ros_x = optical_z = 3.0 (forward)
        # ros_y = -optical_x = -1.0 (left, negative of right)
        # ros_z = -optical_y = -2.0 (up, negative of down)

        transformer = IMUAxisTransformer()
        
        # Transform the vectors
        ros_accel = transformer.transform_vector(
            optical_imu.linear_acceleration.x,
            optical_imu.linear_acceleration.y,
            optical_imu.linear_acceleration.z
        )
        
        ros_gyro = transformer.transform_vector(
            optical_imu.angular_velocity.x,
            optical_imu.angular_velocity.y,
            optical_imu.angular_velocity.z
        )

        # Verify linear acceleration transformation
        self.assertAlmostEqual(ros_accel[0], 3.0, places=6, 
                               msg="Linear acceleration X (forward) should be optical_z")
        self.assertAlmostEqual(ros_accel[1], -1.0, places=6,
                               msg="Linear acceleration Y (left) should be -optical_x")
        self.assertAlmostEqual(ros_accel[2], -2.0, places=6,
                               msg="Linear acceleration Z (up) should be -optical_y")

        # Verify angular velocity transformation
        self.assertAlmostEqual(ros_gyro[0], 6.0, places=6,
                               msg="Angular velocity X (forward) should be optical_z")
        self.assertAlmostEqual(ros_gyro[1], -4.0, places=6,
                               msg="Angular velocity Y (left) should be -optical_x")
        self.assertAlmostEqual(ros_gyro[2], -5.0, places=6,
                               msg="Angular velocity Z (up) should be -optical_y")

    def test_covariance_rotation(self):
        """Test 2: Verify covariance matrix transformation"""
        try:
            from imu_axis_transformer import IMUAxisTransformer
        except ImportError:
            self.fail("Cannot import IMUAxisTransformer - implementation not found")

        transformer = IMUAxisTransformer()

        # Create a test covariance matrix (9-element array representing 3x3 matrix)
        # Simple diagonal matrix for easy verification
        optical_cov = [
            1.0, 0.0, 0.0,  # Row 1: variance in optical_x
            0.0, 4.0, 0.0,  # Row 2: variance in optical_y
            0.0, 0.0, 9.0   # Row 3: variance in optical_z
        ]

        ros_cov = transformer.transform_covariance(optical_cov)

        # Expected result after rotation R × C × R^T where R is:
        # | 0  -1   0 |
        # | 0   0  -1 |
        # | 1   0   0 |
        
        # For diagonal matrix [1, 4, 9], result should be [9, 1, 4]
        # because optical_z variance (9) → ros_x
        # optical_x variance (1) → ros_y
        # optical_y variance (4) → ros_z
        
        expected_cov = [
            9.0, 0.0, 0.0,  # ros_x variance = optical_z variance
            0.0, 1.0, 0.0,  # ros_y variance = optical_x variance
            0.0, 0.0, 4.0   # ros_z variance = optical_y variance
        ]

        for i in range(9):
            self.assertAlmostEqual(ros_cov[i], expected_cov[i], places=6,
                                   msg=f"Covariance element {i} mismatch")

    def test_covariance_rotation_with_cross_terms(self):
        """Test 2b: Verify covariance transformation with off-diagonal elements"""
        try:
            from imu_axis_transformer import IMUAxisTransformer
        except ImportError:
            self.fail("Cannot import IMUAxisTransformer - implementation not found")

        transformer = IMUAxisTransformer()

        # Create a covariance matrix with off-diagonal terms
        optical_cov = [
            1.0, 0.5, 0.0,  # Row 1
            0.5, 4.0, 0.0,  # Row 2
            0.0, 0.0, 9.0   # Row 3
        ]

        ros_cov = transformer.transform_covariance(optical_cov)

        # Verify it's still a valid covariance matrix (symmetric and positive semi-definite)
        # Reshape to 3x3 for easier verification
        cov_matrix = np.array(ros_cov).reshape(3, 3)
        
        # Check symmetry
        self.assertTrue(np.allclose(cov_matrix, cov_matrix.T),
                        msg="Covariance matrix should be symmetric")

        # Check all diagonal elements are non-negative
        for i in range(3):
            self.assertGreaterEqual(cov_matrix[i, i], 0.0,
                                    msg=f"Diagonal element {i} should be non-negative")

    def test_covariance_zero_handling(self):
        """Test 2c: Verify handling of zero covariance (unknown)"""
        try:
            from imu_axis_transformer import IMUAxisTransformer
        except ImportError:
            self.fail("Cannot import IMUAxisTransformer - implementation not found")

        transformer = IMUAxisTransformer()

        # All zeros means covariance is unknown in ROS convention
        zero_cov = [0.0] * 9

        ros_cov = transformer.transform_covariance(zero_cov)

        # Should remain all zeros
        for i in range(9):
            self.assertAlmostEqual(ros_cov[i], 0.0, places=6,
                                   msg="Zero covariance should remain zero")

    def test_frame_id_update(self):
        """Test 3: Verify frame_id is updated correctly"""
        try:
            from imu_axis_transformer import IMUAxisTransformer
        except ImportError:
            self.fail("Cannot import IMUAxisTransformer - implementation not found")

        # Create transformer node
        transformer = IMUAxisTransformer()

        # Create test IMU message
        test_imu = Imu()
        test_imu.header.frame_id = "oak_imu_frame"
        test_imu.header.stamp.sec = 100
        test_imu.header.stamp.nanosec = 500000
        test_imu.linear_acceleration.x = 1.0
        test_imu.linear_acceleration.y = 2.0
        test_imu.linear_acceleration.z = 3.0
        test_imu.angular_velocity.x = 0.1
        test_imu.angular_velocity.y = 0.2
        test_imu.angular_velocity.z = 0.3

        # Store the published message
        published_msg = None
        
        def capture_callback(msg):
            nonlocal published_msg
            published_msg = msg

        # Subscribe to output topic
        subscription = self.test_node.create_subscription(
            Imu,
            '/imu/transformed',
            capture_callback,
            10
        )

        # Manually call the callback
        transformer.imu_callback(test_imu)

        # Spin to process callbacks
        rclpy.spin_once(self.test_node, timeout_sec=0.5)
        rclpy.spin_once(transformer, timeout_sec=0.5)
        rclpy.spin_once(self.test_node, timeout_sec=0.5)

        # Verify frame_id was updated
        self.assertIsNotNone(published_msg, msg="No message was published")
        self.assertEqual(published_msg.header.frame_id, "oak-d-base-frame",
                         msg="Frame ID should be updated to oak-d-base-frame")
        
        # Verify timestamp is preserved
        self.assertEqual(published_msg.header.stamp.sec, 100,
                         msg="Timestamp seconds should be preserved")
        self.assertEqual(published_msg.header.stamp.nanosec, 500000,
                         msg="Timestamp nanoseconds should be preserved")

    def test_enable_disable_parameter(self):
        """Test 4: Verify parameter allows pass-through when disabled"""
        try:
            from imu_axis_transformer import IMUAxisTransformer
        except ImportError:
            self.fail("Cannot import IMUAxisTransformer - implementation not found")

        # Create transformer with disabled transformation
        transformer_disabled = IMUAxisTransformer(enable_transform=False)

        # Create test IMU message
        test_imu = Imu()
        test_imu.header.frame_id = "original_frame"
        test_imu.linear_acceleration.x = 1.0
        test_imu.linear_acceleration.y = 2.0
        test_imu.linear_acceleration.z = 3.0
        test_imu.angular_velocity.x = 4.0
        test_imu.angular_velocity.y = 5.0
        test_imu.angular_velocity.z = 6.0

        published_msg = None
        
        def capture_callback(msg):
            nonlocal published_msg
            published_msg = msg

        # Subscribe to output topic
        subscription = self.test_node.create_subscription(
            Imu,
            '/imu/transformed',
            capture_callback,
            10
        )

        # Call the callback
        transformer_disabled.imu_callback(test_imu)

        # Spin to process
        rclpy.spin_once(self.test_node, timeout_sec=0.5)
        rclpy.spin_once(transformer_disabled, timeout_sec=0.5)
        rclpy.spin_once(self.test_node, timeout_sec=0.5)

        # Verify data passed through unchanged
        self.assertIsNotNone(published_msg, msg="No message was published")
        self.assertEqual(published_msg.header.frame_id, "original_frame",
                         msg="Frame ID should be unchanged when disabled")
        self.assertAlmostEqual(published_msg.linear_acceleration.x, 1.0, places=6)
        self.assertAlmostEqual(published_msg.linear_acceleration.y, 2.0, places=6)
        self.assertAlmostEqual(published_msg.linear_acceleration.z, 3.0, places=6)
        self.assertAlmostEqual(published_msg.angular_velocity.x, 4.0, places=6)
        self.assertAlmostEqual(published_msg.angular_velocity.y, 5.0, places=6)
        self.assertAlmostEqual(published_msg.angular_velocity.z, 6.0, places=6)

    def test_orientation_unchanged(self):
        """Test 5: Verify orientation quaternion is preserved"""
        try:
            from imu_axis_transformer import IMUAxisTransformer
        except ImportError:
            self.fail("Cannot import IMUAxisTransformer - implementation not found")

        transformer = IMUAxisTransformer()

        # Create test IMU with orientation
        test_imu = Imu()
        test_imu.orientation.x = 0.1
        test_imu.orientation.y = 0.2
        test_imu.orientation.z = 0.3
        test_imu.orientation.w = 0.9
        test_imu.linear_acceleration.z = 1.0
        test_imu.angular_velocity.z = 0.1

        published_msg = None
        
        def capture_callback(msg):
            nonlocal published_msg
            published_msg = msg

        subscription = self.test_node.create_subscription(
            Imu,
            '/imu/transformed',
            capture_callback,
            10
        )

        transformer.imu_callback(test_imu)

        rclpy.spin_once(self.test_node, timeout_sec=0.5)
        rclpy.spin_once(transformer, timeout_sec=0.5)
        rclpy.spin_once(self.test_node, timeout_sec=0.5)

        # Verify orientation is unchanged
        self.assertIsNotNone(published_msg)
        self.assertAlmostEqual(published_msg.orientation.x, 0.1, places=6,
                               msg="Orientation X should be unchanged")
        self.assertAlmostEqual(published_msg.orientation.y, 0.2, places=6,
                               msg="Orientation Y should be unchanged")
        self.assertAlmostEqual(published_msg.orientation.z, 0.3, places=6,
                               msg="Orientation Z should be unchanged")
        self.assertAlmostEqual(published_msg.orientation.w, 0.9, places=6,
                               msg="Orientation W should be unchanged")

    # ========== Phase 2 Tests: Node Execution and ROS2 Integration ==========
    
    def test_node_startup(self):
        """Phase 2 Test 1: Verify node can be instantiated"""
        try:
            from imu_axis_transformer import IMUAxisTransformer
        except ImportError:
            self.fail("Cannot import IMUAxisTransformer - implementation not found")

        # Test node creation with default parameters
        try:
            node = IMUAxisTransformer()
            self.assertIsNotNone(node, msg="Node should be created successfully")
            self.assertEqual(node.get_name(), 'imu_axis_transformer',
                             msg="Node name should be 'imu_axis_transformer'")
            self.assertTrue(node.enable_transform,
                            msg="Transformation should be enabled by default")
            node.destroy_node()
        except Exception as e:
            self.fail(f"Node instantiation failed: {e}")

        # Test node creation with transformation disabled
        try:
            node_disabled = IMUAxisTransformer(enable_transform=False)
            self.assertIsNotNone(node_disabled, msg="Node should be created with disabled transform")
            self.assertFalse(node_disabled.enable_transform,
                             msg="Transformation should be disabled")
            node_disabled.destroy_node()
        except Exception as e:
            self.fail(f"Node instantiation with disabled transform failed: {e}")

    def test_topic_subscription(self):
        """Phase 2 Test 2: Verify node subscribes to /imu"""
        try:
            from imu_axis_transformer import IMUAxisTransformer
        except ImportError:
            self.fail("Cannot import IMUAxisTransformer - implementation not found")

        node = IMUAxisTransformer()

        # Verify subscription object exists
        self.assertIsNotNone(node.subscription,
                             msg="Node should have a subscription object")

        # Verify the subscription is for the correct topic
        topic_name = node.subscription.topic_name
        self.assertIn('/imu', topic_name,
                      msg="Node should subscribe to /imu topic")

        node.destroy_node()

    def test_topic_publication(self):
        """Phase 2 Test 3: Verify node publishes to /imu/transformed"""
        try:
            from imu_axis_transformer import IMUAxisTransformer
        except ImportError:
            self.fail("Cannot import IMUAxisTransformer - implementation not found")

        node = IMUAxisTransformer()

        # Verify publisher object exists
        self.assertIsNotNone(node.publisher,
                             msg="Node should have a publisher object")

        # Verify the publisher is for the correct topic
        topic_name = node.publisher.topic_name
        self.assertIn('/imu/transformed', topic_name,
                      msg="Node should publish to /imu/transformed topic")

        node.destroy_node()

    def test_end_to_end_message_flow(self):
        """Phase 2 Test 4: Verify complete message flow from subscription to publication"""
        try:
            from imu_axis_transformer import IMUAxisTransformer
        except ImportError:
            self.fail("Cannot import IMUAxisTransformer - implementation not found")

        # Create transformer node
        transformer = IMUAxisTransformer()

        # Create test message
        input_msg = Imu()
        input_msg.header.stamp = self.test_node.get_clock().now().to_msg()
        input_msg.header.frame_id = "test_imu_frame"
        input_msg.linear_acceleration.x = 1.0
        input_msg.linear_acceleration.y = 2.0
        input_msg.linear_acceleration.z = 3.0
        input_msg.angular_velocity.x = 0.1
        input_msg.angular_velocity.y = 0.2
        input_msg.angular_velocity.z = 0.3

        # Set up subscriber to catch output
        output_received = []
        
        def output_callback(msg):
            output_received.append(msg)

        output_sub = self.test_node.create_subscription(
            Imu,
            '/imu/transformed',
            output_callback,
            10
        )

        # Publish input message
        transformer.imu_callback(input_msg)

        # Spin to process messages
        for _ in range(5):
            rclpy.spin_once(transformer, timeout_sec=0.1)
            rclpy.spin_once(self.test_node, timeout_sec=0.1)

        # Verify message was received
        self.assertGreater(len(output_received), 0,
                           msg="Should receive at least one transformed message")

        output_msg = output_received[0]

        # Verify transformation occurred
        self.assertEqual(output_msg.header.frame_id, "oak-d-base-frame",
                         msg="Frame ID should be updated")
        self.assertAlmostEqual(output_msg.linear_acceleration.x, 3.0, places=5,
                               msg="Linear acceleration should be transformed")
        self.assertAlmostEqual(output_msg.linear_acceleration.y, -1.0, places=5,
                               msg="Linear acceleration should be transformed")
        self.assertAlmostEqual(output_msg.linear_acceleration.z, -2.0, places=5,
                               msg="Linear acceleration should be transformed")

        transformer.destroy_node()


def main():
    """Run the tests"""
    unittest.main()


if __name__ == '__main__':
    main()
