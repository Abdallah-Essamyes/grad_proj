#!/usr/bin/env python3
"""
Tests for oak_d_rgbd_simple.launch.py integration of IMU axis transformer.

Tests verify:
1. Launch file syntax is valid
2. Transformer node is present in launch description
3. Topic remapping chain is correct: /imu → /imu/transformed → /imu/data
"""

import unittest
import os
import sys
import importlib.util
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription


class TestLaunchIntegration(unittest.TestCase):
    """Test suite for IMU axis transformer integration in launch file."""
    
    def setUp(self):
        """Import the launch file module."""
        # Load the launch file
        launch_file = os.path.join(
            os.path.dirname(os.path.dirname(__file__)), 
            'launch',
            'oak_d_rgbd_simple.launch.py'
        )
        
        spec = importlib.util.spec_from_file_location("oak_d_launch", launch_file)
        launch_module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(launch_module)
        
        self.generate_launch_description = launch_module.generate_launch_description
        
    def test_launch_file_syntax(self):
        """Test 1: Verify launch file has valid Python syntax and returns LaunchDescription."""
        ld = self.generate_launch_description()
        self.assertIsInstance(ld, LaunchDescription, 
                            "Launch file should return LaunchDescription object")
        self.assertGreater(len(ld.entities), 0, 
                          "Launch description should contain entities")
    
    def test_transformer_node_present(self):
        """Test 2: Verify imu_axis_transformer node is present in launch description."""
        ld = self.generate_launch_description()
        
        # Find all Node entities
        nodes = [e for e in ld.entities if isinstance(e, Node)]
        
        # Check if transformer node exists by accessing internal attributes
        transformer_found = False
        for node in nodes:
            if hasattr(node, '_Node__package') and hasattr(node, '_Node__node_executable'):
                package = node._Node__package
                executable = node._Node__node_executable
                
                # Handle both string and Substitution types
                if isinstance(package, str):
                    package_str = package
                else:
                    package_str = str(package[0].perform(None))
                    
                if isinstance(executable, str):
                    executable_str = executable
                else:
                    executable_str = str(executable[0].perform(None))
                
                if package_str == 'rtabmap_examples' and 'imu_axis_transformer' in executable_str:
                    transformer_found = True
                    break
        
        self.assertTrue(transformer_found, 
                       "imu_axis_transformer node should be present in launch description")
    
    def test_remapping_chain(self):
        """Test 3: Verify topic remapping chain forms correct pipeline."""
        ld = self.generate_launch_description()
        
        # Find all Node entities
        nodes = [e for e in ld.entities if isinstance(e, Node)]
        
        # Find transformer node and madgwick node
        transformer_remappings = None
        madgwick_remappings = None
        
        for node in nodes:
            if hasattr(node, '_Node__package') and hasattr(node, '_Node__node_executable'):
                package = node._Node__package
                executable = node._Node__node_executable
                
                # Handle both string and Substitution types
                if isinstance(package, str):
                    package_str = package
                else:
                    package_str = str(package[0].perform(None))
                    
                if isinstance(executable, str):
                    executable_str = executable
                else:
                    executable_str = str(executable[0].perform(None))
                
                if package_str == 'rtabmap_examples' and 'imu_axis_transformer' in executable_str:
                    # Extract remappings
                    if hasattr(node, '_Node__remappings'):
                        transformer_remappings = node._Node__remappings
                        
                elif package_str == 'imu_filter_madgwick':
                    # Extract remappings
                    if hasattr(node, '_Node__remappings'):
                        madgwick_remappings = node._Node__remappings
        
        self.assertIsNotNone(transformer_remappings, 
                           "Transformer node should have remappings")
        self.assertIsNotNone(madgwick_remappings, 
                           "Madgwick filter node should have remappings")
        
        # Check that transformer has output to /imu/transformed
        transformer_has_output = False
        for remap in transformer_remappings:
            from_topic = str(remap[0][0].perform(None))
            to_topic = str(remap[1][0].perform(None))
            if 'output' in from_topic and to_topic == '/imu/transformed':
                transformer_has_output = True
                break
        
        # Check that madgwick has input from /imu/transformed
        madgwick_has_input = False
        for remap in madgwick_remappings:
            from_topic = str(remap[0][0].perform(None))
            to_topic = str(remap[1][0].perform(None))
            if 'data_raw' in from_topic and to_topic == '/imu/transformed':
                madgwick_has_input = True
                break
        
        self.assertTrue(transformer_has_output,
                       "Transformer should publish to /imu/transformed")
        self.assertTrue(madgwick_has_input,
                       "Madgwick should subscribe to /imu/transformed")
    
    def test_no_camera_orientation_params(self):
        """Test 4: Verify cam_roll and cam_yaw are removed from camera launch."""
        ld = self.generate_launch_description()
        
        # Find IncludeLaunchDescription for camera driver
        camera_launch = None
        for entity in ld.entities:
            if isinstance(entity, IncludeLaunchDescription):
                # Access launch arguments
                if hasattr(entity, '_IncludeLaunchDescription__launch_arguments'):
                    launch_args = entity._IncludeLaunchDescription__launch_arguments
                    camera_launch = entity
                    break
        
        self.assertIsNotNone(camera_launch, 
                           "Camera launch should be present")
        
        # Verify cam_roll and cam_yaw are NOT in launch arguments
        launch_args = camera_launch._IncludeLaunchDescription__launch_arguments
        arg_names = []
        for arg in launch_args:
            arg_names.append(str(arg[0]))
        
        self.assertNotIn('cam_roll', arg_names,
                        "cam_roll should be removed from camera launch arguments")
        self.assertNotIn('cam_yaw', arg_names,
                        "cam_yaw should be removed from camera launch arguments")


if __name__ == '__main__':
    unittest.main()
