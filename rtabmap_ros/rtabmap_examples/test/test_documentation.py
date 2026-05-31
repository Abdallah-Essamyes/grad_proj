#!/usr/bin/env python3
"""
Unit tests for Documentation Completeness

Verifies that the OAK_MAPPING_README.md contains comprehensive documentation
about the IMU axis transformation fix.
"""

import unittest
import os


class TestDocumentationCompleteness(unittest.TestCase):
    """Test suite for documentation verification"""

    def setUp(self):
        """Set up test fixtures"""
        self.readme_path = '/home/ggsya/ros2_ws/OAK_MAPPING_README.md'
        
        # Required sections that must be documented
        self.required_sections = [
            "IMU Axis Transformation",
            "The IMU Frame Problem",
            "The Solution",
            "Technical Details",
            "Verification Commands",
        ]
        
        # Required technical terms that should be explained
        self.required_terms = [
            "optical frame",
            "ROS REP-103",
            "RDF",
            "FLU",
            "axis transformation",
            "imu_axis_transformer",
        ]
        
        # Required code examples/commands
        self.required_examples = [
            "ros2 topic echo /imu",
            "ros2 topic echo /imu/transformed",
            "ros2 node info /imu_axis_transformer",
        ]

    def test_documentation_exists(self):
        """Test 1: Verify documentation file exists"""
        self.assertTrue(os.path.exists(self.readme_path),
                        msg=f"README file should exist at {self.readme_path}")
        
        # Verify it's not empty
        with open(self.readme_path, 'r') as f:
            content = f.read()
            self.assertGreater(len(content), 0,
                               msg="README should not be empty")

    def test_documentation_completeness(self):
        """Test 2: Verify all required sections are present"""
        with open(self.readme_path, 'r') as f:
            content = f.read()
        
        missing_sections = []
        for section in self.required_sections:
            if section not in content:
                missing_sections.append(section)
        
        self.assertEqual(len(missing_sections), 0,
                         msg=f"Missing required sections: {missing_sections}")

    def test_technical_terms_explained(self):
        """Test 3: Verify technical terms are documented"""
        with open(self.readme_path, 'r') as f:
            content = f.read().lower()  # Case-insensitive search
        
        missing_terms = []
        for term in self.required_terms:
            if term.lower() not in content:
                missing_terms.append(term)
        
        self.assertEqual(len(missing_terms), 0,
                         msg=f"Missing technical terms: {missing_terms}")

    def test_verification_examples_present(self):
        """Test 4: Verify verification commands are documented"""
        with open(self.readme_path, 'r') as f:
            content = f.read()
        
        missing_examples = []
        for example in self.required_examples:
            if example not in content:
                missing_examples.append(example)
        
        self.assertEqual(len(missing_examples), 0,
                         msg=f"Missing verification examples: {missing_examples}")

    def test_pipeline_diagram_present(self):
        """Test 5: Verify data flow pipeline is documented"""
        with open(self.readme_path, 'r') as f:
            content = f.read()
        
        # Check for pipeline components
        pipeline_components = [
            "OAK-D",
            "depthai_ros",
            "imu_axis_transformer",
            "imu_filter_madgwick",
            "rgbd_odometry",
            "rtabmap",
        ]
        
        missing_components = []
        for component in pipeline_components:
            if component not in content:
                missing_components.append(component)
        
        self.assertEqual(len(missing_components), 0,
                         msg=f"Pipeline missing components: {missing_components}")

    def test_troubleshooting_section_present(self):
        """Test 6: Verify troubleshooting guidance is provided"""
        with open(self.readme_path, 'r') as f:
            content = f.read()
        
        # Check for troubleshooting keywords
        troubleshooting_keywords = [
            "troubleshoot",
            "problem",
            "solution",
            "verify",
            "check",
        ]
        
        found_keywords = sum(1 for keyword in troubleshooting_keywords 
                             if keyword.lower() in content.lower())
        
        self.assertGreaterEqual(found_keywords, 3,
                                msg="Documentation should include troubleshooting guidance")

    def test_imu_transformation_math_explained(self):
        """Test 7: Verify transformation mathematics are explained"""
        with open(self.readme_path, 'r') as f:
            content = f.read()
        
        # Look for mathematical explanation
        math_indicators = [
            "optical_x",
            "optical_y", 
            "optical_z",
            "ros_x",
            "ros_y",
            "ros_z",
            "Right-Down-Forward",
            "Forward-Left-Up",
        ]
        
        found_indicators = sum(1 for indicator in math_indicators 
                               if indicator in content)
        
        self.assertGreaterEqual(found_indicators, 5,
                                msg="Transformation mathematics should be explained")


def main():
    """Run the tests"""
    unittest.main()


if __name__ == '__main__':
    main()
