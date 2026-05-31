#!/usr/bin/env python3
"""
Camera Watchdog Node

Monitors camera image topics and restarts the camera node if data stops flowing.
Uses minimal CPU - just a subscription callback and timer (< 1% overhead).

Detects the same condition as rgbd_sync's "Did not receive data since 5 seconds"
but takes action by restarting the camera automatically.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
import subprocess
import time

class CameraWatchdog(Node):
    def __init__(self):
        super().__init__('camera_watchdog')
        
        # Parameters
        self.declare_parameter('timeout_sec', 5.0)
        self.declare_parameter('restart_cooldown_sec', 10.0)
        self.declare_parameter('monitor_topic', '/right/image_rect')
        
        self.timeout_sec = self.get_parameter('timeout_sec').value
        self.restart_cooldown_sec = self.get_parameter('restart_cooldown_sec').value
        self.monitor_topic = self.get_parameter('monitor_topic').value
        
        # State
        self.last_image_time = self.get_clock().now()
        self.last_restart_time = 0.0
        self.restart_in_progress = False
        
        # Subscribe to camera image (Best Effort QoS like the camera publishes)
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=1  # We only care about "is data flowing", not the actual data
        )
        
        self.subscription = self.create_subscription(
            Image,
            self.monitor_topic,
            self.image_callback,
            qos
        )
        
        # Timer to check for timeouts (check every 1 second)
        self.timer = self.create_timer(1.0, self.check_timeout)
        
        self.get_logger().info(
            f'Camera watchdog started - monitoring {self.monitor_topic} '
            f'with {self.timeout_sec}s timeout'
        )
    
    def image_callback(self, msg):
        """Update last received time on any image"""
        self.last_image_time = self.get_clock().now()
    
    def check_timeout(self):
        """Check if we've exceeded timeout and restart if needed"""
        if self.restart_in_progress:
            return
            
        current_time = self.get_clock().now()
        time_since_last_image = (current_time - self.last_image_time).nanoseconds / 1e9
        
        if time_since_last_image > self.timeout_sec:
            # Check restart cooldown
            current_timestamp = time.time()
            if current_timestamp - self.last_restart_time < self.restart_cooldown_sec:
                self.get_logger().warn(
                    f'Camera timeout detected but in cooldown period '
                    f'({current_timestamp - self.last_restart_time:.1f}s < '
                    f'{self.restart_cooldown_sec}s)'
                )
                return
            
            self.get_logger().error(
                f'Camera timeout! No data for {time_since_last_image:.1f}s. '
                'Restarting camera node...'
            )
            self.restart_camera()
    
    def restart_camera(self):
        """Restart the camera node"""
        self.restart_in_progress = True
        self.last_restart_time = time.time()
        
        try:
            # Kill stereo_inertial_node
            self.get_logger().info('Killing stereo_inertial_node...')
            result = subprocess.run(
                ['pkill', '-f', 'stereo_inertial_node'],
                capture_output=True,
                text=True
            )
            
            # Wait for USB re-enumeration
            time.sleep(2)
            
            # Restart camera launch in background
            self.get_logger().info('Restarting camera launch...')
            launch_cmd = [
                'ros2', 'launch', 'depthai_examples',
                'test_stereo_inertial_node.launch.py',
                'depth_aligned:=false',
                'enableRviz:=false',
                'monoResolution:=400p',
                'transform_imu_to_base_frame:=true'
            ]
            
            # Start detached process
            subprocess.Popen(
                launch_cmd,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                start_new_session=True
            )
            
            self.get_logger().info('Camera restart initiated')
            
            # Reset watchdog timer
            self.last_image_time = self.get_clock().now()
            
        except Exception as e:
            self.get_logger().error(f'Failed to restart camera: {e}')
        finally:
            self.restart_in_progress = False

def main(args=None):
    rclpy.init(args=args)
    
    watchdog = CameraWatchdog()
    
    try:
        rclpy.spin(watchdog)
    except KeyboardInterrupt:
        pass
    finally:
        watchdog.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
