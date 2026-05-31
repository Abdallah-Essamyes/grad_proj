# Temporary test launch file to debug wait_imu_to_init issue
# This is identical to oak_d_rgbd_simple.launch.py but for testing

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # C++ IMU axis transformer handles conversion from sensor frame to ROS REP-103
    # (X=forward, Y=left, Z=up) with deterministic performance for 400-500 Hz IMU data
    parameters=[{'frame_id':'oak-d-base-frame',
                 'subscribe_rgbd':True,
                 'subscribe_odom_info':True,
                 'approx_sync':True,  # Disabled - C++ transformer is fast enough
                 'wait_imu_to_init':True,  # Enable visual-inertial odometry
                 'wait_for_transform': 0.5,
                # 'Reg/Force3DoF': 'true',  # UNCOMMENT for strict flat-floor 2D mode (disables Z/roll/pitch) — breaks 3D map but fixes occupancy grid on flat floors
                # 🔽 Point cloud density reduction 🔽
                #Cloud/VoxelSize = 0.03 – 0.05
                #Grid/CellSize = 0.05 – 0.10
                # cloud_voxel_size: voxel grid downsampling applied to the assembled 3D point cloud
                # (the visual /cloud_map published for RViz, NOT the occupancy grid).
                # The depth image is divided into 3D cubes of this side length; all points inside
                # one cube are collapsed into a single point at the cube centre.
                # Smaller → denser cloud, more detail, higher CPU/memory cost.
                # Larger  → sparser cloud, less detail, lower CPU/memory cost.
                # Does NOT affect the occupancy grid or OctoMap — those use Grid/CellSize instead.
                # 0 = disabled (keep every point). Typical range: 0.02 (2cm) – 0.05 (5cm).
                'cloud_voxel_size': 0.03,
                # 🔽 Map publishing control 🔽
               # Grid/MinObstacleHeight does NOT exist in RTAB-Map (verified in Parameters.h — never existed).
                # The lower bound for obstacles is controlled by Grid/MinGroundHeight (default 0=disabled),
                # NOT a MinObstacleHeight param.  Ground/obstacle separation is done by NormalsSegmentation.
                # All height thresholds are in the MAP frame (relative to where the first keyframe was placed).
                # See OBSTACLE_DETECTION.md in ros_ws root for full explanation.
                'Grid/RangeMax': '4.0',             # Only map points within 4m (reduced from 4m)
                'Grid/CellSize': '0.03',             # 3cm cells (finer = more obstacle cells)
                'Grid/MaxObstacleHeight': '0.6',   # cut points above 60cm (excludes ceiling); Z in gravity-aligned camera frame
                'Grid/MinGroundHeight': '-0.05',      # allow floor points up to 1.5m below camera; Z=-0.01 was cutting all floor-level objects
                'Grid/MaxGroundAngle': '25',         # default=45; lower = stricter ground classification; flat clutter with upward normals = ground at 45, = obstacle at 20
                'Grid/RayTracing': 'true',           # Cast free-space rays so obstacle cells can be CLEARED when nothing is there
                'Grid/3D': 'true',                   # 3D OctoMap required for ray tracing to actually clear voxels (WITH_OCTOMAP=ON confirmed)
                'Grid/ClusterRadius': '0.20',         # Max distance (meters) between two points to be in the same cluster (default=0.1)
                'Grid/MinClusterSize': '5',         # Minimum cluster size (default=10). Was 200 — overly agressive, killed real objects.
                'Grid/NoiseFilteringRadius': '0.20',  # 20cm sphere for noise check
                'Grid/NoiseFilteringMinNeighbors': '5', # Need >=5 neighbors in 20cm sphere to survive
                # 🔽 Bayesian occupancy: tuned for FASTER clearing (works with Grid/RayTracing+Grid/3D) 🔽
                'Rtabmap/DetectionRate': '5',        # Process 2 frames/sec (default=1); more keyframes = faster Bayesian clearing
                'RGBD/LinearUpdate': '0',            # Take keyframe regardless of linear displacement (default=0.1m)
                'RGBD/AngularUpdate': '0',           # Take keyframe regardless of angular displacement (default=0.1rad)
                # Bayesian clearing math: steps_to_clear = log(ClampMax/(1-ClampMax)) / |log(ProbMiss/(1-ProbMiss))|
                # ProbClampingMax=0.7 → max logit=+0.85; ProbMiss=0.2 → miss step=-1.39 → ~1 keyframe to clear
                'GridGlobal/OccupancyThr': '0.5',      # Occupied when probability > 50%
                'GridGlobal/ProbMiss': '0.2',           # Aggressive miss decay (was 0.35) → clears in ~1 keyframe
                'GridGlobal/ProbHit': '0.7',            # Probability on a hit
                'GridGlobal/ProbClampingMax': '0.7',    # Lower max (was 0.9) → cells can't get deeply stuck, clears faster
                'GridGlobal/ProbClampingMin': '0.1',    # Min clamped probability
                # 🔽 Sliding-window cloud assembly: only keep the N nearest keyframes 🔽
                # cloud_map / cloud_obstacles now DROP old points as you move away
                # Use /rtabmap/octomap_obstacles in RViz for live-clearing 3D cloud
                'GridGlobal/MaxNodes': '70',            # Assemble map from only 50 nearest keyframes (0=all nodes=never forgets)
                'Optimizer/GravitySigma': '0',      # DISABLED — Madgwick drifts without magnetometer; continuous gravity drag rotates the entire map
                 }]
    
        # parameters=[{'frame_id':'oak-d-base-frame',
        #          'subscribe_rgbd':True,
        #          'subscribe_odom_info':True,
        #          'approx_sync':True,  # Disabled - C++ transformer is fast enough
        #          'wait_imu_to_init':True,  # Enable visual-inertial odometry
        #          'wait_for_transform': 0.5,
        #         # 🔽 Point cloud density reduction 🔽
        #         'cloud_voxel_size': 0.02,        # 2cm voxel grid filter (big reduction!)
        #         # 🔽 Map publishing control 🔽
        #         'Grid/RangeMax': '4.0',             # Only map points within 4m
        #         'Grid/CellSize': '0.01',            # 1cm grid cells (finer)
        #          }]

    # ── IMU topic pipeline ──────────────────────────────────────────────────────────────────
    # /imu            OAK-D hardware output, axis-corrected to oak-d-base-frame.
    #                 Contains raw accel + gyro. NO orientation quaternion.
    #                 At rest pointing level: linear_acceleration.z ≈ +9.81 m/s²
    #                 (+Z is the expected convention — accelerometers measure the reaction
    #                 force pushing UP against gravity, not the gravity vector itself).
    #
    # /imu/leveled    After gravity leveler: a one-time static rotation has been applied so
    #                 the measured gravity vector is EXACTLY [0, 0, +9.81] regardless of
    #                 floor tilt or IMU mounting bias. Still NO orientation quaternion.
    #
    # /imu/madgwick   Madgwick filter output on the leveled data: adds an orientation
    #                 quaternion computed from accel+gyro fusion. Yaw is arbitrary/drifting
    #                 (no magnetometer), but roll & pitch reliably track gravity.
    #                 ← THIS is what RTAB-Map subscribes to. wait_imu_to_init uses the
    #                   orientation once at startup to gravity-align the first camera pose.
    #                   After that, GravitySigma=0 means the optimizer never touches it again.
    # ────────────────────────────────────────────────────────────────────────────────────────
    remappings=[('imu', '/imu/madgwick')]

    # Expose max_depth_mm parameter for hardware depth clipping at camera level
    declare_max_depth_arg = DeclareLaunchArgument(
        'max_depth_mm', 
        default_value='4000', 
        description='Maximum depth in millimeters (hardware clipping at camera level)')

    return LaunchDescription([
        declare_max_depth_arg,

        # Launch camera driver with INLINE IMU TRANSFORMATION ENABLED + HARDWARE THRESHOLD FILTER
        # transform_imu_to_base_frame=True applies rotation at source (zero inter-node latency)
        # i_enable_threshold_filter=True clips depth on-device (min: 200mm, max: configurable)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory('depthai_examples'), 'launch'),
                '/test_stereo_inertial_node.launch.py']),
                launch_arguments={'depth_aligned': 'false',
                                  'enableRviz': 'false',
                                  'monoResolution': '480p',
                                  'stereo_fps': '20',  # OV7251 sensor: only 400p and 480p are supported (720p/800p are for OAK-D Pro/W)
                                  'transform_imu_to_base_frame': 'true',
                                  'i_enable_threshold_filter': 'true',
                                  'i_threshold_filter_min_range': '200',  # 20cm minimum
                                  'i_threshold_filter_max_range': LaunchConfiguration('max_depth_mm')
                                  }.items(),
        ),

        #Node(
        #    package='rtabmap_examples',
        #    executable='camera_reconnect_monitor.py',
        #    name='camera_watchdog',
        #    output='screen',
        #    parameters=[{
        #        'timeout_sec': 5.0,          # Match rgbd_sync warning threshold
        #        'restart_cooldown_sec': 10.0,  # Prevent restart spam
        #        'monitor_topic': '/right/image_rect'
        #    }]
        #),

        # NOTE: Separate C++ transformer node NOT needed when inline transform is enabled
        # IMU data from camera is already in oak-d-base-frame (transformed at source)
        # See the /imu pipeline comment above for full topic flow.
        # GravitySigma=0: IMU sets initial gravity-aligned pose ONLY — no continuous map rotation

        # ── Step 1: gravity leveler ──────────────────────────────────────────────────
        # Collects the first 50 accel samples at rest, computes the mean gravity vector
        # (floor tilt + static IMU bias included), then applies a fixed rotation to ALL
        # subsequent messages so Madgwick always sees gravity pointing exactly at +Z.
        # Input:  /imu  → Output: /imu/leveled
        Node(
            package='rtabmap_examples', executable='imu_gravity_leveler.py',
            name='imu_gravity_leveler', output='screen',
            parameters=[{'num_calibration_samples': 50}]),

        # Sync right/depth/camera_info together (depth already clipped by hardware)
        Node(   
            package='rtabmap_sync', executable='rgbd_sync', output='screen',
            parameters=parameters,
            remappings=[('rgb/image', '/right/image_rect'),
                        ('rgb/camera_info', '/right/camera_info'),
                        ('depth/image', '/stereo/depth')]),

        # ── Step 2: Madgwick orientation filter ─────────────────────────────────────
        # Fuses gravity-leveled accel+gyro into an orientation quaternion.
        # Roll & pitch are reliable (gravity-anchored). Yaw drifts (no magnetometer).
        # Input:  /imu/leveled  → Output: /imu/madgwick
        Node(
            package='imu_filter_madgwick', executable='imu_filter_madgwick_node', output='screen',
            parameters=[{'use_mag': False,
                         'world_frame': 'enu',
                         'publish_tf': False,
                         'use_best_effort_qos': True}],  # match BEST_EFFORT from gravity leveler
            remappings=[('imu/data_raw', '/imu/leveled'),
                        ('imu/data',     '/imu/madgwick')]),

        # Visual odometry
        Node(
            package='rtabmap_odom', executable='rgbd_odometry', output='screen',
            parameters=parameters,
            remappings=remappings),

        # VSLAM - ENABLED for full load testing
        Node(
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=parameters,
            remappings=remappings,
            arguments=['-d']),

       # # Visualization - ENABLED for full load testing
        Node(
            package='rtabmap_viz', executable='rtabmap_viz', output='screen',
            parameters=parameters,
            remappings=remappings),

        # ── Real-time depth → PointCloud2 for Nav2 costmap ──────────────────────────
        # depth_image_proc/point_cloud_xyz_node converts the live stereo depth image to
        # a PointCloud2 at the full camera frame rate (20Hz).  This is what Nav2's
        # VoxelLayer / ObstacleLayer subscribes to for real-time obstacle marking AND
        # clearing.  Without this, Nav2 never sees any sensor data (no LiDAR exists).
        #
        # Output topic: /depth_cloud/points  (frame: oak_right_camera_optical_frame)
        # Nav2 costmap does its own TF lookup to transform the cloud into odom/map.
        # ────────────────────────────────────────────────────────────────────────────
        Node(
            package='depth_image_proc',
            executable='point_cloud_xyz_node',
            name='depth_to_cloud',
            output='screen',
            remappings=[
                ('image_rect',  '/stereo/depth'),
                ('camera_info', '/right/camera_info'),
                ('points',      '/depth_cloud/points'),
            ],
            parameters=[{'queue_size': 5}],
        ),

        # Static TF: make odom->base_link identity so Nav2 local costmaps can start
        Node(
            package='tf2_ros', executable='static_transform_publisher', output='screen',
            arguments=['0','0','0','0','0','0','odom','base_link'],
        ),

        # Static TF: make map->odom identity for testing (map==odom==base_link)
        Node(
            package='tf2_ros', executable='static_transform_publisher', output='screen',
            arguments=['0','0','0','0','0','0','map','odom'],
        ),

        # Static TF: make base_link->oak_imu_frame identity so oak imu frame equals base
        Node(
            package='tf2_ros', executable='static_transform_publisher', output='screen',
            arguments=['0','0','0','0','0','0','base_link','oak_imu_frame'],
        ),


    ])
