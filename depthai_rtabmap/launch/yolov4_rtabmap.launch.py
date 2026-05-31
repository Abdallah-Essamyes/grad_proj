"""
yolov4_rtabmap.launch.py

Single-process launch that combines:
  - yolov4_rtabmap_node  (OAK-D driver: color + depth + YOLO + right mono + IMU)
  - IMU gravity leveler  (static tilt correction)
  - Madgwick filter      (orientation quaternion from accel+gyro)
  - RTAB-Map rgbd_sync   (syncs color/image 1280×720 + stereo/depth 1280×720)
  - rtabmap_odom         (rgbd visual-inertial odometry)
  - rtabmap              (SLAM)
  - rtabmap_viz          (optional 3D visualizer)
  - depth → PointCloud2  (for Nav2 costmap)

Depth is always aligned to CAM_A (color camera, 1280×720).
YOLO spatial detections run on the 416×416 RGB preview with CAM_A-aligned depth.
rgbd_sync uses color/image (1280×720) + stereo/depth (1280×720) — exact dimension match.
Depth is wired directly from the stereo node (not YOLO passthrough) so it
publishes at full stereo frame rate.
"""


# THIS FILE WAS MANUALLY SYMLINKED TO THE INSTALL/SHARE/DEPTHAI_RTABMAP/LAUNCH/ FOLDER BECAUSE FUCK TS IT WONT SYMLINK INSTALL BY COLCON CUZ FUCK ME IG

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    depthai_examples_share = get_package_share_directory('depthai_examples')
    default_resources_path = os.path.join(depthai_examples_share, 'resources')

    # ── Launch arguments ──────────────────────────────────────────────────
    declare_mono_res = DeclareLaunchArgument(
        'monoResolution', default_value='480p',
        description='Mono camera resolution: 400p | 480p | 720p | 800p. '
                    'OAK-D-Lite supports only 400p/480p.')

    declare_max_depth = DeclareLaunchArgument(
        'max_depth_mm', default_value='4000',
        description='Maximum depth in mm passed to RTAB-Map Grid/RangeMax '
                    '(informational; depth hardware clipping is not available '
                    'via this driver — use the threshold filter in the spatial node).')

    declare_stereo_fps = DeclareLaunchArgument(
        'stereo_fps', default_value='10',
        description='Target stereo frame rate (frames per second).')

    declare_viz = DeclareLaunchArgument(
        'viz', default_value='True',
        description='Launch rtabmap_viz 3D visualizer (adds significant CPU load).')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation clock.')

    declare_greyscale_slam = DeclareLaunchArgument(
        'greyscale_slam', default_value='false',
        description="'true' = RTAB-Map uses right mono (greyscale) + stereo/depth_right "
                    "(CAM_C-aligned depth, same resolution as right/image_rect). "
                    "'false' = RTAB-Map uses color (1280x720) + stereo/depth (CAM_A-aligned). "
                    "YOLO always uses the 416x416 color preview with CAM_A depth regardless.")

    # ── RTAB-Map shared parameters ────────────────────────────────────────
    # Depth is aligned to CAM_A (color camera, 1280×720).
    # rgbd_sync uses color/image (1280×720) + stereo/depth (1280×720) — exact match,
    # no dimension mismatch. YOLO spatial detections also use CAM_A-aligned depth.
    rtabmap_parameters = [{
        'frame_id': 'oak-d-base-frame',
        'subscribe_rgbd': True,
        'subscribe_odom_info': True,
        'approx_sync': True,
        # Allow up to 150 ms timestamp gap between color (~22 Hz) and depth (~15 Hz).
        # Without this, rgbd_sync spams a warning on every pair because the two
        # cameras have independent clocks and natural diffs of 33-133 ms.
        'approx_sync_max_interval': 0.15,
        # wait_imu_to_init=False: Madgwick needs ~5s to converge. During that window
        # the orientation output is wrong, causing the IMU pose guess to drift 17°+
        # and making registration fail on every frame. Starting without IMU init lets
        # visual odometry initialize cleanly; IMU is still fused once it stabilises.
        'wait_imu_to_init': True,
        'wait_for_transform': 0.5,
        'use_sim_time': LaunchConfiguration('use_sim_time'),

        # ── Point cloud density ──────────────────────────────────────────
        'cloud_decimation': 2,        # take every 4th pixel when projecting depth → cloud
        'cloud_voxel_size': 0.05,     # voxel-downsample assembled map cloud to 5 cm

        # ── Occupancy grid ───────────────────────────────────────────────
        'Grid/RangeMax':              '4.0',
        'Grid/CellSize':              '0.03',
        'Grid/MaxObstacleHeight':     '0.6',
        'Grid/MinGroundHeight':       '-0.05',
        'Grid/MaxGroundAngle':        '25',
        'Grid/RayTracing':            'true',
        'Grid/3D':                    'true',
        'Grid/ClusterRadius':         '0.20',
        'Grid/MinClusterSize':        '5',
        'Grid/NoiseFilteringRadius':  '0.20',
        'Grid/NoiseFilteringMinNeighbors': '5',

        # ── Odometry: color/image is now 1280×720 (ISP-scaled via setIspScale(2,3)).
        # Decimation 2 → 640×360, which matches the stereo depth output at 720p.
        # For greyscale mode (right mono 640×480), decimation 2 → 320×240 (still fine).
        'Odom/ImageDecimation': '2',
        # Lower min-inliers: stereo depth is sparser than RGB-D (default 20 is too tight)
        'Vis/MinInliers': '10',

        # ── Detection rate & keyframe policy ─────────────────────────────
        'Rtabmap/DetectionRate': '5',  # 2 Hz — was 5 Hz, major CPU reduction
        'RGBD/LinearUpdate':     '0',
        'RGBD/AngularUpdate':    '0',

        # ── Bayesian occupancy (fast clearing) ────────────────────────────
        'GridGlobal/OccupancyThr':     '0.5',
        'GridGlobal/ProbMiss':         '0.2',
        'GridGlobal/ProbHit':          '0.7',
        'GridGlobal/ProbClampingMax':  '0.7',
        'GridGlobal/ProbClampingMin':  '0.1',

        # ── Sliding-window assembly ───────────────────────────────────────
        'GridGlobal/MaxNodes': '30',

        # ── IMU gravity optimizer disabled (no magnetometer, Madgwick drifts) ─
        'Optimizer/GravitySigma': '0',
    }]

    # IMU topic remapping — odometry + slam subscribe to 'imu', remapped to madgwick output
    imu_remapping = [('imu', '/imu/madgwick')]

    # ── Nodes ─────────────────────────────────────────────────────────────

    # 1. OAK-D combined driver: YOLO + color + depth + right mono + IMU
    yolov4_rtabmap_node = Node(
        package='depthai_rtabmap',
        executable='yolov4_rtabmap_node',
        name='yolov4_rtabmap_node',
        output='screen',
        parameters=[{
            'tf_prefix': 'oak',
            'monoResolution': LaunchConfiguration('monoResolution'),
            'resourceBaseFolder': default_resources_path,
            # Enable inline IMU axis transformation (BNO086 → ROS REP-103).
            # Same rotation applied in test_stereo_inertial_node when
            # transform_imu_to_base_frame=true:
            #   ros_x = -sensor_z,  ros_y = -sensor_x,  ros_z = sensor_y
            
            'transform_imu_to_base_frame': True,
            'imuMode': 1,
            'angularVelCovariance': 0.02,
            'linearAccelCovariance': 0.0,
            'sync_nn': True,
            'subpixel': True,
            'confidence': 200,
            'LRchecktresh': 5,
            # depth_align_to_mono mirrors greyscale_slam:
            #   false → depth aligned to CAM_A (color frame, 1280×720)
            #   true  → depth aligned to CAM_C (right mono frame, 640×480)
            'depth_align_to_mono': False,
        }],
    )

    # 2. IMU gravity leveler — collects 50 samples at rest, removes static tilt/bias.
    #    Input: /imu  →  Output: /imu/leveled
    imu_gravity_leveler = Node(
        package='rtabmap_examples',
        executable='imu_gravity_leveler.py',
        name='imu_gravity_leveler',
        output='screen',
        parameters=[{'num_calibration_samples': 50}],
    )

    # 3. Madgwick filter — fuses gravity-leveled accel+gyro into orientation quaternion.
    #    Input: /imu/leveled  →  Output: /imu/madgwick
    imu_madgwick = Node(
        package='imu_filter_madgwick',
        executable='imu_filter_madgwick_node',
        name='imu_filter_madgwick',
        output='screen',
        parameters=[{
            'use_mag': False,
            'world_frame': 'enu',
            'publish_tf': False,
            'use_best_effort_qos': True,
        }],
        remappings=[
            ('imu/data_raw', '/imu/leveled'),
            ('imu/data',     '/imu/madgwick'),
        ],
    )

    # 4. rgbd_sync — COLOR mode (greyscale_slam=false, default)
    #     color/image (1280×720) + stereo/depth (1280×720, CAM_A). Exact size match.
    rgbd_sync = Node(
        package='rtabmap_sync',
        executable='rgbd_sync',
        name='rgbd_sync',
        output='screen',
        parameters=rtabmap_parameters,
        condition=UnlessCondition(LaunchConfiguration('greyscale_slam')),
        remappings=[
            ('rgb/image',       '/color/image'),
            ('rgb/camera_info', '/color/camera_info'),
            ('depth/image',     '/stereo/depth'),
        ],
    )

    # 4b. rgbd_sync — GREYSCALE mode (greyscale_slam=true)
    #     right/image_rect (640×480 greyscale) + stereo/depth_right (640×480, CAM_C).
    #     Both topics share CAM_C intrinsics → exact dimension and frame match.
    rgbd_sync_grey = Node(
        package='rtabmap_sync',
        executable='rgbd_sync',
        name='rgbd_sync',
        output='screen',
        parameters=rtabmap_parameters,
        condition=IfCondition(LaunchConfiguration('greyscale_slam')),
        remappings=[
            ('rgb/image',       '/right/image_rect'),
            ('rgb/camera_info', '/right/camera_info'),
            ('depth/image',     '/stereo/depth_right'),
        ],
    )

    # 5. Visual-inertial odometry
    rgbd_odometry = Node(
        package='rtabmap_odom',
        executable='rgbd_odometry',
        name='rgbd_odometry',
        output='screen',
        parameters=rtabmap_parameters,
        remappings=imu_remapping,
    )

    # 6. RTAB-Map SLAM
    rtabmap = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        parameters=rtabmap_parameters,
        remappings=imu_remapping,
        arguments=['-d'],
    )

    # 7. RTAB-Map visualizer (optional — disabled by default to reduce CPU/temp)
    rtabmap_viz = Node(
        package='rtabmap_viz',
        executable='rtabmap_viz',
        name='rtabmap_viz',
        output='screen',
        parameters=rtabmap_parameters,
        remappings=imu_remapping,
        condition=IfCondition(LaunchConfiguration('viz')),
    )

    # 8. Real-time depth → PointCloud2 for Nav2 costmap.
    #    stereo/depth publishes at full stereo rate (direct from stereo node, not YOLO passthrough).
    #    stereo/camera_info is published by the same BridgePublisher as stereo/depth,
    #    so timestamps always match — avoids the approx_sync mismatch with color/camera_info.
    depth_to_cloud = Node(
        package='depth_image_proc',
        executable='point_cloud_xyz_node',
        name='depth_to_cloud',
        output='screen',
        remappings=[
            ('image_rect',  '/stereo/depth'),
            ('camera_info', '/stereo/camera_info'),
            ('points',      '/depth_cloud/points'),
        ],
        parameters=[{'queue_size': 10}],
    )

    # 9. Static TF: odom → base_link (identity, robot base provides the real one)
    static_odom_base = Node(
        package='tf2_ros', executable='static_transform_publisher', output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link'],
    )

    # 10. Static TF: map → odom (identity, rtabmap corrects this at runtime)
    static_map_odom = Node(
        package='tf2_ros', executable='static_transform_publisher', output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
    )

    # 11. Static TF: base_link → oak_imu_frame (identity — IMU is already
    #     transformed to oak-d-base-frame by the driver, so rtabmap treats
    #     imu_frame == base_frame)
    static_base_imu = Node(
        package='tf2_ros', executable='static_transform_publisher', output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'oak_imu_frame'],
    )

    return LaunchDescription([
        declare_mono_res,
        declare_max_depth,
        declare_stereo_fps,
        declare_use_sim_time,
        declare_viz,
        declare_greyscale_slam,

        yolov4_rtabmap_node,
        imu_gravity_leveler,
        imu_madgwick,
        rgbd_sync,
        rgbd_sync_grey,
        rgbd_odometry,
        rtabmap,
        rtabmap_viz,
        depth_to_cloud,
        static_odom_base,
        static_map_odom,
        static_base_imu,
    ])
