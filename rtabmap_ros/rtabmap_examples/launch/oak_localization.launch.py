# Localization-only launch — loads an existing RTAB-Map database and localizes
# within it WITHOUT adding new nodes or modifying the map.
#
# Usage:
#   ros2 launch rtabmap_examples oak_localization.launch.py
#   ros2 launch rtabmap_examples oak_localization.launch.py database_path:=/home/ggsya/.ros/my_map.db
#   ros2 launch rtabmap_examples oak_localization.launch.py database_path:=/home/ggsya/maps/lab_v2.db max_depth_mm:=3000
#
# The map is read-only: RTAB-Map will localize but never write new nodes to the DB.
# Loop closures and the map→odom TF correction still happen — only new keyframe
# creation is suppressed.

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    # ── Declare launch arguments ─────────────────────────────────────────────────
    declare_database_arg = DeclareLaunchArgument(
        'database_path',
        default_value=os.path.expanduser('~/.ros/rtabmap.db'),
        description='Absolute path to the .db map file to localize in')

    declare_max_depth_arg = DeclareLaunchArgument(
        'max_depth_mm',
        default_value='4000',
        description='Maximum depth in millimeters (hardware clipping at camera level)')

    # ── RTAB-Map / odometry parameters ──────────────────────────────────────────
    # Identical sensor/grid settings as oak_temp.launch.py; only the memory/SLAM
    # section differs (IncrementalMemory=false turns on pure localization mode).
    parameters = [{
        'frame_id':             'oak-d-base-frame',
        'subscribe_rgbd':       True,
        'subscribe_odom_info':  True,
        'approx_sync':          True,
        'wait_imu_to_init':     True,
        'wait_for_transform':   0.5,

        # ── Localization-mode keys ───────────────────────────────────────────────
        # Mem/IncrementalMemory=false  → no new nodes added; read-only localization
        # Mem/InitWMWithAllNodes=true  → load every node from the DB into working
        #                                memory at startup so loop-closure search
        #                                covers the entire saved map immediately.
        # RGBD/StartAtOrigin=false     → start from the best-matching location in
        #                                the map (global re-localization at launch).
        # Mem/LocalizationReadOnly     → never write back to the DB file.
        'Mem/IncrementalMemory':    'false',
        'Mem/InitWMWithAllNodes':   'true',
        'RGBD/StartAtOrigin':       'false',
        'Mem/LocalizationReadOnly': 'true',
        'database_path':            LaunchConfiguration('database_path'),

        # ── Point cloud ──────────────────────────────────────────────────────────
        'cloud_voxel_size': 0.03,

        # ── Occupancy grid ───────────────────────────────────────────────────────
        'Grid/RangeMax':          '4.0',
        'Grid/CellSize':          '0.03',
        'Grid/MaxObstacleHeight': '0.6',
        'Grid/MinGroundHeight':   '-0.05',
        'Grid/MaxGroundAngle':    '25',
        'Grid/RayTracing':        'true',
        'Grid/3D':                'true',
        'Grid/ClusterRadius':     '0.20',
        'Grid/MinClusterSize':    '5',
        'Grid/NoiseFilteringRadius':      '0.20',
        'Grid/NoiseFilteringMinNeighbors': '5',

        # ── Detection rate / update thresholds ───────────────────────────────────
        'Rtabmap/DetectionRate': '5',
        'RGBD/LinearUpdate':     '0',
        'RGBD/AngularUpdate':    '0',

        # ── Bayesian occupancy ───────────────────────────────────────────────────
        'GridGlobal/OccupancyThr':    '0.5',
        'GridGlobal/ProbMiss':        '0.2',
        'GridGlobal/ProbHit':         '0.7',
        'GridGlobal/ProbClampingMax': '0.7',
        'GridGlobal/ProbClampingMin': '0.1',
        'GridGlobal/MaxNodes':        '70',

        # ── Gravity / optimizer ──────────────────────────────────────────────────
        'Optimizer/GravitySigma': '0',
    }]

    remappings = [('imu', '/imu/madgwick')]

    # ────────────────────────────────────────────────────────────────────────────
    return LaunchDescription([
        declare_database_arg,
        declare_max_depth_arg,

        # ── Camera driver ────────────────────────────────────────────────────────
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(get_package_share_directory('depthai_examples'), 'launch'),
                '/test_stereo_inertial_node.launch.py']),
            launch_arguments={
                'depth_aligned':                  'false',
                'enableRviz':                     'false',
                'monoResolution':                 '480p',
                'stereo_fps':                     '20',
                'transform_imu_to_base_frame':    'true',
                'i_enable_threshold_filter':      'true',
                'i_threshold_filter_min_range':   '200',
                'i_threshold_filter_max_range':   LaunchConfiguration('max_depth_mm'),
            }.items(),
        ),

        # ── IMU gravity leveler ──────────────────────────────────────────────────
        Node(
            package='rtabmap_examples', executable='imu_gravity_leveler.py',
            name='imu_gravity_leveler', output='screen',
            parameters=[{'num_calibration_samples': 50}]),

        # ── RGBD sync ────────────────────────────────────────────────────────────
        Node(
            package='rtabmap_sync', executable='rgbd_sync', output='screen',
            parameters=parameters,
            remappings=[
                ('rgb/image',       '/right/image_rect'),
                ('rgb/camera_info', '/right/camera_info'),
                ('depth/image',     '/stereo/depth'),
            ]),

        # ── Madgwick orientation filter ──────────────────────────────────────────
        Node(
            package='imu_filter_madgwick', executable='imu_filter_madgwick_node', output='screen',
            parameters=[{
                'use_mag':             False,
                'world_frame':         'enu',
                'publish_tf':          False,
                'use_best_effort_qos': True,
            }],
            remappings=[
                ('imu/data_raw', '/imu/leveled'),
                ('imu/data',     '/imu/madgwick'),
            ]),

        # ── Visual odometry ──────────────────────────────────────────────────────
        Node(
            package='rtabmap_odom', executable='rgbd_odometry', output='screen',
            parameters=parameters,
            remappings=remappings),

        # ── RTAB-Map SLAM node (localization mode — NO -d flag) ──────────────────
        # NOTE: No '-d' argument here. '-d' deletes the database on launch.
        #       In localization mode we MUST load the existing DB unchanged.
        Node(
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=parameters,
            remappings=remappings),
            # arguments=['-d']  ← intentionally omitted: do NOT wipe the map

        # ── Visualization ────────────────────────────────────────────────────────
        Node(
            package='rtabmap_viz', executable='rtabmap_viz', output='screen',
            parameters=parameters,
            remappings=remappings),

        # ── Real-time depth → PointCloud2 for Nav2 costmap ──────────────────────
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

        # ── Static TFs ───────────────────────────────────────────────────────────
        Node(
            package='tf2_ros', executable='static_transform_publisher', output='screen',
            arguments=['0','0','0','0','0','0','odom','base_link']),
        Node(
            package='tf2_ros', executable='static_transform_publisher', output='screen',
            arguments=['0','0','0','0','0','0','map','odom']),
        Node(
            package='tf2_ros', executable='static_transform_publisher', output='screen',
            arguments=['0','0','0','0','0','0','base_link','oak_imu_frame']),
    ])
