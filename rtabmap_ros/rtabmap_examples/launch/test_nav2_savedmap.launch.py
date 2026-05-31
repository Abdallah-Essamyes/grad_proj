import os

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.actions import TimerAction, ExecuteProcess


def generate_launch_description():
    params_file = LaunchConfiguration('params_file')
    map_file = LaunchConfiguration('map')
    declare_params = DeclareLaunchArgument(
        'params_file',
        default_value="/home/ggsya/ros_ws/src/rtabmap_ros/rtabmap_examples/config/nav2_params.yaml",
        description='Full path to the ROS2 parameters file for Nav2'
    )
    declare_map = DeclareLaunchArgument(
        'map',
        default_value="/home/ggsya/ros_ws/src/rtabmap_ros/rtabmap_examples/scripts/nav2_white_map.yaml",
        description='Full path to map yaml file to load (overrides yaml_filename in params)'
    )

    # Minimal set: map server, costmap server (global), planner server, and lifecycle manager
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[params_file, {'yaml_filename': map_file}],
    )

    # Static TFs for simple testing
    static_tf_map_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_map_odom',
        output='screen',
        arguments=['0.1', '0.1', '0.1', '0', '0', '0', 'map', 'odom'],
    )

    static_tf_odom_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_odom_base_link',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link'],
    )

    global_costmap = Node(
    package='nav2_costmap_2d',
    executable='nav2_costmap_2d',
    namespace='global_costmap',
    name='global_costmap',
    parameters=[params_file],
    output='screen'
)

    local_costmap = Node(
        package='nav2_costmap_2d',
        executable='nav2_costmap_2d',
        namespace='local_costmap',
        name='local_costmap',
        parameters=[params_file],
        output='screen'
    )

    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[params_file],
    )


    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'autostart': True,
            'node_names': [
                'map_server',
                'planner_server',
            ]
        }],
    )

    # After nodes are up, explicitly request configure->activate transitions
    # Start a small helper that reads the GetCostmap service and republishes
    # it on /global_costmap/costmap at a given frequency (Hz).
    publish_costmap = Node(
        package='rtabmap_examples',
        executable='publish_costmap_from_service.py',
        name='publish_costmap',
        output='screen',
        arguments=['--frequency', '1'],
        )

    return LaunchDescription([
        declare_params,
        declare_map,
        # map server needs the yaml_filename parameter; allow overriding via 'map' launch arg
        map_server,
        planner_server,
        static_tf_map_odom,
        static_tf_odom_base,
        lifecycle_manager,
        publish_costmap,
    ])
