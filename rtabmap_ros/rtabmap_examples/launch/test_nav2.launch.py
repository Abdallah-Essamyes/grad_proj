import os

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from pathlib import Path


def generate_launch_description():

    params_file = LaunchConfiguration('params_file')

    declare_params = DeclareLaunchArgument(
        'params_file',
        default_value=str(Path(__file__).resolve().parent.parent / "config" /"nav2_params_liveSLAM.yaml"),
        description='Full path to the ROS2 parameters file for Nav2'
    )

    # Static TFs for simple testing
    static_tf_map_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_map_odom',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
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
                'planner_server'
            ]
        }],
    )

    publish_costmap = Node(
        package='rtabmap_examples',
        executable='publish_costmap_from_service.py',
        name='publish_costmap',
        output='screen',
        arguments=['--frequency', '1'],
        )

    return LaunchDescription([
        declare_params,
        static_tf_map_odom,
        static_tf_odom_base,
        global_costmap,
        local_costmap,
        planner_server,
        lifecycle_manager,
        #publish_costmap,
    ])
