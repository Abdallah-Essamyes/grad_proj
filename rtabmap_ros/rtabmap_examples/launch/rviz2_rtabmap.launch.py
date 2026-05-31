from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os


def generate_launch_description():
    rviz_config = os.path.abspath(
        os.path.join(os.path.dirname(__file__), '..', 'config', 'rviz_rtabmap.rviz')
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false', description='Use simulation time'),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            # use --display-config and set window geometry to be wider
            arguments=['--display-config', rviz_config, '--geometry', '1600x900'],
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        ),
    ])
