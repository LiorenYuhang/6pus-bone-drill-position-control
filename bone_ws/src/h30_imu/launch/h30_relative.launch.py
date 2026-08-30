from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory('h30_imu'))
    return LaunchDescription([
        DeclareLaunchArgument(
            'relative_config',
            default_value=str(share / 'config' / 'h30_relative_attitude.yaml')),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(share / 'launch' / 'h30_dual.launch.py')),
        ),
        Node(
            package='h30_imu',
            executable='h30_relative_attitude_node',
            name='h30_relative_attitude',
            output='screen',
            parameters=[LaunchConfiguration('relative_config')],
        ),
    ])
