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
        DeclareLaunchArgument('base_csv_path', default_value=''),
        DeclareLaunchArgument('platform_csv_path', default_value=''),
        DeclareLaunchArgument('terminal_rate_hz', default_value='5.0'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(share / 'launch' / 'h30_dual.launch.py')),
            launch_arguments={
                'base_csv_path': LaunchConfiguration('base_csv_path'),
                'platform_csv_path': LaunchConfiguration('platform_csv_path'),
                'terminal_rate_hz': LaunchConfiguration('terminal_rate_hz'),
            }.items()),
        Node(
            package='h30_imu',
            executable='h30_relative_attitude_node',
            name='h30_relative_attitude',
            output='screen',
            parameters=[LaunchConfiguration('relative_config')],
        ),
    ])
