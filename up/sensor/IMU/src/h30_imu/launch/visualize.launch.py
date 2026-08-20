from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'output_dir',
            default_value=PathJoinSubstitution([
                EnvironmentVariable('HOME'), 'bone_drill', 'up', 'sensor',
                'IMU', 'data'
            ]),
            description='Directory used for timestamped IMU CSV files',
        ),
        Node(
            package='h30_imu', executable='imu_csv_visualizer',
            name='imu_csv_visualizer', output='screen',
            parameters=[{'output_dir': LaunchConfiguration('output_dir')}],
        ),
    ])
