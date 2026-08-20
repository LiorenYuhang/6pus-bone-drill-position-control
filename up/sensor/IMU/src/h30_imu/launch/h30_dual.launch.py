from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


BASE_PORT = '/dev/serial/by-id/usb-1a86_USB_Single_Serial_5B32028000-if00'
PLATFORM_PORT = '/dev/serial/by-id/usb-1a86_USB_Single_Serial_5B0B029000-if00'


def generate_launch_description():
    default_config = str(Path(get_package_share_directory('h30_imu')) / 'config' / 'h30.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=default_config),
        DeclareLaunchArgument('base_csv_path', default_value=''),
        DeclareLaunchArgument('platform_csv_path', default_value=''),
        DeclareLaunchArgument('terminal_rate_hz', default_value='5.0'),
        Node(
            package='h30_imu',
            executable='h30_imu_node',
            namespace='pus6/imu_base',
            name='h30_imu',
            output='screen',
            parameters=[LaunchConfiguration('config'), {
                'port': BASE_PORT,
                'baud_rate': 460800,
                'frame_id': 'pus6_base_imu_link',
                'world_frame': 'world',
                'imu_topic': '/pus6/imu_base/data',
                'euler_topic': '/pus6/imu_base/euler',
                'temperature_topic': '/pus6/imu_base/temperature',
                'timed_topic': '/pus6/imu_base/timed',
                'publish_euler': True,
                'publish_tf': False,
                'csv_path': LaunchConfiguration('base_csv_path'),
                'terminal_rate_hz': ParameterValue(
                    LaunchConfiguration('terminal_rate_hz'), value_type=float),
            }],
        ),
        Node(
            package='h30_imu',
            executable='h30_imu_node',
            namespace='pus6/imu_platform',
            name='h30_imu',
            output='screen',
            parameters=[LaunchConfiguration('config'), {
                'port': PLATFORM_PORT,
                'baud_rate': 460800,
                'frame_id': 'pus6_platform_imu_link',
                'world_frame': 'world',
                'imu_topic': '/pus6/imu_platform/data',
                'euler_topic': '/pus6/imu_platform/euler',
                'temperature_topic': '/pus6/imu_platform/temperature',
                'timed_topic': '/pus6/imu_platform/timed',
                'publish_euler': True,
                'publish_tf': False,
                'csv_path': LaunchConfiguration('platform_csv_path'),
                'terminal_rate_hz': ParameterValue(
                    LaunchConfiguration('terminal_rate_hz'), value_type=float),
            }],
        ),
    ])
