from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    default_config = str(Path(get_package_share_directory('h30_imu')) / 'config' / 'h30.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=default_config),
        DeclareLaunchArgument('port', default_value='/dev/serial/by-id/usb-1a86_USB_Single_Serial_5B32028000-if00'),
        Node(package='h30_imu', executable='h30_imu_node', name='h30_imu', output='screen',
             parameters=[LaunchConfiguration('config'), {'port': LaunchConfiguration('port')}]),
    ])
