from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    default_config = str(Path(get_package_share_directory('h30_imu')) / 'config' / 'h30.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=default_config),
        DeclareLaunchArgument('namespace', default_value=''),
        DeclareLaunchArgument('node_name', default_value='h30_imu'),
        DeclareLaunchArgument('port', default_value='/dev/serial/by-id/usb-1a86_USB_Single_Serial_5B32028000-if00'),
        DeclareLaunchArgument('csv_path', default_value=''),
        DeclareLaunchArgument('terminal_rate_hz', default_value='5.0'),
        DeclareLaunchArgument('frame_id', default_value='imu_link'),
        DeclareLaunchArgument('world_frame', default_value='world'),
        DeclareLaunchArgument('imu_topic', default_value='imu/data'),
        DeclareLaunchArgument('euler_topic', default_value='imu/euler'),
        DeclareLaunchArgument('temperature_topic', default_value='imu/temperature'),
        DeclareLaunchArgument('publish_tf', default_value='true'),
        Node(package='h30_imu', executable='h30_imu_node',
             namespace=LaunchConfiguration('namespace'),
             name=LaunchConfiguration('node_name'), output='screen',
             parameters=[LaunchConfiguration('config'), {
                 'port': LaunchConfiguration('port'),
                 'csv_path': LaunchConfiguration('csv_path'),
                 'frame_id': LaunchConfiguration('frame_id'),
                 'world_frame': LaunchConfiguration('world_frame'),
                 'imu_topic': LaunchConfiguration('imu_topic'),
                 'euler_topic': LaunchConfiguration('euler_topic'),
                 'temperature_topic': LaunchConfiguration('temperature_topic'),
                 'publish_tf': ParameterValue(LaunchConfiguration('publish_tf'), value_type=bool),
                 'terminal_rate_hz': ParameterValue(
                     LaunchConfiguration('terminal_rate_hz'), value_type=float),
             }]),
    ])
