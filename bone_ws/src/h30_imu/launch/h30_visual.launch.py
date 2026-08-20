from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    launch_dir = Path(get_package_share_directory('h30_imu')) / 'launch'
    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(launch_dir / 'h30.launch.py'))),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(launch_dir / 'visualize.launch.py'))),
    ])
