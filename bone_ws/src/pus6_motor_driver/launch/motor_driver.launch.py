import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("pus6_motor_driver")
    default_config = os.path.join(pkg_share, "config", "motors.yaml")

    mode_arg = DeclareLaunchArgument(
        "hardware_mode",
        default_value="mock",
        description="Hardware execution mode: 'mock' (safe default), 'read_only', or 'active'",
    )

    config_arg = DeclareLaunchArgument(
        "config_file",
        default_value=default_config,
        description="Path to motor driver config yaml",
    )

    driver_node = Node(
        package="pus6_motor_driver",
        executable="motor_driver_node",
        name="pus6_motor_driver_node",
        output="screen",
        parameters=[
            LaunchConfiguration("config_file"),
            {"hardware_mode": LaunchConfiguration("hardware_mode")},
        ],
    )

    return LaunchDescription([
        mode_arg,
        config_arg,
        driver_node,
    ])
