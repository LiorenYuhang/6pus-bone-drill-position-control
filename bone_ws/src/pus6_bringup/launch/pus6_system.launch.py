import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_share = get_package_share_directory("pus6_bringup")
    default_config = os.path.join(bringup_share, "config", "system_params.yaml")

    mode_arg = DeclareLaunchArgument(
        "hardware_mode",
        default_value="mock",
        description="Hardware execution mode: 'mock' (safe default), 'read_only', or 'active'",
    )

    launch_imu_arg = DeclareLaunchArgument(
        "launch_imu",
        default_value="false",
        description="Whether to launch H30 IMU node",
    )

    config_arg = DeclareLaunchArgument(
        "config_file",
        default_value=default_config,
        description="Path to system parameters yaml",
    )

    motor_driver_node = Node(
        package="pus6_motor_driver",
        executable="motor_driver_node",
        name="pus6_motor_driver_node",
        output="screen",
        parameters=[
            LaunchConfiguration("config_file"),
            {"hardware_mode": LaunchConfiguration("hardware_mode")},
        ],
    )

    kinematics_node = Node(
        package="pus6_kinematics",
        executable="kinematics_node",
        name="pus6_kinematics_node",
        output="screen",
        parameters=[LaunchConfiguration("config_file")],
    )

    control_node = Node(
        package="pus6_control",
        executable="control_node",
        name="pus6_control_node",
        output="screen",
        parameters=[LaunchConfiguration("config_file")],
    )

    imu_node = Node(
        package="h30_imu",
        executable="h30_imu_node",
        name="h30_imu_node",
        output="screen",
        condition=IfCondition(LaunchConfiguration("launch_imu")),
    )

    return LaunchDescription([
        mode_arg,
        launch_imu_arg,
        config_arg,
        motor_driver_node,
        kinematics_node,
        control_node,
        imu_node,
    ])
