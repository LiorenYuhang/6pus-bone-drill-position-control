import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_share = get_package_share_directory("pus6_bringup")
    h30_share = get_package_share_directory("h30_imu")
    default_config = os.path.join(bringup_share, "config", "system_params.yaml")

    mode_arg = DeclareLaunchArgument(
        "hardware_mode",
        default_value="mock",
        description="Hardware execution mode: 'mock' (safe default), 'read_only', or 'active'",
    )

    launch_imu_arg = DeclareLaunchArgument(
        "launch_imu",
        default_value="false",
        description="Whether to launch the legacy single H30 IMU node",
    )

    launch_dual_imu_arg = DeclareLaunchArgument(
        "launch_dual_imu",
        default_value="false",
        description="Whether to launch dual timed H30 IMU nodes without relative attitude",
    )

    launch_relative_imu_arg = DeclareLaunchArgument(
        "launch_relative_imu",
        default_value="false",
        description="Whether to launch dual timed H30 IMU nodes plus relative attitude observer",
    )

    launch_fk_arg = DeclareLaunchArgument(
        "launch_fk",
        default_value="true",
        description="Whether to launch pus6_fk_node",
    )

    fixed_lag_delay_arg = DeclareLaunchArgument(
        "fixed_lag_delay_ms",
        default_value="160.0",
        description="Fixed-Lag query delay in ms",
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

    fk_node = Node(
        package="pus6_kinematics",
        executable="fk_node",
        name="pus6_fk_node",
        output="screen",
        parameters=[
            LaunchConfiguration("config_file"),
            {"fixed_lag_delay_ms": LaunchConfiguration("fixed_lag_delay_ms")},
        ],
        condition=IfCondition(LaunchConfiguration("launch_fk")),
    )

    imu_node = Node(
        package="h30_imu",
        executable="h30_imu_node",
        name="h30_imu_node",
        output="screen",
        condition=IfCondition(LaunchConfiguration("launch_imu")),
    )

    dual_imu_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(h30_share, "launch", "h30_dual.launch.py")),
        condition=IfCondition(LaunchConfiguration("launch_dual_imu")),
    )

    relative_imu_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(h30_share, "launch", "h30_relative.launch.py")),
        condition=IfCondition(LaunchConfiguration("launch_relative_imu")),
    )

    return LaunchDescription([
        mode_arg,
        launch_imu_arg,
        launch_dual_imu_arg,
        launch_relative_imu_arg,
        launch_fk_arg,
        fixed_lag_delay_arg,
        config_arg,
        motor_driver_node,
        kinematics_node,
        control_node,
        fk_node,
        imu_node,
        dual_imu_launch,
        relative_imu_launch,
    ])
