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

    servo_scheduler_mode_arg = DeclareLaunchArgument(
        "servo_scheduler_mode",
        default_value="ros_timer",
        description="Servo scheduler mode: 'ros_timer' (default) or 'dedicated_thread'",
    )

    telemetry_publish_mode_arg = DeclareLaunchArgument(
        "telemetry_publish_mode",
        default_value="auto",
        description="Telemetry publication mode: 'auto' (default: inline for ros_timer, async_latest for dedicated_thread), 'inline', or 'async_latest'",
    )

    feedback_loss_policy_arg = DeclareLaunchArgument(
        "feedback_loss_policy",
        default_value="default",
        description="Feedback loss policy: 'default' (J7/J8 strict 4-miss gate) or 'age_based' (J11 research mode: per-axis age gate)",
    )

    motor_driver_node = Node(
        package="pus6_motor_driver",
        executable="motor_driver_node",
        name="pus6_motor_driver_node",
        output="screen",
        parameters=[
            LaunchConfiguration("config_file"),
            {
                "hardware_mode": LaunchConfiguration("hardware_mode"),
                "servo_scheduler_mode": LaunchConfiguration("servo_scheduler_mode"),
                "telemetry_publish_mode": LaunchConfiguration("telemetry_publish_mode"),
                "feedback_loss_policy": LaunchConfiguration("feedback_loss_policy"),
            },
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

    return LaunchDescription([
        mode_arg,
        launch_imu_arg,
        launch_fk_arg,
        fixed_lag_delay_arg,
        config_arg,
        servo_scheduler_mode_arg,
        telemetry_publish_mode_arg,
        feedback_loss_policy_arg,
        motor_driver_node,
        kinematics_node,
        control_node,
        fk_node,
        imu_node,
    ])
