"""pus6_motor_driver: ROS 2 driver package for 6PUS Inspire electric cylinders."""

from .motor import Motor
from .protocol import StatusFeedback
from .safety import (
    MOTOR_IDS,
    NUM_MOTORS,
    DriverError,
    SafetyViolation,
    TargetValidationError,
    validate_status_feedback,
    validate_target_vector,
)
from .six_motor_controller import SixMotorController
from .uart_bus import UartBus

__all__ = [
    "MOTOR_IDS",
    "NUM_MOTORS",
    "DriverError",
    "SafetyViolation",
    "TargetValidationError",
    "validate_target_vector",
    "validate_status_feedback",
    "SixMotorController",
    "Motor",
    "UartBus",
    "StatusFeedback",
]
