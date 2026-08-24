"""Safety invariants, bounds checking, and validation rules for 6PUS motor driver."""

from __future__ import annotations

from typing import Mapping, Sequence
import sys
from pathlib import Path

try:
    from pus6_kinematics.geometry_config import load_geometry_config
except ImportError:
    _kinematics_pkg = Path(__file__).resolve().parents[2] / "pus6_kinematics"
    if str(_kinematics_pkg) not in sys.path:
        sys.path.insert(0, str(_kinematics_pkg))
    from pus6_kinematics.geometry_config import load_geometry_config

_GEOMETRY_CONFIG = load_geometry_config()

try:
    from .protocol import StatusFeedback
except ImportError:
    from protocol import StatusFeedback


MOTOR_IDS: tuple[int, ...] = (1, 2, 3, 4, 5, 6)
NUM_MOTORS: int = 6

CONTROL_MODE_ADDR: int = 0x25
TARGET_SPEED_ADDR: int = 0x28
TARGET_POSITION_ADDR: int = 0x29
ACTUAL_POSITION_ADDR: int = 0x2A
LOWER_LIMIT_ADDR: int = 0x24
UPPER_LIMIT_ADDR: int = 0x23
PAUSE_ADDR: int = 0x1A

SPEED_MODE: int = 2
DEFAULT_SPEED: int = 50
MAX_CURRENT_MA: int = 500
# Formal control-layer deadband for static position checks.  This is an
# acceptance band around the selected reference; raw register/status values
# are never modified or normalized.
STATIC_POSITION_TOLERANCE_STEPS: int = 2

ABSOLUTE_RAW_LOWER_LIMIT: int = _GEOMETRY_CONFIG.absolute_raw_min_steps
ABSOLUTE_RAW_UPPER_LIMIT: int = _GEOMETRY_CONFIG.absolute_raw_max_steps
SOFTWARE_LOWER_LIMIT: int = _GEOMETRY_CONFIG.software_command_min_steps
SOFTWARE_UPPER_LIMIT: int = _GEOMETRY_CONFIG.software_command_max_steps
NORMAL_MOTION_LOWER_LIMIT: int = _GEOMETRY_CONFIG.normal_motion_min_steps
NORMAL_MOTION_UPPER_LIMIT: int = _GEOMETRY_CONFIG.normal_motion_max_steps

# Commanded targets and encoder feedback have different safety contracts. A
# target is never allowed outside the absolute raw command range. Feedback is
# permitted a small signed zero-reference deviation so values such as -2 from
# an encoder at software zero do not abort an otherwise valid motion command.
FEEDBACK_POSITION_TOLERANCE_STEPS: int = 5
FEEDBACK_LOWER_LIMIT: int = ABSOLUTE_RAW_LOWER_LIMIT - FEEDBACK_POSITION_TOLERANCE_STEPS
FEEDBACK_UPPER_LIMIT: int = ABSOLUTE_RAW_UPPER_LIMIT + FEEDBACK_POSITION_TOLERANCE_STEPS

# Validated standalone-runner UART default. Keep this aligned with
# pus6_motor_driver/config/motors.yaml and the ROS bringup configuration.
DEFAULT_SERIAL_PORT: str = "/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0"

# Hardware Dual-Pose Constants (Storage Park vs Operating Home)
STORAGE_PARK_RAW_STEPS: tuple[int, ...] = _GEOMETRY_CONFIG.storage_park_raw_steps
OPERATING_HOME_RAW_STEPS: tuple[int, ...] = _GEOMETRY_CONFIG.operating_home_raw_steps
STORAGE_PARK_RAW_STEP: int = STORAGE_PARK_RAW_STEPS[0]
OPERATING_HOME_RAW_STEP: int = OPERATING_HOME_RAW_STEPS[0]

# SI / Step Unit Conversion Constants (40 step/mm = 40,000 step/m)
STEPS_PER_METER: float = _GEOMETRY_CONFIG.steps_per_meter
STEPS_PER_MM: float = STEPS_PER_METER / 1000.0
METERS_PER_STEP: float = _GEOMETRY_CONFIG.meters_per_step
MM_PER_STEP: float = METERS_PER_STEP * 1000.0


def steps_to_meters(steps: int | float) -> float:
    """Convert device step counts to SI meters."""
    return float(steps) * METERS_PER_STEP


def uint16_to_int16(value: int) -> int:
    """Decode one uint16 transport value as a signed int16 step count.

    Position feedback registers are transported as uint16 on the wire but
    represent signed step counts. This conversion is explicit and applies
    no offset and no safety-limit relaxation.
    """
    if not isinstance(value, int) or isinstance(value, bool):
        raise TypeError(f"uint16 register value must be an integer, got {type(value).__name__}")
    if not 0 <= value <= 0xFFFF:
        raise ValueError(f"uint16 register value outside [0, 65535]: {value}")
    return value - 0x10000 if value >= 0x8000 else value


def meters_to_steps(meters: float) -> int:
    """Convert SI meters to nearest integer device step counts."""
    return int(round(meters * STEPS_PER_METER))


def steps_to_mm(steps: int | float) -> float:
    """Convert device step counts to millimeters."""
    return float(steps) * MM_PER_STEP


def mm_to_steps(mm: float) -> int:
    """Convert millimeters to nearest integer device step counts."""
    return int(round(mm * STEPS_PER_MM))


def raw_to_relative_steps(raw_steps: int | Sequence[int]) -> int | list[int]:
    """Convert raw device step counts to software relative steps (q_rel = q_raw - 1000)."""
    if isinstance(raw_steps, (int, float)):
        return int(round(raw_steps)) - OPERATING_HOME_RAW_STEP
    return [int(round(r)) - OPERATING_HOME_RAW_STEP for r in raw_steps]


def relative_to_raw_steps(rel_steps: int | Sequence[int]) -> int | list[int]:
    """Convert software relative steps to raw device step counts (q_raw = q_rel + 1000)."""
    if isinstance(rel_steps, (int, float)):
        return int(round(rel_steps)) + OPERATING_HOME_RAW_STEP
    return [int(round(r)) + OPERATING_HOME_RAW_STEP for r in rel_steps]



from enum import Enum


class HardwareMode(str, Enum):
    """Execution mode for SixMotorController and Pus6MotorDriverNode."""
    MOCK = "mock"            # No real serial port opened; uses fake/mock backend
    READ_ONLY = "read_only"  # Real serial port opened; telemetry reads allowed; ALL writes strictly blocked (0 writes)
    ACTIVE = "active"        # Real serial port opened; validated hardware writes permitted


def parse_hardware_mode(mode: str | HardwareMode) -> HardwareMode:
    """Parse and validate hardware mode string."""
    if isinstance(mode, HardwareMode):
        return mode
    val = str(mode).strip().lower()
    for member in HardwareMode:
        if member.value == val or member.name.lower() == val:
            return member
    raise ValueError(
        f"Invalid hardware mode '{mode}'. Must be one of: {[m.value for m in HardwareMode]}"
    )


class DriverError(Exception):
    """Base exception for pus6_motor_driver errors."""


class SafetyViolation(DriverError):
    """Raised when a hardware safety condition or boundary constraint is violated."""


class ReadOnlyModeError(SafetyViolation):
    """Raised when a write operation is attempted while in READ_ONLY hardware mode."""


class J2F0SafetyViolation(SafetyViolation):
    """Raised when J2-F0 safety invariants (current threshold, fault codes) are violated."""


class J2F0Stop(SafetyViolation):
    """Raised when J2-F0 real-time STOP conditions (>3 step drift, bus errors) are triggered."""


class TargetValidationError(SafetyViolation):
    """Raised when target position vector validation fails before write execution."""


def validate_target_vector(
    targets: Sequence[int],
    limits: Mapping[int, tuple[int, int]] | None = None,
) -> list[int]:
    """Atomically validate a 6-element target vector for ID1..ID6.
    This function validates commanded trajectory targets only; feedback positions
    must be kept separate and must not be passed here.

    Args:
        targets: 6-element integer sequence corresponding to ID1..ID6.
        limits: Optional mapping of motor_id -> (lower_limit, upper_limit).

    Returns:
        Validated target positions as a list of integers.

    Raises:
        TargetValidationError: If length != 6, non-integer, or target outside safe limits.
    """
    if len(targets) != NUM_MOTORS:
        raise TargetValidationError(
            f"Target position vector length must be exactly {NUM_MOTORS} (for ID1..ID6), got {len(targets)}"
        )

    validated: list[int] = []
    for idx, mid in enumerate(MOTOR_IDS):
        val = targets[idx]
        if not isinstance(val, int) or isinstance(val, bool):
            raise TargetValidationError(
                f"Target position for ID{mid} must be an integer, got {type(val).__name__} ({val})"
            )

        # This is a commanded trajectory target, so use the absolute raw
        # command range. The normal-motion envelope (currently [200, 1800])
        # is a separate planning policy and must not reject PARK/HOME targets.
        lower = ABSOLUTE_RAW_LOWER_LIMIT
        upper = ABSOLUTE_RAW_UPPER_LIMIT

        if limits is not None and mid in limits:
            hw_lower, hw_upper = limits[mid]
            lower = max(lower, hw_lower)
            upper = min(upper, hw_upper)

        if not (lower <= val <= upper):
            raise TargetValidationError(
                f"Motor ID{mid} target {val} outside safe boundary [{lower}, {upper}]"
            )
        validated.append(val)

    return validated


def validate_status_feedback(
    status: StatusFeedback,
    motor_id: int,
    limits: tuple[int, int] | None = None,
) -> None:
    """Validate status feedback health and the independent encoder range.

    The optional ``limits`` argument is retained for API compatibility with
    callers that also validate commanded targets; feedback position checking
    intentionally uses the dedicated zero-offset tolerance below.

    Args:
        status: Feedback status frame.
        motor_id: Logical motor ID (1..6).
        limits: Optional command-limit tuple retained for call-site compatibility.

    Raises:
        SafetyViolation: If the status reports a fault, overcurrent, or
            feedback position outside the monitoring range.
    """
    if status.error_code != 0:
        raise SafetyViolation(
            f"Motor ID{motor_id} reported hardware fault error code 0x{status.error_code:02X}"
        )

    if status.current_mA > MAX_CURRENT_MA:
        raise SafetyViolation(
            f"Motor ID{motor_id} current {status.current_mA}mA exceeds threshold {MAX_CURRENT_MA}mA"
        )

    if not (FEEDBACK_LOWER_LIMIT <= status.current_position <= FEEDBACK_UPPER_LIMIT):
        raise SafetyViolation(
            f"Motor ID{motor_id} actual position {status.current_position} "
            f"outside feedback range [{FEEDBACK_LOWER_LIMIT}, {FEEDBACK_UPPER_LIMIT}]"
        )
