"""Safety invariants, bounds checking, and validation rules for 6PUS motor driver."""

from dataclasses import dataclass
from enum import Enum
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

# Startup State Acceptance Bands (Provisional +/- 5 step tolerance)
PARK_MEASUREMENT_BAND_LOWER: int = STORAGE_PARK_RAW_STEP - FEEDBACK_POSITION_TOLERANCE_STEPS  # -5
PARK_MEASUREMENT_BAND_UPPER: int = STORAGE_PARK_RAW_STEP + FEEDBACK_POSITION_TOLERANCE_STEPS  # +5
HOME_MEASUREMENT_BAND_LOWER: int = OPERATING_HOME_RAW_STEP - FEEDBACK_POSITION_TOLERANCE_STEPS  # 995
HOME_MEASUREMENT_BAND_UPPER: int = OPERATING_HOME_RAW_STEP + FEEDBACK_POSITION_TOLERANCE_STEPS  # 1005


class StartupClassification(str, Enum):
    """Authoritative physical startup classification for 6PUS platform."""
    VALID_PARK = "valid_park"              # All 6 axes within provisional PARK measurement band [-5, 5] steps
    VALID_HOME = "valid_home"              # All 6 axes within HOME measurement band [995, 1005] steps
    VALID_IN_ENVELOPE = "valid_in_envelope" # All 6 axes within valid commanded envelope [0, 2000] steps
    INVALID_OUT_OF_RANGE = "invalid_out_of_range" # One or more axes outside feedback limits [-5, 2005] steps
    MIXED_UNCLASSIFIED = "mixed_unclassified" # Inconsistent/mixed positions that cannot be safely classified


@dataclass(frozen=True)
class StartupStateAssessment:
    classification: StartupClassification
    measured_positions: tuple[int, ...]
    planning_seed: tuple[int, ...]
    is_park: bool
    is_home: bool
    is_valid: bool
    description: str


def classify_startup_state(
    measured_positions: Sequence[int],
    *,
    history_samples: Sequence[Sequence[int]] | None = None,
    stability_tolerance_steps: int = STATIC_POSITION_TOLERANCE_STEPS,
) -> StartupStateAssessment:
    """Classify 6-axis measured startup state and derive authoritative planning seed.

    Contract:
    1. If any measurement is outside [FEEDBACK_LOWER_LIMIT, FEEDBACK_UPPER_LIMIT] ([-5, 2005]):
       -> INVALID_OUT_OF_RANGE (is_valid=False, cannot plan)
    2. If history_samples is provided and position drift on any axis > stability_tolerance_steps:
       -> MIXED_UNCLASSIFIED (is_valid=False, readings unstable)
    3. If ALL 6 axes are within the provisional PARK measurement band [-5, +5]:
       -> VALID_PARK (is_valid=True, planning_seed is set to exactly [0, 0, 0, 0, 0, 0])
       Raw telemetry is preserved; motion planning starts from [0]*6 so all commanded targets remain >= 0.
    4. If ALL 6 axes are within the HOME measurement band [995, 1005]:
       -> VALID_HOME (is_valid=True, planning_seed is set to [1000, 1000, 1000, 1000, 1000, 1000])
    5. If ALL 6 axes are strictly within the command envelope [0, 2000]:
       -> VALID_IN_ENVELOPE (is_valid=True, planning_seed is set to measured_positions)
    6. Otherwise (e.g. mixed state with negative measurements on some axes without all being at PARK):
       -> MIXED_UNCLASSIFIED (is_valid=False, cannot plan; generic clamping is strictly forbidden)
    """
    if len(measured_positions) != NUM_MOTORS:
        raise ValueError(f"Expected {NUM_MOTORS} measured positions, got {len(measured_positions)}")

    raw_tuple = tuple(int(p) for p in measured_positions)

    # 1. Feedback bounds check
    for mid, pos in zip(MOTOR_IDS, raw_tuple):
        if not (FEEDBACK_LOWER_LIMIT <= pos <= FEEDBACK_UPPER_LIMIT):
            return StartupStateAssessment(
                classification=StartupClassification.INVALID_OUT_OF_RANGE,
                measured_positions=raw_tuple,
                planning_seed=(),
                is_park=False,
                is_home=False,
                is_valid=False,
                description=f"Motor ID{mid} measured position {pos} outside valid feedback limits [{FEEDBACK_LOWER_LIMIT}, {FEEDBACK_UPPER_LIMIT}]",
            )

    # 2. Stability check across history samples if provided
    if history_samples and len(history_samples) > 1:
        for idx, mid in enumerate(MOTOR_IDS):
            axis_vals = [s[idx] for s in history_samples]
            drift = max(axis_vals) - min(axis_vals)
            if drift > stability_tolerance_steps:
                return StartupStateAssessment(
                    classification=StartupClassification.MIXED_UNCLASSIFIED,
                    measured_positions=raw_tuple,
                    planning_seed=(),
                    is_park=False,
                    is_home=False,
                    is_valid=False,
                    description=f"Motor ID{mid} position unstable across preflight samples (drift {drift} > {stability_tolerance_steps} steps)",
                )

    # 3. Classify VALID_PARK: all 6 axes within [-5, +5] steps
    if all(PARK_MEASUREMENT_BAND_LOWER <= p <= PARK_MEASUREMENT_BAND_UPPER for p in raw_tuple):
        return StartupStateAssessment(
            classification=StartupClassification.VALID_PARK,
            measured_positions=raw_tuple,
            planning_seed=STORAGE_PARK_RAW_STEPS,
            is_park=True,
            is_home=False,
            is_valid=True,
            description=f"All 6 axes within provisional PARK measurement band [{PARK_MEASUREMENT_BAND_LOWER}, {PARK_MEASUREMENT_BAND_UPPER}] steps",
        )

    # 4. Classify VALID_HOME: all 6 axes within [995, 1005] steps
    if all(HOME_MEASUREMENT_BAND_LOWER <= p <= HOME_MEASUREMENT_BAND_UPPER for p in raw_tuple):
        return StartupStateAssessment(
            classification=StartupClassification.VALID_HOME,
            measured_positions=raw_tuple,
            planning_seed=OPERATING_HOME_RAW_STEPS,
            is_park=False,
            is_home=True,
            is_valid=True,
            description=f"All 6 axes within HOME measurement band [{HOME_MEASUREMENT_BAND_LOWER}, {HOME_MEASUREMENT_BAND_UPPER}] steps",
        )

    # 5. Classify VALID_IN_ENVELOPE: all 6 axes strictly within [0, 2000] steps
    if all(ABSOLUTE_RAW_LOWER_LIMIT <= p <= ABSOLUTE_RAW_UPPER_LIMIT for p in raw_tuple):
        return StartupStateAssessment(
            classification=StartupClassification.VALID_IN_ENVELOPE,
            measured_positions=raw_tuple,
            planning_seed=raw_tuple,
            is_park=False,
            is_home=False,
            is_valid=True,
            description=f"All 6 axes within commanded envelope [{ABSOLUTE_RAW_LOWER_LIMIT}, {ABSOLUTE_RAW_UPPER_LIMIT}] steps",
        )

    # 6. Mixed / unclassified state (e.g. some axes in [-5, -1] while others are > 5)
    return StartupStateAssessment(
        classification=StartupClassification.MIXED_UNCLASSIFIED,
        measured_positions=raw_tuple,
        planning_seed=(),
        is_park=False,
        is_home=False,
        is_valid=False,
        description=f"Mixed/unclassified startup state {list(raw_tuple)}: non-uniform negative offsets cannot be safely seeded",
    )

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
