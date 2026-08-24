import sys
from pathlib import Path

PKG_ROOT = Path(__file__).resolve().parent.parent
if str(PKG_ROOT) not in sys.path:
    sys.path.insert(0, str(PKG_ROOT))

import pytest
from pus6_motor_driver.protocol import StatusFeedback
from pus6_motor_driver.safety import (
    MOTOR_IDS,
    NUM_MOTORS,
    SafetyViolation,
    TargetValidationError,
    validate_status_feedback,
    validate_target_vector,
    uint16_to_int16,
)


def test_validate_target_vector_valid():
    targets = [100, 200, 300, 400, 500, 600]
    res = validate_target_vector(targets)
    assert res == targets


def test_validate_target_vector_length_mismatch():
    with pytest.raises(TargetValidationError, match="length must be exactly 6"):
        validate_target_vector([100, 200, 300, 400, 500])

    with pytest.raises(TargetValidationError, match="length must be exactly 6"):
        validate_target_vector([100, 200, 300, 400, 500, 600, 700])


def test_validate_target_vector_type_error():
    with pytest.raises(TargetValidationError, match="must be an integer"):
        validate_target_vector([100, 200, "300", 400, 500, 600])

    with pytest.raises(TargetValidationError, match="must be an integer"):
        validate_target_vector([100, 200, True, 400, 500, 600])


def test_validate_target_vector_global_bounds():
    assert validate_target_vector([2000] * 6) == [2000] * 6
    assert validate_target_vector([2000] * 6, limits={mid: (0, 2000) for mid in MOTOR_IDS}) == [2000] * 6

    with pytest.raises(TargetValidationError, match="outside safe boundary"):
        validate_target_vector([-1, 100, 100, 100, 100, 100])

    with pytest.raises(TargetValidationError, match="outside safe boundary"):
        validate_target_vector([100, 100, 100, 100, 100, 2001])

    # A signed feedback value is never a valid motion target.
    with pytest.raises(TargetValidationError, match="outside safe boundary"):
        validate_target_vector([-2, 0, 0, 0, 0, 0])


def test_validate_target_vector_hardware_limits():
    limits = {
        1: (10, 1000),
        2: (20, 800),
        3: (0, 1800),
        4: (0, 1800),
        5: (0, 1800),
        6: (0, 1800),
    }
    # ID1 = 5 (below hw lower limit 10)
    with pytest.raises(TargetValidationError, match="ID1 target 5 outside safe boundary"):
        validate_target_vector([5, 100, 100, 100, 100, 100], limits=limits)

    # ID2 = 850 (above hw upper limit 800)
    with pytest.raises(TargetValidationError, match="ID2 target 850 outside safe boundary"):
        validate_target_vector([50, 850, 100, 100, 100, 100], limits=limits)


def test_validate_status_feedback_normal():
    status = StatusFeedback(
        target_position=100,
        current_position=100,
        current_mA=50,
        force_g=100,
        force_adc=200,
        temperature=30,
        error_code=0,
    )
    validate_status_feedback(status, motor_id=1, limits=(0, 1800))


def test_validate_status_feedback_fault():
    status = StatusFeedback(
        target_position=100,
        current_position=100,
        current_mA=50,
        force_g=100,
        force_adc=200,
        temperature=30,
        error_code=0x01,
    )
    with pytest.raises(SafetyViolation, match="hardware fault error code 0x01"):
        validate_status_feedback(status, motor_id=1)


def test_validate_status_feedback_overcurrent():
    status = StatusFeedback(
        target_position=100,
        current_position=100,
        current_mA=501,
        force_g=100,
        force_adc=200,
        temperature=30,
        error_code=0,
    )
    with pytest.raises(SafetyViolation, match="exceeds threshold 500mA"):
        validate_status_feedback(status, motor_id=1)


def test_validate_status_feedback_uses_feedback_range_not_command_limits():
    # A status position may exceed a caller's nominal command envelope.
    within_feedback_range = StatusFeedback(
        target_position=100,
        current_position=1500,
        current_mA=50,
        force_g=0,
        force_adc=0,
        temperature=30,
        error_code=0,
    )
    validate_status_feedback(within_feedback_range, motor_id=1, limits=(0, 1000))

    outside_feedback_range = StatusFeedback(
        target_position=100,
        current_position=2006,
        current_mA=50,
        force_g=0,
        force_adc=0,
        temperature=30,
        error_code=0,
    )
    with pytest.raises(SafetyViolation, match="outside feedback range"):
        validate_status_feedback(outside_feedback_range, motor_id=1, limits=(0, 1000))


def test_validate_status_feedback_accepts_zero_offset_tolerance_only():
    def status_at(position):
        return StatusFeedback(
            target_position=1000,
            current_position=position,
            current_mA=50,
            force_g=0,
            force_adc=0,
            temperature=30,
            error_code=0,
        )

    for position in (-5, 2005):
        validate_status_feedback(status_at(position), motor_id=3, limits=(0, 1800))
    for position in (-6, 2006):
        with pytest.raises(SafetyViolation, match="outside feedback range"):
            validate_status_feedback(status_at(position), motor_id=3, limits=(0, 1800))


def test_uint16_feedback_decode_is_explicit_and_offset_free():
    assert [uint16_to_int16(value) for value in (0xFFFE, 0xFFFF, 0x0000, 0x03E8)] == [-2, -1, 0, 1000]
    with pytest.raises(ValueError):
        uint16_to_int16(0x10000)


def test_unit_conversions():
    from pus6_motor_driver.safety import (
        meters_to_steps,
        mm_to_steps,
        steps_to_meters,
        steps_to_mm,
    )

    # 40 step/mm
    assert steps_to_mm(40) == 1.0
    assert steps_to_mm(0) == 0.0
    assert mm_to_steps(1.0) == 40
    assert mm_to_steps(0.025) == 1

    # 40000 step/m
    assert steps_to_meters(40000) == 1.0
    assert steps_to_meters(1) == 0.000025
    assert meters_to_steps(0.000025) == 1
    assert meters_to_steps(0.045) == 1800
