import csv
import importlib.util
import math
import sys
import threading
from types import SimpleNamespace
from pathlib import Path

import pytest

SCRIPT = Path(__file__).resolve().parent.parent / "scripts" / "j10_a1_single_candidate_workspace_measurement.py"
spec = importlib.util.spec_from_file_location("j10_a1_single_candidate_workspace_measurement", SCRIPT)
mod = importlib.util.module_from_spec(spec)
assert spec.loader is not None
sys.modules[spec.name] = mod
spec.loader.exec_module(mod)


def test_preflight_pass_for_small_x_candidate():
    plan = mod.compute_workspace_candidate_preflight("x", "positive", 0.5, "mm")
    assert plan.qualification == "PASS"
    assert plan.limit_reason == ""
    assert plan.limiting_actuator.startswith("ID")
    assert plan.predicted_target_steps == [999, 1003, 1004, 997, 997, 1001]
    assert max(plan.actuator_margin) > 0
    assert plan.actuator_margin == plan.absolute_margin
    assert plan.historical_envelope_exceeded is False
    assert plan.maintenance_mode_required is False


def test_preflight_limits_on_task_gate():
    plan = mod.compute_workspace_candidate_preflight("x", "positive", 20.0, "mm")
    assert plan.qualification == "LIMIT"
    assert "task_space_gate" in plan.limit_reason
    assert plan.home_return_result == "not_executed"


def test_j10_gate_extension_is_explicit_for_x_and_y_only():
    assert mod.compute_workspace_candidate_preflight("x", "positive", 20.0, "mm").qualification == "LIMIT"
    extended_20 = mod.compute_workspace_candidate_preflight(
        "x", "positive", 20.0, "mm", j10_extended_task_space_gate=True
    )
    extended_40 = mod.compute_workspace_candidate_preflight(
        "x", "positive", 40.0, "mm", j10_extended_task_space_gate=True
    )
    extended_43_5 = mod.compute_workspace_candidate_preflight(
        "x", "positive", 43.5, "mm", j10_extended_task_space_gate=True
    )
    extended_45_75 = mod.compute_workspace_candidate_preflight(
        "x", "positive", 45.75, "mm", j10_extended_task_space_gate=True
    )
    extended_46 = mod.compute_workspace_candidate_preflight(
        "x", "positive", 46.0, "mm", j10_extended_task_space_gate=True
    )
    assert extended_20.qualification == "PASS"
    assert extended_40.qualification == "PASS"
    assert extended_40.task_space_gate_passed is True
    assert extended_43_5.qualification == "PASS"
    assert extended_43_5.task_space_gate_passed is True
    assert extended_45_75.qualification == "PASS"
    assert extended_45_75.task_space_gate_passed is True
    assert extended_46.qualification == "PASS"
    assert extended_46.limit_reason == ""
    assert extended_46.task_space_gate_passed is False
    assert mod.compute_workspace_candidate_preflight("x", "negative", 20.0, "mm").qualification == "LIMIT"
    extended_negative = mod.compute_workspace_candidate_preflight(
        "x", "negative", 20.0, "mm", j10_extended_task_space_gate=True
    )
    assert extended_negative.qualification == "PASS"
    assert extended_negative.task_space_gate_passed is True
    assert mod.compute_workspace_candidate_preflight("y", "positive", 20.0, "mm").qualification == "LIMIT"
    y_positive = mod.compute_workspace_candidate_preflight(
        "y", "positive", 49.0, "mm", j10_extended_task_space_gate=True
    )
    y_negative = mod.compute_workspace_candidate_preflight(
        "y", "negative", 45.5, "mm", j10_extended_task_space_gate=True
    )
    y_positive_over_gate = mod.compute_workspace_candidate_preflight(
        "y", "positive", 49.5, "mm", j10_extended_task_space_gate=True
    )
    assert y_positive.qualification == "PASS"
    assert y_positive.task_space_gate_passed is True
    assert y_negative.qualification == "PASS"
    assert y_negative.task_space_gate_passed is True
    assert y_positive_over_gate.qualification == "PASS"
    assert y_positive_over_gate.task_space_gate_passed is False
    assert mod.compute_workspace_candidate_preflight("roll", "positive", 20.0, "deg").qualification == "LIMIT"
    roll_positive = mod.compute_workspace_candidate_preflight(
        "roll", "positive", 62.5, "deg", j10_extended_task_space_gate=True
    )
    roll_positive_over_gate = mod.compute_workspace_candidate_preflight(
        "roll", "positive", 63.0, "deg", j10_extended_task_space_gate=True
    )
    roll_negative = mod.compute_workspace_candidate_preflight(
        "roll", "negative", 69.0, "deg", j10_extended_task_space_gate=True
    )
    assert roll_positive.qualification == "PASS"
    assert roll_positive.task_space_gate_passed is True
    assert roll_positive_over_gate.qualification == "PASS"
    assert roll_positive_over_gate.task_space_gate_passed is False
    assert roll_negative.qualification == "PASS"
    assert roll_negative.task_space_gate_passed is True
    assert mod.compute_workspace_candidate_preflight("pitch", "positive", 10.0, "deg").qualification == "LIMIT"
    pitch_positive = mod.compute_workspace_candidate_preflight(
        "pitch", "positive", 35.0, "deg", j10_extended_task_space_gate=True
    )
    pitch_negative = mod.compute_workspace_candidate_preflight(
        "pitch", "negative", 35.0, "deg", j10_extended_task_space_gate=True
    )
    pitch_at_gate = mod.compute_workspace_candidate_preflight(
        "pitch", "positive", 55.0, "deg", j10_extended_task_space_gate=True
    )
    pitch_over_gate = mod.compute_workspace_candidate_preflight(
        "pitch", "positive", 55.1, "deg", j10_extended_task_space_gate=True
    )
    assert pitch_positive.qualification == "PASS"
    assert pitch_negative.qualification == "PASS"
    assert pitch_at_gate.qualification == "PASS"
    assert pitch_at_gate.task_space_gate_passed is True
    assert pitch_over_gate.qualification == "LIMIT"
    assert pitch_over_gate.task_space_gate_passed is False
    assert mod.compute_workspace_candidate_preflight("yaw", "positive", 10.0, "deg").qualification == "LIMIT"
    yaw_positive = mod.compute_workspace_candidate_preflight(
        "yaw", "positive", 30.0, "deg", j10_extended_task_space_gate=True
    )
    yaw_at_gate = mod.compute_workspace_candidate_preflight(
        "yaw", "positive", 97.5, "deg", j10_extended_task_space_gate=True
    )
    yaw_over_gate = mod.compute_workspace_candidate_preflight(
        "yaw", "positive", 98.0, "deg", j10_extended_task_space_gate=True
    )
    assert yaw_positive.qualification == "PASS"
    assert yaw_at_gate.qualification == "PASS"
    assert yaw_at_gate.task_space_gate_passed is True
    assert yaw_over_gate.qualification == "PASS"
    assert yaw_over_gate.task_space_gate_passed is False
    assert mod.compute_workspace_candidate_preflight("z", "positive", 20.0, "mm").qualification == "PASS"
    assert mod.compute_workspace_candidate_preflight("z", "positive", 20.0, "mm").task_space_gate_passed is True
    assert mod.compute_workspace_candidate_preflight("z", "positive", 21.0, "mm").qualification == "LIMIT"
    z_positive = mod.compute_workspace_candidate_preflight(
        "z", "positive", 24.5, "mm", j10_extended_task_space_gate=True
    )
    z_negative = mod.compute_workspace_candidate_preflight(
        "z", "negative", 24.5, "mm", j10_extended_task_space_gate=True
    )
    z_positive_at_gate = mod.compute_workspace_candidate_preflight(
        "z", "positive", 25.0, "mm", j10_extended_task_space_gate=True
    )
    z_positive_over_gate = mod.compute_workspace_candidate_preflight(
        "z", "positive", 25.1, "mm", j10_extended_task_space_gate=True
    )
    assert z_positive.qualification == "PASS"
    assert z_positive.task_space_gate_passed is True
    assert z_negative.qualification == "PASS"
    assert z_negative.task_space_gate_passed is True
    assert z_positive_at_gate.qualification == "PASS"
    assert z_positive_at_gate.task_space_gate_passed is True
    assert z_positive_over_gate.qualification == "LIMIT"
    assert z_positive_over_gate.task_space_gate_passed is False


class _FakeKinInvalid:
    nominal_height = 0.13261226796206452

    def differential_inverse_kinematics_steps(self, pose, reference_pose):
        raise RuntimeError("IK invalid")

    def inverse_kinematics(self, pose):
        raise RuntimeError("IK invalid")


class _FakeKinOOR:
    nominal_height = 0.13261226796206452

    def differential_inverse_kinematics_steps(self, pose, reference_pose):
        return [0, 0, 0, 0, 0, 850]

    def inverse_kinematics(self, pose):
        return [0.05] * 6


class _FakeKinAbsoluteOOR:
    nominal_height = 0.13261226796206452

    def __init__(self, delta):
        self.delta = delta

    def differential_inverse_kinematics_steps(self, pose, reference_pose):
        return [0, 0, 0, 0, 0, self.delta]

    def inverse_kinematics(self, pose):
        return [0.05] * 6


def test_preflight_limits_on_invalid_ik():
    plan = mod.compute_workspace_candidate_preflight("pitch", "positive", 0.5, "deg", kin=_FakeKinInvalid())
    assert plan.qualification == "LIMIT"
    assert "ik_invalid" in plan.limit_reason


def test_preflight_pass_on_extended_exploratory_range():
    plan = mod.compute_workspace_candidate_preflight("z", "positive", 0.5, "mm", kin=_FakeKinOOR())
    assert plan.qualification == "PASS"
    assert plan.limit_reason == ""
    assert plan.predicted_target_steps[-1] == 1850
    assert plan.absolute_margin[-1] == 150
    assert plan.absolute_margin_min == 150
    assert plan.historical_envelope_exceeded is True
    assert plan.maintenance_mode_required is True


def test_historical_or_former_exploratory_range_does_not_limit_j10():
    plan = mod.compute_workspace_candidate_preflight("z", "positive", 0.5, "mm", kin=_FakeKinAbsoluteOOR(950))
    assert plan.qualification == "PASS"
    assert plan.limit_reason == ""
    assert plan.predicted_target_steps[-1] == 1950
    assert plan.absolute_margin[-1] == 50
    assert plan.absolute_margin_min == 50
    assert plan.historical_envelope_exceeded is True
    assert plan.maintenance_mode_required is True


@pytest.mark.parametrize(
    ("delta", "expected_steps", "qualification", "margin", "maintenance"),
    [
        (790, 1790, "PASS", 210, False),
        (850, 1850, "PASS", 150, True),
        (901, 1901, "PASS", 99, True),
        (999, 1999, "PASS", 1, True),
        (1000, 2000, "PASS", 0, True),
        (1001, 2001, "LIMIT", -1, True),
        (-1001, -1, "LIMIT", -1, True),
    ],
)
def test_j10_absolute_actuator_policy_regression(delta, expected_steps, qualification, margin, maintenance):
    plan = mod.compute_workspace_candidate_preflight(
        "z", "positive", 0.5, "mm", kin=_FakeKinAbsoluteOOR(delta)
    )
    assert plan.predicted_target_steps[-1] == expected_steps
    assert plan.qualification == qualification
    assert plan.limit_reason == ("predicted_step_outside_absolute_range" if qualification == "LIMIT" else "")
    assert plan.absolute_margin[-1] == margin
    assert plan.absolute_margin_min == margin
    assert plan.historical_envelope_exceeded is (expected_steps < 200 or expected_steps > 1800)
    assert plan.maintenance_mode_required is maintenance


def test_preflight_limits_on_absolute_range():
    for delta, expected in ((-1001, -1), (1001, 2001)):
        plan = mod.compute_workspace_candidate_preflight("z", "positive", 0.5, "mm", kin=_FakeKinAbsoluteOOR(delta))
        assert plan.qualification == "LIMIT"
        assert plan.limit_reason == "predicted_step_outside_absolute_range"
        assert plan.predicted_target_steps[-1] == expected
        assert plan.historical_envelope_exceeded is True
        assert plan.maintenance_mode_required is True


def test_csv_fields_include_new_j10_columns():
    for field in (
        "axis",
        "direction",
        "target_amplitude",
        "requested_target_steps_1",
        "predicted_target_steps_1",
        "actuator_margin_1",
        "absolute_margin_1",
        "absolute_margin_min",
        "actuator_margin_semantics",
        "limiting_actuator",
        "qualification",
        "limit_reason",
        "historical_envelope_exceeded",
        "absolute_min",
        "absolute_max",
        "historical_min",
        "historical_max",
        "exploratory_min",
        "exploratory_max",
        "exploratory_envelope_deprecated",
        "exploratory_limits_hard_gate",
        "maintenance_mode_required",
        "home_return_result",
        "motion_executed",
        "uart_cycle_count",
        "six_axis_0x29_frame_count",
    ):
        assert field in mod.CSV_FIELDS
    assert "hardware_write_count" not in mod.CSV_FIELDS


def test_dry_run_report_and_artifact(tmp_path: Path):
    report = mod.run_workspace_measurement_experiment(
        axis="x",
        direction="positive",
        amplitude=0.5,
        units="mm",
        output_dir=tmp_path,
        execute_motion=False,
        dry_run=True,
        emit_artifacts=True,
    )
    assert report["uart_cycle_count"] == 0
    assert report["six_axis_0x29_frame_count"] == 0
    assert report["motion_executed"] is False
    assert report["maintenance_mode"] is False
    assert report["plan"]["qualification"] == "PASS"
    assert report["plan"]["historical_envelope_exceeded"] is False
    assert report["plan"]["maintenance_mode_required"] is False
    assert report["default_task_space_limit_x"] == 15.0
    assert report["active_task_space_limit_x_positive"] == 15.0
    assert report["j10_extended_task_space_gate"] is False

    csv_path = Path(report["csv_path"])
    rows = list(csv.DictReader(csv_path.open("r", encoding="utf-8")))
    assert len(rows) == 1
    assert rows[0]["qualification"] == "PASS"
    assert rows[0]["uart_cycle_count"] == "0"
    assert rows[0]["six_axis_0x29_frame_count"] == "0"
    assert rows[0]["historical_envelope_exceeded"] == "False"
    assert rows[0]["maintenance_mode_required"] == "False"
    assert rows[0]["motion_executed"] == "False"
    assert csv_path.name.startswith("J10_")
    assert Path(report["metadata_path"]).name.startswith("J10_")


def test_analysis_reports_final_csv_consistency():
    rows = [
        {"qualification": "PASS", "axis": "x", "direction": "positive",
         "limit_reason": "", "historical_envelope_exceeded": "False",
         "maintenance_mode_required": "False", "home_return_result": "completed",
         "uart_cycle_count": "12", "six_axis_0x29_frame_count": "72"},
        {"qualification": "PASS", "axis": "x", "direction": "positive",
         "limit_reason": "", "historical_envelope_exceeded": "False",
         "maintenance_mode_required": "False", "home_return_result": "completed",
         "uart_cycle_count": "15", "six_axis_0x29_frame_count": "90"},
    ]
    analysis = mod._analysis_from_rows(rows)
    assert analysis["row_count"] == 2
    assert analysis["qualification"] == "PASS"
    assert analysis["home_return_result"] == "completed"
    assert analysis["uart_cycle_count"] == 15
    assert analysis["six_axis_0x29_frame_count"] == 90


def test_analysis_fk_summary_uses_final_measurement_window_only():
    common = {
        "qualification": "PASS", "axis": "x", "direction": "positive",
        "limit_reason": "", "historical_envelope_exceeded": "False",
        "maintenance_mode_required": "False", "home_return_result": "completed",
        "uart_cycle_count": "1", "six_axis_0x29_frame_count": "6",
    }
    transition = dict(common, stage="candidate_transition", fk_x_mm="2", fk_y_mm="9",
                      fk_z_mm="132", fk_roll_deg="1", fk_pitch_deg="1", fk_yaw_deg="1")
    measured = dict(common, stage="candidate_measurement", fk_x_mm="15", fk_y_mm="0",
                    fk_z_mm="132.6", fk_roll_deg="0", fk_pitch_deg="0", fk_yaw_deg="0")
    analysis = mod._analysis_from_rows([transition, measured])
    assert analysis["measurement_window_stage"] == "candidate_measurement"
    assert analysis["measurement_window_row_count"] == 1
    assert analysis["measurement_window_fk_median"] == [15.0, 0.0, 132.6, 0.0, 0.0, 0.0]


def test_post_run_teardown_is_separate_from_motion_stop_schema():
    assert "post_run_teardown_warning" not in mod.CSV_FIELDS
    assert "motion_stop" not in mod.CSV_FIELDS
    assert "fault_stop" not in mod.CSV_FIELDS


def test_fk_callback_accepts_current_pose_message_layout():
    harness = object.__new__(mod.WorkspaceMeasurementHarness)
    harness.lock = threading.Lock()
    harness.fk_xyz_m = [0.0, 0.0, 0.0]
    harness.fk_rpy_rad = [0.0, 0.0, 0.0]
    msg = SimpleNamespace(
        pose=SimpleNamespace(position=SimpleNamespace(x=0.01, y=-0.002, z=0.1326)),
        euler_angles_rad=[0.001, -0.002, 0.003],
    )
    mod.WorkspaceMeasurementHarness._on_fk_state(harness, msg)
    assert harness.fk_xyz_m == [0.01, -0.002, 0.1326]
    assert harness.fk_rpy_rad == [0.001, -0.002, 0.003]


def test_cycle_sequence_is_exported_as_count_not_zero_based_id():
    harness = object.__new__(mod.WorkspaceMeasurementHarness)
    harness.lock = threading.Lock()
    harness.latest_fb_steps = [1000] * 6
    harness.latest_fault_codes = [0] * 6
    harness.uart_cycle_count = 0
    harness.six_axis_0x29_frame_count = 0
    harness.feedback_sample_count = 0
    harness.kin = SimpleNamespace(forward_kinematics=lambda _: [0.0] * 6)
    msg = SimpleNamespace(raw_steps=[1000] * 6, fault_codes=[0] * 6, uart_cycle_sequence=0)
    mod.WorkspaceMeasurementHarness._on_timed_feedback(harness, msg)
    assert harness.uart_cycle_count == 1
    assert harness.six_axis_0x29_frame_count == 6


def test_home_band_reuses_j7_safety_confirmation_range():
    assert mod._home_feedback_in_band([995, 996, 997, 998, 999, 1005]) is True
    assert mod._home_feedback_in_band([994, 996, 997, 998, 999, 1000]) is False
    assert mod._home_feedback_in_band([995, 996, 997, 998, 999]) is False


def test_pre_candidate_home_timeout_uses_observed_feedback_distance():
    timeout = mod._home_timeout_from_feedback([1530, 1000, 530, 530, 1000, 1530], [1000] * 6)
    assert math.isclose(timeout, 530 / mod.OBSERVED_FEEDBACK_TRAVEL_RATE_STEPS_S + mod.TRAVEL_TIMEOUT_MARGIN_S)
    assert timeout > mod._estimate_step_travel_timeout_s([1000] * 6, [1000] * 6)


def test_home_confirmation_cli_defaults_are_bounded():
    args = mod.build_arg_parser().parse_args([
        "--axis", "x", "--direction", "positive", "--amplitude", "20",
    ])
    assert args.candidate_timeout_s is None
    assert args.home_timeout_s is None
    assert args.home_consecutive_samples == 5


def test_travel_timeouts_scale_with_actual_step_distance():
    timeout = mod._estimate_step_travel_timeout_s([1000] * 6, [1173, 1404, 1461, 1007, 1057, 1279])
    assert math.isclose(timeout, 461 / mod.OBSERVED_FEEDBACK_TRAVEL_RATE_STEPS_S + mod.TRAVEL_TIMEOUT_MARGIN_S)
    assert timeout > 6.0
    assert mod._estimate_step_travel_timeout_s([1000] * 6, [1000] * 6) == mod.TRAVEL_TIMEOUT_MARGIN_S


def test_build_arg_parser_has_expected_cli():
    parser = mod.build_arg_parser()
    args = parser.parse_args(["--axis", "x", "--direction", "positive", "--amplitude", "0.5", "--maintenance-mode"])
    assert args.axis == "x"
    assert args.direction == "positive"
    assert args.maintenance_mode is True
    assert math.isclose(args.amplitude, 0.5)


def test_yaw_105_prohibited_from_motion_execution(tmp_path: Path):
    """Verify that Yaw +105 deg (exceeding J10 Yaw safe gate of 100 deg) is strictly prohibited from executing motion."""
    report = mod.run_workspace_measurement_experiment(
        axis="yaw",
        direction="positive",
        amplitude=105.0,
        units="deg",
        output_dir=tmp_path,
        execute_motion=True,
        dry_run=False,
        emit_artifacts=True,
        j10_extended_task_space_gate=True,
    )
    assert report["motion_executed"] is False
    assert report["uart_cycle_count"] == 0
    assert report["six_axis_0x29_frame_count"] == 0
    assert report["status"] == "LIMIT"
    assert report["plan"]["task_space_gate_passed"] is False
    assert "task_space_gate" in report["candidate_reach_failure"]
