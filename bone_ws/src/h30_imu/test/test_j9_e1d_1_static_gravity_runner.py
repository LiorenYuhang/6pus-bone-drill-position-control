from __future__ import annotations

import importlib.util
import sys
from types import SimpleNamespace
from pathlib import Path

import numpy as np


PKG = Path(__file__).resolve().parents[1]
SCRIPT = PKG / "scripts" / "j9_e1d_1_static_gravity_calibration.py"
spec = importlib.util.spec_from_file_location("j9_e1d_1_static_gravity_calibration", SCRIPT)
runner = importlib.util.module_from_spec(spec)
assert spec.loader is not None
sys.modules[spec.name] = runner
spec.loader.exec_module(runner)


def test_static_grid_and_traversals_are_exact_and_distinct() -> None:
    assert len(runner.STATIC_POSES) == 9
    assert len(runner.FIT_ORDER) == 9
    assert len(runner.VALIDATION_ORDER) == 9
    assert set(runner.FIT_ORDER) == set(runner.VALIDATION_ORDER) == set(runner.STATIC_POSES)
    assert len(set(runner.FIT_ORDER)) == 9
    assert len(set(runner.VALIDATION_ORDER)) == 9
    assert runner.FIT_ORDER != runner.VALIDATION_ORDER


def test_static_pose_directions_and_observability() -> None:
    report = runner.trajectory_preflight()
    assert report["status"] == "PASS"
    assert report["all_poses_within_orientation_envelope"] is True
    assert report["joint_step_min"] >= -800
    assert report["joint_step_max"] <= 800
    assert report["rotation_observability"]["rank"] == 3
    assert report["rotation_bias_nuisance_observability"]["rank"] == 6
    assert report["gravity_rotation_information_eigenvalues"][0] > 0.0
    assert report["gravity_rotation_information_eigenvalues"][-1] > 0.0


def test_acceleration_vector_stability_metric_rejects_direction_change() -> None:
    stable = np.tile(np.array([[-0.32, 0.09, 9.80]], dtype=float), (200, 1))
    stable_report = runner.acceleration_stability_metrics(stable)
    assert max(stable_report["axis_std_m_s2"]) < 1e-12
    assert stable_report["gravity_direction_p95_rad"] < 1e-12

    unstable = stable.copy()
    unstable[-20:, 0] += 0.20
    unstable_report = runner.acceleration_stability_metrics(unstable)
    assert unstable_report["gravity_direction_p95_rad"] > runner.STATIC_MAX_GRAVITY_DIRECTION_P95_RAD


def test_gravity_direction_and_wahba_recover_known_rotation() -> None:
    poses = [runner.STATIC_POSES[name] for name in runner.FIT_ORDER]
    u_p = np.vstack([runner.platform_gravity_direction(pose) for pose in poses])
    r_true = runner.Rotation.from_euler("xyz", [0.12, -0.08, 0.20])
    u_i = r_true.inv().apply(u_p)
    r_est = runner.solve_gravity_wahba(u_p, u_i)
    error = runner.Rotation.from_matrix(r_est @ r_true.as_matrix().T).magnitude()
    assert error < 1e-10


def test_static_gate_violation_reports_first_failed_metric() -> None:
    report = {
        "pass": False,
        "fk_converged": True,
        "joint_span_steps": [0, 1, 3, 0, 0, 0],
        "gyro_norm_p95_rad_s": 0.001,
        "accel_norm_mean_m_s2": runner.GRAVITY_M_S2,
        "accel_axis_std_m_s2": [0.001, 0.001, 0.001],
        "gravity_direction_p95_rad": 0.001,
    }
    violation = runner.first_static_gate_violation(report, 123)
    assert violation is not None
    assert violation["metric"] == "joint_span_steps"
    assert violation["measured"] == 3
    assert violation["threshold"] == runner.STATIC_MAX_JOINT_SPAN_STEPS


def test_run_static_pose_invalidates_entire_dwell_on_first_violation(monkeypatch) -> None:
    monkeypatch.setattr(runner, "publish_target", lambda *args, **kwargs: None)
    monkeypatch.setattr(runner, "STATIC_SETTLE_MIN_S", 0.0)
    monkeypatch.setattr(runner, "STATIC_SETTLE_TIMEOUT_S", 0.1)
    monkeypatch.setattr(runner, "STATIC_DWELL_S", 0.02)

    good = {
        "pass": True,
        "fk_converged": True,
        "joint_span_steps": [0, 0, 0, 0, 0, 0],
        "gyro_norm_p95_rad_s": 0.001,
        "accel_norm_mean_m_s2": runner.GRAVITY_M_S2,
        "accel_axis_std_m_s2": [0.001, 0.001, 0.001],
        "gravity_direction_p95_rad": 0.001,
    }
    bad = dict(good)
    bad.update({"pass": False, "gravity_direction_p95_rad": runner.STATIC_MAX_GRAVITY_DIRECTION_P95_RAD + 1e-4})

    class Observer:
        def __init__(self) -> None:
            self.calls = 0

        def static_gate(self, now_ns: int) -> dict[str, object]:
            self.calls += 1
            return good if self.calls <= 2 else bad

    record = runner.run_static_pose(None, None, Observer(), runner.STATIC_POSES["home"], 0.0)
    assert record["dwell_gate"]["valid"] is False
    assert record["dwell_gate"]["accepted_for_calibration"] is False
    assert record["dwell_gate"]["first_violation"]["metric"] == "gravity_direction_p95_rad"


class FakeWriter:
    def __init__(self) -> None:
        self.rows = []

    def enqueue(self, stream: str, row: dict[str, object]) -> bool:
        self.rows.append((stream, row))
        return True


def fake_stamp() -> SimpleNamespace:
    return SimpleNamespace(sec=1, nanosec=2)


def fake_diag(level: object, values: dict[str, str] | None = None) -> SimpleNamespace:
    key_values = [SimpleNamespace(key=key, value=value) for key, value in (values or {}).items()]
    status = SimpleNamespace(
        name="/pus6/imu_platform",
        level=level,
        message="streaming",
        hardware_id="platform_imu",
        values=key_values,
    )
    return SimpleNamespace(header=SimpleNamespace(stamp=fake_stamp()), status=[status])


def observer_without_ros() -> object:
    observer = runner.E1DObserverNode.__new__(runner.E1DObserverNode)
    observer.writer = FakeWriter()
    observer.lock = runner.threading.Lock()
    observer.latest_feedback_steps = None
    observer.latest_fk_converged = False
    observer.latest_fk_stamp_ns = 0
    observer.latest_driver_diagnostics = None
    observer.latest_h30_diagnostics = None
    observer.imu_samples = []
    observer.feedback_samples = []
    return observer


def fake_imu() -> SimpleNamespace:
    vector = SimpleNamespace(x=0.0, y=0.0, z=0.0)
    accel = SimpleNamespace(x=0.0, y=0.0, z=runner.GRAVITY_M_S2)
    return SimpleNamespace(
        header=SimpleNamespace(stamp=fake_stamp()),
        publisher_sequence=1,
        tid=2,
        has_sample_timestamp=True,
        sample_timestamp_us=3,
        host_monotonic_arrival_time_ns=runner.time.monotonic_ns(),
        imu=SimpleNamespace(angular_velocity=vector, linear_acceleration=accel),
    )


def fake_feedback(stamp_ns: int) -> SimpleNamespace:
    return SimpleNamespace(
        header=SimpleNamespace(stamp=fake_stamp()),
        snapshot_start_monotonic_ns=stamp_ns - 1_000_000,
        snapshot_end_monotonic_ns=stamp_ns,
        uart_cycle_sequence=1,
        raw_steps=[1000, 1000, 1000, 1000, 1000, 1000],
        currents_ma=[0, 0, 0, 0, 0, 0],
        fault_codes=[0, 0, 0, 0, 0, 0],
    )


def fake_fk() -> SimpleNamespace:
    return SimpleNamespace(
        header=SimpleNamespace(stamp=fake_stamp()),
        query_monotonic_timestamp_ns=runner.time.monotonic_ns(),
        converged=True,
        residual_norm_meters=0.0,
        compute_time_us=10.0,
        pose=SimpleNamespace(orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0)),
    )


def test_decode_ros_uint8_accepts_int_and_single_byte_buffers() -> None:
    assert runner.decode_ros_uint8(0, "level") == 0
    assert runner.decode_ros_uint8(255, "level") == 255
    assert runner.decode_ros_uint8(b"\x00", "level") == 0
    assert runner.decode_ros_uint8(bytearray(b"\x01"), "level") == 1
    assert runner.decode_ros_uint8(memoryview(b"\x02"), "level") == 2


def test_decode_ros_uint8_rejects_malformed_values() -> None:
    for value in (-1, 256, b"", b"\x00\x01", bytearray(b"\x00\x01"), "0"):
        try:
            runner.decode_ros_uint8(value, "level")
        except (TypeError, ValueError):
            continue
        raise AssertionError(f"accepted malformed uint8 value {value!r}")


def test_h30_diagnostics_callback_accepts_runtime_byte_levels() -> None:
    observer = observer_without_ros()
    values = {
        "source_capture_queue_capacity": "131072",
        "source_capture_enqueued_rows": "10",
        "source_capture_written_rows": "9",
        "source_capture_overflow_drops": "0",
        "source_capture_writer_errors": "0",
    }
    for level in (0, b"\x00", b"\x01"):
        runner.E1DObserverNode._on_h30_diagnostics(observer, fake_diag(level, values))
    summary = runner.E1DObserverNode.artifact_summary(observer)
    assert summary["source_capture"] == {
        "available": True,
        "capacity": 131072,
        "enqueued": 10,
        "written": 9,
        "overflow": 0,
        "errors": 0,
    }
    assert len(observer.writer.rows) == 3


def test_h30_diagnostics_callback_records_malformed_byte_level_without_crashing() -> None:
    observer = observer_without_ros()
    runner.E1DObserverNode._on_h30_diagnostics(observer, fake_diag(b"\x00\x01"))
    summary = runner.E1DObserverNode.artifact_summary(observer)
    status = summary["h30_diagnostics"]["statuses"][0]
    assert status["level"] is None
    assert "expected one uint8 byte" in status["level_decode_error"]
    assert observer.writer.rows[-1][0] == "h30_diagnostics"


def test_observer_liveness_after_diagnostics_callbacks_populates_static_gate() -> None:
    observer = observer_without_ros()
    for _ in range(3):
        runner.E1DObserverNode._on_h30_diagnostics(observer, fake_diag(b"\x00"))
    runner.E1DObserverNode._on_fk(observer, fake_fk())
    for _ in range(25):
        runner.E1DObserverNode._on_imu(observer, fake_imu())
    now_ns = runner.time.monotonic_ns()
    for _ in range(6):
        runner.E1DObserverNode._on_feedback(observer, fake_feedback(now_ns))
    gate = runner.E1DObserverNode.static_gate(observer, runner.time.monotonic_ns())
    assert gate["pass"] is True
    assert gate["imu_count"] >= 20
    assert gate["feedback_count"] >= 5
    streams = [stream for stream, _ in observer.writer.rows]
    assert "h30_diagnostics" in streams
    assert "imu" in streams
    assert "joint_feedback" in streams
    assert "fk" in streams


def test_stage_aware_raw_artifact_name_infers_e1d_run_number() -> None:
    out_dir = Path("/tmp/j9_e1d_2_static_gravity_calibration_20260829_run1")
    stage = runner.infer_stage_label(out_dir)
    assert stage == "J9-E1D-2"
    assert runner.raw_artifact_path(out_dir, stage).name == "j9_e1d_2_raw.json"
    explicit = runner.infer_stage_label(Path("/tmp/custom"), "J9-E1D-3")
    assert runner.raw_artifact_path(Path("/tmp/custom"), explicit).name == "j9_e1d_3_raw.json"



class FakeSerial:
    def __init__(self) -> None:
        self.close_count = 0

    def close(self) -> None:
        self.close_count += 1


class FakeAdapter:
    def __init__(self) -> None:
        self.serial = FakeSerial()


class FakeLifecycleNode:
    def __init__(self) -> None:
        self.destroyed = False

    def destroy_node(self) -> None:
        self.destroyed = True


class FakeTimer:
    def __init__(self) -> None:
        self.canceled = False

    def cancel(self) -> None:
        self.canceled = True


class FakeMotorLifecycleNode(FakeLifecycleNode):
    def __init__(self) -> None:
        super().__init__()
        self.j7_adapter = FakeAdapter()
        self.write_timer = FakeTimer()
        self._uart_cycle_seq = 0
        self._deadline_miss_count = 0
        self._j7_fault_reason = None
        self.bus = None
        self.j7_writer = None
        self.j7_feedback = None


class FakeExecutor:
    def __init__(self) -> None:
        self.shutdown_called = False

    def shutdown(self) -> None:
        self.shutdown_called = True


class FakeThread:
    def __init__(self, alive: bool = True) -> None:
        self.alive = alive
        self.join_count = 0

    def is_alive(self) -> bool:
        return self.alive

    def join(self, timeout: float | None = None) -> None:
        self.join_count += 1
        self.alive = False


def fake_active_pipeline() -> runner.ActiveMotionPipeline:
    return runner.ActiveMotionPipeline(
        executor=FakeExecutor(),
        spin_thread=FakeThread(alive=True),
        target_node=FakeLifecycleNode(),
        publisher=object(),
        kin_node=FakeLifecycleNode(),
        control_node=FakeLifecycleNode(),
        motor_node=FakeMotorLifecycleNode(),
        fk_node=FakeLifecycleNode(),
        z0=0.1326,
    )


def test_uart_ownership_rejects_standalone_home_while_active_ros_is_owned(monkeypatch) -> None:
    ownership = runner.UartOwnershipAudit()
    ownership.enter_active_ros("fit_active_ros")
    called = False

    def forbidden(*args, **kwargs):
        nonlocal called
        called = True
        raise AssertionError("standalone HOME must not be called while ACTIVE_ROS owns UART")

    monkeypatch.setattr(runner, "run_servo_home_park", forbidden, raising=False)
    try:
        runner.run_exclusive_standalone_action("home", ownership, "home_between_traversals")
    except RuntimeError as exc:
        assert "UART ownership must be RELEASED" in str(exc)
    else:
        raise AssertionError("overlapping ownership was accepted")
    assert called is False
    assert ownership.state == runner.UartOwnershipState.ACTIVE_ROS


def test_release_active_motion_pipeline_stops_publication_executor_and_closes_uart() -> None:
    ownership = runner.UartOwnershipAudit()
    ownership.enter_active_ros("fit_active_ros")
    pipeline = fake_active_pipeline()
    release = runner.release_active_motion_pipeline(pipeline, ownership, "fit_active_ros_complete")

    assert release["released"] is True
    assert release["target_publication_stopped"] is True
    assert release["worker_timer_canceled"] is True
    assert release["executor_shutdown"] is True
    assert release["spin_thread_alive_after_join"] is False
    assert release["active_uart_closed"] is True
    assert pipeline.released is True
    assert pipeline.target_node.destroyed is True
    assert pipeline.motor_node.destroyed is True
    assert pipeline.motor_node.write_timer.canceled is True
    assert pipeline.motor_node.j7_adapter.serial.close_count == 1
    assert ownership.state == runner.UartOwnershipState.RELEASED


def test_motor_worker_shutdown_ordering_guarantees_join_before_serial_close() -> None:
    order_events: list[str] = []
    ownership = runner.UartOwnershipAudit()
    ownership.enter_active_ros("fit_active_ros")

    class OrderedTargetNode:
        def destroy_node(self) -> None:
            order_events.append("target_stopped")

    class OrderedTimer:
        def cancel(self) -> None:
            order_events.append("worker_timer_canceled")

    class OrderedExecutor:
        def shutdown(self) -> None:
            order_events.append("executor_shutdown")

    class OrderedThread:
        def __init__(self) -> None:
            self.alive = True

        def is_alive(self) -> bool:
            return self.alive

        def join(self, timeout: float | None = None) -> None:
            order_events.append("spin_thread_joined")
            self.alive = False

    class OrderedSerial:
        def close(self) -> None:
            order_events.append("serial_closed")

    class OrderedMotorNode:
        def __init__(self) -> None:
            self.write_timer = OrderedTimer()
            self.j7_adapter = SimpleNamespace(serial=OrderedSerial())
            self._uart_cycle_seq = 0
            self._deadline_miss_count = 0
            self._j7_fault_reason = None
            self.bus = None
            self.j7_writer = None
            self.j7_feedback = None

        def destroy_node(self) -> None:
            order_events.append("motor_node_destroyed")

    pipeline = runner.ActiveMotionPipeline(
        executor=OrderedExecutor(),
        spin_thread=OrderedThread(),
        target_node=OrderedTargetNode(),
        publisher=object(),
        kin_node=FakeLifecycleNode(),
        control_node=FakeLifecycleNode(),
        motor_node=OrderedMotorNode(),
        fk_node=FakeLifecycleNode(),
        z0=0.1326,
    )

    release = runner.release_active_motion_pipeline(pipeline, ownership, "fit_active_ros_complete")
    assert release["released"] is True
    assert ownership.state == runner.UartOwnershipState.RELEASED
    assert order_events == [
        "target_stopped",
        "worker_timer_canceled",
        "executor_shutdown",
        "spin_thread_joined",
        "motor_node_destroyed",
        "serial_closed",
    ]


def test_static_gate_rejects_stale_pre_handoff_fk() -> None:
    observer = observer_without_ros()
    now_ns = runner.time.monotonic_ns()
    # Populate IMU and Feedback with fresh samples
    for _ in range(25):
        observer.imu_samples.append((now_ns - 100_000_000, np.zeros(3), np.array([0.0, 0.0, runner.GRAVITY_M_S2])))
    for _ in range(6):
        observer.feedback_samples.append((now_ns - 100_000_000, [1000] * 6))

    # Case A: FK was received 5 seconds ago (before handoff / during standalone home)
    observer.latest_fk_converged = True
    observer.latest_fk_stamp_ns = now_ns - int(5.0 * 1e9)
    gate_stale = runner.E1DObserverNode.static_gate(observer, now_ns)
    assert gate_stale["pass"] is False
    assert gate_stale["fk_fresh"] is False
    assert gate_stale["reason"] == "insufficient_recent_feedback_imu_or_fk"

    # Case B: Fresh FK arrives from current active pipeline
    observer.latest_fk_stamp_ns = now_ns - 50_000_000
    gate_fresh = runner.E1DObserverNode.static_gate(observer, now_ns)
    assert gate_fresh["pass"] is True
    assert gate_fresh["fk_fresh"] is True
    assert gate_fresh["fk_converged"] is True


def test_validation_restart_stale_target_guard_rejects_any_prior_target_state() -> None:
    clean = SimpleNamespace(
        _latest_target=None,
        _last_commanded_steps=None,
        _last_target_rx_time_ns=0,
        _target_in_count=0,
    )
    audit = runner.assert_no_stale_startup_targets(clean)
    assert audit["latest_target_is_none"] is True
    assert audit["last_commanded_steps_is_none"] is True

    stale_cases = [
        SimpleNamespace(_latest_target=object(), _last_commanded_steps=None, _last_target_rx_time_ns=0, _target_in_count=0),
        SimpleNamespace(_latest_target=None, _last_commanded_steps=[1000] * 6, _last_target_rx_time_ns=0, _target_in_count=0),
        SimpleNamespace(_latest_target=None, _last_commanded_steps=None, _last_target_rx_time_ns=123, _target_in_count=0),
        SimpleNamespace(_latest_target=None, _last_commanded_steps=None, _last_target_rx_time_ns=0, _target_in_count=1),
    ]
    for stale in stale_cases:
        try:
            runner.assert_no_stale_startup_targets(stale)
        except RuntimeError as exc:
            assert "stale target state" in str(exc)
        else:
            raise AssertionError("stale startup state was accepted")


def test_standalone_home_and_park_use_released_exclusive_ownership(monkeypatch) -> None:
    calls = []

    def fake_home_park(action: str, **kwargs):
        calls.append((action, kwargs))
        return SimpleNamespace(passed=True, action=action)

    monkeypatch.setattr(runner, "run_servo_home_park", fake_home_park, raising=False)
    ownership = runner.UartOwnershipAudit()
    home = runner.run_exclusive_standalone_action("home", ownership, "home_between_traversals")
    park = runner.run_exclusive_standalone_action("park", ownership, "final_park")

    assert home.passed is True
    assert park.passed is True
    assert calls[0][0] == "home"
    assert calls[1][0] == "park"
    assert calls[0][1]["real_hardware"] is True
    assert calls[0][1]["confirm_motion"] is True
    assert ownership.state == runner.UartOwnershipState.RELEASED
    event_states = [event.to_state for event in ownership.events]
    assert event_states == [
        "RELEASED",
        "STANDALONE_HOME",
        "RELEASED",
        "STANDALONE_PARK",
        "RELEASED",
    ]



def _write_static_calibration_artifacts(tmp_path: Path, rotation: object) -> dict[str, object]:
    segments = []
    imu_lines = ["callback_monotonic_ns,ros_header_stamp_ns,publisher_sequence,tid,has_sample_timestamp,sample_timestamp_us,host_monotonic_arrival_time_ns,gx_rad_s,gy_rad_s,gz_rad_s,ax_m_s2,ay_m_s2,az_m_s2\n"]
    fk_lines = ["callback_monotonic_ns,ros_header_stamp_ns,query_monotonic_timestamp_ns,converged,residual_norm_meters,compute_time_us,quat_xyzw\n"]
    t = 1_000_000_000
    sequence = 0
    for traversal, order in (("fit", runner.FIT_ORDER), ("validation", runner.VALIDATION_ORDER)):
        for pose_name in order:
            start_ns = t
            end_ns = t + 100_000_000
            pose = runner.STATIC_POSES[pose_name]
            u_p = runner.platform_gravity_direction(pose)
            u_i = rotation.inv().apply(u_p)
            accel = -runner.GRAVITY_M_S2 * u_i
            for offset in (10_000_000, 40_000_000, 70_000_000):
                sequence += 1
                sample_ns = start_ns + offset
                imu_lines.append(
                    f"{sample_ns},{sample_ns},{sequence},{sequence},1,{sequence},{sample_ns},0,0,0,"
                    f"{accel[0]},{accel[1]},{accel[2]}\n"
                )
            q = runner.Rotation.from_euler("xyz", pose.rpy_rad).as_quat().tolist()
            fk_lines.append(f"{start_ns + 50_000_000},{start_ns + 50_000_000},{start_ns + 50_000_000},1,0,1,\"{runner.json.dumps(q)}\"\n")
            segments.append(
                {
                    "traversal": traversal,
                    "pose": pose_name,
                    "dwell_start_monotonic_ns": start_ns,
                    "dwell_end_monotonic_ns": end_ns,
                    "dwell_gate": {"valid": True, "accepted_for_calibration": True, "first_violation": None},
                }
            )
            t += 200_000_000
    (tmp_path / "imu.csv").write_text("".join(imu_lines), encoding="utf-8")
    (tmp_path / "fk.csv").write_text("".join(fk_lines), encoding="utf-8")
    return {"segments": segments, "invalid_dwell_poses": []}


def test_static_gravity_reporting_serializes_fit_validation_and_observability(tmp_path) -> None:
    true_rotation = runner.Rotation.from_euler("xyz", [0.01, 0.02, 1.58])
    raw = _write_static_calibration_artifacts(tmp_path, true_rotation)

    report = runner.build_static_gravity_calibration_result(tmp_path, raw)

    assert report["data_acquisition_status"]["status"] == "PASS"
    assert report["data_acquisition_status"]["fit_valid_count"] == 9
    assert report["data_acquisition_status"]["validation_valid_count"] == 9
    calibration = report["calibration_result"]
    assert calibration["status"] == "PASS"
    assert calibration["estimator"] == "solve_gravity_wahba"
    assert calibration["fit"]["pose_count"] == 9
    assert calibration["validation"]["pose_count"] == 9
    assert calibration["fit"]["gravity_direction_residual_rms_rad"] < 1e-7
    assert calibration["validation"]["gravity_direction_residual_p95_rad"] < 1e-7
    assert calibration["fit"]["observability"]["rank"] == 3
    assert calibration["fit"]["observability"]["condition_number"] > 1.0
    assert len(calibration["fit"]["rotation_matrix_R_P_IP"]) == 3
    assert len(calibration["validation"]["euler_xyz_deg_R_P_IP"]) == 3
    assert calibration["fit_vs_validation_so3_delta_deg"] < 1e-7
    estimated = np.asarray(calibration["fit"]["rotation_matrix_R_P_IP"])
    assert runner.rotation_delta_deg(estimated, true_rotation.as_matrix()) < 1e-7


def test_failed_final_park_is_separate_from_static_calibration_result(tmp_path) -> None:
    true_rotation = runner.Rotation.from_euler("xyz", [0.01, 0.02, 1.58])
    result = _write_static_calibration_artifacts(tmp_path, true_rotation)
    result.update(
        {
            "park_result": {"passed": False, "summary": "PARK failed after accepted dwells"},
            "final_readonly_audit": {
                "status": "PASS",
                "initial_positions_steps": {str(i): 60 for i in range(1, 7)},
                "initial_currents_ma": {str(i): 0 for i in range(1, 7)},
                "initial_faults": {str(i): 0 for i in range(1, 7)},
            },
        }
    )

    result.update(runner.build_static_gravity_calibration_result(tmp_path, result))
    result["hardware_teardown_result"] = runner.build_hardware_teardown_result(result)
    result["scientific_closure"] = runner.build_e1d_scientific_closure(result)

    assert result["data_acquisition_status"]["status"] == "PASS"
    assert result["calibration_result"]["status"] == "PASS"
    assert result["hardware_teardown_result"]["status"] == "FAIL"
    assert result["scientific_closure"]["status"] == "STATIC_GRAVITY_SCIENTIFIC_CLOSURE_NOT_FROZEN"
    assert result["calibration_result"]["freeze_recommendation"] == "DO_NOT_FREEZE_R_P_IP"


def test_static_gravity_reporting_does_not_compute_incomplete_dwell_set(tmp_path) -> None:
    raw = {"segments": [], "invalid_dwell_poses": ["home"]}

    report = runner.build_static_gravity_calibration_result(tmp_path, raw)

    assert report["data_acquisition_status"]["status"] == "NO_ACCEPTED_DWELLS"
    assert report["calibration_result"]["status"] == "NOT_COMPUTED"
