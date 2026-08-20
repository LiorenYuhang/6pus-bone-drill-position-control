import subprocess
import sys
from pathlib import Path

PKG_ROOT = Path(__file__).resolve().parent.parent
WORKSPACE_ROOT = Path("/home/lyapunov/bone_drill/bone_ws").resolve()
for p in [PKG_ROOT, WORKSPACE_ROOT, WORKSPACE_ROOT / "scripts"]:
    if str(p) not in sys.path:
        sys.path.insert(0, str(p))

import pytest
import numpy as np

from pus6_motor_driver.safety import (
    ACTUAL_POSITION_ADDR,
    HardwareMode,
    J2F0SafetyViolation,
    J2F0Stop,
    MOTOR_IDS,
    ReadOnlyModeError,
)
from pus6_motor_driver.protocol import (
    CMD_WR_REGISTER,
    pack_read_status_cmd,
    pack_write_register_cmd,
)
from pus6_motor_driver.six_motor_controller import SixMotorController, SixAxisPositionsSnapshot
from pus6_motor_driver.uart_bus import UartBus, RetryExhaustedError
from pus6_motor_driver.benchmark_stats import (
    compute_cycle_timing_stats,
    compute_e2e_latency_stats,
    evaluate_sweep_point,
)
from pus6_motor_driver.fake_bus import FakeBus


class QueueSerial:
    """Mock serial port that feeds configured byte sequences for successive writes."""

    def __init__(self, responses_per_write: list[bytes]) -> None:
        self.responses_queue = list(responses_per_write)
        self.current_buf = bytearray()
        self.write_count = 0

    def write(self, b: bytes) -> int:
        self.write_count += 1
        if self.responses_queue:
            self.current_buf = bytearray(self.responses_queue.pop(0))
        else:
            self.current_buf = bytearray()
        return len(b)

    def read(self, n: int = 1) -> bytes:
        if not self.current_buf:
            return b""
        chunk = bytes(self.current_buf[:n])
        self.current_buf = self.current_buf[n:]
        return chunk

    def flush(self) -> None:
        pass


def test_read_positions_snapshot_with_fake_bus():
    fake_bus = FakeBus()
    controller = SixMotorController(bus=fake_bus, hardware_mode=HardwareMode.READ_ONLY)
    controller.connect()

    snapshot = controller.read_positions_snapshot()
    assert isinstance(snapshot, SixAxisPositionsSnapshot)
    assert len(snapshot.positions_steps) == 6
    assert len(snapshot.positions_meters) == 6
    assert snapshot.timestamp_end_ns >= snapshot.timestamp_start_ns > 0
    assert len(snapshot.per_axis_timestamps_ns) == 6

    # Verify that exactly 6 read register commands (0x31) were sent and 0 writes
    read_events = [e for e in fake_bus.events if e[1] == 0x31]
    assert len(read_events) == 6
    for idx, mid in enumerate(MOTOR_IDS):
        assert read_events[idx][0] == mid
        assert read_events[idx][2] == ACTUAL_POSITION_ADDR

    assert controller.write_count == 0
    controller.close()
    assert controller.write_count == 0


def test_bus_level_pre_write_rejection_in_read_only():
    dummy = QueueSerial([])
    bus = UartBus(serial_instance=dummy, read_only=True)
    write_frame = pack_write_register_cmd(1, 0x29, [100])

    with pytest.raises(ReadOnlyModeError, match="Bus-level write protection"):
        bus.request(write_frame)

    # Assert serial.write was never called
    assert dummy.write_count == 0
    assert bus.tx_cmd_counts.get(CMD_WR_REGISTER, 0) == 0


def test_uart_write_audit_records_each_0x32_frame_before_transmission():
    response = bytearray([
        0xAA, 0x55, 0x0F, 0x04, 0x32, 0x29, 0x00,
        0x35, 0x00, 0x35, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x1E, 0x00,
    ])
    response.append(sum(response[2:]) & 0xFF)
    serial_mock = QueueSerial([bytes(response)])
    bus = UartBus(serial_instance=serial_mock, timeout=0.01, retries=0)

    bus.request(
        pack_write_register_cmd(4, 0x29, [53]),
        expected_id=4,
        expected_command=CMD_WR_REGISTER,
        retries=0,
    )

    assert serial_mock.write_count == 1
    assert len(bus.write_audit) == 1
    assert bus.write_audit[0]["timestamp_ns"] > 0
    assert bus.write_audit[0]["timestamp_monotonic_ns"] > 0
    assert bus.write_audit[0]["motor_id"] == 4
    assert bus.write_audit[0]["register_address"] == 0x29
    assert bus.write_audit[0]["payload"] == [53]
    assert bus.get_telemetry()["write_audit"] == bus.write_audit


def test_uart_telemetry_retry_recovered():
    valid_resp = bytes([
        0xAA, 0x55, 0x0F, 0x01, 0x30, 0x00, 0x00,
        0x32, 0x00, 0x32, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1E, 0x00, 0xC2
    ])

    # 1st write gets timeout (empty bytes), 2nd write gets valid_resp
    serial_mock = QueueSerial([b"", valid_resp])
    bus = UartBus(serial_instance=serial_mock, timeout=0.01, retries=2)
    frame = pack_read_status_cmd(1)

    resp = bus.request(frame)
    assert resp == valid_resp

    tel = bus.get_telemetry()
    assert tel["transaction_count"] == 1
    assert tel["attempt_count"] == 2
    assert tel["retry_count"] == 1
    assert tel["timeout_count"] == 1
    assert tel["failed_transaction_count"] == 0


def test_uart_telemetry_timeout_counting():
    # All 3 attempts return empty bytes
    serial_mock = QueueSerial([b"", b"", b""])
    bus = UartBus(serial_instance=serial_mock, timeout=0.01, retries=2)
    frame = pack_read_status_cmd(1)

    with pytest.raises(RetryExhaustedError):
        bus.request(frame)

    tel = bus.get_telemetry()
    assert tel["transaction_count"] == 1
    assert tel["attempt_count"] == 3  # 1 initial + 2 retries
    assert tel["retry_count"] == 2
    assert tel["timeout_count"] == 3
    assert tel["failed_transaction_count"] == 1


def test_uart_telemetry_checksum_error_counting():
    # Bad checksum frame
    bad_resp = bytes([
        0xAA, 0x55, 0x0F, 0x01, 0x30, 0x00, 0x00,
        0x32, 0x00, 0x32, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1E, 0x00, 0x00  # Bad checksum
    ])
    valid_resp = bytes([
        0xAA, 0x55, 0x0F, 0x01, 0x30, 0x00, 0x00,
        0x32, 0x00, 0x32, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1E, 0x00, 0xC2
    ])

    serial_mock = QueueSerial([bad_resp, valid_resp])
    bus = UartBus(serial_instance=serial_mock, timeout=0.01, retries=2)
    frame = pack_read_status_cmd(1)

    resp = bus.request(frame)
    assert resp == valid_resp

    tel = bus.get_telemetry()
    assert tel["transaction_count"] == 1
    assert tel["attempt_count"] == 2
    assert tel["retry_count"] == 1
    assert tel["checksum_error_count"] == 1
    assert tel["failed_transaction_count"] == 0


def test_realtime_position_drift_stop_logic():
    from run_j2_f0_benchmark import J2F0BenchmarkHarness

    harness = J2F0BenchmarkHarness(mock=True)
    baseline = [50, 50, 50, 50, 50, 50]

    # Valid within +/- 3 steps
    valid_positions = [51, 49, 52, 48, 53, 47]
    harness._check_realtime_position_drift(valid_positions, baseline, "test")

    # Drift of 4 steps triggers J2F0Stop
    drifted_positions = [50, 50, 55, 50, 50, 50]  # ID3 drifted by 5
    with pytest.raises(J2F0Stop, match="Real-time position drift STOP: Motor ID3"):
        harness._check_realtime_position_drift(drifted_positions, baseline, "test")


def test_baseline_admission_strict_zero_current():
    from run_j2_f0_benchmark import J2F0BenchmarkHarness

    fake_bus = FakeBus()
    controller = SixMotorController(bus=fake_bus, hardware_mode=HardwareMode.READ_ONLY)
    controller.connect()

    harness = J2F0BenchmarkHarness(mock=True)

    # Normal static baseline (0 mA) passes admission
    snap = harness._verify_pre_post_0x30_snapshot(controller, "admission_test")
    assert snap["fault_set"] == ["0x00"]
    assert max(snap["currents_mA"]) == 0

    # Current > 0 mA fails admission
    fake_bus.currents[1] = 5  # 5 mA > 0 mA
    with pytest.raises(J2F0SafetyViolation, match="baseline admission failed: non-zero current 5mA > 0mA"):
        harness._verify_pre_post_0x30_snapshot(controller, "admission_test")


def test_f0_a_exception_closes_bus():
    from run_j2_f0_benchmark import J2F0BenchmarkHarness

    class DriftingFakeBus(FakeBus):
        def __init__(self):
            super().__init__()
            self.read_count = 0

        def request(self, frame: bytes, **kwargs) -> bytes:
            self.read_count += 1
            if self.read_count > 6:  # After pre-flight 0x30 snapshot (6 reads), drift ID1
                self.positions[1] = 999
            return super().request(frame, **kwargs)

    fake_bus = DriftingFakeBus()
    harness = J2F0BenchmarkHarness(mock=True)
    harness._create_controller = lambda: SixMotorController(bus=fake_bus, hardware_mode=HardwareMode.READ_ONLY)

    with pytest.raises(J2F0Stop, match="Real-time position drift STOP"):
        harness.run_f0_a(duration_sec=1.0)

    # Verify that the controller and bus were closed in the finally block!
    assert fake_bus.closed is True


@pytest.mark.parametrize("target_hz", [25.0, 50.0])
def test_mock_ros_auditor_end_to_end_at_25_and_50_hz(target_hz, tmp_path):
    """Exercise the real cross-process Auditor path without opening a UART."""
    from run_j2_f0_benchmark import J2F0BenchmarkHarness

    harness = J2F0BenchmarkHarness(mock=True, log_dir=str(tmp_path))
    result = harness._run_ros_timed_window_with_auditor(
        target_hz=target_hz,
        duration_sec=1.5,
        read_mode="full_state",
        publish_raw=True,
        theoretical_wire_ms=1.823,
        stage_label=f"MOCK-DIAG@{target_hz:.1f}Hz",
    )

    assert result["auditor_subscription_count"] >= 1
    assert result["driver_publish_timestamp_count"] > 1
    assert result["auditor_receive_rate_hz"] > 0.0
    assert result["auditor_delivery_ratio"] >= 0.95

    interval_stats = result["ros_publish_interval_stats"]
    assert interval_stats is not None
    assert interval_stats["sample_count"] == result["driver_publish_timestamp_count"] - 1
    assert interval_stats["median_p50_ms"] > 0.0
    assert interval_stats["p95_ms"] > 0.0
    assert interval_stats["p99_ms"] > 0.0
    assert interval_stats["max_ms"] > 0.0

    # Deadline/on-time values must come from measured intervals, not an empty
    # sample fallback.
    assert result["on_time_ratio"] >= 0.0
    assert result["deadline_miss_count"] >= 0
    assert result["deadline_miss_ratio"] >= 0.0
    assert "No publish interval samples" not in result["fail_reasons"]

    telemetry = result["uart_telemetry"]
    for key in (
        "transaction_count",
        "attempt_count",
        "retry_count",
        "timeout_count",
        "checksum_error_count",
        "protocol_error_count",
        "failed_transaction_count",
    ):
        assert key in telemetry
    assert telemetry["transaction_count"] > 0
    assert telemetry["attempt_count"] >= telemetry["transaction_count"]
    assert result["write_audit"]["controller_write_count"] == 0
    assert result["write_audit"]["cmd_0x32_tx_count"] == 0


def test_per_axis_start_and_end_timestamps():
    fake_bus = FakeBus()
    controller = SixMotorController(bus=fake_bus, hardware_mode=HardwareMode.READ_ONLY)
    controller.connect()

    # 1. Full-state snapshot
    snap_state = controller.read_state_snapshot()
    assert len(snap_state.per_axis_start_timestamps_ns) == 6
    assert len(snap_state.per_axis_end_timestamps_ns) == 6
    for mid in MOTOR_IDS:
        start_ns = snap_state.per_axis_start_timestamps_ns[mid]
        end_ns = snap_state.per_axis_end_timestamps_ns[mid]
        assert end_ns >= start_ns > 0
    # Sequential ordering: ID6 start occurs after ID1 start
    assert snap_state.per_axis_start_timestamps_ns[6] >= snap_state.per_axis_start_timestamps_ns[1]

    # 2. Position-only snapshot
    snap_pos = controller.read_positions_snapshot()
    assert len(snap_pos.per_axis_start_timestamps_ns) == 6
    assert len(snap_pos.per_axis_end_timestamps_ns) == 6
    for mid in MOTOR_IDS:
        start_ns = snap_pos.per_axis_start_timestamps_ns[mid]
        end_ns = snap_pos.per_axis_end_timestamps_ns[mid]
        assert end_ns >= start_ns > 0

    controller.close()


def test_publish_interval_deadline_calculation_isolated():
    # Simulate publish intervals: target period = 20.0ms (50Hz)
    target_period_ms = 20.0

    # 5 intervals: [20.1ms (on-time), 19.9ms (on-time), 22.5ms (deadline miss), 20.0ms (on-time), 19.8ms (on-time)]
    # All are within |dt - 20| < 2.0 except 22.5ms (diff = 2.5)
    t_pub_starts_ns = [
        0,
        20_100_000,
        40_000_000,
        62_500_000,
        82_500_000,
        102_300_000,
    ]
    pub_intervals_ns = list(np.diff(t_pub_starts_ns))
    pub_intervals_ms = np.array(pub_intervals_ns, dtype=np.float64) / 1e6

    deadline_miss_count = 0
    max_consecutive_miss = 0
    current_consecutive_miss = 0
    on_time_count = 0

    for interval in pub_intervals_ms:
        if abs(interval - target_period_ms) < 2.0:
            on_time_count += 1
        if interval >= target_period_ms:
            deadline_miss_count += 1
            current_consecutive_miss += 1
            if current_consecutive_miss > max_consecutive_miss:
                max_consecutive_miss = current_consecutive_miss
        else:
            current_consecutive_miss = 0

    on_time_ratio = on_time_count / len(pub_intervals_ms)
    assert on_time_count == 4
    assert on_time_ratio == 0.8
    assert max_consecutive_miss == 2  # 20.1ms and 22.5ms (interval 1 was 20.1, interval 2 was 19.9 -> reset, interval 3 was 22.5, interval 4 was 20.0 -> 2 consecutive!)


def test_cli_fail_closed_on_real_hardware_all():
    script_path = WORKSPACE_ROOT / "scripts" / "run_j2_f0_benchmark.py"
    # Running --real-hardware with --group ALL must be rejected with exit code 1
    res = subprocess.run(
        [sys.executable, str(script_path), "--real-hardware", "--group", "ALL"],
        capture_output=True,
        text=True,
    )
    assert res.returncode == 1
    assert "FAIL-CLOSED REJECTION" in res.stdout
