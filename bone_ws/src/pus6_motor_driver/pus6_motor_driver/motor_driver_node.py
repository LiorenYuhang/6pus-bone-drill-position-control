"""ROS 2 Node wrapping SixMotorController for 6PUS robot platform with decoupled UART worker."""

from __future__ import annotations

import logging
import threading
import time
from dataclasses import dataclass
from enum import Enum
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, Int32MultiArray, Int64MultiArray
from std_srvs.srv import Trigger

try:
    from .safety import (
        MOTOR_IDS,
        NUM_MOTORS,
        ABSOLUTE_RAW_LOWER_LIMIT,
        ABSOLUTE_RAW_UPPER_LIMIT,
        NORMAL_MOTION_LOWER_LIMIT,
        NORMAL_MOTION_UPPER_LIMIT,
        DriverError,
        HardwareMode,
        J2F0SafetyViolation,
        J2F0Stop,
        ReadOnlyModeError,
        SafetyViolation,
        TargetValidationError,
        meters_to_steps,
        parse_hardware_mode,
        steps_to_meters,
    )
    from .six_motor_controller import SixAxisStateSnapshot, SixMotorController
except ImportError:
    from safety import (
        MOTOR_IDS,
        NUM_MOTORS,
        ABSOLUTE_RAW_LOWER_LIMIT,
        ABSOLUTE_RAW_UPPER_LIMIT,
        NORMAL_MOTION_LOWER_LIMIT,
        NORMAL_MOTION_UPPER_LIMIT,
        DriverError,
        HardwareMode,
        J2F0SafetyViolation,
        J2F0Stop,
        ReadOnlyModeError,
        SafetyViolation,
        TargetValidationError,
        meters_to_steps,
        parse_hardware_mode,
        steps_to_meters,
    )
    from six_motor_controller import SixAxisStateSnapshot, SixMotorController

try:
    from pus6_msgs.msg import TimedJointCommand, TimedJointFeedback
except ImportError:
    TimedJointCommand = None  # type: ignore
    TimedJointFeedback = None  # type: ignore


@dataclass
class TargetEnvelope:
    """Latest target container for non-blocking producer-consumer buffering."""
    targets_steps: list[int]
    seq: int
    stamp_ns: int
    consumed: bool = False


class Pus6MotorDriverNode(Node):
    """ROS 2 Node providing decoupled command dispatch and sensor telemetry for 6PUS platform."""

    def __init__(
        self,
        *,
        hardware_mode: str | HardwareMode | None = None,
        write_rate_hz: float | None = None,
        poll_rate_hz: float | None = None,
        read_mode: str | None = None,
        port: str | None = None,
        baudrate: int | None = None,
        timeout: float | None = None,
        auto_connect: bool | None = None,
        auto_configure: bool | None = None,
        publish_raw_diagnostics: bool | None = None,
        enable_j2_f0_instrumentation: bool | None = None,
        maintenance_mode: bool | None = None,
        node_name: str = "pus6_motor_driver_node",
        **kwargs,
    ) -> None:
        super().__init__(node_name, **kwargs)

        # Declare parameters (default to safe mock mode)
        self.declare_parameter("port", "/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0")
        self.declare_parameter("baudrate", 921600)
        self.declare_parameter("timeout", 0.05)
        self.declare_parameter("poll_rate_hz", 10.0)        # Background low-rate telemetry
        self.declare_parameter("write_rate_hz", 50.0)       # Dedicated high-rate UART dispatch loop
        self.declare_parameter("target_timeout_sec", 0.10)  # Watchdog threshold for target stream (100 ms)
        self.declare_parameter("timeout_behavior", "stop_write") # "stop_write" | "hold" | "pause"
        self.declare_parameter("reconnect_max_jump_steps", 10)   # Max jump allowed when recovering from timeout (10 steps = 0.25mm)
        self.declare_parameter("hardware_mode", "mock")     # Default must be "mock"
        self.declare_parameter("mock_hardware", True)       # Deprecated backward-compat alias
        self.declare_parameter("auto_connect", True)
        self.declare_parameter("auto_configure", False)     # Never auto-configure on launch
        self.declare_parameter("read_mode", "full_state")   # "full_state" | "position_only"
        self.declare_parameter("publish_raw_diagnostics", True)
        self.declare_parameter("enable_j2_f0_instrumentation", False)
        self.declare_parameter("maintenance_mode", False)

        p_port = port if port is not None else self.get_parameter("port").get_parameter_value().string_value
        p_baudrate = baudrate if baudrate is not None else self.get_parameter("baudrate").get_parameter_value().integer_value
        p_timeout = timeout if timeout is not None else self.get_parameter("timeout").get_parameter_value().double_value
        p_poll_rate = poll_rate_hz if poll_rate_hz is not None else self.get_parameter("poll_rate_hz").get_parameter_value().double_value
        self.write_rate_hz = write_rate_hz if write_rate_hz is not None else self.get_parameter("write_rate_hz").get_parameter_value().double_value
        self.target_timeout_sec = self.get_parameter("target_timeout_sec").get_parameter_value().double_value
        self.timeout_behavior = self.get_parameter("timeout_behavior").get_parameter_value().string_value
        self.reconnect_max_jump_steps = self.get_parameter("reconnect_max_jump_steps").get_parameter_value().integer_value
        mode_str = hardware_mode if hardware_mode is not None else self.get_parameter("hardware_mode").get_parameter_value().string_value
        p_auto_connect = auto_connect if auto_connect is not None else self.get_parameter("auto_connect").get_parameter_value().bool_value
        p_auto_configure = auto_configure if auto_configure is not None else self.get_parameter("auto_configure").get_parameter_value().bool_value
        self.read_mode = read_mode if read_mode is not None else self.get_parameter("read_mode").get_parameter_value().string_value
        self.publish_raw_diagnostics = publish_raw_diagnostics if publish_raw_diagnostics is not None else self.get_parameter("publish_raw_diagnostics").get_parameter_value().bool_value
        self.enable_j2_f0_instrumentation = enable_j2_f0_instrumentation if enable_j2_f0_instrumentation is not None else self.get_parameter("enable_j2_f0_instrumentation").get_parameter_value().bool_value
        self.maintenance_mode = maintenance_mode if maintenance_mode is not None else self.get_parameter("maintenance_mode").get_parameter_value().bool_value

        self.hardware_mode = parse_hardware_mode(mode_str)
        self.runtime_error: Exception | None = None
        self._last_state_snapshot: SixAxisStateSnapshot | None = None
        self._last_positions_snapshot: Any = None
        self._last_commanded_steps: list[int] | None = None

        self.get_logger().info(
            f"Initializing Pus6MotorDriverNode (mode={self.hardware_mode.value}, "
            f"write_rate={self.write_rate_hz}Hz, timeout={self.target_timeout_sec*1e3:.0f}ms, "
            f"poll_rate={p_poll_rate}Hz, port={p_port}, baud={p_baudrate})"
        )

        self.controller = SixMotorController(
            port=p_port,
            baudrate=p_baudrate,
            timeout=p_timeout,
            hardware_mode=self.hardware_mode,
        )

        # Standard ROS 2 Feedback Publisher
        self.joint_state_pub = self.create_publisher(JointState, "/pus6/joint_states", 10)

        # Raw Diagnostic Telemetry Publishers
        self.raw_steps_pub = self.create_publisher(Int32MultiArray, "/pus6/joint_steps_raw", 10)
        self.raw_currents_pub = self.create_publisher(Int32MultiArray, "/pus6/joint_currents_raw", 10)
        self.raw_forces_pub = self.create_publisher(Int32MultiArray, "/pus6/joint_forces_raw", 10)
        self.raw_temps_pub = self.create_publisher(Int32MultiArray, "/pus6/joint_temperatures_raw", 10)
        self.raw_faults_pub = self.create_publisher(Int32MultiArray, "/pus6/joint_faults_raw", 10)

        # Driver Performance & Scheduling Diagnostics Publisher
        # [target_in_count, overwrite_count, consumed_count, deadline_miss_count, target_age_ms, write_duration_ms]
        self.driver_diagnostics_pub = self.create_publisher(
            Float64MultiArray, "/pus6/driver_diagnostics", 10
        )

        # Dedicated Benchmark Stamp Publisher [seq, t_pub_mono_ns] (gated behind param)
        if self.enable_j2_f0_instrumentation:
            self.benchmark_stamp_pub = self.create_publisher(
                Int64MultiArray, "/pus6/benchmark_stamp", qos_profile_sensor_data
            )
        else:
            self.benchmark_stamp_pub = None
        self._benchmark_seq_id = 0
        self.benchmark_publish_timestamps_ns: list[int] = []

        # J5-C1 Timed Telemetry Publishers (Host CLOCK_MONOTONIC Timing)
        if TimedJointFeedback is not None:
            self.timed_feedback_pub = self.create_publisher(
                TimedJointFeedback, "/pus6/joint_feedback/timed", qos_profile_sensor_data
            )
            self.timed_command_pub = self.create_publisher(
                TimedJointCommand, "/pus6/joint_command/timed", qos_profile_sensor_data
            )
        else:
            self.timed_feedback_pub = None
            self.timed_command_pub = None
        self._uart_cycle_seq = 0

        # Latest-Value-Wins decoupling buffer and statistics
        self._target_lock = threading.Lock()
        self._latest_target: TargetEnvelope | None = None
        self._target_in_count: int = 0
        self._target_overwrite_count: int = 0
        self._target_consumed_count: int = 0
        self._deadline_miss_count: int = 0
        self._last_target_age_ms: float = 0.0
        self._last_write_duration_ms: float = 0.0
        self._last_write_stamp_ns: int = 0
        self._last_target_rx_time_ns: int = 0
        self._stale_target_timeout_count: int = 0
        self._stream_timed_out: bool = False
        self._consecutive_comm_failures: int = 0

        # Segmented Timing and Contention Instrumentation
        self._last_worker_start_ns: int = 0
        self._target_fetch_ms: float = 0.0
        self._final_preflight_ms: float = 0.0
        self._uart_lock_wait_ms: float = 0.0
        self._six_axis_uart_transaction_ms: float = 0.0
        self._feedback_parse_ms: float = 0.0
        self._ros_publish_ms: float = 0.0
        self._worker_total_ms: float = 0.0
        self._worker_start_to_start_ms: float = 0.0
        self._per_motor_ms: dict[int, float] = {}
        self._background_poll_count: int = 0
        self._background_poll_during_active_count: int = 0
        self._uart_lock_contention_count: int = 0
        self._timeout_events: list[dict[str, Any]] = []
        self._slew_recovery_count: int = 0
        self._last_slew_alpha: float = 1.0
        self._last_slew_original_gap: int = 0
        self._last_slew_sent_gap: int = 0
        self._last_slew_remaining_gap: int = 0

        # Dedicated Callback Groups for thread decoupling in MultiThreadedExecutor
        self.sub_cb_group = MutuallyExclusiveCallbackGroup()
        self.worker_cb_group = MutuallyExclusiveCallbackGroup()
        self.poll_cb_group = MutuallyExclusiveCallbackGroup()

        # Dedicated QoS for high-rate continuous control stream: Latest-Value-Wins (Keep last 1, Best Effort)
        control_cmd_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.declare_parameter("command_ingress_mode", "steps") # "steps" (authoritative) | "meters" | "all"
        self.command_ingress_mode = self.get_parameter("command_ingress_mode").get_parameter_value().string_value

        # Subscriptions (Non-blocking callbacks isolated in sub_cb_group)
        # Default authoritative command ingress: steps only (prevents dual-topic overwrite race)
        if self.command_ingress_mode in ("steps", "all"):
            self.cmd_steps_sub = self.create_subscription(
                Int32MultiArray,
                "/pus6/command_joint_steps",
                self._handle_command_joint_steps,
                control_cmd_qos,
                callback_group=self.sub_cb_group,
            )
        else:
            self.cmd_steps_sub = None

        if self.command_ingress_mode in ("meters", "all"):
            self.cmd_positions_sub = self.create_subscription(
                Float64MultiArray,
                "/pus6/command_joint_positions",
                self._handle_command_positions_meters,
                control_cmd_qos,
                callback_group=self.sub_cb_group,
            )
        else:
            self.cmd_positions_sub = None

        self.cmd_single_step_sub = self.create_subscription(
            Int32MultiArray,
            "/pus6/command_single_step",
            self._handle_command_single_step,
            10,
            callback_group=self.sub_cb_group,
        )

        # Services
        self.pause_srv = self.create_service(
            Trigger,
            "/pus6/pause_all",
            self._handle_pause_all,
            callback_group=self.sub_cb_group,
        )
        self.config_srv = self.create_service(
            Trigger,
            "/pus6/configure_motors",
            self._handle_configure_motors,
            callback_group=self.sub_cb_group,
        )

        self.joint_names = [f"motor_{mid}" for mid in MOTOR_IDS]

        if p_auto_connect:
            try:
                self.controller.connect()
                if p_auto_configure:
                    if self.hardware_mode == HardwareMode.READ_ONLY:
                        self.get_logger().warn("auto_configure skipped: Node is in READ_ONLY mode.")
                    else:
                        self.controller.configure_all()
            except Exception as exc:
                self.get_logger().error(f"Failed to connect to hardware on startup: {exc}")

        # Dedicated high-rate UART dispatch worker timer (Latest-Value-Wins consumer in worker_cb_group)
        if self.write_rate_hz > 0.0:
            write_interval = 1.0 / max(1.0, self.write_rate_hz)
            self.write_timer = self.create_timer(
                write_interval, self._uart_worker_cycle, callback_group=self.worker_cb_group
            )
        else:
            self.write_timer = None

        # Background low-rate polling timer (in poll_cb_group)
        if p_poll_rate > 0.0:
            poll_interval = 1.0 / max(1.0, p_poll_rate)
            self.poll_timer = self.create_timer(
                poll_interval, self._background_poll_and_publish, callback_group=self.poll_cb_group
            )
        else:
            self.poll_timer = None

    def _handle_command_positions_meters(self, msg: Float64MultiArray) -> None:
        """Non-blocking ingestion of 6-axis position targets in SI meters."""
        if self.hardware_mode == HardwareMode.READ_ONLY:
            self.get_logger().error("Command rejected: pus6_motor_driver is in READ_ONLY mode.")
            return

        if len(msg.data) != NUM_MOTORS:
            self.get_logger().error(
                f"Invalid target meters vector length: {len(msg.data)} (expected {NUM_MOTORS}). Ignored."
            )
            return

        targets_steps = [meters_to_steps(p) for p in msg.data]
        self._enqueue_latest_target(targets_steps)

    def _handle_command_joint_steps(self, msg: Int32MultiArray) -> None:
        """Non-blocking ingestion of 6-axis position targets in integer steps."""
        if self.hardware_mode == HardwareMode.READ_ONLY:
            self.get_logger().error("Command rejected: pus6_motor_driver is in READ_ONLY mode.")
            return

        targets_steps = list(msg.data)
        if len(targets_steps) != NUM_MOTORS:
            self.get_logger().error(
                f"Invalid target steps vector length: {len(targets_steps)} (expected {NUM_MOTORS}). Ignored."
            )
            return

        self._enqueue_latest_target(targets_steps)

    def _enqueue_latest_target(self, targets_steps: list[int]) -> None:
        """Atomic Latest-Value-Wins buffer update with timeout recovery gating."""
        now_ns = time.perf_counter_ns()
        with self._target_lock:
            # If recovering from a timed-out state, verify jump threshold against last confirmed position
            if self._stream_timed_out:
                last_known = self._last_commanded_steps or (
                    self._last_state_snapshot.positions_steps if self._last_state_snapshot else None
                )
                if last_known is not None:
                    max_jump = max(abs(t - p) for t, p in zip(targets_steps, last_known))
                    if max_jump > self.reconnect_max_jump_steps:
                        self.get_logger().error(
                            f"Timeout recovery target rejected: max jump {max_jump} steps > limit {self.reconnect_max_jump_steps} steps. "
                            "Target generator must re-anchor before resuming.",
                            throttle_duration_sec=1.0,
                        )
                        return
                self.get_logger().info(
                    f"Target stream re-armed after timeout (max jump: {max_jump if last_known else 0} steps)."
                )
                self._stream_timed_out = False

            self._last_target_rx_time_ns = now_ns
            if self._latest_target is not None and not self._latest_target.consumed:
                self._target_overwrite_count += 1
            self._latest_target = TargetEnvelope(
                targets_steps=targets_steps,
                seq=self._target_in_count,
                stamp_ns=now_ns,
                consumed=False,
            )
            self._target_in_count += 1

    def _execute_safety_preflight(self, targets_steps: list[int]) -> bool:
        """Exhaustive safety preflight check right before issuing 6-axis UART command."""
        # 1. Connection check
        if not self.controller.is_connected:
            self.get_logger().error("Safety preflight FAIL: Controller not connected.")
            return False

        # 2. Dimensions check
        if len(targets_steps) != NUM_MOTORS:
            self.get_logger().error(f"Safety preflight FAIL: Target length {len(targets_steps)} != {NUM_MOTORS}.")
            return False

        # 3. Hardware absolute range [0, 2000] steps (0 ~ 50mm)
        for i, s in enumerate(targets_steps):
            if not (ABSOLUTE_RAW_LOWER_LIMIT <= s <= ABSOLUTE_RAW_UPPER_LIMIT):
                self.get_logger().error(f"Safety preflight FAIL: Motor {i+1} target {s} out of absolute range [{ABSOLUTE_RAW_LOWER_LIMIT}, {ABSOLUTE_RAW_UPPER_LIMIT}].")
                return False

        # 4. Normal J4 commands stay in the operating envelope. A raw PARK
        # target is accepted only when the node is explicitly in maintenance mode.
        if not self.maintenance_mode:
            for i, s in enumerate(targets_steps):
                if not (NORMAL_MOTION_LOWER_LIMIT <= s <= NORMAL_MOTION_UPPER_LIMIT):
                    self.get_logger().error(
                        f"Safety preflight FAIL: Motor {i+1} target {s} out of normal operating envelope "
                        f"[{NORMAL_MOTION_LOWER_LIMIT}, {NORMAL_MOTION_UPPER_LIMIT}]. "
                        "Set maintenance_mode=true only for an authorized PARK workflow."
                    )
                    return False

        # 5. Inter-frame delta check (against last commanded steps)
        if self._last_commanded_steps is not None:
            max_step_delta = 20  # 20 steps = 0.50 mm (final hard safety preflight gate)
            for i, (t_new, t_old) in enumerate(zip(targets_steps, self._last_commanded_steps)):
                if abs(t_new - t_old) > max_step_delta:
                    self.get_logger().error(
                        f"Safety preflight FAIL: Motor {i+1} inter-frame delta {abs(t_new - t_old)} > {max_step_delta} steps."
                    )
                    return False

        # 6. Fresh feedback & Motor Fault Gate: check if previous snapshot reported fault != 0
        if self._last_state_snapshot is not None:
            now_ns = time.perf_counter_ns()
            feedback_age_ms = (now_ns - self._last_state_snapshot.timestamp_end_ns) / 1e6
            if feedback_age_ms > 1000.0 and self.hardware_mode == HardwareMode.ACTIVE:
                self.get_logger().error(
                    f"Safety preflight FAIL: State feedback is stale ({feedback_age_ms:.1f}ms > 1000ms)."
                )
                return False

            for i, f in enumerate(self._last_state_snapshot.fault_codes):
                if f != 0:
                    self.get_logger().error(f"Safety preflight FAIL: Motor {i+1} has active fault code {f}.")
                    return False

        elif self.hardware_mode == HardwareMode.ACTIVE:
            self.get_logger().error("Safety preflight FAIL: No valid state snapshot available prior to physical write.")
            return False

        return True

    def _apply_vector_slew_recovery(
        self, candidate_targets_steps: list[int], now_ns: int
    ) -> tuple[list[int], bool, float, int, int, int]:
        """Applies Time-Aware Vector Slew Recovery if candidate targets step gap exceeds dynamic allowance.

        Preserves exact 6-axis vector proportion without silent per-axis independent clamping.
        Caps single-dispatch step delta strictly <= 20 steps.

        Returns:
            (dispatched_steps, recovery_active, alpha, original_gap, sent_gap, remaining_gap)
        """
        # Anchor is the last successfully dispatched 6-axis vector
        anchor = self._last_commanded_steps
        if anchor is None:
            if self._last_state_snapshot is not None:
                anchor = list(self._last_state_snapshot.positions_steps)
            else:
                anchor = list(candidate_targets_steps)

        # Elapsed time since last successful write dispatch
        if self._last_write_stamp_ns > 0:
            dt_elapsed_s = max(0.010, (now_ns - self._last_write_stamp_ns) / 1e9)
        else:
            dt_elapsed_s = 1.0 / max(1.0, self.write_rate_hz)

        # Allowance calculation based on shaper max speed (0.005 m/s = 200 steps/s)
        # Cap strictly <= 20 steps (the final hard safety gate)
        vmax_steps_per_sec = 200.0
        raw_allowance = int(vmax_steps_per_sec * dt_elapsed_s)
        allowance = min(20, max(5, raw_allowance))

        gaps = [c - a for c, a in zip(candidate_targets_steps, anchor)]
        max_gap = max(abs(g) for g in gaps)

        if max_gap <= allowance:
            self._last_slew_alpha = 1.0
            self._last_slew_original_gap = max_gap
            self._last_slew_sent_gap = max_gap
            self._last_slew_remaining_gap = 0
            return list(candidate_targets_steps), False, 1.0, max_gap, max_gap, 0

        # Vector proportional scaling across all 6 axes
        alpha = float(allowance) / float(max_gap)
        dispatched = [int(round(a + alpha * g)) for a, g in zip(anchor, gaps)]

        # Strict safety cap <= 20 steps from anchor for all axes
        for i in range(len(dispatched)):
            d = dispatched[i] - anchor[i]
            if abs(d) > 20:
                dispatched[i] = anchor[i] + (20 if d > 0 else -20)

        sent_gap = max(abs(d - a) for d, a in zip(dispatched, anchor))
        remaining_gap = max_gap - sent_gap

        self._slew_recovery_count += 1
        self._last_slew_alpha = alpha
        self._last_slew_original_gap = max_gap
        self._last_slew_sent_gap = sent_gap
        self._last_slew_remaining_gap = remaining_gap

        self.get_logger().warn(
            f"Vector slew recovery active: original_gap={max_gap} steps > allowance={allowance} steps (dt={dt_elapsed_s*1000:.1f}ms). "
            f"Scaling 6-axis vector by alpha={alpha:.3f} -> sent_gap={sent_gap} steps (remaining={remaining_gap} steps).",
            throttle_duration_sec=1.0,
        )

        return dispatched, True, alpha, max_gap, sent_gap, remaining_gap

    def _uart_worker_cycle(self) -> None:
        """High-rate UART worker: dispatches latest target with watchdog, preflight, and segmented telemetry."""
        if not self.controller.is_connected:
            return

        t_entry_ns = time.perf_counter_ns()
        if self._last_worker_start_ns > 0:
            self._worker_start_to_start_ms = (t_entry_ns - self._last_worker_start_ns) / 1e6
        self._last_worker_start_ns = t_entry_ns

        target_env: TargetEnvelope | None = None

        t_fetch_start_ns = time.perf_counter_ns()
        with self._target_lock:
            # Watchdog check: If active streaming has started and target stream timed out
            if self._last_target_rx_time_ns > 0:
                age_since_rx_ms = (t_entry_ns - self._last_target_rx_time_ns) / 1e6
                if age_since_rx_ms > (self.target_timeout_sec * 1000.0):
                    if not self._stream_timed_out:
                        self._stream_timed_out = True
                        self._stale_target_timeout_count += 1
                        self.get_logger().warn(
                            f"Target stream timed out ({age_since_rx_ms:.1f}ms > {self.target_timeout_sec*1000:.0f}ms). "
                            f"Halting continuous writes (behavior={self.timeout_behavior}).",
                            throttle_duration_sec=2.0,
                        )
                        if self.timeout_behavior == "pause" and self.hardware_mode != HardwareMode.READ_ONLY:
                            try:
                                self.controller.pause_all()
                            except Exception as p_exc:
                                self.get_logger().error(f"Failed to pause on timeout: {p_exc}")
                    # In timeout state: do NOT repeat stale targets over UART!
                    return

            if self._latest_target is not None and not self._latest_target.consumed:
                target_env = self._latest_target
                target_env.consumed = True
                self._target_consumed_count += 1

        t_fetch_end_ns = time.perf_counter_ns()
        self._target_fetch_ms = (t_fetch_end_ns - t_fetch_start_ns) / 1e6

        if target_env is None:
            return

        # Time-Aware Vector Slew Recovery before preflight
        dispatched_targets, rec_active, alpha, orig_gap, sent_gap, rem_gap = self._apply_vector_slew_recovery(
            target_env.targets_steps, t_entry_ns
        )

        # Final safety preflight right before hardware dispatch
        t_preflight_start_ns = time.perf_counter_ns()
        preflight_ok = self._execute_safety_preflight(dispatched_targets)
        t_preflight_end_ns = time.perf_counter_ns()
        self._final_preflight_ms = (t_preflight_end_ns - t_preflight_start_ns) / 1e6

        if not preflight_ok:
            self.get_logger().error("Aborting UART write cycle due to safety preflight failure.")
            return

        age_ms = (t_entry_ns - target_env.stamp_ns) / 1e6
        self._last_target_age_ms = age_ms

        t_write_start_ns = time.perf_counter_ns()
        try:
            cmd_result = self.controller.command_positions_with_metrics(dispatched_targets)
            self._consecutive_comm_failures = 0
            self._last_commanded_steps = list(dispatched_targets)
            t_write_end_ns = time.perf_counter_ns()
            write_dur_ms = (t_write_end_ns - t_write_start_ns) / 1e6
            self._six_axis_uart_transaction_ms = write_dur_ms
            self._last_write_duration_ms = write_dur_ms
            self._last_write_stamp_ns = t_write_end_ns

            # Per-motor transaction breakdown
            for mid in MOTOR_IDS:
                t_m_start = cmd_result.per_axis_write_timestamps_ns.get(mid, 0)
                t_m_end = cmd_result.per_axis_write_completion_timestamps_ns.get(mid, 0)
                if t_m_start > 0 and t_m_end > 0:
                    self._per_motor_ms[mid] = (t_m_end - t_m_start) / 1e6

            target_period_ms = 1000.0 / max(1.0, self.write_rate_hz)
            if write_dur_ms > target_period_ms:
                self._deadline_miss_count += 1

            # Convert 0x29 write responses directly into a SixAxisStateSnapshot
            t_parse_start_ns = time.perf_counter_ns()
            snapshot = cmd_result.to_state_snapshot()
            self._last_state_snapshot = snapshot
            t_parse_end_ns = time.perf_counter_ns()
            self._feedback_parse_ms = (t_parse_end_ns - t_parse_start_ns) / 1e6

            # Shared sequence ID for 1-to-1 pairing of command and companion feedback
            current_uart_seq = self._uart_cycle_seq
            self._uart_cycle_seq += 1

            # ROS publish telemetry
            t_pub_start_ns = time.perf_counter_ns()
            self._publish_snapshot_telemetry(snapshot, current_uart_seq)
            self._publish_driver_diagnostics()

            # J5-C1 Timed Command Publisher (Dispatched target with actual transaction timestamps)
            if self.timed_command_pub is not None:
                tc_msg = TimedJointCommand()
                tc_msg.header.stamp = self.get_clock().now().to_msg()
                t_start_cmd = cmd_result.per_axis_write_timestamps_ns.get(1, 0)
                t_end_cmd = cmd_result.per_axis_write_completion_timestamps_ns.get(6, 0)
                tc_msg.transaction_start_monotonic_ns = t_start_cmd
                tc_msg.transaction_end_monotonic_ns = t_end_cmd
                tc_msg.per_axis_tx_start_monotonic_ns = [
                    cmd_result.per_axis_write_timestamps_ns.get(mid, 0) for mid in MOTOR_IDS
                ]
                tc_msg.per_axis_tx_done_monotonic_ns = [
                    cmd_result.per_axis_write_completion_timestamps_ns.get(mid, 0) for mid in MOTOR_IDS
                ]
                tc_msg.target_positions_meters = [steps_to_meters(s) for s in dispatched_targets]
                tc_msg.target_steps = list(dispatched_targets)
                tc_msg.uart_cycle_sequence = current_uart_seq
                self.timed_command_pub.publish(tc_msg)

            t_pub_end_ns = time.perf_counter_ns()
            self._ros_publish_ms = (t_pub_end_ns - t_pub_start_ns) / 1e6

            self._worker_total_ms = (t_pub_end_ns - t_entry_ns) / 1e6

        except (SafetyViolation, J2F0SafetyViolation, J2F0Stop, ReadOnlyModeError) as exc:
            self.runtime_error = exc
            self.get_logger().fatal(f"FATAL safety violation in UART worker write: {exc}")
            raise
        except Exception as exc:
            t_err_ns = time.perf_counter_ns()
            self._consecutive_comm_failures += 1
            err_record = {
                "timestamp_ns": t_err_ns,
                "error": str(exc),
                "target_steps": list(target_env.targets_steps),
                "consecutive_count": self._consecutive_comm_failures,
                "duration_before_err_ms": (t_err_ns - t_write_start_ns) / 1e6,
            }
            self._timeout_events.append(err_record)
            self.get_logger().warn(
                f"UART transient communication error (count={self._consecutive_comm_failures}): {exc}"
            )
            # Delivery Uncertain State Machine:
            # 1. Hardware already paused via pause_all() in six_motor_controller exception handler.
            # 2. Attempt to read fresh 6-axis state snapshot to establish true ground-truth anchor.
            try:
                if self.controller.is_connected:
                    recovery_snap = self.controller.read_state_snapshot()
                    # Validate feedback completeness & faults
                    if (
                        recovery_snap is not None
                        and all(f == 0 for f in recovery_snap.fault_codes)
                        and all(200 <= p <= 1800 for p in recovery_snap.positions_steps)
                    ):
                        self._last_state_snapshot = recovery_snap
                        # Re-anchor command anchor to true physical positions
                        self._last_commanded_steps = list(recovery_snap.positions_steps)
                        self._last_write_stamp_ns = time.perf_counter_ns()
                        self.get_logger().info(
                            f"Delivery-uncertain resync SUCCESS: Re-anchored to physical positions {self._last_commanded_steps}."
                        )
            except Exception as resync_exc:
                self.get_logger().warn(f"Delivery-uncertain snapshot read failed: {resync_exc}")

            if self._consecutive_comm_failures >= 3:
                self.runtime_error = DriverError(
                    f"STOP: {self._consecutive_comm_failures} consecutive UART communication failures: {exc}"
                )
                self.get_logger().fatal(f"FATAL: 3 consecutive UART communication failures. Halting.")
                raise self.runtime_error

    def _background_poll_and_publish(self) -> None:
        """Low-rate background telemetry (active only during idle or READ_ONLY mode)."""
        if not self.controller.is_connected:
            return

        self._background_poll_count += 1
        now_ns = time.perf_counter_ns()
        # If an active target command stream is ongoing (< 500ms since last target arrival/write dispatch),
        # skip standalone poll to prevent bus contention.
        has_active_stream = (
            (self._last_target_rx_time_ns > 0 and (now_ns - self._last_target_rx_time_ns) < 500_000_000)
            or (self._last_write_stamp_ns > 0 and (now_ns - self._last_write_stamp_ns) < 500_000_000)
        )
        if has_active_stream and self.hardware_mode != HardwareMode.READ_ONLY:
            self._background_poll_during_active_count += 1
            return

        t_cb_start_ns = time.perf_counter_ns()
        try:
            t_scan_start_ns = time.perf_counter_ns()
            if self.read_mode == "position_only":
                pos_snap = self.controller.read_positions_snapshot()
                self._last_positions_snapshot = pos_snap
                t_scan_end_ns = time.perf_counter_ns()
                t_pub_start_ns = time.perf_counter_ns()
                msg = JointState()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.name = self.joint_names
                msg.position = pos_snap.positions_meters
                self.joint_state_pub.publish(msg)
                if self.publish_raw_diagnostics:
                    self._publish_array(self.raw_steps_pub, pos_snap.positions_steps)
                t_pub_end_ns = time.perf_counter_ns()
            else:
                snap = self.controller.read_state_snapshot()
                self._last_state_snapshot = snap
                t_scan_end_ns = time.perf_counter_ns()
                t_pub_start_ns = time.perf_counter_ns()
                self._publish_snapshot_telemetry(snap)
                t_pub_end_ns = time.perf_counter_ns()

            t_cb_end_ns = time.perf_counter_ns()
            if self.benchmark_stamp_pub is not None:
                bench_msg = Int64MultiArray()
                bench_msg.data = [
                    self._benchmark_seq_id,
                    t_cb_start_ns,
                    t_scan_start_ns,
                    t_scan_end_ns,
                    t_pub_start_ns,
                    t_pub_end_ns,
                    t_cb_end_ns,
                ]
                self.benchmark_stamp_pub.publish(bench_msg)
                self.benchmark_publish_timestamps_ns.append(t_pub_start_ns)
                self._benchmark_seq_id += 1

        except Exception as exc:
            self.get_logger().warn(f"Error in background poll: {exc}", throttle_duration_sec=2.0)

    def _poll_and_publish(self) -> None:
        """Alias for backward compatibility with existing tests and audit tools."""
        self._background_poll_and_publish()

    def _publish_snapshot_telemetry(self, snapshot: SixAxisStateSnapshot, uart_seq: int | None = None) -> None:
        """Publish standard JointState and raw diagnostic arrays from a snapshot."""
        # Standard JointState (ROS header.stamp retains standard ROS node clock semantics)
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = snapshot.positions_meters
        msg.velocity = []
        msg.effort = []
        self.joint_state_pub.publish(msg)

        # J5-C1 Timed Telemetry Feedback (Host CLOCK_MONOTONIC Timing)
        if self.timed_feedback_pub is not None:
            tf_msg = TimedJointFeedback()
            tf_msg.header.stamp = msg.header.stamp
            tf_msg.snapshot_start_monotonic_ns = snapshot.timestamp_start_ns
            tf_msg.snapshot_end_monotonic_ns = snapshot.timestamp_end_ns
            tf_msg.per_axis_rx_done_monotonic_ns = [
                snapshot.per_axis_end_timestamps_ns.get(mid, 0) for mid in MOTOR_IDS
            ]
            tf_msg.positions_meters = list(snapshot.positions_meters)
            tf_msg.raw_steps = list(snapshot.positions_steps)
            tf_msg.currents_ma = list(snapshot.currents_mA)
            tf_msg.forces_g_raw = list(snapshot.forces_g_raw)
            tf_msg.temperatures_c = list(snapshot.temperatures_C)
            tf_msg.fault_codes = list(snapshot.fault_codes)
            if uart_seq is not None:
                tf_msg.uart_cycle_sequence = uart_seq
            else:
                tf_msg.uart_cycle_sequence = self._uart_cycle_seq
                self._uart_cycle_seq += 1
            self.timed_feedback_pub.publish(tf_msg)

        # Raw Telemetry
        if self.publish_raw_diagnostics:
            self._publish_array(self.raw_steps_pub, snapshot.positions_steps)
            self._publish_array(self.raw_currents_pub, snapshot.currents_mA)
            self._publish_array(self.raw_forces_pub, snapshot.forces_g_raw)
            self._publish_array(self.raw_temps_pub, snapshot.temperatures_C)
            self._publish_array(self.raw_faults_pub, snapshot.fault_codes)

    def _publish_driver_diagnostics(self) -> None:
        """Publish scheduling and deadline metrics."""
        diag_msg = Float64MultiArray()
        # [target_in_count, overwrite_count, consumed_count, deadline_miss_count, target_age_ms, write_duration_ms]
        diag_msg.data = [
            float(self._target_in_count),
            float(self._target_overwrite_count),
            float(self._target_consumed_count),
            float(self._deadline_miss_count),
            float(self._last_target_age_ms),
            float(self._last_write_duration_ms),
        ]
        self.driver_diagnostics_pub.publish(diag_msg)

    def _publish_array(self, pub, data: list[int]) -> None:
        arr_msg = Int32MultiArray()
        arr_msg.data = data
        pub.publish(arr_msg)

    def _handle_command_single_step(self, msg: Int32MultiArray) -> None:
        """Handle isolated single-axis bring-up command: data = [motor_id, target_step]."""
        if self.hardware_mode == HardwareMode.READ_ONLY:
            self.get_logger().error("Single-axis command rejected: pus6_motor_driver is in READ_ONLY mode.")
            return

        if len(msg.data) != 2:
            self.get_logger().error(
                f"Single-axis command requires [motor_id, target_step], got {len(msg.data)} elements."
            )
            return

        motor_id = int(msg.data[0])
        target_step = int(msg.data[1])

        try:
            self.controller.command_single_axis_with_metrics(motor_id, target_step)
            self.get_logger().info(
                f"Executed isolated single-axis move: ID{motor_id} -> {target_step} step"
            )
        except Exception as exc:
            self.get_logger().error(f"Single-axis move error on ID{motor_id}: {exc}")

    def _handle_pause_all(
        self,
        request: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:
        if self.hardware_mode == HardwareMode.READ_ONLY:
            response.success = False
            response.message = "Rejected: Pause write is forbidden in READ_ONLY mode."
            self.get_logger().warn(response.message)
            return response

        try:
            res = self.controller.pause_all()
            all_ok = all(res.values())
            response.success = all_ok
            response.message = f"Pause all results: {res}"
            self.get_logger().warn(f"Pause all triggered: {res}")
        except Exception as exc:
            response.success = False
            response.message = f"Pause all exception: {exc}"
            self.get_logger().error(f"Pause all failed: {exc}")
        return response

    def _handle_configure_motors(
        self,
        request: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:
        if self.hardware_mode == HardwareMode.READ_ONLY:
            response.success = False
            response.message = "Rejected: Motor configure is forbidden in READ_ONLY mode."
            self.get_logger().warn(response.message)
            return response

        try:
            self.controller.configure_all()
            response.success = True
            response.message = "Configured mode=2, speed=50 on all 6 motors successfully."
            self.get_logger().info("Configured all motors successfully via service.")
        except Exception as exc:
            response.success = False
            response.message = f"Configuration failed: {exc}"
            self.get_logger().error(f"Configuration failed: {exc}")
        return response

    def destroy_node(self) -> None:
        """Safe shutdown: execute best-effort pause_all ONLY if active real hardware is connected."""
        if self.controller.is_connected:
            if self.hardware_mode == HardwareMode.ACTIVE:
                try:
                    self.get_logger().info("Active node shutting down: executing best-effort hardware pause_all.")
                    self.controller.pause_all()
                except Exception as exc:
                    self.get_logger().error(f"Error during shutdown pause_all: {exc}")
            self.controller.close()
        super().destroy_node()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = Pus6MotorDriverNode()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
