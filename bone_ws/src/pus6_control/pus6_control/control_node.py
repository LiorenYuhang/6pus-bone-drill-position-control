"""ROS 2 Node managing control algorithms and dispatching motor commands."""

from __future__ import annotations

import rclpy
import json
import os
import time
from pathlib import Path
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, Int32MultiArray, Int64MultiArray
from std_srvs.srv import Trigger

try:
    from pus6_kinematics.geometry_config import load_geometry_config
except ImportError:
    from pathlib import Path
    import sys
    _kinematics_pkg = Path(__file__).resolve().parents[2] / "pus6_kinematics"
    if str(_kinematics_pkg) not in sys.path:
        sys.path.insert(0, str(_kinematics_pkg))
    from pus6_kinematics.geometry_config import load_geometry_config

try:
    from .controller_interface import ControlError
    from .joint_trajectory_controller import JointTrajectoryController, OuterLoopControllerHook
except ImportError:
    from controller_interface import ControlError
    from joint_trajectory_controller import JointTrajectoryController, OuterLoopControllerHook


class Pus6ControlNode(Node):
    """ROS 2 Node managing continuous control algorithms and dispatching motor commands."""

    def __init__(self) -> None:
        super().__init__("pus6_control_node")

        self.declare_parameter("max_speed_m_per_sec", 0.005)      # 200 step/s = 5.0 mm/s = 0.005 m/s
        self.declare_parameter("max_accel_m_per_sec2", 0.050)     # 2000 step/s^2 = 50.0 mm/s^2
        self.declare_parameter("max_jerk_m_per_sec3", 0.500)      # 20000 step/s^3 = 500.0 mm/s^3
        self.declare_parameter("omega_n", 20.0)                   # Filter bandwidth (rad/s)
        self.declare_parameter("control_rate_hz", 50.0)           # Nominal continuous control rate: 50 Hz
        geometry_cfg = load_geometry_config()
        self.declare_parameter("geometry_config_path", str(geometry_cfg.path))
        self.declare_parameter("maintenance_mode", False)

        max_spd = self.get_parameter("max_speed_m_per_sec").get_parameter_value().double_value
        max_acc = self.get_parameter("max_accel_m_per_sec2").get_parameter_value().double_value
        max_jrk = self.get_parameter("max_jerk_m_per_sec3").get_parameter_value().double_value
        omega_n = self.get_parameter("omega_n").get_parameter_value().double_value
        rate = self.get_parameter("control_rate_hz").get_parameter_value().double_value
        config_path = self.get_parameter("geometry_config_path").get_parameter_value().string_value
        geometry_cfg = load_geometry_config(config_path)
        spm = geometry_cfg.steps_per_meter
        self.operating_home_steps = geometry_cfg.operating_home_raw_steps
        self.storage_park_steps = geometry_cfg.storage_park_raw_steps
        self.home_step = self.operating_home_steps[0]
        self.maintenance_mode = self.get_parameter("maintenance_mode").get_parameter_value().bool_value

        self.controller = JointTrajectoryController(
            max_speed_m_per_sec=max_spd,
            max_accel_m_per_sec2=max_acc,
            max_jerk_m_per_sec3=max_jrk,
            omega_n=omega_n,
            steps_per_meter=spm,
        )

        self._current_positions: list[float] | None = None
        self._target_positions: list[float] | None = [0.0] * 6  # Start at OPERATING_HOME (relative = 0)
        self._last_time = self.get_clock().now()
        self._is_homed = True
        self._seq = 0

        from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

        # Dedicated QoS for continuous control commands: Latest-Value-Wins (Keep last 1, Best Effort)
        control_cmd_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        # Publishers
        self.cmd_pub = self.create_publisher(
            Float64MultiArray, "/pus6/command_joint_positions", control_cmd_qos
        )
        self.cmd_steps_pub = self.create_publisher(
            Int32MultiArray, "/pus6/command_joint_steps", control_cmd_qos
        )
        self.diag_pub = self.create_publisher(
            Float64MultiArray, "/pus6/control_diagnostics", 10
        )

        # Subscriptions
        self.joint_state_sub = self.create_subscription(
            JointState, "/pus6/joint_states", self._handle_joint_states, 10
        )
        self.desired_deltas_sub = self.create_subscription(
            Float64MultiArray, "/pus6/desired_joint_deltas", self._handle_desired_deltas, 10
        )
        # Communication recovery resets existing shaper state; its algorithm,
        # gains, rates and existing command/feedback QoS remain unchanged.
        self._last_communication_anchor_ns = 0
        self._last_communication_anchor_payload = None
        self._static_qualification = os.environ.get('PUS6_J14_RECOVERY_STAGE') == 'static_resume'
        self._static_recovery_reference = None
        self._pending_static_anchor_ack = None
        self._static_sync_record = None
        self._communication_anchor_ack = self.create_publisher(
            Int64MultiArray, '/pus6/communication_reanchor_ack', 10)
        self._communication_anchor_sub = self.create_subscription(
            Int64MultiArray, '/pus6/communication_reanchor', self._handle_communication_reanchor, 10)

        # Services
        self.startup_srv = self.create_service(
            Trigger, "/pus6/startup_home", self._handle_startup_home
        )
        self.park_srv = self.create_service(
            Trigger, "/pus6/controlled_park", self._handle_controlled_park
        )

        # Control loop timer
        interval = 1.0 / max(1.0, rate)
        self.timer = self.create_timer(interval, self._control_loop)
        self.get_logger().info(
            f"Pus6ControlNode initialized (rate={rate:.1f}Hz, max_speed={max_spd*1e3:.1f}mm/s, "
            f"max_accel={max_acc*1e3:.1f}mm/s^2, omega_n={omega_n:.1f}, home_step={self.home_step})."
        )

    def _handle_joint_states(self, msg: JointState) -> None:
        if len(msg.position) == 6:
            # Convert absolute meters to relative displacement from OPERATING_HOME
            self._current_positions = [
                float(p) - float(home) / self.controller.steps_per_meter
                for p, home in zip(msg.position, self.operating_home_steps)
            ]

    def _handle_desired_deltas(self, msg: Float64MultiArray) -> None:
        if self._static_recovery_reference is not None:
            return  # A1.5 holds the synchronized reference; old Cartesian debt is not pursued.
        if len(msg.data) == 6:
            self._target_positions = [float(v) for v in msg.data]

    def _handle_communication_reanchor(self, msg: Int64MultiArray) -> None:
        if len(msg.data) != 7 or msg.data[0] <= 0:
            return
        epoch, *positions = list(msg.data)
        if any(p < 0 or p > 2000 for p in positions) or epoch < self._last_communication_anchor_ns:
            return
        if epoch == self._last_communication_anchor_ns and list(msg.data) != self._last_communication_anchor_payload:
            return
        if epoch > self._last_communication_anchor_ns:
            if self._static_qualification:
                self._static_sync_record = dict(epoch_ns=epoch,
                    old_target=self._target_positions, old_motion_state=self.controller.get_motion_state(),
                    fresh_raw_steps=list(positions), sync_start_ns=time.monotonic_ns())
            measured = [(float(p)-float(home))/self.controller.steps_per_meter
                        for p,home in zip(positions,self.operating_home_steps)]
            self.controller.reset(initial_positions=measured)
            self._current_positions = list(measured)
            self._target_positions = list(measured)
            self._last_time = self.get_clock().now()
            self._last_communication_anchor_ns = epoch
            self._last_communication_anchor_payload = list(msg.data)
            if self._static_qualification:
                self._static_recovery_reference = list(measured)
                self._pending_static_anchor_ack = [epoch, *positions]
                self._static_sync_record.update(reanchor_target_meters=list(measured),
                    reset_motion_state=self.controller.get_motion_state(), sync_done_ns=time.monotonic_ns(),
                    reset_api='JointTrajectoryController.reset -> StatefulJerkLimitedShaper.reset_state')
        if self._static_qualification:
            return  # ACK follows the next ordinary Control/Shaper cycle, not a raw-command shortcut.
        ack = Int64MultiArray()
        ack.data = [epoch, *positions]
        self._communication_anchor_ack.publish(ack)

    def _handle_startup_home(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        self._target_positions = [0.0] * 6
        self.controller.reset(initial_positions=[0.0] * 6)
        self._is_homed = True
        response.success = True
        response.message = "Set target to OPERATING_HOME [0.0]*6 m"
        self.get_logger().info("Service called: Startup to OPERATING_HOME")
        return response

    def _handle_controlled_park(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        if not self.maintenance_mode:
            response.success = False
            response.message = "Rejected: STORAGE_PARK is maintenance-only; set maintenance_mode=true or use the J3 maintenance runner."
            self.get_logger().warn(response.message)
            return response
        park_m = [
            (float(park) - float(home)) / self.controller.steps_per_meter
            for park, home in zip(self.storage_park_steps, self.operating_home_steps)
        ]
        self._target_positions = park_m
        self._is_homed = False
        response.success = True
        response.message = f"Set target to STORAGE_PARK raw={list(self.storage_park_steps)}"
        self.get_logger().info("Service called: Controlled park to STORAGE_PARK")
        return response

    def _control_loop(self) -> None:
        if self._current_positions is None or self._target_positions is None:
            return

        now = self.get_clock().now()
        dt = (now - self._last_time).nanoseconds / 1e9
        self._last_time = now

        try:
            cmd = self.controller.update(
                current_state=self._current_positions,
                target_state=self._target_positions,
                dt=dt,
            )

            # Publish SI meters command
            out_msg = Float64MultiArray()
            out_msg.data = [float(v) for v in cmd]
            self.cmd_pub.publish(out_msg)

            # Publish raw steps command (mapped to home_step)
            raw_steps = self.controller.meters_to_raw_steps(cmd, self.operating_home_steps)
            steps_msg = Int32MultiArray()
            steps_msg.data = raw_steps
            self.cmd_steps_pub.publish(steps_msg)

            if self._pending_static_anchor_ack is not None:
                ack = Int64MultiArray(data=[*self._pending_static_anchor_ack, *raw_steps])
                self._static_sync_record.update(first_shaped_raw_target=list(raw_steps),
                    first_control_command_ns=time.monotonic_ns(),
                    first_command_motion_state=self.controller.get_motion_state(), shaper_sync_result=True)
                directory = os.environ.get('PUS6_J14_RECOVERY_AUDIT_DIR')
                if directory:
                    with (Path(directory)/'shaper_reanchor_audit.jsonl').open('a') as stream:
                        stream.write(json.dumps(self._static_sync_record)+'\n')
                print('J14_SHAPER_SYNC '+json.dumps(self._static_sync_record), flush=True)
                self._communication_anchor_ack.publish(ack)
                self._pending_static_anchor_ack = None

            # Publish tracking diagnostics [max_error_mm, mean_error_mm, dt, seq]
            errors_mm = [abs(c - cur) * 1e3 for c, cur in zip(cmd, self._current_positions)]
            diag_msg = Float64MultiArray()
            diag_msg.data = [
                max(errors_mm) if errors_mm else 0.0,
                sum(errors_mm) / max(1, len(errors_mm)),
                dt,
                float(self._seq),
            ]
            self.diag_pub.publish(diag_msg)
            self._seq += 1

        except ControlError as exc:
            self.get_logger().error(f"Control loop error: {exc}")


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = Pus6ControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
