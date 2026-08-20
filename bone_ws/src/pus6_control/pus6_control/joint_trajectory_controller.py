"""Stateful jerk-limited online reference shaper and joint controller for 6PUS platform.

Implements a continuous 3rd-order state filter with triple pole (s + omega_n)^3
and saturation bounds on velocity, acceleration, and jerk.
"""

from __future__ import annotations

import math
from typing import Sequence

try:
    from pus6_kinematics.geometry_config import load_geometry_config
except ImportError:
    from pathlib import Path
    import sys
    _kinematics_pkg = Path(__file__).resolve().parents[2] / "pus6_kinematics"
    if str(_kinematics_pkg) not in sys.path:
        sys.path.insert(0, str(_kinematics_pkg))
    from pus6_kinematics.geometry_config import load_geometry_config

_GEOMETRY_CONFIG = load_geometry_config()

try:
    from .controller_interface import ControlError, ControllerInterface
except ImportError:
    from controller_interface import ControlError, ControllerInterface


def quintic_progress(tau: float) -> float:
    """Compute normalized S-curve quintic polynomial progress s(tau) in [0, 1].

    Formula: s(tau) = 10*tau^3 - 15*tau^4 + 6*tau^5
    Note: Intended for single offline trajectory segments with zero boundary velocities.
    """
    if tau <= 0.0:
        return 0.0
    if tau >= 1.0:
        return 1.0
    return 10.0 * (tau**3) - 15.0 * (tau**4) + 6.0 * (tau**5)


def compute_synchronized_trajectory_point(
    start_positions: Sequence[float | int],
    target_positions: Sequence[float | int],
    t: float,
    total_duration: float,
) -> list[float]:
    """Compute synchronized 6-axis joint positions at time t using normalized progress s(t)."""
    if len(start_positions) != len(target_positions):
        raise ControlError(
            f"Start ({len(start_positions)}) and target ({len(target_positions)}) length mismatch"
        )
    tau = t / max(0.001, total_duration)
    s = quintic_progress(tau)
    return [
        float(q0) + s * float(q1 - q0)
        for q0, q1 in zip(start_positions, target_positions)
    ]


def compute_synchronized_step_point(
    start_steps: Sequence[int],
    target_steps: Sequence[int],
    t: float,
    total_duration: float,
) -> list[int]:
    """Compute integer step coordinates at time t using normalized quintic progress."""
    pt_float = compute_synchronized_trajectory_point(start_steps, target_steps, t, total_duration)
    return [int(round(p)) for p in pt_float]


class OuterLoopControllerHook:
    """Extension hook for outer-loop controllers (PID, ADRC, SAC RL, dual-IMU observer)."""

    def compute_correction(
        self,
        nominal_joint_cmd: Sequence[float],
        measured_joint_state: Sequence[float],
        dt: float,
    ) -> list[float]:
        """Compute additive joint correction vector in SI meters.

        Default base implementation is pass-through (zero correction).
        """
        return [0.0] * len(nominal_joint_cmd)


class StatefulJerkLimitedShaper:
    """Stateful 3rd-order online reference shaper with jerk, acceleration, and velocity saturation.

    Transfer function corresponds to triple real pole at s = -omega_n:
        H(s) = omega_n^3 / (s + omega_n)^3 = omega_n^3 / (s^3 + 3*omega_n*s^2 + 3*omega_n^2*s + omega_n^3)

    Continuous differential equation:
        q_triple_dot = omega_n^3 * (q_target - q) - 3*omega_n^2 * q_dot - 3*omega_n * q_ddot

    Maintains persistent state [q, q_dot, q_ddot] across control cycles.
    When a new target arrives, q_dot and q_ddot are NOT reset, guaranteeing C^2 continuity.
    """

    def __init__(
        self,
        num_joints: int = 6,
        omega_n: float = 20.0,
        max_speed_m_per_sec: float = 0.005,      # 200 step/s = 5.0 mm/s
        max_accel_m_per_sec2: float = 0.050,     # 2000 step/s^2 = 50.0 mm/s^2
        max_jerk_m_per_sec3: float = 0.500,      # 20000 step/s^3 = 500.0 mm/s^3
        initial_positions: Sequence[float] | None = None,
    ) -> None:
        self.num_joints = num_joints
        self.omega_n = omega_n
        self.max_speed = max_speed_m_per_sec
        self.max_accel = max_accel_m_per_sec2
        self.max_jerk = max_jerk_m_per_sec3

        # Persistent continuous state: [q, dq, ddq]
        if initial_positions is not None:
            if len(initial_positions) != num_joints:
                raise ValueError(f"Initial positions length {len(initial_positions)} != {num_joints}")
            self.q = [float(p) for p in initial_positions]
        else:
            self.q = [0.0] * num_joints
        self.dq = [0.0] * num_joints
        self.ddq = [0.0] * num_joints
        self.dddq = [0.0] * num_joints

    def reset_state(
        self,
        positions: Sequence[float],
        velocities: Sequence[float] | None = None,
        accelerations: Sequence[float] | None = None,
    ) -> None:
        """Explicitly re-anchor state (e.g. during system homing or hard initialization)."""
        if len(positions) != self.num_joints:
            raise ValueError(f"Positions length {len(positions)} != {self.num_joints}")
        self.q = [float(p) for p in positions]
        self.dq = [float(v) for v in velocities] if velocities else [0.0] * self.num_joints
        self.ddq = [float(a) for a in accelerations] if accelerations else [0.0] * self.num_joints
        self.dddq = [0.0] * self.num_joints

    def update(
        self,
        target_positions: Sequence[float],
        dt: float,
    ) -> list[float]:
        """Advance the 3rd-order state shaper by dt towards target_positions.

        Does NOT reset velocity or acceleration on target updates.
        """
        if len(target_positions) != self.num_joints:
            raise ControlError(
                f"Target length {len(target_positions)} != {self.num_joints}"
            )
        if dt <= 0.0:
            return list(self.q)

        # Numerical integration sub-steps for numerical stiffness at high omega_n
        substeps = max(1, int(math.ceil(dt / 0.002)))  # Substep <= 2ms
        sub_dt = dt / float(substeps)

        w = self.omega_n
        w2 = w * w
        w3 = w2 * w

        for _ in range(substeps):
            for i in range(self.num_joints):
                tgt = float(target_positions[i])
                pos = self.q[i]
                vel = self.dq[i]
                acc = self.ddq[i]

                # Raw 3rd order characteristic differential equation
                jerk_raw = w3 * (tgt - pos) - 3.0 * w2 * vel - 3.0 * w * acc

                # Nonlinear jerk saturation
                jerk = max(-self.max_jerk, min(self.max_jerk, jerk_raw))
                self.dddq[i] = jerk

                # Acceleration integration & saturation
                acc_next = max(-self.max_accel, min(self.max_accel, acc + jerk * sub_dt))
                self.ddq[i] = acc_next

                # Velocity integration & saturation
                vel_next = max(-self.max_speed, min(self.max_speed, vel + acc_next * sub_dt))
                self.dq[i] = vel_next

                # Position integration
                pos_next = pos + vel_next * sub_dt
                self.q[i] = pos_next

        return list(self.q)


class JointTrajectoryController(ControllerInterface):
    """Interpolates setpoints and rate-limits commands in joint space using a stateful 3rd-order shaper."""

    def __init__(
        self,
        max_speed_m_per_sec: float = 0.005,      # 200 step/s = 5.0 mm/s
        max_accel_m_per_sec2: float = 0.050,     # 2000 step/s^2 = 50.0 mm/s^2
        max_jerk_m_per_sec3: float = 0.500,      # 20000 step/s^3 = 500.0 mm/s^3
        omega_n: float = 20.0,
        max_step_delta_m: float | None = None,   # Legacy parameter alias
        steps_per_meter: float | None = None,
        num_joints: int = 6,
        outer_loop_hook: OuterLoopControllerHook | None = None,
    ) -> None:
        self.steps_per_meter = _GEOMETRY_CONFIG.steps_per_meter if steps_per_meter is None else float(steps_per_meter)
        self.num_joints = num_joints
        self.outer_loop_hook = outer_loop_hook or OuterLoopControllerHook()

        self.shaper = StatefulJerkLimitedShaper(
            num_joints=num_joints,
            omega_n=omega_n,
            max_speed_m_per_sec=max_speed_m_per_sec,
            max_accel_m_per_sec2=max_accel_m_per_sec2,
            max_jerk_m_per_sec3=max_jerk_m_per_sec3,
        )
        self._initialized = False

    @property
    def max_speed(self) -> float:
        return self.shaper.max_speed

    @property
    def max_accel(self) -> float:
        return self.shaper.max_accel

    @property
    def max_jerk(self) -> float:
        return self.shaper.max_jerk

    def reset(self, initial_positions: Sequence[float] | None = None) -> None:
        if initial_positions is not None:
            self.shaper.reset_state(initial_positions)
            self._initialized = True
        else:
            self.shaper.reset_state([0.0] * self.num_joints)
            self._initialized = False

    def update(
        self,
        current_state: Sequence[float],
        target_state: Sequence[float],
        dt: float,
    ) -> list[float]:
        """Compute smooth, jerk-limited joint commands towards target with outer-loop correction.

        Preserves continuous state across calls and handles dynamic target variations.
        """
        if len(current_state) != self.num_joints or len(target_state) != self.num_joints:
            raise ControlError(
                f"State and target must have length {self.num_joints}, "
                f"got {len(current_state)} and {len(target_state)}"
            )

        if not self._initialized:
            self.shaper.reset_state(current_state)
            self._initialized = True

        # Shape reference trajectory (C^2 continuous)
        shaped_cmd = self.shaper.update(target_state, dt)

        # Apply outer-loop correction hook
        correction = self.outer_loop_hook.compute_correction(shaped_cmd, current_state, dt)
        final_cmd = [c + corr for c, corr in zip(shaped_cmd, correction)]

        return final_cmd

    def get_motion_state(self) -> dict[str, list[float]]:
        """Return live continuous states [q, dq, ddq, dddq]."""
        return {
            "position": list(self.shaper.q),
            "velocity": list(self.shaper.dq),
            "acceleration": list(self.shaper.ddq),
            "jerk": list(self.shaper.dddq),
        }

    def meters_to_raw_steps(
        self, positions_m: Sequence[float], operating_home_step: int | Sequence[int] | None = None
    ) -> list[int]:
        """Convert relative SI positions to raw steps using YAML HOME by default."""
        reference = _GEOMETRY_CONFIG.operating_home_raw_steps if operating_home_step is None else operating_home_step
        refs = [reference] * self.num_joints if isinstance(reference, int) else list(reference)
        if len(refs) != len(positions_m):
            raise ControlError("operating home reference and position vectors must both have six elements")
        return [int(round(ref + p * self.steps_per_meter)) for p, ref in zip(positions_m, refs)]

    def raw_steps_to_meters(
        self, raw_steps: Sequence[int], operating_home_step: int | Sequence[int] | None = None
    ) -> list[float]:
        """Convert raw steps to relative SI positions using YAML HOME by default."""
        reference = _GEOMETRY_CONFIG.operating_home_raw_steps if operating_home_step is None else operating_home_step
        refs = [reference] * self.num_joints if isinstance(reference, int) else list(reference)
        if len(refs) != len(raw_steps):
            raise ControlError("operating home reference and raw step vectors must both have six elements")
        return [float(s - ref) / self.steps_per_meter for s, ref in zip(raw_steps, refs)]
