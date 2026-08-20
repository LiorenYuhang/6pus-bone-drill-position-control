"""Analytical formulation and geometric model for 6PUS parallel manipulator.

Authoritative geometry verified from hardware/调试工具/Kinematics.zip (params.m, inver.m).
"""

from __future__ import annotations

import math
from pathlib import Path
from typing import Sequence

try:
    from .kinematics_interface import (
        ForwardKinematicsResult,
        KinematicsError,
        KinematicsInterface,
        SingularityError,
        WorkspaceLimitError,
    )
except ImportError:
    from kinematics_interface import (
        ForwardKinematicsResult,
        KinematicsError,
        KinematicsInterface,
        SingularityError,
        WorkspaceLimitError,
    )

try:
    from .geometry_config import GeometryConfig, load_geometry_config
except ImportError:
    from geometry_config import GeometryConfig, load_geometry_config


# Compatibility aliases are derived from the authoritative YAML.
_DEFAULT_GEOMETRY_CONFIG = load_geometry_config()
DEFAULT_STEPS_PER_METER: float = _DEFAULT_GEOMETRY_CONFIG.steps_per_meter

# Authoritative geometric constants from Kinematics/params.m
DEFAULT_BASE_RADIUS_M: float = _DEFAULT_GEOMETRY_CONFIG.base_radius_m        # c = 40 mm
DEFAULT_PLATFORM_RADIUS_M: float = _DEFAULT_GEOMETRY_CONFIG.platform_radius_m    # d = 27.5 mm
DEFAULT_STRUT_LENGTH_M: float = _DEFAULT_GEOMETRY_CONFIG.strut_length_m       # e = 84 mm
DEFAULT_LAMBDA_RAD: float = _DEFAULT_GEOMETRY_CONFIG.lambda_rad   # λ = 30 deg (base pair separation)
DEFAULT_MU_RAD: float = _DEFAULT_GEOMETRY_CONFIG.mu_rad       # μ = 60 deg (platform pair separation)

# Nominal neutral height (m) where slider stroke q = 50 mm (mid-stroke):
# h_zero = sqrt(c^2 + d^2 - 2*c*d*cos(pi/3 - (lambda+mu)/2)) = 15.2057 mm
# sqrt_term = sqrt(e^2 - h_zero^2) = 82.6123 mm
# z_nom = 50.0 + 82.6123 = 132.6123 mm = 0.132612268 m
DEFAULT_NOMINAL_HEIGHT_M: float = _DEFAULT_GEOMETRY_CONFIG.nominal_height_m


class Analytical6PusKinematics(KinematicsInterface):
    """Analytical kinematics model for the authoritative 6PUS parallel platform."""

    def __init__(
        self,
        base_radius_m: float | None = None,
        platform_radius_m: float | None = None,
        strut_length_m: float | None = None,
        lambda_rad: float | None = None,
        mu_rad: float | None = None,
        nominal_height_m: float | None = None,
        min_stroke_m: float | None = None,
        max_stroke_m: float | None = None,
        steps_per_meter: float | None = None,
        config_path: str | Path | None = None,
    ) -> None:
        # The default model is now loaded from YAML. Explicit constructor
        # arguments remain supported for legacy tests and parameter overrides.
        self.geometry_config: GeometryConfig = load_geometry_config(config_path)
        cfg = self.geometry_config
        self.base_radius = cfg.base_radius_m if base_radius_m is None else base_radius_m
        self.platform_radius = cfg.platform_radius_m if platform_radius_m is None else platform_radius_m
        self.strut_length = cfg.strut_length_m if strut_length_m is None else strut_length_m
        self.lambda_rad = cfg.lambda_rad if lambda_rad is None else lambda_rad
        self.mu_rad = cfg.mu_rad if mu_rad is None else mu_rad
        self.nominal_height = cfg.nominal_height_m if nominal_height_m is None else nominal_height_m
        self.min_stroke_m = cfg.stroke_min_m if min_stroke_m is None else min_stroke_m
        self.max_stroke_m = cfg.stroke_max_m if max_stroke_m is None else max_stroke_m
        self.steps_per_meter = cfg.steps_per_meter if steps_per_meter is None else steps_per_meter
        self.workspace_height_min_m = cfg.workspace_height_min_m
        self.workspace_height_max_m = cfg.workspace_height_max_m

        c = self.base_radius
        d = self.platform_radius
        lam = self.lambda_rad
        mu = self.mu_rad

        # Use explicit YAML anchors for the default model. If a caller
        # overrides either radius/angle, retain legacy generated-anchor behavior.
        if base_radius_m is None and lambda_rad is None:
            self.base_anchors = [list(anchor) for anchor in cfg.base_anchors_m]
        else:
            self.base_anchors = [
                [-c * math.sin(lam / 2.0), c * math.cos(lam / 2.0), 0.0],
                [-c * math.cos(math.pi / 6.0 - lam / 2.0), -c * math.sin(math.pi / 6.0 - lam / 2.0), 0.0],
                [-c * math.cos(math.pi / 6.0 + lam / 2.0), -c * math.sin(math.pi / 6.0 + lam / 2.0), 0.0],
                [c * math.cos(math.pi / 6.0 + lam / 2.0), -c * math.sin(math.pi / 6.0 + lam / 2.0), 0.0],
                [c * math.cos(math.pi / 6.0 - lam / 2.0), -c * math.sin(math.pi / 6.0 - lam / 2.0), 0.0],
                [c * math.sin(lam / 2.0), c * math.cos(lam / 2.0), 0.0],
            ]

        if platform_radius_m is None and mu_rad is None:
            self.platform_anchors_body = [list(anchor) for anchor in cfg.platform_anchors_body_m]
        else:
            self.platform_anchors_body = [
                [-d * math.cos(math.pi / 6.0 + mu / 2.0), d * math.sin(math.pi / 6.0 + mu / 2.0), 0.0],
                [-d * math.cos(math.pi / 6.0 - mu / 2.0), d * math.sin(math.pi / 6.0 - mu / 2.0), 0.0],
                [-d * math.sin(mu / 2.0), -d * math.cos(mu / 2.0), 0.0],
                [d * math.sin(mu / 2.0), -d * math.cos(mu / 2.0), 0.0],
                [d * math.cos(math.pi / 6.0 - mu / 2.0), d * math.sin(math.pi / 6.0 - mu / 2.0), 0.0],
                [d * math.cos(math.pi / 6.0 + mu / 2.0), d * math.sin(math.pi / 6.0 + mu / 2.0), 0.0],
            ]

    @staticmethod
    def _rotation_matrix_zyx(alpha: float, beta: float, gamma: float) -> list[list[float]]:
        """Compute R = Rz(gamma) * Ry(beta) * Rx(alpha) matching inver.m."""
        ca, sa = math.cos(alpha), math.sin(alpha)
        cb, sb = math.cos(beta), math.sin(beta)
        cg, sg = math.cos(gamma), math.sin(gamma)

        return [
            [cg * cb, cg * sb * sa - sg * ca, cg * sb * ca + sg * sa],
            [sg * cb, sg * sb * sa + cg * ca, sg * sb * ca - cg * sa],
            [-sb, cb * sa, cb * ca],
        ]

    def inverse_kinematics(self, pose: Sequence[float]) -> list[float]:
        """Compute 6PUS actuator slider positions [q1..q6] from platform pose.

        Args:
            pose: [x, y, z, roll, pitch, yaw] (x, y, z in meters; roll=alpha, pitch=beta, yaw=gamma in radians).

        Returns:
            6-element list of slider positions [q1..q6] in SI meters.

        Raises:
            KinematicsError: If pose length is not 6.
            WorkspaceLimitError: If pose is unreachable or out of stroke.
        """
        if len(pose) != 6:
            raise KinematicsError(
                f"Pose vector must have length 6 [x, y, z, roll, pitch, yaw], got {len(pose)}"
            )

        x, y, z, alpha, beta, gamma = pose

        # Physical platform height workspace check
        if z < self.workspace_height_min_m or z > self.workspace_height_max_m:
            raise WorkspaceLimitError(
                f"Target z={z:.4f}m is outside reachable height range "
                f"[{self.workspace_height_min_m:.2f}, {self.workspace_height_max_m:.2f}]"
            )

        R = self._rotation_matrix_zyx(alpha, beta, gamma)
        slider_positions: list[float] = []

        for i in range(6):
            # Global platform anchor position BB_i = P + R * b_i
            b_i = self.platform_anchors_body[i]
            px = x + (R[0][0] * b_i[0] + R[0][1] * b_i[1] + R[0][2] * b_i[2])
            py = y + (R[1][0] * b_i[0] + R[1][1] * b_i[1] + R[1][2] * b_i[2])
            pz = z + (R[2][0] * b_i[0] + R[2][1] * b_i[1] + R[2][2] * b_i[2])

            a_i = self.base_anchors[i]
            dx = px - a_i[0]
            dy = py - a_i[1]

            radicand = self.strut_length**2 - dx**2 - dy**2
            if radicand < 0.0:
                if radicand > -1e-9:
                    radicand = 0.0
                else:
                    raise WorkspaceLimitError(
                        f"Limb {i+1} horizontal span {math.sqrt(dx**2 + dy**2):.4f}m exceeds strut length {self.strut_length:.4f}m"
                    )

            qi = pz - math.sqrt(radicand)
            slider_positions.append(qi)

        return slider_positions

    def differential_inverse_kinematics(
        self,
        desired_pose: Sequence[float],
        reference_pose: Sequence[float],
    ) -> list[float]:
        """Compute differential actuator displacement Δq = IK(x_des) - IK(x_ref)."""
        q_des = self.inverse_kinematics(desired_pose)
        q_ref = self.inverse_kinematics(reference_pose)
        return [q_d - q_r for q_d, q_r in zip(q_des, q_ref)]

    def differential_inverse_kinematics_steps(
        self,
        desired_pose: Sequence[float],
        reference_pose: Sequence[float],
    ) -> list[int]:
        """Compute integer differential step displacement Δq_steps for hardware dispatch."""
        delta_m = self.differential_inverse_kinematics(desired_pose, reference_pose)
        return [int(round(dm * self.steps_per_meter)) for dm in delta_m]

    @staticmethod
    def _drot_dalpha(alpha: float, beta: float, gamma: float) -> list[list[float]]:
        """Partial derivative dR/d(alpha) for ZYX Euler angles."""
        ca, sa = math.cos(alpha), math.sin(alpha)
        cb, sb = math.cos(beta), math.sin(beta)
        cg, sg = math.cos(gamma), math.sin(gamma)
        return [
            [0.0, cg * sb * ca + sg * sa, -cg * sb * sa + sg * ca],
            [0.0, sg * sb * ca - cg * sa, -sg * sb * sa - cg * ca],
            [0.0, cb * ca, -cb * sa],
        ]

    @staticmethod
    def _drot_dbeta(alpha: float, beta: float, gamma: float) -> list[list[float]]:
        """Partial derivative dR/d(beta) for ZYX Euler angles."""
        ca, sa = math.cos(alpha), math.sin(alpha)
        cb, sb = math.cos(beta), math.sin(beta)
        cg, sg = math.cos(gamma), math.sin(gamma)
        return [
            [-cg * sb, cg * cb * sa, cg * cb * ca],
            [-sg * sb, sg * cb * sa, sg * cb * ca],
            [-cb, -sb * sa, -sb * ca],
        ]

    @staticmethod
    def _drot_dgamma(alpha: float, beta: float, gamma: float) -> list[list[float]]:
        """Partial derivative dR/d(gamma) for ZYX Euler angles."""
        ca, sa = math.cos(alpha), math.sin(alpha)
        cb, sb = math.cos(beta), math.sin(beta)
        cg, sg = math.cos(gamma), math.sin(gamma)
        return [
            [-sg * cb, -sg * sb * sa - cg * ca, -sg * sb * ca + cg * sa],
            [cg * cb, cg * sb * sa - sg * ca, cg * sb * ca + sg * sa],
            [0.0, 0.0, 0.0],
        ]

    def forward_kinematics_with_metrics(
        self,
        joint_positions: Sequence[float],
        initial_guess: Sequence[float] | None = None,
        max_iter: int = 30,
        tol: float = 1e-10,
    ) -> ForwardKinematicsResult:
        """Compute exact 6PUS platform pose from actuator lengths with convergence metrics.

        Solves the system of 6 nonlinear geometric strut-length constraints:
            f_i(p, R) = || p + R * b_i - s_i || - L = 0  (for i = 1..6)
        using Levenberg-Marquardt on SE(3) parameter space.

        Args:
            joint_positions: 6-element slider positions [q1..q6] in SI meters.
            initial_guess: Optional initial [x, y, z, roll, pitch, yaw] guess (default: OPERATING_HOME).
            max_iter: Maximum iterations before terminating.
            tol: L2 norm constraint residual convergence threshold (meters).

        Returns:
            ForwardKinematicsResult containing pose, SO(3) matrix, residual norm, and iterations.
        """
        if len(joint_positions) != 6:
            raise KinematicsError(
                f"Joint positions vector must have length 6, got {len(joint_positions)}"
            )

        import numpy as np

        q_target = np.asarray(joint_positions, dtype=np.float64)

        if initial_guess is not None:
            if len(initial_guess) != 6:
                raise KinematicsError(f"Initial guess must have length 6, got {len(initial_guess)}")
            x_k = np.asarray(initial_guess, dtype=np.float64).copy()
        else:
            # Default warm start: home position at nominal height
            avg_q = float(np.mean(q_target))
            h_zero_sq = (
                self.base_radius**2
                + self.platform_radius**2
                - 2.0
                * self.base_radius
                * self.platform_radius
                * math.cos(self.mu_rad - (self.lambda_rad + self.mu_rad) / 2.0)
            )
            z_nom = avg_q + math.sqrt(max(0.0, self.strut_length**2 - h_zero_sq))
            x_k = np.array([0.0, 0.0, z_nom, 0.0, 0.0, 0.0], dtype=np.float64)

        base_a = np.asarray(self.base_anchors, dtype=np.float64)
        plat_b = np.asarray(self.platform_anchors_body, dtype=np.float64)
        strut_L = float(self.strut_length)

        damping = 1e-6
        converged = False
        res_norm = 0.0
        final_iter = 0

        for it in range(max_iter):
            final_iter = it + 1
            x, y, z, alpha, beta, gamma = x_k

            R = np.asarray(self._rotation_matrix_zyx(alpha, beta, gamma), dtype=np.float64)
            dRa = np.asarray(self._drot_dalpha(alpha, beta, gamma), dtype=np.float64)
            dRb = np.asarray(self._drot_dbeta(alpha, beta, gamma), dtype=np.float64)
            dRg = np.asarray(self._drot_dgamma(alpha, beta, gamma), dtype=np.float64)

            p_origin = np.array([x, y, z], dtype=np.float64)
            f = np.zeros(6, dtype=np.float64)
            J = np.zeros((6, 6), dtype=np.float64)

            for i in range(6):
                b_i = plat_b[i]
                p_i = p_origin + R @ b_i
                s_i = np.array([base_a[i, 0], base_a[i, 1], q_target[i]], dtype=np.float64)
                diff = p_i - s_i
                dist = float(np.linalg.norm(diff))
                if dist < 1e-9:
                    raise SingularityError(f"Limb {i+1} degenerate configuration: length is near zero.")
                f[i] = dist - strut_L
                u_i = diff / dist

                J[i, 0:3] = u_i
                J[i, 3] = u_i @ (dRa @ b_i)
                J[i, 4] = u_i @ (dRb @ b_i)
                J[i, 5] = u_i @ (dRg @ b_i)

            res_norm = float(np.linalg.norm(f))
            if res_norm <= tol:
                converged = True
                break

            # Damped Gauss-Newton / Levenberg-Marquardt step
            H = J.T @ J + damping * np.eye(6, dtype=np.float64)
            g_vec = J.T @ f
            try:
                dx = -np.linalg.solve(H, g_vec)
            except np.linalg.LinAlgError:
                raise SingularityError("Kinematic Jacobian is singular during forward kinematics iteration.")

            # Simple Armijo line search
            step_scale = 1.0
            best_x = x_k + dx
            x_k = best_x

        final_R = self._rotation_matrix_zyx(float(x_k[3]), float(x_k[4]), float(x_k[5]))
        pose_list = [float(v) for v in x_k]
        pos_list = [float(x_k[0]), float(x_k[1]), float(x_k[2])]

        return ForwardKinematicsResult(
            pose=pose_list,
            position_m=pos_list,
            rotation_matrix=final_R,
            converged=converged,
            iterations=final_iter,
            residual_norm=res_norm,
        )

    def forward_kinematics(
        self,
        joint_positions: Sequence[float],
        initial_guess: Sequence[float] | None = None,
        max_iter: int = 30,
        tol: float = 1e-10,
    ) -> list[float]:
        """Compute platform pose [x, y, z, roll, pitch, yaw] from joint lengths [q1..q6].

        Args:
            joint_positions: 6-element sequence [q1..q6] in SI meters.
            initial_guess: Optional initial pose guess.
            max_iter: Maximum solver iterations.
            tol: Residual norm convergence threshold in meters.

        Returns:
            6-element list [x, y, z, roll, pitch, yaw].

        Raises:
            WorkspaceLimitError: If solver fails to converge.
        """
        res = self.forward_kinematics_with_metrics(
            joint_positions, initial_guess=initial_guess, max_iter=max_iter, tol=tol
        )
        if not res.converged:
            raise WorkspaceLimitError(
                f"Forward kinematics solver failed to converge within {max_iter} iterations (residual={res.residual_norm:.4e}m > tol={tol:.4e}m)."
            )
        return res.pose

