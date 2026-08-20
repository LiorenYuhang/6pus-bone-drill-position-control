from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import Sequence


@dataclass
class ForwardKinematicsResult:
    """Detailed forward kinematics solution record."""
    pose: list[float]                        # [x, y, z, roll, pitch, yaw] in SI meters and radians
    position_m: list[float]                  # [x, y, z] in SI meters
    rotation_matrix: list[list[float]]       # 3x3 SO(3) rotation matrix
    converged: bool                          # True if residual norm <= tolerance
    iterations: int                          # Number of iterations executed
    residual_norm: float                     # L2 norm of kinematic constraint vector (meters)


class KinematicsError(Exception):
    """Base exception for kinematics calculation errors."""


class SingularityError(KinematicsError):
    """Raised when inverse/forward kinematics encounters a singular configuration."""


class WorkspaceLimitError(KinematicsError):
    """Raised when target pose exceeds the reachable workspace of the 6PUS mechanism."""


class KinematicsInterface(ABC):
    """Abstract interface for 6PUS mechanism forward and inverse kinematics."""

    @abstractmethod
    def inverse_kinematics(self, pose: Sequence[float]) -> list[float]:
        """Compute actuator joint lengths [q1..q6] from platform pose [x, y, z, roll, pitch, yaw].

        Args:
            pose: 6-element sequence [x, y, z, roll, pitch, yaw] (meters and radians).

        Returns:
            6-element list of actuator stroke/length values [q1, q2, q3, q4, q5, q6].
        """
        pass

    @abstractmethod
    def forward_kinematics(self, joint_positions: Sequence[float]) -> list[float]:
        """Compute platform pose [x, y, z, roll, pitch, yaw] from joint lengths [q1..q6].

        Args:
            joint_positions: 6-element sequence [q1..q6].

        Returns:
            6-element list [x, y, z, roll, pitch, yaw].
        """
        pass

    @abstractmethod
    def differential_inverse_kinematics(
        self,
        desired_pose: Sequence[float],
        reference_pose: Sequence[float],
    ) -> list[float]:
        """Compute differential actuator displacement Δq = IK(x_des) - IK(x_ref).

        Args:
            desired_pose: Target pose [x, y, z, roll, pitch, yaw].
            reference_pose: Reference baseline pose [x0, y0, z0, roll0, pitch0, yaw0].

        Returns:
            6-element list of differential joint displacements [Δq1..Δq6] in SI meters.
        """
        pass

