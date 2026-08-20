"""Controller interface and abstract base class for 6PUS platform control."""

from __future__ import annotations

from abc import ABC, abstractmethod
from typing import Sequence


class ControlError(Exception):
    """Base exception for control module errors."""


class ControllerInterface(ABC):
    """Abstract interface for 6PUS controllers (PID, ADRC, RL/SAC)."""

    @abstractmethod
    def reset(self) -> None:
        """Reset internal controller states, integrators, and observers."""
        pass

    @abstractmethod
    def update(
        self,
        current_state: Sequence[float],
        target_state: Sequence[float],
        dt: float,
    ) -> list[float]:
        """Compute control action.

        Args:
            current_state: Current state vector (positions or pose).
            target_state: Target state vector.
            dt: Time step in seconds.

        Returns:
            Control action vector (e.g. commanded joint positions or velocities).
        """
        pass
