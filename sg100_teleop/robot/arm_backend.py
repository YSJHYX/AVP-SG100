"""Arm backend abstraction (v3 §A1.2 / §B2.1)."""

from __future__ import annotations

from abc import ABC, abstractmethod

import numpy as np

from sg100_teleop.robot.arm_state import ArmCommandSent, ArmState


class ArmBackend(ABC):
    """The single seam between teleop logic and arm hardware.

    ``send_joint_target`` only *publishes*; backends own their sender
    timing.  Real backends must pass every target through their safety
    chain (continuity gate at the producer side, limiter/watchdogs at
    the sender side) before it reaches hardware.
    """

    @abstractmethod
    def connect(self) -> None:
        """Establish the link and validate the arm (dof, state)."""

    @abstractmethod
    def disconnect(self) -> None:
        """Stop threads and close the link safely."""

    @property
    @abstractmethod
    def enabled(self) -> bool:
        """Whether the sender may stream targets to the arm."""

    @abstractmethod
    def set_enabled(self, enabled: bool) -> None:
        """Enable/disable streaming (ARM/DISARM, pause semantics)."""

    @abstractmethod
    def send_joint_target(
        self,
        q_rad: np.ndarray,
        *,
        command_bundle_id: int | None = None,
    ) -> None:
        """Publish a new joint target (non-blocking)."""

    @abstractmethod
    def drain_command_events(self) -> tuple[ArmCommandSent, ...]:
        """Return and remove successful post-limiter command events."""

    @abstractmethod
    def joint_state(self) -> ArmState | None:
        """Latest feedback sample, ``None`` before the first one."""

    @abstractmethod
    def stop_smooth(self) -> int:
        """Decelerated stop; returns SDK status code (0 = success)."""

    @abstractmethod
    def stop_immediate(self) -> int:
        """Immediate stop; returns SDK status code (0 = success)."""

    @abstractmethod
    def joint_limits_rad(self) -> tuple[np.ndarray, np.ndarray]:
        """(lower, upper) hard joint limits in radians."""
