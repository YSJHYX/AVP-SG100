"""Stable adapter seam between physical joint17 episodes and future models."""

from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from sg100_teleop.data_collection.schema import PHYSICAL_DIM, SCHEMA_NAME


@dataclass(frozen=True)
class PhysicalEpisode:
    state: np.ndarray
    action: np.ndarray
    next_state: np.ndarray

    def validate(self) -> None:
        for name, value in (
            ("state", self.state),
            ("action", self.action),
            ("next_state", self.next_state),
        ):
            array = np.asarray(value)
            if array.ndim != 2 or array.shape[1] != PHYSICAL_DIM:
                raise ValueError(f"{name} must have shape [N,{PHYSICAL_DIM}]")
            if not np.all(np.isfinite(array)):
                raise ValueError(f"{name} contains non-finite physical data")


def load_physical_episode(path: str | Path) -> PhysicalEpisode:
    with np.load(Path(path), allow_pickle=False) as archive:
        schema = str(archive["schema_name"].item())
        if schema != SCHEMA_NAME:
            raise ValueError(f"unsupported schema: {schema}")
        episode = PhysicalEpisode(
            state=np.asarray(archive["state"], dtype=np.float64),
            action=np.asarray(archive["action"], dtype=np.float64),
            next_state=np.asarray(archive["next_state"], dtype=np.float64),
        )
    episode.validate()
    return episode


class ArmHandDatasetAdapter(ABC):
    """Future OpenPI/LeRobot exporters implement this physical-data seam."""

    source_schema = SCHEMA_NAME
    physical_dimension = PHYSICAL_DIM

    @abstractmethod
    def export(self, episode: PhysicalEpisode, output: str | Path) -> Path:
        """Normalize/pad/map cameras outside the hardware recorder."""
