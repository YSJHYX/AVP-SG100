"""Episode numeric buffer；每个 append 对应唯一 master tick。"""

from __future__ import annotations

from typing import Any, Mapping

import numpy as np


_VECTOR_WIDTHS = {
    "arm_q_goal_rad": 6,
    "arm_qcmd_sent_rad": 6,
    "arm_qpos_rad": 6,
    "hand_qcmd_retargeted_canonical_rad": 11,
    "hand_qcmd_effective_canonical_rad": 11,
    "hand_qcmd_sdk_deg": 11,
    "hand_feedback_sdk_rad": 11,
    "hand_feedback_modes": 11,
    "hand_qpos_canonical_rad": 11,
    "robot_qcmd_17d_rad": 17,
    "robot_desired_qcmd_17d_rad": 17,
    "robot_qpos_17d_rad": 17,
}


class VlaRawBuffer:
    def __init__(self) -> None:
        self._rows: list[dict[str, Any]] = []

    def append(self, row: Mapping[str, Any]) -> None:
        materialized = dict(row)
        missing = set(_VECTOR_WIDTHS) - set(materialized)
        if missing:
            raise ValueError(f"tick row 缺少字段: {sorted(missing)}")
        for name, width in _VECTOR_WIDTHS.items():
            value = np.asarray(materialized[name])
            if value.shape != (width,):
                raise ValueError(f"{name} 必须是 [{width}]")
            materialized[name] = value.copy()
        self._rows.append(materialized)

    def clear(self) -> None:
        self._rows.clear()

    def __len__(self) -> int:
        return len(self._rows)

    @property
    def rows(self) -> tuple[dict[str, Any], ...]:
        return tuple(self._rows)

    def arrays(self) -> dict[str, np.ndarray]:
        if not self._rows:
            return {
                name: np.empty((0, width), dtype=np.float64)
                for name, width in _VECTOR_WIDTHS.items()
            }
        keys = tuple(self._rows[0])
        if any(tuple(row) != keys for row in self._rows):
            raise ValueError("tick rows schema 不一致")
        result: dict[str, np.ndarray] = {}
        for key in keys:
            values = [row[key] for row in self._rows]
            result[key] = np.stack(values) if key in _VECTOR_WIDTHS else np.asarray(values)
        return result
