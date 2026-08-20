from dataclasses import dataclass
from pathlib import Path

import numpy as np
import yaml


SCHEMA_VERSION = 1
ROTATION_ATOL = 1e-6


def validate_avp_to_tcp_local_rotation(value) -> np.ndarray:
    """Validate the proper rotation mapping fixed AVP axes to robot axes."""
    try:
        basis = np.asarray(value, dtype=np.float64)
    except (TypeError, ValueError) as exc:
        raise ValueError(
            "avp_to_tcp_local_rotation must be a numeric 3x3 matrix"
        ) from exc

    if basis.shape != (3, 3):
        raise ValueError(
            "avp_to_tcp_local_rotation must have shape (3,3), "
            f"got {basis.shape}"
        )
    if not np.all(np.isfinite(basis)):
        raise ValueError(
            "avp_to_tcp_local_rotation must contain only finite values"
        )

    gram = basis.T @ basis
    if not np.allclose(gram, np.eye(3), atol=ROTATION_ATOL, rtol=0.0):
        raise ValueError(
            "avp_to_tcp_local_rotation must be orthonormal; scale and shear "
            "belong outside the coordinate basis"
        )

    determinant = float(np.linalg.det(basis))
    if not np.isclose(determinant, 1.0, atol=ROTATION_ATOL, rtol=0.0):
        raise ValueError(
            "avp_to_tcp_local_rotation must have determinant +1; "
            f"got {determinant:.9f}"
        )

    return basis.copy()


@dataclass(frozen=True)
class AVPToTCPConfig:
    """Resolved fixed-axis AVP-world to robot-world controller mapping."""

    translation_scale: float
    avp_to_tcp_local_rotation: np.ndarray
    source: str


def load_avp_to_tcp_config(path: str | Path) -> AVPToTCPConfig:
    path = Path(path)
    with path.open("r", encoding="utf-8") as stream:
        data = yaml.safe_load(stream)

    if not isinstance(data, dict):
        raise ValueError("AVP-to-TCP config must contain a YAML mapping")
    if data.get("schema_version") != SCHEMA_VERSION:
        raise ValueError(
            "Unsupported AVP-to-TCP config schema_version="
            f"{data.get('schema_version')}"
        )

    translation_scale = float(data["translation_scale"])
    if not np.isfinite(translation_scale) or translation_scale <= 0.0:
        raise ValueError("translation_scale must be finite and > 0")

    source = data.get("source")
    if not isinstance(source, str) or not source.strip():
        raise ValueError("AVP-to-TCP config source must be a non-empty string")

    return AVPToTCPConfig(
        translation_scale=translation_scale,
        avp_to_tcp_local_rotation=validate_avp_to_tcp_local_rotation(
            data["avp_to_tcp_local_rotation"]
        ),
        source=source.strip(),
    )
