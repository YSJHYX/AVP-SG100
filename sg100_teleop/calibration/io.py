from pathlib import Path
from datetime import datetime, timezone

import yaml

from sg100_teleop.hand.hand_calibration import (
    HandCalibration,
    UnidirectionalCalibration,
    BidirectionalCalibration,
)


SCHEMA_VERSION = 1


def calibration_to_dict(
    calibration: HandCalibration,
    metadata: dict | None = None,
) -> dict:

    return {
        "schema_version": SCHEMA_VERSION,

        "metadata": {
            "created_at_utc": (
                datetime.now(
                    timezone.utc
                ).isoformat()
            ),
            **(metadata or {}),
        },

        "unidirectional": {
            name: {
                "neutral": float(
                    cfg.neutral
                ),
                "active": float(
                    cfg.active
                ),
            }
            for name, cfg
            in calibration.unidirectional.items()
        },

        "bidirectional": {
            name: {
                "negative": float(
                    cfg.negative
                ),
                "neutral": float(
                    cfg.neutral
                ),
                "positive": float(
                    cfg.positive
                ),
            }
            for name, cfg
            in calibration.bidirectional.items()
        },
    }


def calibration_from_dict(
    data: dict,
) -> HandCalibration:

    version = data.get(
        "schema_version"
    )

    if version != SCHEMA_VERSION:
        raise ValueError(
            f"Unsupported calibration "
            f"schema_version={version}"
        )

    uni = {
        name:
            UnidirectionalCalibration(
                neutral=float(
                    cfg["neutral"]
                ),
                active=float(
                    cfg["active"]
                ),
            )
        for name, cfg
        in data["unidirectional"].items()
    }

    bi = {
        name:
            BidirectionalCalibration(
                negative=float(
                    cfg["negative"]
                ),
                neutral=float(
                    cfg["neutral"]
                ),
                positive=float(
                    cfg["positive"]
                ),
            )
        for name, cfg
        in data["bidirectional"].items()
    }

    return HandCalibration(
        unidirectional=uni,
        bidirectional=bi,
    )


def save_calibration(
    calibration: HandCalibration,
    path: str | Path,
    metadata: dict | None = None,
) -> None:

    path = Path(path)

    path.parent.mkdir(
        parents=True,
        exist_ok=True,
    )

    data = calibration_to_dict(
        calibration,
        metadata=metadata,
    )

    with path.open(
        "w",
        encoding="utf-8",
    ) as f:
        yaml.safe_dump(
            data,
            f,
            sort_keys=False,
            allow_unicode=True,
        )


def load_calibration(
    path: str | Path,
) -> HandCalibration:

    path = Path(path)

    with path.open(
        "r",
        encoding="utf-8",
    ) as f:
        data = yaml.safe_load(f)

    return calibration_from_dict(
        data
    )
