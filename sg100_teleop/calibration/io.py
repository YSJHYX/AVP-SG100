from pathlib import Path
from datetime import datetime, timezone

import yaml

from sg100_teleop.hand.hand_calibration import (
    HandCalibration,
    UnidirectionalCalibration,
    BidirectionalCalibration,
)


SCHEMA_VERSION = 3

# schema_version 1 captured index_adduction as a UNIDIRECTIONAL feature.
# The retarget contract requires a bidirectional (signed) index
# adduction calibration: -1 toward thumb / 0 neutral / +1 toward middle.
#
# schema_version 2 captured index/middle/little distal as BIDIRECTIONAL
# features.  Human DIP joints are not operable in reverse, so these are
# now UNIDIRECTIONAL: 0 = straight/open baseline, 1 = distal flexed
# toward palm (no hyperextension endpoint).
#
# Legacy files are rejected with an actionable message instead of being
# silently reinterpreted.
LEGACY_SCHEMA_VERSIONS = (1, 2)


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

    if version in LEGACY_SCHEMA_VERSIONS or (
        "index_adduction"
        in (data.get("unidirectional") or {})
    ) or (
        "index_distal"
        in (data.get("bidirectional") or {})
    ):
        raise ValueError(
            "Legacy calibration schema "
            f"(schema_version={version}); rerun AVP hand calibration. "
            "The current contract requires signed bidirectional "
            "index_adduction (toward thumb / neutral / toward middle) and "
            "unidirectional finger-distal flexion "
            "(straight baseline / distal flex toward palm, no "
            "hyperextension endpoint) "
            "(请重新运行 AVP 手部标定：食指内收需双向端点，"
            "手指末节为伸直基线+屈曲端点，不再采集反伸端点)."
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
