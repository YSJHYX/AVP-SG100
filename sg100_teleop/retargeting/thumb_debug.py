"""Low-frequency thumb retargeting diagnostics shared by the sim
runners' ``--thumb-retarget-debug`` mode.

Everything here is reporting-only: none of it changes the retargeting
pipeline.  The printed block mirrors the structure requested for
thumb debugging:

    grasp score / engagement / lock weight
    human canonical thumb features
    raw SG100 thumb q (base retargeting, pre-lock)
    applied lock bias per joint
    final SG100 thumb q (post-lock, clipped)
    MuJoCo thumb tip geometry vs the index-middle closure region
"""

from pathlib import Path

import mujoco
import numpy as np
import yaml

from sg100_teleop.retargeting.sg100_retargeter import SG100Retargeter


DEFAULT_SEMANTICS_PATH = (
    Path(__file__).resolve().parents[1]
    / "config"
    / "sg100_joint_semantics.yaml"
)

# Diagnostic tip references; matches
# sg100_teleop/sim/inspect_sg100_thumb_kinematics.py.  The production
# models define no tip sites, so distal body COM points are used.
THUMB_TIP_BODY = "thumb_link3"
INDEX_TIP_BODY = "index_link3"
MIDDLE_TIP_BODY = "middle_link2"


def build_lock_disabled_retargeter(
    semantics_path: Path | str = DEFAULT_SEMANTICS_PATH,
) -> SG100Retargeter:
    """Retargeter with power_grasp_thumb_lock forced off.

    The base retargeting (including pinch coordination) is identical
    to production; only the grasp lock bias is disabled, so its output
    is exactly the RAW (pre-lock) command used in the debug block.
    """

    import tempfile

    semantics_path = Path(semantics_path)

    with semantics_path.open("r") as f:
        config = yaml.safe_load(f)

    section = config.get("power_grasp_thumb_lock")

    if section is not None:
        section["enabled"] = False

    with tempfile.NamedTemporaryFile(
        "w",
        suffix=".yaml",
        delete=False,
    ) as f:
        yaml.safe_dump(config, f)
        temp_path = f.name

    return SG100Retargeter(Path(temp_path))


def hand_tip_geometry_mm(
    model: mujoco.MjModel,
    data: mujoco.MjData,
) -> dict[str, float]:
    """Thumb tip distances (mm) from the CURRENT MuJoCo state.

    Uses the live qpos-driven FK of the passed model/data, so during
    teleop this reports the actual robot hand geometry, not the
    commanded target.
    """

    def body_com(name: str) -> np.ndarray:
        bid = mujoco.mj_name2id(
            model,
            mujoco.mjtObj.mjOBJ_BODY,
            name,
        )
        if bid < 0:
            raise ValueError(
                f"thumb debug geometry: body {name!r} not in model"
            )
        return data.xipos[bid]

    thumb = body_com(THUMB_TIP_BODY)
    index = body_com(INDEX_TIP_BODY)
    middle = body_com(MIDDLE_TIP_BODY)
    lock = 0.5 * (index + middle)

    return {
        "thumb_index_mm": float(
            np.linalg.norm(thumb - index) * 1000.0
        ),
        "thumb_middle_mm": float(
            np.linalg.norm(thumb - middle) * 1000.0
        ),
        "thumb_lock_region_mm": float(
            np.linalg.norm(thumb - lock) * 1000.0
        ),
    }


def format_thumb_retarget_debug(
    info: dict,
    features: dict,
    geometry_mm: dict[str, float] | None = None,
) -> str:
    """Render the thumb retarget debug block (multi-line string)."""

    def q3(mapping: dict) -> str:
        return " / ".join(
            f"{np.degrees(mapping.get(j, 0.0)):+7.2f}"
            for j in ("thumb_j1", "thumb_j2", "thumb_j3")
        )

    lines = [
        "=" * 52,
        "SG100 THUMB RETARGET",
        "=" * 52,
        f"grasp_score    : {info.get('grasp_score', 0.0):.3f}",
        f"thumb_engagement: {info.get('thumb_engagement', 0.0):.3f}",
        f"pinch_gate     : {info.get('pinch_gate', 0.0):.3f}",
        f"lock_weight    : {info.get('lock_weight', 0.0):.3f}",
        "HUMAN  opposition / abduction / distal:",
        "        "
        f"{features.get('thumb_opposition', 0.0):+.3f} / "
        f"{features.get('thumb_abduction', 0.0):+.3f} / "
        f"{features.get('thumb_distal', 0.0):+.3f}",
        "RAW SG100  j1 / j2 / j3 (deg): "
        + q3(info.get("raw_q", {})),
        "LOCK BIAS  j1 / j2 / j3 (deg): "
        + q3(info.get("bias_q", {})),
        "FINAL SG100 j1 / j2 / j3 (deg): "
        + q3(info.get("final_q", {})),
    ]

    if geometry_mm is not None:
        lines.extend([
            "GEOMETRY (live FK)",
            f"  thumb-index      : "
            f"{geometry_mm.get('thumb_index_mm', float('nan')):7.1f} mm",
            f"  thumb-middle     : "
            f"{geometry_mm.get('thumb_middle_mm', float('nan')):7.1f} mm",
            f"  thumb-lock-region: "
            f"{geometry_mm.get('thumb_lock_region_mm', float('nan')):7.1f} mm",
        ])

    return "\n".join(lines)
