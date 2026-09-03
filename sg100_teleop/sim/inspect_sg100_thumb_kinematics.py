"""SG100 thumb kinematics diagnostic (read-only FK, no viewer).

Answers two questions with the production SG100 model:

1.  JOINT SEMANTICS — what does each thumb joint's numeric q+ direction
    physically do (palm-relative), verified by a direct single-joint
    sweep min -> neutral -> max with every other hand joint held at
    the kinematic reference (q = 0)?

2.  OPERATING POINTS — where does the thumb distal phalanx actually
    land for canonical hand states driven through the production
    retargeter (open / power grasp / pinch), measured against the
    index-middle grasp closure region?

Palm frame (built from FK, same construction as
tests/test_sg100_retarget_semantics.py::PalmFrameHarness):

    origin  = mean(index MCP, middle MCP, little MCP body positions)
    lateral = index MCP - little MCP      (+ = thumb side of the palm)
    forward = origin - palm base          (out of the palm along the
                                           fingers)
    normal  = cross(lateral, forward)     (+ = palm interior side)

References (no synthetic tip sites exist in the production model, so
distal body COM points xipos are used, matching
test_sg100_physical_joint_semantics.py):

    thumb distal   = xipos(thumb_link3)
    index distal   = xipos(index_link3)
    middle distal  = xipos(middle_link2)
    lock region    = midpoint(index distal, middle distal) evaluated in
                     the SAME hand pose (the grasp closure region moves
                     with the fingers)

Usage:

    python -m sg100_teleop.sim.inspect_sg100_thumb_kinematics
    python -m sg100_teleop.sim.inspect_sg100_thumb_kinematics --sweep-only
    python -m sg100_teleop.sim.inspect_sg100_thumb_kinematics --operating-points-only
"""

import argparse

import mujoco
import numpy as np

from sg100_teleop.hand.canonical_hand import CanonicalHandState
from sg100_teleop.retargeting.sg100_retargeter import SG100Retargeter
from sg100_teleop.retargeting.thumb_debug import (
    build_lock_disabled_retargeter,
)


MODEL_PATH = (
    "sg100_teleop/assets/sg100/mjcf/sg100.xml"
)

THUMB_JOINTS = ("thumb_j1", "thumb_j2", "thumb_j3")

# Real AVP operating points (canonical), from the recorded calibration
# session used by tests/test_sg100_retarget_semantics.py::MEASURED_POINTS
# plus thumb distal values recorded in sg100_joint_semantics.yaml.
OPERATING_POINTS = {
    "OPEN_HAND": CanonicalHandState(
        thumb_opposition=0.0,
        thumb_abduction=0.0,
        thumb_distal=0.0,
    ),
    "NEUTRAL": CanonicalHandState(),
    "NORMAL_GRASP": CanonicalHandState(
        thumb_opposition=0.7595,
        thumb_abduction=0.8610,
        thumb_distal=0.918,
        index_adduction=0.0,
        index_flexion=0.9795,
        index_distal=0.8,
        middle_flexion=1.0,
        middle_distal=0.8,
        little_outward=0.0,
        little_flexion=1.0,
        little_distal=0.8,
    ),
    "INDEX_PINCH": CanonicalHandState(
        thumb_opposition=0.5724,
        thumb_abduction=1.0,
        thumb_distal=0.5,
        index_adduction=0.0,
        index_flexion=0.3892,
        index_distal=0.3,
        middle_flexion=0.0,
        middle_distal=0.0,
        little_outward=0.0,
        little_flexion=0.0,
        little_distal=0.0,
    ),
}


class ThumbKinematicsHarness:
    def __init__(
        self,
        model_path: str = MODEL_PATH,
        retargeter: SG100Retargeter | None = None,
    ):
        self.model = mujoco.MjModel.from_xml_path(model_path)
        self.data = mujoco.MjData(self.model)
        self.retargeter = (
            retargeter
            if retargeter is not None
            else SG100Retargeter()
        )

        self.joint_ids = {
            name: mujoco.mj_name2id(
                self.model,
                mujoco.mjtObj.mjOBJ_JOINT,
                name,
            )
            for name in self.retargeter.joint_order
        }
        assert all(jid >= 0 for jid in self.joint_ids.values())

        self._set_hand(np.zeros(11))
        self._build_palm_frame()

    # -- pose handling --------------------------------------------------

    def _set_hand(self, q_hand: np.ndarray) -> None:
        self.data.qpos[:] = 0.0
        self.data.qvel[:] = 0.0
        for i, name in enumerate(self.retargeter.joint_order):
            adr = int(
                self.model.jnt_qposadr[self.joint_ids[name]]
            )
            self.data.qpos[adr] = q_hand[i]
        mujoco.mj_forward(self.model, self.data)

    def _body_pos(self, name: str) -> np.ndarray:
        bid = mujoco.mj_name2id(
            self.model,
            mujoco.mjtObj.mjOBJ_BODY,
            name,
        )
        if bid < 0:
            raise ValueError(f"Unknown body: {name}")
        return self.data.xipos[bid].copy()

    def _build_palm_frame(self) -> None:
        index_mcp = self._body_pos("index_link1")
        middle_mcp = self._body_pos("middle_link1")
        little_mcp = self._body_pos("little_link1")
        palm_base = self._body_pos("thumb_link1") * 0.0  # placeholder

        # Palm base: the base_link body COM anchors the palm root.
        palm_base = self.data.xipos[
            mujoco.mj_name2id(
                self.model,
                mujoco.mjtObj.mjOBJ_BODY,
                "base_link",
            )
        ].copy()

        origin = (index_mcp + middle_mcp + little_mcp) / 3.0
        lateral_raw = index_mcp - little_mcp
        forward = origin - palm_base
        forward /= np.linalg.norm(forward)
        lateral = lateral_raw - np.dot(lateral_raw, forward) * forward
        lateral /= np.linalg.norm(lateral)
        normal = np.cross(lateral, forward)
        normal /= np.linalg.norm(normal)

        # Sign the palm normal physically: index MCP flexion curls the
        # index toward the palm interior.
        straight = self._body_pos("index_link3")
        q_flex = np.zeros(11)
        q_flex[
            self.retargeter.joint_order.index("index_j2")
        ] = np.deg2rad(60.0)
        self._set_hand(q_flex)
        flexed = self._body_pos("index_link3")
        normal *= np.sign(np.dot(flexed - straight, normal))
        assert abs(np.dot(flexed - straight, normal)) > 1e-4

        self._set_hand(np.zeros(11))

        self.palm_origin = origin
        self.palm_lateral = lateral  # + = thumb side
        self.palm_forward = forward  # + = out of the palm
        self.palm_normal = normal    # + = palm interior

    # -- measurements ----------------------------------------------------

    def palm_coords(self, point: np.ndarray) -> tuple[float, float, float]:
        delta = point - self.palm_origin
        return (
            float(np.dot(delta, self.palm_lateral)),
            float(np.dot(delta, self.palm_forward)),
            float(np.dot(delta, self.palm_normal)),
        )

    def geometry(self) -> dict:
        thumb = self._body_pos("thumb_link3")
        index = self._body_pos("index_link3")
        middle = self._body_pos("middle_link2")
        lock = 0.5 * (index + middle)

        def dist(a, b):
            return float(np.linalg.norm(a - b))

        return {
            "thumb": thumb,
            "d_thumb_index_mm": dist(thumb, index) * 1000.0,
            "d_thumb_middle_mm": dist(thumb, middle) * 1000.0,
            "d_thumb_lock_mm": dist(thumb, lock) * 1000.0,
        }

    def lock_region(self) -> np.ndarray:
        index = self._body_pos("index_link3")
        middle = self._body_pos("middle_link2")
        return 0.5 * (index + middle)

    # -- sweeps ------------------------------------------------------------

    def sweep_joint(self, joint_name: str, samples: int = 11) -> list[dict]:
        jid = self.joint_ids[joint_name]
        adr = int(self.model.jnt_qposadr[jid])
        lower, upper = (
            float(self.model.jnt_range[jid][0]),
            float(self.model.jnt_range[jid][1]),
        )

        rows = []
        for value in np.linspace(lower, upper, samples):
            q = np.zeros(11)
            q[adr] = value
            self._set_hand(q)
            geo = self.geometry()
            lat, fwd, nrm = self.palm_coords(geo["thumb"])
            rows.append({
                "q": value,
                "lateral": lat,
                "forward": fwd,
                "normal": nrm,
                **geo,
            })
        self._set_hand(np.zeros(11))
        return rows

    def operating_point(self, state: CanonicalHandState) -> dict:
        q = self.retargeter.retarget(state)
        self._set_hand(q)
        geo = self.geometry()
        lat, fwd, nrm = self.palm_coords(geo["thumb"])
        lock_lat, lock_fwd, lock_nrm = self.palm_coords(
            self.lock_region()
        )
        result = {
            "q": q,
            "thumb_lateral": lat,
            "thumb_forward": fwd,
            "thumb_normal": nrm,
            "lock_lateral": lock_lat,
            "lock_forward": lock_fwd,
            "lock_normal": lock_nrm,
            **geo,
        }
        self._set_hand(np.zeros(11))
        return result


def _fmt(value: float) -> str:
    return f"{value:+.4f}"


def describe_direction(rows: list[dict], key: str) -> str:
    """Summarize how ``key`` changes from q_min to q_max."""
    lo, hi = rows[0][key], rows[-1][key]
    delta = hi - lo
    if abs(delta) < 1e-6:
        return "no change"
    return f"{delta:+.4f} (min {_fmt(lo)} -> max {_fmt(hi)})"


def run_sweep(harness: ThumbKinematicsHarness) -> None:
    print()
    print("=" * 72)
    print("SG100 THUMB SINGLE-JOINT SWEEP (other hand joints at q=0)")
    print("=" * 72)
    print("palm frame: lateral + = thumb side, normal + = palm interior,")
    print("            forward + = out of the palm toward finger tips")
    print("references: thumb distal = xipos(thumb_link3) COM,")
    print("            lock region  = midpoint(index_link3, middle_link2)")
    print("            COMs evaluated in the SAME pose")
    print()

    for joint in THUMB_JOINTS:
        jid = harness.joint_ids[joint]
        lower, upper = harness.model.jnt_range[jid]
        rows = harness.sweep_joint(joint)

        print(f"--- {joint}: range [{lower:+.3f}, {upper:+.3f}] rad ---")
        print(
            f"{'q(rad)':>8s} {'lat(m)':>9s} {'fwd(m)':>9s} {'nrm(m)':>9s}"
            f" {'d_idx(mm)':>10s} {'d_mid(mm)':>10s} {'d_lock(mm)':>11s}"
        )
        for row in rows:
            print(
                f"{row['q']:+8.3f}"
                f" {row['lateral']:+9.4f}"
                f" {row['forward']:+9.4f}"
                f" {row['normal']:+9.4f}"
                f" {row['d_thumb_index_mm']:10.1f}"
                f" {row['d_thumb_middle_mm']:10.1f}"
                f" {row['d_thumb_lock_mm']:11.1f}"
            )
        print()
        print(f"  lateral (thumb-side +): {describe_direction(rows, 'lateral')}")
        print(f"  normal  (palm-interior +): {describe_direction(rows, 'normal')}")
        print(f"  forward (palm-outward +): {describe_direction(rows, 'forward')}")
        print(f"  d_thumb_lock: {describe_direction(rows, 'd_thumb_lock_mm')}")
        print()


def run_operating_points(
    harness: ThumbKinematicsHarness,
    harness_no_lock: ThumbKinematicsHarness | None = None,
) -> None:
    print()
    print("=" * 72)
    print("SG100 THUMB OPERATING POINTS (production retargeter)")
    if harness_no_lock is not None:
        print("BEFORE = power_grasp_thumb_lock disabled (base mapping)")
        print("AFTER  = production config (lock enabled)")
    print("=" * 72)
    print()

    for name, state in OPERATING_POINTS.items():
        features = state.as_dict()

        before = (
            harness_no_lock.operating_point(state)
            if harness_no_lock is not None
            else None
        )

        result = harness.operating_point(state)
        q_deg = np.degrees(result["q"][:3])
        gate = harness.retargeter.pinch_gate(features)
        weight = harness.retargeter.thumb_lock_weight(features)

        # Applied bias diagnostics use the pre-lock command as the
        # base (BEFORE q when the lock-disabled harness is available).
        diag = harness.retargeter.thumb_lock_diagnostics(
            before["q"] if before is not None else result["q"],
            features,
        )

        print(f"--- {name} ---")
        print(
            "  canonical: opp="
            f"{state.thumb_opposition:.3f} abd="
            f"{state.thumb_abduction:.3f} dst="
            f"{state.thumb_distal:+.3f}"
        )
        print(
            "  gates    : pinch_gate="
            f"{gate:.3f} grasp_score="
            f"{harness.retargeter.grasp_score(features):.3f}"
            f" engagement="
            f"{harness.retargeter.thumb_engagement(features):.3f}"
            f" lock_weight={weight:.3f}"
        )
        print(
            "  AFTER  q (deg): thumb_j1="
            f"{q_deg[0]:+7.2f} thumb_j2={q_deg[1]:+7.2f}"
            f" thumb_j3={q_deg[2]:+7.2f}"
        )
        if before is not None:
            bq = np.degrees(before["q"][:3])
            bias = {
                joint: round(float(np.degrees(value)), 2)
                for joint, value in diag["bias_q"].items()
            }
            print(
                "  BEFORE q (deg): thumb_j1="
                f"{bq[0]:+7.2f} thumb_j2={bq[1]:+7.2f}"
                f" thumb_j3={bq[2]:+7.2f}   applied bias (deg):"
                f" {bias}"
            )
            print(
                "  d_lock (mm): BEFORE "
                f"{before['d_thumb_lock_mm']:6.1f} -> AFTER "
                f"{result['d_thumb_lock_mm']:6.1f}   d_thumb_index:"
                f" {before['d_thumb_index_mm']:6.1f} ->"
                f" {result['d_thumb_index_mm']:6.1f}"
                f"   d_thumb_middle:"
                f" {before['d_thumb_middle_mm']:6.1f} ->"
                f" {result['d_thumb_middle_mm']:6.1f}"
            )
        print(
            "  thumb palm coords (m): lateral="
            f"{result['thumb_lateral']:+.4f} forward="
            f"{result['thumb_forward']:+.4f} normal="
            f"{result['thumb_normal']:+.4f}"
        )
        print(
            "  lock  palm coords (m): lateral="
            f"{result['lock_lateral']:+.4f} forward="
            f"{result['lock_forward']:+.4f} normal="
            f"{result['lock_normal']:+.4f}"
        )
        print(
            "  AFTER distances (mm): d_thumb_index="
            f"{result['d_thumb_index_mm']:.1f}"
            f" d_thumb_middle={result['d_thumb_middle_mm']:.1f}"
            f" d_thumb_lock={result['d_thumb_lock_mm']:.1f}"
        )
        print()


def main() -> None:
    parser = argparse.ArgumentParser(
        description=(
            "SG100 thumb kinematics diagnostic: single-joint sweeps "
            "and canonical operating points (read-only FK)."
        )
    )
    parser.add_argument(
        "--model",
        default=MODEL_PATH,
        help="SG100 MJCF model path (default: production sg100.xml)",
    )
    parser.add_argument(
        "--sweep-only",
        action="store_true",
        help="Only run the single-joint sweeps",
    )
    parser.add_argument(
        "--operating-points-only",
        action="store_true",
        help="Only run the canonical operating-point measurements",
    )
    args = parser.parse_args()

    harness = ThumbKinematicsHarness(args.model)

    # BEFORE/AFTER comparison: base mapping vs production lock config.
    harness_no_lock = ThumbKinematicsHarness(
        args.model,
        retargeter=build_lock_disabled_retargeter(),
    )

    if not args.operating_points_only:
        run_sweep(harness)
    if not args.sweep_only:
        run_operating_points(harness, harness_no_lock)

    print("DIAGNOSTIC COMPLETE — all values are FK/COM measurements,")
    print("no SG100 hardware involved.")


if __name__ == "__main__":
    main()
