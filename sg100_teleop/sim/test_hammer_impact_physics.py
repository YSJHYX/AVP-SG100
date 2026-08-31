"""Isolated production-parameter Hammer/Peg velocity sweep.

The tool loads the production scene afresh for each velocity, gives the free
hammer a controlled initial velocity, and never writes model/controller
parameters back to XML.
"""

from __future__ import annotations

import argparse
from datetime import datetime
from pathlib import Path

import mujoco
import numpy as np

from sg100_teleop.sim.combined_robot_layout import resolve_robot_layout
from sg100_teleop.sim.hammer_impact_diagnostic import (
    HAMMER_BODY,
    HammerImpactDiagnostic,
)


ROOT = Path(__file__).resolve().parents[2]
DEFAULT_MODEL = (
    ROOT
    / "sg100_teleop/assets/rm65_sg100/mjcf/"
    "rm65_sg100_demo_hammer_peg_assembly_scene.xml"
)


def _id(model, objtype, name: str) -> int:
    object_id = mujoco.mj_name2id(model, objtype, name)
    if object_id < 0:
        raise RuntimeError(f"MuJoCo object not found: {name}")
    return int(object_id)


def _reset(model):
    data = mujoco.MjData(model)
    key = _id(model, mujoco.mjtObj.mjOBJ_KEY, "demo_ready")
    mujoco.mj_resetDataKeyframe(model, data, key)
    layout = resolve_robot_layout(model)
    data.ctrl[layout.arm_actuator_ids] = data.qpos[layout.arm_qpos_indices]
    data.ctrl[layout.hand_actuator_ids] = 0.0
    mujoco.mj_forward(model, data)
    return data, layout


def _hold_robot_bias(model, data, layout) -> None:
    mujoco.mj_forward(model, data)
    data.qfrc_applied[:] = 0.0
    data.qfrc_applied[layout.dof_indices] = -data.qfrc_bias[
        layout.dof_indices
    ]


def run_synthetic_impact(
    model_path: Path,
    speed: float,
    output_dir: Path,
) -> dict:
    if not np.isfinite(speed) or speed <= 0.0:
        raise ValueError("impact speed must be finite and > 0")
    model = mujoco.MjModel.from_xml_path(str(model_path))
    data, layout = _reset(model)
    hammer_body = _id(model, mujoco.mjtObj.mjOBJ_BODY, HAMMER_BODY)
    hammer_joint = int(model.body_jntadr[hammer_body])
    hammer_qpos = int(model.jnt_qposadr[hammer_joint])
    hammer_dof = int(model.jnt_dofadr[hammer_joint])

    # Same clean alignment used by the existing production-scene physics
    # regression: head center xy=(0.520, 0.200), lower face 5 mm above cap.
    data.qpos[hammer_qpos : hammer_qpos + 7] = np.array(
        [0.415, 0.209, 0.152, 1.0, 0.0, 0.0, 0.0]
    )
    data.qvel[hammer_dof : hammer_dof + 6] = 0.0
    data.qvel[hammer_dof + 2] = -float(speed)
    mujoco.mj_forward(model, data)

    diagnostic = HammerImpactDiagnostic(
        model,
        output_dir,
        pre_frames=25,
        post_frames=5,
        print_events=False,
    )
    q_goal = data.qpos[layout.arm_qpos_indices].copy()
    diagnostic.sample(data, q_goal)
    max_steps = int(np.ceil(0.8 / model.opt.timestep))
    for _ in range(max_steps):
        _hold_robot_bias(model, data, layout)
        mujoco.mj_step(model, data)
        diagnostic.sample(data, q_goal)
        if diagnostic.events:
            break
    diagnostic.close()
    if not diagnostic.events:
        return {
            "input_speed": speed,
            "contact_detected": False,
            "actual_impact_speed": np.nan,
            "peak_normal_force": 0.0,
            "estimated_impulse": 0.0,
            "peg_displacement_mm": 0.0,
        }
    event = diagnostic.events[0]
    return {
        "input_speed": speed,
        "contact_detected": True,
        "actual_impact_speed": event["pre_impact_head_speed"],
        "normal_impact_speed": event["head_normal_impact_velocity"],
        "peak_normal_force": event["peak_normal_force"],
        "estimated_impulse": event["estimated_normal_impulse_Ns"],
        "peg_displacement_mm": event["peg_displacement_mm"],
        "contact_duration_ms": event["contact_duration_s"] * 1000.0,
        "trace_csv": event["trace_csv"],
        "trace_npz": event["trace_npz"],
    }


def run_peg_force_sanity(model_path: Path, force_n: float = 7.0) -> dict:
    """Temporary low-speed check that the production peg is not locked."""

    model = mujoco.MjModel.from_xml_path(str(model_path))
    data, layout = _reset(model)
    joint = _id(model, mujoco.mjtObj.mjOBJ_JOINT, "demo_peg_slide")
    qpos = int(model.jnt_qposadr[joint])
    dof = int(model.jnt_dofadr[joint])
    before = float(data.qpos[qpos])
    force_steps = int(np.ceil(0.05 / model.opt.timestep))
    settle_steps = int(np.ceil(0.20 / model.opt.timestep))
    for _ in range(force_steps):
        _hold_robot_bias(model, data, layout)
        data.qfrc_applied[dof] = float(force_n)
        mujoco.mj_step(model, data)
    for _ in range(settle_steps):
        _hold_robot_bias(model, data, layout)
        mujoco.mj_step(model, data)
    return {
        "force_n": force_n,
        "duration_s": force_steps * float(model.opt.timestep),
        "settle_s": settle_steps * float(model.opt.timestep),
        "displacement_mm": (float(data.qpos[qpos]) - before) * 1000.0,
    }


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Synthetic Hammer/Peg impact sweep using production parameters"
    )
    parser.add_argument("--model", type=Path, default=DEFAULT_MODEL)
    parser.add_argument(
        "--speeds",
        nargs="+",
        type=float,
        default=(0.3, 0.5, 0.8),
        metavar="M_PER_S",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=None,
        help="Diagnostic trace directory (default: recording/diagnostics/...)",
    )
    return parser


def main() -> None:
    args = build_arg_parser().parse_args()
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    output_dir = args.output_dir or (
        ROOT / "recording/diagnostics" / f"hammer_synthetic_{stamp}"
    )
    results = []
    for speed in args.speeds:
        speed_dir = output_dir / f"{speed:.3f}_mps"
        results.append(run_synthetic_impact(args.model, speed, speed_dir))

    print("=" * 50)
    print("SYNTHETIC HAMMER IMPACT")
    print("=" * 50)
    for result in results:
        print(f"input head velocity: {result['input_speed']:.3f} m/s")
        print(f"contact detected: {'YES' if result['contact_detected'] else 'NO'}")
        print(f"actual impact velocity: {result['actual_impact_speed']:.6f} m/s")
        if result["contact_detected"]:
            print(f"normal impact velocity: {result['normal_impact_speed']:.6f} m/s")
            print(f"peak normal force: {result['peak_normal_force']:.6f} N")
            print(
                f"estimated impulse: {result['estimated_impulse']:.9g} N*s "
                "(ESTIMATED FROM DISCRETE CONTACT FORCE)"
            )
            print(f"contact duration: {result['contact_duration_ms']:.3f} ms")
        print(f"peg displacement: {result['peg_displacement_mm']:.6f} mm")
        print("-" * 50)

    successful = [
        result
        for result in results
        if 0.5 <= result["input_speed"] <= 0.8
        and result["peg_displacement_mm"] >= 1.0
    ]
    high_speed = [
        result for result in results if result["input_speed"] >= 0.8
    ]
    print("DIAGNOSTIC HEURISTIC (1 mm = visibly moving, not a hard physics threshold)")
    if successful:
        print("HAMMER/PEG PHYSICS: FUNCTIONAL")
        print("CONTROL-SIDE BOTTLENECK: LIKELY if AVP impacts are slower than this sweep")
    elif high_speed and all(r["peg_displacement_mm"] < 1.0 for r in high_speed):
        print("HAMMER/PEG PHYSICS: SUSPECT")
        print("CONTROL-SIDE BOTTLENECK: INCONCLUSIVE")
    else:
        print("HAMMER/PEG PHYSICS: INCONCLUSIVE")
        print("CONTROL-SIDE BOTTLENECK: INCONCLUSIVE")

    sanity = run_peg_force_sanity(args.model)
    print("\nPEG LOW-SPEED FORCE/MOTION SANITY CHECK")
    print(
        f"temporary applied joint force: {sanity['force_n']:.3f} N for "
        f"{sanity['duration_s']:.3f} s, then settle {sanity['settle_s']:.3f} s"
    )
    print(f"peg displacement: {sanity['displacement_mm']:.6f} mm")
    print(f"trace root: {output_dir}")
    print("Production XML/controller changes: NONE")


if __name__ == "__main__":
    main()
