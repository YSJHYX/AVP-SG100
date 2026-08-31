"""Print compiled Hammer/Peg/RM65 physics parameters without mutation."""

from __future__ import annotations

import argparse
from pathlib import Path

import mujoco

from sg100_teleop.sim.hammer_impact_diagnostic import (
    audit_hammer_model,
    format_static_audit,
)
from sg100_teleop.tracking.avp_to_tcp_config import load_avp_to_tcp_config


ROOT = Path(__file__).resolve().parents[2]
DEFAULT_MODEL = (
    ROOT
    / "sg100_teleop/assets/rm65_sg100/mjcf/"
    "rm65_sg100_demo_hammer_peg_assembly_scene.xml"
)
DEFAULT_TCP_CONFIG = (
    ROOT / "sg100_teleop/config/avp_to_tcp_identity.yaml"
)


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Read-only compiled-model Hammer impact physics audit"
    )
    parser.add_argument("--model", type=Path, default=DEFAULT_MODEL)
    parser.add_argument(
        "--avp-to-tcp-config", type=Path, default=DEFAULT_TCP_CONFIG
    )
    parser.add_argument("--tracking-hz", type=float, default=60.0)
    parser.add_argument("--arm-filter-tau", type=float, default=0.03)
    parser.add_argument("--max-arm-velocity", type=float, default=3.0)
    return parser


def main() -> None:
    args = build_arg_parser().parse_args()
    model = mujoco.MjModel.from_xml_path(str(args.model))
    audit = audit_hammer_model(model)
    print(format_static_audit(audit))

    config = load_avp_to_tcp_config(args.avp_to_tcp_config)
    tcp_filter = config.filter
    print("\nCONTROL PATH EFFECTIVE LIMITS")
    print(f"AVP input / IK update rate: {args.tracking_hz:.9g} Hz")
    print(f"simulation / actuator update rate: {1.0 / model.opt.timestep:.9g} Hz")
    print(f"translation scale: {config.translation_scale:.9g}")
    print(f"TCP position/rotation tau: {tcp_filter.position_tau:.9g} / {tcp_filter.rotation_tau:.9g} s")
    print(f"TCP max linear velocity: {tcp_filter.max_linear_velocity:.9g} m/s")
    print(f"TCP max angular velocity: {tcp_filter.max_angular_velocity:.9g} rad/s")
    print(f"arm q-goal filter tau: {args.arm_filter_tau:.9g} s")
    print(f"arm q-command velocity limit: {args.max_arm_velocity:.9g} rad/s")
    print("real RM65 backend limiter active in this sim path: NO")
    print("potential strike attenuation: INCONCLUSIVE (runtime measurement required)")


if __name__ == "__main__":
    main()
