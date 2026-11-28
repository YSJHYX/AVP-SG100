"""Run the existing Biyi control loop in its own process and native viewer."""
from __future__ import annotations

import argparse

from .channel import OperatorBridge


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--channel", required=True)
    parser.add_argument("--parent-pid", type=int)
    args, runner_args = parser.parse_known_args()
    bridge = OperatorBridge(args.channel, parent_pid=args.parent_pid)
    bridge.start()
    try:
        from sg100_teleop.sim.run_avp_arm_hand_sim import main as teleop_main
        teleop_main(runner_args, operator_bridge=bridge)
    except BaseException as exc:
        bridge.update(phase="ERROR", error=str(exc))
        raise
    else:
        bridge.update(phase="STOPPED", arm_enabled=False, hand_enabled=False)
    finally:
        bridge.close()


if __name__ == "__main__":
    main()
