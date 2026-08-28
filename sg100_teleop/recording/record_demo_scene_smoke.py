"""Record short scripted smoke sessions for each demo scene.

Produces one ``mujoco_recording.npz`` per demo (single shared state
timeline), replayable by ``render_mujoco_video`` from any camera.
The scripts are kinematic puppets: the arm interpolates between the
audited IK waypoints, the hand closes, grasped objects are welded to
the palm frame (recorded relative transform at grasp time), and
task joints (peg / drawer) are driven along their scripted
progress.  Physics still steps underneath so free bodies settle
realistically; welded objects are placed, not simulated, which is
appropriate for a camera smoke reel.

Run::

    conda run -n mujoco-learn python -m \\
        sg100_teleop.recording.record_demo_scene_smoke
"""

from __future__ import annotations

import os
from pathlib import Path

os.environ.setdefault("MUJOCO_GL", "egl")

import mujoco
import numpy as np

from sg100_teleop.kinematics.rm65_ik import RM65IKSolver
from sg100_teleop.recording.mujoco_state_recorder import (
    MujocoStateRecorder,
)
from sg100_teleop.sim.combined_robot_layout import (
    resolve_robot_layout,
)
from sg100_teleop.sim.demo_task_validation import (
    V8_READY_ARM_Q,
    audit_ik,
)

ROOT = Path(__file__).resolve().parents[2]
MJCF = ROOT / "sg100_teleop/assets/rm65_sg100/mjcf"
OUT_ROOT = ROOT / "recording/demo_scene_previews"

RECORD_FPS = 30.0
HAND_OPEN = np.zeros(11)
HAND_CLOSED = np.array(
    [0.55, -0.75, 0.45, 0.9, 1.35, 0.5, -0.7, 0.9, 1.0, -0.35, 0.8]
)


def _qadr(model, joint):
    jid = mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_JOINT, joint
    )
    return int(model.jnt_qposadr[jid])


def _body_id(model, name):
    return mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_BODY, name
    )


def _quat_between(a: np.ndarray, b: np.ndarray, s: float):
    """SLERP-free nlerp on quaternions (wxyz), fine for smoke reels."""
    if float(np.dot(a, b)) < 0.0:
        b = -b
    q = a + s * (b - a)
    return q / np.linalg.norm(q)


class Puppet:
    """Drives one scene through a scripted timeline."""

    def __init__(self, xml: Path, out_dir: Path):
        self.model = mujoco.MjModel.from_xml_path(str(xml))
        self.data = mujoco.MjData(self.model)
        self.layout = resolve_robot_layout(self.model)
        self.ik = RM65IKSolver(
            self.model, tcp_body_name="sg100_base_link"
        )
        key = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_KEY, "demo_ready"
        )
        mujoco.mj_resetDataKeyframe(self.model, self.data, key)
        self.data.ctrl[self.layout.arm_actuator_ids] = V8_READY_ARM_Q
        self.data.ctrl[
            self.layout.hand_actuator_ids
        ] = HAND_OPEN
        self.out_dir = out_dir
        self.recorder = MujocoStateRecorder(
            self.model,
            scene_path=xml,
            recording_dir=out_dir,
            record_fps=RECORD_FPS,
            camera_name="workstation_front",
        )
        self.recorder.start(self.data)
        self._palm = _body_id(self.model, "sg100_base_link")
        self._welds: dict[int, tuple[np.ndarray, np.ndarray]] = {}

    # -- helpers ---------------------------------------------------
    def solve_arm(self, target):
        q, res = self.ik.solve(
            q_seed=self.current_arm_q(),
            target_position=np.asarray(target),
            target_rotation=None,
            position_only=True,
            posture_reference=V8_READY_ARM_Q,
        )
        assert res.converged, f"IK failed for {target}"
        return q

    def current_arm_q(self):
        return self.data.qpos[
            self.layout.arm_qpos_indices
        ].copy()

    def set_arm(self, q):
        self.data.qpos[self.layout.arm_qpos_indices] = q
        self.data.ctrl[self.layout.arm_actuator_ids] = q

    def set_hand(self, q):
        self.data.qpos[self.layout.hand_qpos_indices] = q
        self.data.ctrl[self.layout.hand_actuator_ids] = q

    def weld(self, joint_name):
        """Freeze a freejoint's relative pose to the palm frame."""
        jadr = _qadr(self.model, joint_name)
        mujoco.mj_forward(self.model, self.data)
        palm_pos = self.data.xpos[self._palm].copy()
        palm_mat = self.data.xmat[self._palm].reshape(3, 3).copy()
        inv_palm = np.eye(4)
        inv_palm[:3, :3] = palm_mat.T
        inv_palm[:3, 3] = -palm_mat.T @ palm_pos
        obj_pos = self.data.qpos[jadr:jadr + 3].copy()
        obj_quat = self.data.qpos[jadr + 3:jadr + 7].copy()
        obj_mat = np.zeros(9)
        mujoco.mju_quat2Mat(obj_mat, obj_quat)
        rel = inv_palm @ np.block(
            [[obj_mat.reshape(3, 3), obj_pos[:, None]],
             [np.zeros((1, 3)), np.ones((1, 1))]]
        )
        self._welds[jadr] = (rel[:3, :3], rel[:3, 3])

    def unweld(self, joint_name):
        self._welds.pop(_qadr(self.model, joint_name), None)

    def _apply_welds(self):
        if not self._welds:
            return
        mujoco.mj_kinematics(self.model, self.data)
        palm_pos = self.data.xpos[self._palm].copy()
        palm_mat = self.data.xmat[self._palm].reshape(3, 3).copy()
        for jadr, (rel_R, rel_p) in self._welds.items():
            world_R = palm_mat @ rel_R
            world_p = palm_mat @ rel_p + palm_pos
            self.data.qpos[jadr:jadr + 3] = world_p
            quat = np.zeros(4)
            mujoco.mju_mat2Quat(quat, world_R.reshape(9))
            self.data.qpos[jadr + 3:jadr + 7] = quat
            self.data.qvel[jadr:jadr + 6] = 0.0

    # -- timeline ----------------------------------------------------
    def hold(self, seconds: float, record=True):
        steps = int(round(seconds * RECORD_FPS))
        for _ in range(steps):
            self._apply_welds()
            mujoco.mj_forward(self.model, self.data)
            self.data.qfrc_applied[:] = 0.0
            self.data.qfrc_applied[
                self.layout.dof_indices
            ] = -self.data.qfrc_bias[self.layout.dof_indices]
            if record:
                self.recorder.maybe_record(self.data)
            for _ in range(
                int(round(1.0 / RECORD_FPS / self.model.opt.timestep))
            ):
                self._apply_welds()
                mujoco.mj_step(self.model, self.data)

    def move_arm(self, target, seconds: float, close_hand=False):
        """Interpolate the arm to a waypoint (fresh IK solution)."""
        q_start = self.current_arm_q()
        q_goal = self.solve_arm(target)
        hand_start = self.data.qpos[
            self.layout.hand_qpos_indices
        ].copy()
        hand_goal = HAND_CLOSED if close_hand else hand_start
        frames = max(1, int(round(seconds * RECORD_FPS)))
        for f in range(1, frames + 1):
            s = f / frames
            smooth = s * s * (3.0 - 2.0 * s)
            self.set_arm(q_start + smooth * (q_goal - q_start))
            self.set_hand(
                hand_start + smooth * (hand_goal - hand_start)
            )
            self._apply_welds()
            mujoco.mj_forward(self.model, self.data)
            self.data.qfrc_applied[:] = 0.0
            self.data.qfrc_applied[
                self.layout.dof_indices
            ] = -self.data.qfrc_bias[self.layout.dof_indices]
            self.recorder.maybe_record(self.data)
            for _ in range(
                int(
                    round(
                        1.0
                        / RECORD_FPS
                        / self.model.opt.timestep
                    )
                )
            ):
                self._apply_welds()
                mujoco.mj_step(self.model, self.data)

    def drive_joint(self, joint, value):
        jadr = _qadr(self.model, joint)
        self.data.qpos[jadr] = value
        self.data.qvel[self.model.jnt_dofadr[
            mujoco.mj_name2id(
                self.model, mujoco.mjtObj.mjOBJ_JOINT, joint
            )
        ]] = 0.0

    def finish(self) -> Path:
        return self.recorder.stop()


# ------------------------------------------------------------------
# Demo scripts
# ------------------------------------------------------------------

def record_demo01():
    """V2: hammer rests raised on the two-pier cradle."""
    xml = (
        MJCF / "rm65_sg100_demo_hammer_peg_assembly_scene.xml"
    )
    out = OUT_ROOT / "hammer_peg_assembly"
    p = Puppet(xml, out)

    p.hold(0.5)
    p.move_arm((0.34, -0.13, 0.12), 0.8)          # pregrasp (raised)
    p.move_arm((0.34, -0.13, 0.077), 0.8, close_hand=True)
    p.weld("demo_hammer_free")
    p.hold(0.3)
    p.move_arm((0.34, -0.13, 0.22), 0.8)          # lift off the cradle
    p.move_arm((0.415, 0.209, 0.22), 1.0)         # pre-strike
    for tap in range(3):
        p.move_arm((0.415, 0.209, 0.180), 0.25)
        p.move_arm((0.415, 0.209, 0.155), 0.25)
        p.move_arm((0.415, 0.209, 0.185), 0.25)
        # scripted peg progress after each tap (kinematic smoke)
        p.drive_joint("demo_peg_slide", 0.014 * (tap + 1))
    p.move_arm((0.34, -0.13, 0.22), 0.8)
    p.move_arm((0.34, -0.13, 0.13), 0.8)          # back above the cradle
    p.move_arm((0.34, -0.12, 0.09), 0.6)
    p.set_hand(HAND_OPEN)
    p.unweld("demo_hammer_free")
    p.hold(0.4)                                    # settle onto the piers
    p.move_arm((0.36, 0.02, 0.14), 0.8)
    p.hold(0.5)
    return p.finish()


def record_demo02():
    """V2: Drawer Loading & Closing - open, load the tape, close."""
    xml = (
        MJCF
        / "rm65_sg100_demo_drawer_loading_and_closing_scene.xml"
    )
    out = OUT_ROOT / "drawer_loading_and_closing"
    p = Puppet(xml, out)

    p.hold(0.5)
    # hook the horizontal bar handle
    p.move_arm((0.458, 0.18, 0.19), 0.8)
    p.move_arm((0.458, 0.18, 0.145), 0.8, close_hand=True)
    p.hold(0.2)
    # pull: drawer follows a scripted open profile while the hand
    # retracts with the bar
    for i in range(1, 25):
        s = i / 24.0
        p.drive_joint("demo_drawer_slide", 0.15 * s)
        p.move_arm((0.458 - 0.128 * s, 0.18, 0.145), 1.0 / 30.0)
    p.hold(0.3)
    # release the handle, fetch the tape from the table
    p.set_hand(HAND_OPEN)
    p.move_arm((0.40, 0.05, 0.20), 0.8)
    p.move_arm((0.30, -0.18, 0.14), 0.9)
    p.move_arm((0.30, -0.18, 0.08), 0.7, close_hand=True)
    p.weld("demo_drawer_item_free")
    p.hold(0.3)
    p.move_arm((0.30, -0.18, 0.18), 0.7)
    p.move_arm((0.47, 0.18, 0.20), 1.0)           # over the cavity
    p.move_arm((0.47, 0.18, 0.11), 0.7)           # into the drawer
    p.set_hand(HAND_OPEN)
    p.unweld("demo_drawer_item_free")
    p.hold(0.4)                                    # tape settles inside
    p.move_arm((0.47, 0.18, 0.20), 0.7)
    # re-hook the opened handle and push closed
    p.move_arm((0.35, 0.18, 0.145), 0.9, close_hand=True)
    for i in range(1, 25):
        s = i / 24.0
        p.drive_joint("demo_drawer_slide", 0.15 * (1.0 - s))
        p.move_arm((0.33 + 0.128 * s, 0.18, 0.145), 1.0 / 30.0)
    p.hold(0.3)
    p.set_hand(HAND_OPEN)
    p.move_arm((0.36, 0.0, 0.16), 0.8)
    p.hold(0.5)
    return p.finish()


def record_demo03():
    """V2: exactly two task objects (box + screwdriver)."""
    xml = (
        MJCF / "rm65_sg100_demo_maintenance_kit_packing_scene.xml"
    )
    out = OUT_ROOT / "maintenance_kit_packing"
    p = Puppet(xml, out)

    def cycle(joint, grasp_xy, grasp_z, goal_xy, goal_z, lift=0.20):
        p.move_arm((grasp_xy[0], grasp_xy[1], grasp_z + 0.07), 0.7)
        p.move_arm(
            (grasp_xy[0], grasp_xy[1], grasp_z), 0.7, close_hand=True
        )
        p.weld(joint)
        p.move_arm(
            (grasp_xy[0], grasp_xy[1], lift), 0.7
        )
        p.move_arm((goal_xy[0], goal_xy[1], goal_z + 0.10), 0.9)
        p.move_arm((goal_xy[0], goal_xy[1], goal_z + 0.02), 0.6)
        p.set_hand(HAND_OPEN)
        p.unweld(joint)
        p.hold(0.3)
        p.move_arm((goal_xy[0], goal_xy[1], lift), 0.7)

    p.hold(0.5)
    cycle(
        "kit_object_a_free", (0.44, 0.34), 0.13,
        (0.32, -0.12), 0.0125,
    )
    cycle(
        "kit_object_b_free", (0.54, 0.20), 0.10,
        (0.42, -0.295), 0.008,
    )
    p.move_arm((0.36, 0.0, 0.16), 0.8)
    p.hold(0.6)
    return p.finish()


def main() -> int:
    print("recording demo 01 smoke ...")
    path = record_demo01()
    print("  ->", path)
    print("recording demo 02 smoke ...")
    path = record_demo02()
    print("  ->", path)
    print("recording demo 03 smoke ...")
    path = record_demo03()
    print("  ->", path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
