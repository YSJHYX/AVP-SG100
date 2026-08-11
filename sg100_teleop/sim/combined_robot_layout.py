from __future__ import annotations

from dataclasses import dataclass

import mujoco
import numpy as np


RM65_JOINT_ORDER = (
    "joint_1",
    "joint_2",
    "joint_3",
    "joint_4",
    "joint_5",
    "joint_6",
)

SG100_JOINT_ORDER = (
    "thumb_j1",
    "thumb_j2",
    "thumb_j3",
    "index_j1",
    "index_j2",
    "index_j3",
    "middle_j1",
    "middle_j2",
    "little_j1",
    "little_j2",
    "little_j3",
)

ROBOT_JOINT_ORDER = (
    RM65_JOINT_ORDER
    + SG100_JOINT_ORDER
)

ARM_DOF = len(RM65_JOINT_ORDER)
HAND_DOF = len(SG100_JOINT_ORDER)
TOTAL_DOF = len(ROBOT_JOINT_ORDER)


@dataclass(frozen=True)
class CombinedRobotLayout:
    joint_ids: np.ndarray
    qpos_indices: np.ndarray
    dof_indices: np.ndarray
    actuator_ids: np.ndarray

    @property
    def arm_qpos_indices(self) -> np.ndarray:
        return self.qpos_indices[:ARM_DOF]

    @property
    def hand_qpos_indices(self) -> np.ndarray:
        return self.qpos_indices[ARM_DOF:]

    @property
    def arm_actuator_ids(self) -> np.ndarray:
        return self.actuator_ids[:ARM_DOF]

    @property
    def hand_actuator_ids(self) -> np.ndarray:
        return self.actuator_ids[ARM_DOF:]


def _require_strict_contiguous_order(
    values: np.ndarray,
    *,
    label: str,
) -> None:
    if not np.array_equal(
        np.diff(values),
        np.ones(TOTAL_DOF - 1, dtype=np.int64),
    ):
        raise RuntimeError(
            f"Robot {label} must be contiguous and follow "
            f"the authoritative 17-joint order; got "
            f"{values.tolist()}"
        )


def resolve_robot_layout(
    model: mujoco.MjModel,
) -> CombinedRobotLayout:
    """Resolve and strictly validate the named 17-DoF robot layout."""
    joint_ids = []
    qpos_indices = []
    dof_indices = []
    actuator_ids = []

    for joint_name in ROBOT_JOINT_ORDER:
        joint_id = mujoco.mj_name2id(
            model,
            mujoco.mjtObj.mjOBJ_JOINT,
            joint_name,
        )
        if joint_id < 0:
            raise RuntimeError(
                "Required robot joint not found in MuJoCo model: "
                f"{joint_name}"
            )

        if model.jnt_type[joint_id] != mujoco.mjtJoint.mjJNT_HINGE:
            raise RuntimeError(
                f"Robot joint {joint_name} must be a hinge joint"
            )

        actuator_name = f"{joint_name}_pos"
        actuator_id = mujoco.mj_name2id(
            model,
            mujoco.mjtObj.mjOBJ_ACTUATOR,
            actuator_name,
        )
        if actuator_id < 0:
            raise RuntimeError(
                "Required robot actuator not found in MuJoCo model: "
                f"{actuator_name}"
            )

        actuator_joint_id = int(
            model.actuator_trnid[actuator_id, 0]
        )
        if actuator_joint_id != joint_id:
            actual_joint_name = mujoco.mj_id2name(
                model,
                mujoco.mjtObj.mjOBJ_JOINT,
                actuator_joint_id,
            )
            raise RuntimeError(
                f"Actuator {actuator_name} should drive {joint_name}, "
                f"but drives {actual_joint_name}"
            )

        joint_ids.append(joint_id)
        qpos_indices.append(int(model.jnt_qposadr[joint_id]))
        dof_indices.append(int(model.jnt_dofadr[joint_id]))
        actuator_ids.append(actuator_id)

    layout = CombinedRobotLayout(
        joint_ids=np.asarray(joint_ids, dtype=np.int64),
        qpos_indices=np.asarray(qpos_indices, dtype=np.int64),
        dof_indices=np.asarray(dof_indices, dtype=np.int64),
        actuator_ids=np.asarray(actuator_ids, dtype=np.int64),
    )

    _require_strict_contiguous_order(
        layout.joint_ids,
        label="joint IDs",
    )
    _require_strict_contiguous_order(
        layout.qpos_indices,
        label="qpos addresses",
    )
    _require_strict_contiguous_order(
        layout.dof_indices,
        label="DoF addresses",
    )
    _require_strict_contiguous_order(
        layout.actuator_ids,
        label="actuator order",
    )

    return layout


def _as_robot_command(
    q_arm: np.ndarray,
    q_hand: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    q_arm = np.asarray(q_arm, dtype=np.float64)
    q_hand = np.asarray(q_hand, dtype=np.float64)

    if q_arm.shape != (ARM_DOF,):
        raise ValueError(
            f"q_arm must have shape ({ARM_DOF},), got {q_arm.shape}"
        )
    if q_hand.shape != (HAND_DOF,):
        raise ValueError(
            f"q_hand must have shape ({HAND_DOF},), got {q_hand.shape}"
        )
    if not np.all(np.isfinite(q_arm)):
        raise ValueError("q_arm contains non-finite values")
    if not np.all(np.isfinite(q_hand)):
        raise ValueError("q_hand contains non-finite values")

    return q_arm, q_hand


def write_robot_controls(
    data: mujoco.MjData,
    layout: CombinedRobotLayout,
    q_arm: np.ndarray,
    q_hand: np.ndarray,
) -> None:
    q_arm, q_hand = _as_robot_command(q_arm, q_hand)
    data.ctrl[layout.arm_actuator_ids] = q_arm
    data.ctrl[layout.hand_actuator_ids] = q_hand


def initialize_robot_state(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    layout: CombinedRobotLayout,
    q_arm: np.ndarray,
    q_hand: np.ndarray,
) -> None:
    """Reset to model defaults, then initialize only named robot state."""
    q_arm, q_hand = _as_robot_command(q_arm, q_hand)

    data.qpos[:] = model.qpos0
    data.qpos[layout.arm_qpos_indices] = q_arm
    data.qpos[layout.hand_qpos_indices] = q_hand
    write_robot_controls(data, layout, q_arm, q_hand)
    mujoco.mj_forward(model, data)
