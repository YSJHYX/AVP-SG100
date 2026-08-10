from __future__ import annotations

from dataclasses import dataclass
import math

import mujoco
import numpy as np


RM65_JOINT_NAMES = (
    "joint_1",
    "joint_2",
    "joint_3",
    "joint_4",
    "joint_5",
    "joint_6",
)


@dataclass
class IKResult:
    converged: bool
    iterations: int

    position_error: float
    rotation_error: float

    sigma_min: float
    condition_number: float
    damping: float

    reason: str


def so3_log(R: np.ndarray) -> np.ndarray:
    """
    SO(3) logarithm.

    Returns a rotation vector whose direction is the rotation
    axis and whose magnitude is the rotation angle [rad].
    """
    value = (np.trace(R) - 1.0) / 2.0
    value = float(np.clip(value, -1.0, 1.0))

    theta = math.acos(value)

    if theta < 1e-9:
        return 0.5 * np.array(
            [
                R[2, 1] - R[1, 2],
                R[0, 2] - R[2, 0],
                R[1, 0] - R[0, 1],
            ],
            dtype=np.float64,
        )

    return (
        theta / (2.0 * math.sin(theta))
    ) * np.array(
        [
            R[2, 1] - R[1, 2],
            R[0, 2] - R[2, 0],
            R[1, 0] - R[0, 1],
        ],
        dtype=np.float64,
    )


class RM65IKSolver:
    """
    6D Cartesian IK for RM65.

    TCP:
        sg100_base_link

    Solver:
        adaptive damped least-squares Jacobian IK

    Important:
        Only RM65's six DoFs participate in IK.
        SG100's eleven joints are intentionally excluded.
    """

    def __init__(
        self,
        model: mujoco.MjModel,
        tcp_body_name: str = "sg100_base_link",
        *,
        orientation_weight: float = 0.20,
        position_tolerance: float = 1e-4,
        rotation_tolerance: float = 1e-3,
        max_iterations: int = 200,
        step_gain: float = 0.8,
        max_joint_step: float = 0.08,
        max_step_norm: float = 0.15,
        joint_limit_margin: float = 1e-4,
    ):
        self.model = model
        self.data = mujoco.MjData(model)

        self.orientation_weight = float(
            orientation_weight
        )

        self.position_tolerance = float(
            position_tolerance
        )

        self.rotation_tolerance = float(
            rotation_tolerance
        )

        self.max_iterations = int(
            max_iterations
        )

        self.step_gain = float(
            step_gain
        )

        self.max_joint_step = float(
            max_joint_step
        )

        self.max_step_norm = float(
            max_step_norm
        )

        self.joint_limit_margin = float(
            joint_limit_margin
        )

        self.tcp_body_id = mujoco.mj_name2id(
            model,
            mujoco.mjtObj.mjOBJ_BODY,
            tcp_body_name,
        )

        if self.tcp_body_id < 0:
            raise RuntimeError(
                f"TCP body not found: {tcp_body_name}"
            )

        self.joint_ids = []
        self.qpos_indices = []
        self.dof_indices = []

        lower = []
        upper = []

        for name in RM65_JOINT_NAMES:
            jid = mujoco.mj_name2id(
                model,
                mujoco.mjtObj.mjOBJ_JOINT,
                name,
            )

            if jid < 0:
                raise RuntimeError(
                    f"RM65 joint not found: {name}"
                )

            self.joint_ids.append(jid)

            qadr = int(
                model.jnt_qposadr[jid]
            )

            dadr = int(
                model.jnt_dofadr[jid]
            )

            self.qpos_indices.append(qadr)
            self.dof_indices.append(dadr)

            lower.append(
                model.jnt_range[jid, 0]
            )

            upper.append(
                model.jnt_range[jid, 1]
            )

        self.qpos_indices = np.asarray(
            self.qpos_indices,
            dtype=np.int64,
        )

        self.dof_indices = np.asarray(
            self.dof_indices,
            dtype=np.int64,
        )

        self.lower = np.asarray(
            lower,
            dtype=np.float64,
        )

        self.upper = np.asarray(
            upper,
            dtype=np.float64,
        )

    # ---------------------------------------------------------
    # Kinematics
    # ---------------------------------------------------------

    def _set_arm_q(
        self,
        q_arm: np.ndarray,
    ) -> None:
        q_arm = np.asarray(
            q_arm,
            dtype=np.float64,
        )

        if q_arm.shape != (6,):
            raise ValueError(
                f"Expected q_arm shape (6,), "
                f"got {q_arm.shape}"
            )

        self.data.qpos[:] = 0.0
        self.data.qvel[:] = 0.0

        self.data.qpos[
            self.qpos_indices
        ] = q_arm

        mujoco.mj_forward(
            self.model,
            self.data,
        )

    def fk(
        self,
        q_arm: np.ndarray,
    ):
        self._set_arm_q(
            q_arm
        )

        p = self.data.xpos[
            self.tcp_body_id
        ].copy()

        R = (
            self.data.xmat[
                self.tcp_body_id
            ]
            .reshape(3, 3)
            .copy()
        )

        return p, R

    def _jacobian(self):
        jacp = np.zeros(
            (3, self.model.nv),
            dtype=np.float64,
        )

        jacr = np.zeros(
            (3, self.model.nv),
            dtype=np.float64,
        )

        point = self.data.xpos[
            self.tcp_body_id
        ].copy()

        mujoco.mj_jac(
            self.model,
            self.data,
            jacp,
            jacr,
            point,
            self.tcp_body_id,
        )

        J = np.vstack(
            [
                jacp[
                    :,
                    self.dof_indices,
                ],
                jacr[
                    :,
                    self.dof_indices,
                ],
            ]
        )

        return J

    # ---------------------------------------------------------
    # Error
    # ---------------------------------------------------------

    def _pose_error(
        self,
        target_position,
        target_rotation,
    ):
        p_current = self.data.xpos[
            self.tcp_body_id
        ]

        R_current = (
            self.data.xmat[
                self.tcp_body_id
            ]
            .reshape(3, 3)
        )

        position_error = (
            target_position
            - p_current
        )

        # World-frame orientation error.
        #
        # This matches MuJoCo jacr, whose angular velocity
        # components are represented in world coordinates.
        R_error = (
            target_rotation
            @ R_current.T
        )

        rotation_error = so3_log(
            R_error
        )

        return (
            position_error,
            rotation_error,
        )

    # ---------------------------------------------------------
    # Adaptive damping
    # ---------------------------------------------------------

    @staticmethod
    def _adaptive_damping(
        sigma_min: float,
    ) -> float:
        """
        Increase damping close to singular configurations.

        The Jacobian tests showed sigma_min roughly in the
        0.015-0.03 range for several representative RM65
        configurations, so we deliberately start damping
        before sigma_min becomes extremely small.
        """
        base = 0.002
        threshold = 0.05
        maximum = 0.05

        if sigma_min >= threshold:
            return base

        ratio = (
            threshold - sigma_min
        ) / threshold

        return (
            base
            + (maximum - base)
            * ratio * ratio
        )

    # ---------------------------------------------------------
    # IK
    # ---------------------------------------------------------

    def solve(
        self,
        q_seed: np.ndarray,
        target_position: np.ndarray,
        target_rotation: np.ndarray,
    ):
        q = np.asarray(
            q_seed,
            dtype=np.float64,
        ).copy()

        target_position = np.asarray(
            target_position,
            dtype=np.float64,
        )

        target_rotation = np.asarray(
            target_rotation,
            dtype=np.float64,
        )

        if q.shape != (6,):
            raise ValueError(
                "q_seed must have shape (6,)"
            )

        if target_position.shape != (3,):
            raise ValueError(
                "target_position must have shape (3,)"
            )

        if target_rotation.shape != (3, 3):
            raise ValueError(
                "target_rotation must have shape (3,3)"
            )

        lower = (
            self.lower
            + self.joint_limit_margin
        )

        upper = (
            self.upper
            - self.joint_limit_margin
        )

        q = np.clip(
            q,
            lower,
            upper,
        )

        last_sigma_min = float("nan")
        last_condition = float("nan")
        last_damping = float("nan")

        for iteration in range(
            self.max_iterations
        ):
            self._set_arm_q(q)

            (
                e_pos,
                e_rot,
            ) = self._pose_error(
                target_position,
                target_rotation,
            )

            pos_norm = float(
                np.linalg.norm(e_pos)
            )

            rot_norm = float(
                np.linalg.norm(e_rot)
            )

            if (
                pos_norm <= self.position_tolerance
                and rot_norm <= self.rotation_tolerance
            ):
                # Compute diagnostics at the actual converged pose.
                # This also handles zero-iteration convergence without
                # returning NaN singularity metrics.
                J = self._jacobian()

                J_weighted = J.copy()
                J_weighted[3:, :] *= self.orientation_weight

                S = np.linalg.svd(
                    J_weighted,
                    compute_uv=False,
                )

                sigma_min = float(S[-1])

                if sigma_min > 1e-12:
                    condition = float(
                        S[0] / sigma_min
                    )
                else:
                    condition = float("inf")

                damping = self._adaptive_damping(
                    sigma_min
                )

                return q, IKResult(
                    converged=True,
                    iterations=iteration,
                    position_error=pos_norm,
                    rotation_error=rot_norm,
                    sigma_min=sigma_min,
                    condition_number=condition,
                    damping=damping,
                    reason="converged",
                )

            J = self._jacobian()

            # ------------------------------------------------
            # Mixed metric:
            #
            # translation [m]
            # rotation    [rad]
            #
            # orientation_weight converts the rotational
            # portion into a comparable optimization scale.
            # ------------------------------------------------
            J_weighted = J.copy()

            J_weighted[
                3:,
                :,
            ] *= self.orientation_weight

            error = np.concatenate(
                [
                    e_pos,
                    self.orientation_weight
                    * e_rot,
                ]
            )

            U, S, Vt = np.linalg.svd(
                J_weighted,
                full_matrices=False,
            )

            sigma_min = float(S[-1])

            if sigma_min > 1e-12:
                condition = float(
                    S[0] / sigma_min
                )
            else:
                condition = float("inf")

            damping = (
                self._adaptive_damping(
                    sigma_min
                )
            )

            # Damped pseudoinverse:
            #
            # J+_lambda =
            # V diag(s / (s^2 + lambda^2)) U^T
            factors = (
                S
                / (
                    S * S
                    + damping * damping
                )
            )

            dq = (
                Vt.T
                @ (
                    factors
                    * (
                        U.T
                        @ error
                    )
                )
            )

            dq *= self.step_gain

            # Per-joint step protection.
            dq = np.clip(
                dq,
                -self.max_joint_step,
                self.max_joint_step,
            )

            # Whole-vector step protection.
            dq_norm = float(
                np.linalg.norm(dq)
            )

            if dq_norm > self.max_step_norm:
                dq *= (
                    self.max_step_norm
                    / dq_norm
                )

            q_next = np.clip(
                q + dq,
                lower,
                upper,
            )

            last_sigma_min = sigma_min
            last_condition = condition
            last_damping = damping

            # If clipping or singularity prevents any further
            # movement, stop rather than loop indefinitely.
            if np.max(
                np.abs(q_next - q)
            ) < 1e-10:
                q = q_next
                break

            q = q_next

        # -----------------------------------------------------
        # Final evaluation
        # -----------------------------------------------------
        self._set_arm_q(q)

        e_pos, e_rot = self._pose_error(
            target_position,
            target_rotation,
        )

        pos_norm = float(
            np.linalg.norm(e_pos)
        )

        rot_norm = float(
            np.linalg.norm(e_rot)
        )

        return q, IKResult(
            converged=False,
            iterations=self.max_iterations,
            position_error=pos_norm,
            rotation_error=rot_norm,
            sigma_min=last_sigma_min,
            condition_number=last_condition,
            damping=last_damping,
            reason="max_iterations_or_stalled",
        )
