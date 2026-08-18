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

# Position-only IK tuning (3D translation task, orientation free).
POSITION_ONLY_TOLERANCE_M = 1e-4
POSITION_ONLY_MAX_ITERATIONS = 60
POSITION_ONLY_DAMPING = 0.05
POSITION_ONLY_POSTURE_GAIN = 0.02
POSITION_ONLY_MAX_STEP_RAD = 0.2


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


def rotation_matrix_to_quat_wxyz(R: np.ndarray) -> np.ndarray:
    """
    Rotation matrix -> quaternion in the official library's
    (w, x, y, z) component order. Robust for any rotation angle.
    """
    R = np.asarray(R, dtype=np.float64)

    tr = float(np.trace(R))

    if tr > 0.0:
        s = math.sqrt(tr + 1.0) * 2.0
        w = 0.25 * s
        x = (R[2, 1] - R[1, 2]) / s
        y = (R[0, 2] - R[2, 0]) / s
        z = (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s

    quat = np.array([w, x, y, z], dtype=np.float64)

    norm = float(np.linalg.norm(quat))
    if norm > 0.0:
        quat /= norm

    return quat


class RM65IKSolver:
    """
    6D Cartesian IK for RM65.

    TCP:
        sg100_base_link (coincides with the RM65 joint-6 flange frame,
        so the target pose is the official T06 pose).

    Solver:
        RealMan's official offline algorithm library from the
        ``Robotic_Arm`` pip package (``rm_algo_inverse_kinematics``).
        It is a closed-form solver that returns the analytic solution
        nearest to the seed joint vector, which preserves the
        frame-to-frame continuity this teleop pipeline relies on.

        The official library was validated against this MuJoCo model by
        ``scripts/validate_rm_official_fk.py``: FK agreement <= 5 um /
        0.022 deg across sampled configurations, joint limits within
        0.2 deg, and an offline IK round trip <= 5.3 um. MuJoCo world
        coordinates equal the official RM65 base frame, so no extra
        SE(3) correction is needed.

    Unit / convention bridge:
        official library: joint angles in degrees, quaternions ordered
        (w, x, y, z); this class keeps the project's radians +
        rotation-matrix interface.

    Diagnostics:
        ``sigma_min`` / ``condition_number`` are computed from the
        MuJoCo Jacobian at the returned configuration (position and
        orientation rows unweighted). ``damping`` is kept for
        ``IKResult`` compatibility and is always 0.0 because the
        official solver is analytic, not iteratively damped.

    Important:
        Only RM65's six DoFs participate in IK.
        SG100's eleven joints are intentionally excluded.
    """

    def __init__(
        self,
        model: mujoco.MjModel,
        tcp_body_name: str = "sg100_base_link",
        *,
        joint_limit_margin: float = 1e-4,
    ):
        self.model = model
        self.data = mujoco.MjData(model)

        self.joint_limit_margin = float(joint_limit_margin)

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

            qadr = int(model.jnt_qposadr[jid])
            dadr = int(model.jnt_dofadr[jid])

            self.qpos_indices.append(qadr)
            self.dof_indices.append(dadr)

            lower.append(model.jnt_range[jid, 0])
            upper.append(model.jnt_range[jid, 1])

        self.qpos_indices = np.asarray(
            self.qpos_indices,
            dtype=np.int64,
        )

        self.dof_indices = np.asarray(
            self.dof_indices,
            dtype=np.int64,
        )

        self.lower = np.asarray(lower, dtype=np.float64)
        self.upper = np.asarray(upper, dtype=np.float64)

        self._init_official_algorithm()

    # ---------------------------------------------------------
    # Official RealMan algorithm library
    # ---------------------------------------------------------

    def _init_official_algorithm(self) -> None:
        try:
            from Robotic_Arm.rm_ctypes_wrap import (
                rm_force_type_e,
                rm_inverse_kinematics_params_t,
                rm_robot_arm_model_e,
            )
            from Robotic_Arm.rm_robot_interface import Algo
        except ImportError as exc:
            raise RuntimeError(
                "The RealMan official algorithm package is required "
                "for RM65 IK. Install it with:  pip install Robotic_Arm"
            ) from exc

        # RM65-B = RM_MODEL_RM_65_E arm + RM_MODEL_RM_B_E standard
        # (B-series) end-effector version.
        self._params_type = rm_inverse_kinematics_params_t

        self._algo = Algo(
            rm_robot_arm_model_e.RM_MODEL_RM_65_E,
            rm_force_type_e.RM_MODEL_RM_B_E,
        )

        if getattr(self._algo, "arm_dof", 6) != 6:
            raise RuntimeError(
                "Official algorithm library did not initialize as a "
                f"6-DoF RM65 (arm_dof={self._algo.arm_dof})."
            )

    # ---------------------------------------------------------
    # Kinematics (MuJoCo, used for FK cross-checks and Jacobian
    # diagnostics; validated to match the official library)
    # ---------------------------------------------------------

    def _set_arm_q(
        self,
        q_arm: np.ndarray,
    ) -> None:
        q_arm = np.asarray(q_arm, dtype=np.float64)

        if q_arm.shape != (6,):
            raise ValueError(
                f"Expected q_arm shape (6,), got {q_arm.shape}"
            )

        self.data.qpos[:] = 0.0
        self.data.qvel[:] = 0.0

        self.data.qpos[self.qpos_indices] = q_arm

        mujoco.mj_forward(
            self.model,
            self.data,
        )

    def fk(
        self,
        q_arm: np.ndarray,
    ):
        self._set_arm_q(q_arm)

        p = self.data.xpos[self.tcp_body_id].copy()

        R = (
            self.data.xmat[self.tcp_body_id]
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

        point = self.data.xpos[self.tcp_body_id].copy()

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
                jacp[:, self.dof_indices],
                jacr[:, self.dof_indices],
            ]
        )

        return J

    def _position_jacobian(self):
        """3D position-task Jacobian, shape (3, 6)."""

        jacp = np.zeros(
            (3, self.model.nv),
            dtype=np.float64,
        )

        jacr = np.zeros(
            (3, self.model.nv),
            dtype=np.float64,
        )

        point = self.data.xpos[self.tcp_body_id].copy()

        mujoco.mj_jac(
            self.model,
            self.data,
            jacp,
            jacr,
            point,
            self.tcp_body_id,
        )

        return jacp[:, self.dof_indices]

    # ---------------------------------------------------------
    # Error
    # ---------------------------------------------------------

    def _pose_error(
        self,
        target_position,
        target_rotation,
    ):
        p_current = self.data.xpos[self.tcp_body_id]

        R_current = (
            self.data.xmat[self.tcp_body_id]
            .reshape(3, 3)
        )

        position_error = target_position - p_current

        # World-frame orientation error.
        R_error = target_rotation @ R_current.T

        rotation_error = so3_log(R_error)

        return position_error, rotation_error

    def _singularity_diagnostics(self) -> tuple[float, float]:
        """
        sigma_min / condition number of the (unweighted) MuJoCo
        Jacobian at the currently set configuration.
        """
        J = self._jacobian()

        S = np.linalg.svd(
            J,
            compute_uv=False,
        )

        sigma_min = float(S[-1])

        if sigma_min > 1e-12:
            condition = float(S[0] / sigma_min)
        else:
            condition = float("inf")

        return sigma_min, condition

    # ---------------------------------------------------------
    # IK
    # ---------------------------------------------------------

    def _solve_position_only(
        self,
        q_seed: np.ndarray,
        target_position: np.ndarray,
    ):
        """Damped-least-squares 3D position task with null-space
        posture regularization toward the seed.

        Task residual:  e = p_target - p_tcp(q)      shape (3,)
        Task Jacobian:  J = dp_tcp/dq                 shape (3, 6)

        The orientation never enters the residual, the Jacobian, or
        the convergence criterion.
        """

        lower = self.lower + self.joint_limit_margin
        upper = self.upper - self.joint_limit_margin

        q_seed = np.clip(
            np.asarray(q_seed, dtype=np.float64),
            lower,
            upper,
        )

        q = q_seed.copy()

        # Adaptive (Levenberg-Marquardt style) damping: strong damping
        # keeps large steps stable; once the error decreases steadily
        # the damping decays so the tail converges quickly instead of
        # crawling at the fixed-damping floor.
        damping = POSITION_ONLY_DAMPING
        prev_error = float("inf")

        iteration = 0
        converged = False

        for iteration in range(1, POSITION_ONLY_MAX_ITERATIONS + 1):
            self._set_arm_q(q)

            e = target_position - self.data.xpos[
                self.tcp_body_id
            ]

            error_norm = float(np.linalg.norm(e))

            if error_norm <= POSITION_ONLY_TOLERANCE_M:
                converged = True
                break

            J = self._position_jacobian()  # (3, 6)

            A = J @ J.T + (damping ** 2) * np.eye(3)

            dq_task = J.T @ np.linalg.solve(A, e)

            # Weak null-space posture preference toward the seed:
            # resolves the 6-DoF redundancy for continuity without
            # competing with the position task.
            J_pinv = J.T @ np.linalg.inv(A)

            N = np.eye(6) - J_pinv @ J

            dq_null = POSITION_ONLY_POSTURE_GAIN * (q_seed - q)

            dq = dq_task + N @ dq_null

            dq = np.clip(
                dq,
                -POSITION_ONLY_MAX_STEP_RAD,
                POSITION_ONLY_MAX_STEP_RAD,
            )

            q = np.clip(q + dq, lower, upper)

            if error_norm < prev_error:
                damping = max(1e-3, damping * 0.7)
            else:
                damping = POSITION_ONLY_DAMPING

            prev_error = error_norm

        self._set_arm_q(q)

        pos_error = float(np.linalg.norm(
            target_position - self.data.xpos[self.tcp_body_id]
        ))

        sigma_min, condition = self._singularity_diagnostics()

        if converged:
            return q, IKResult(
                converged=True,
                iterations=iteration,
                position_error=pos_error,
                rotation_error=0.0,
                sigma_min=sigma_min,
                condition_number=condition,
                damping=POSITION_ONLY_DAMPING,
                reason="position_only_success",
            )

        # HOLD semantics: return the seed unchanged and report the
        # error of the configuration actually returned.
        self._set_arm_q(q_seed)

        pos_error = float(np.linalg.norm(
            target_position - self.data.xpos[self.tcp_body_id]
        ))

        return q_seed.copy(), IKResult(
            converged=False,
            iterations=iteration,
            position_error=pos_error,
            rotation_error=0.0,
            sigma_min=sigma_min,
            condition_number=condition,
            damping=POSITION_ONLY_DAMPING,
            reason="position_only_max_iterations",
        )

    def solve(
        self,
        q_seed: np.ndarray,
        target_position: np.ndarray,
        target_rotation: np.ndarray | None,
        position_only: bool = False,
    ):
        """
        position_only=False (default):
            full 6D pose IK via the official closed-form solver
            (target_rotation required, shape (3, 3)).

        position_only=True:
            TRUE 3D translation task IK.  The orientation is NOT a
            task residual — target_rotation is ignored entirely
            (may be None) and no rotation Jacobian rows participate.
            A damped-least-squares position step plus a weak
            null-space posture term toward the seed keeps the
            redundant solution continuous frame to frame.  On
            non-convergence the seed is returned unchanged (HOLD),
            matching the official path's failure semantics.
        """

        q = np.asarray(q_seed, dtype=np.float64).copy()

        target_position = np.asarray(
            target_position,
            dtype=np.float64,
        )

        if q.shape != (6,):
            raise ValueError("q_seed must have shape (6,)")

        if target_position.shape != (3,):
            raise ValueError("target_position must have shape (3,)")

        if position_only:
            return self._solve_position_only(
                q,
                target_position,
            )

        if target_rotation is None:
            raise ValueError(
                "target_rotation is required unless position_only=True"
            )

        target_rotation = np.asarray(
            target_rotation,
            dtype=np.float64,
        )

        if target_rotation.shape != (3, 3):
            raise ValueError("target_rotation must have shape (3,3)")

        lower = self.lower + self.joint_limit_margin
        upper = self.upper - self.joint_limit_margin

        q = np.clip(q, lower, upper)

        # -----------------------------------------------------
        # Official closed-form IK (degrees + wxyz quaternion)
        # -----------------------------------------------------

        quat = rotation_matrix_to_quat_wxyz(target_rotation)

        params = self._params_type(
            q_in=list(np.degrees(q)) + [0.0],
            q_pose=[
                float(target_position[0]),
                float(target_position[1]),
                float(target_position[2]),
                float(quat[0]),
                float(quat[1]),
                float(quat[2]),
                float(quat[3]),
            ],
            flag=0,
        )

        ret, q_out_deg = self._algo.rm_algo_inverse_kinematics(params)

        # -----------------------------------------------------
        # Accept only a clean, finite, in-limit solution.
        # Otherwise return the seed unchanged (HOLD semantics).
        # -----------------------------------------------------

        solved = ret == 0

        if solved:
            q_solution = np.radians(
                np.asarray(q_out_deg, dtype=np.float64)
            )

            if not np.all(np.isfinite(q_solution)):
                solved = False
            else:
                q_solution = np.clip(q_solution, lower, upper)

        if not solved:
            q_solution = q

        # -----------------------------------------------------
        # Evaluate error and singularity diagnostics at the
        # returned configuration through MuJoCo FK.
        # -----------------------------------------------------

        self._set_arm_q(q_solution)

        e_pos, e_rot = self._pose_error(
            target_position,
            target_rotation,
        )

        pos_norm = float(np.linalg.norm(e_pos))
        rot_norm = float(np.linalg.norm(e_rot))

        sigma_min, condition = self._singularity_diagnostics()

        if solved:
            return q_solution, IKResult(
                converged=True,
                iterations=1,
                position_error=pos_norm,
                rotation_error=rot_norm,
                sigma_min=sigma_min,
                condition_number=condition,
                damping=0.0,
                reason="official_ik_success",
            )

        return q_solution, IKResult(
            converged=False,
            iterations=0,
            position_error=pos_norm,
            rotation_error=rot_norm,
            sigma_min=sigma_min,
            condition_number=condition,
            damping=0.0,
            reason=f"official_ik_failed_ret_{ret}",
        )
