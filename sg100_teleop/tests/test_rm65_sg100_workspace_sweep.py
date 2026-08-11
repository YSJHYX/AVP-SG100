"""
RM65 + SG100 operational workspace sweep.

This test measures the continuous Cartesian workspace reachable from the
validated RM65 teleoperation-ready configuration.

Stage 1 keeps the SG100 TCP orientation fixed at the ready orientation and
sweeps the TCP along six Cartesian directions:

    outward / inward
    left / right
    up / down

The purpose is to identify:

    GOOD  : healthy operational workspace
    EDGE  : reachable but close to singularity or joint limits
    FAIL  : no longer continuously reachable from the ready configuration

This is a kinematic-only test.

It does not test:
    - collision
    - table geometry
    - object contact
    - grasp force
    - MoveIt planning
    - real AVP tracking
"""
from pathlib import Path

import mujoco
import numpy as np

from sg100_teleop.kinematics.rm65_ik import RM65IKSolver

# ============================================================
# Repository / model paths
# ============================================================

REPO_ROOT = Path(__file__).resolve().parents[2]

MODEL_PATH = (
    REPO_ROOT
    / "sg100_teleop"
    / "assets"
    / "rm65_sg100"
    / "mjcf"
    / "rm65_sg100_sim.xml"
)

TCP_BODY = "sg100_base_link"

# ============================================================
# Robot dimensions
# ============================================================

ARM_DOF = 6
HAND_DOF = 11
TOTAL_DOF = 17

# ============================================================
# Validated RM65 teleoperation-ready configuration
# ============================================================

Q_TELEOP_READY = np.array(
    [
         0.32488,
        -0.76769,
         0.97621,
         0.22463,
        -1.80649,
        -0.12430,
    ],
    dtype=np.float64,
)


# ============================================================
# RM65 joint limits
# ============================================================

RM65_JOINT_LOWER = np.array(
    [
        -3.1060,
        -2.2689,
        -2.3560,
        -3.1060,
        -2.2340,
        -6.2800,
    ],
    dtype=np.float64,
)

RM65_JOINT_UPPER = np.array(
    [
         3.1060,
         2.2689,
         2.3560,
         3.1060,
         2.2340,
         6.2800,
    ],
    dtype=np.float64,
)

# ============================================================
# Workspace classification thresholds
# ============================================================

SIGMA_MIN_GOOD = 0.015
CONDITION_MAX_GOOD = 80.0
JOINT_MARGIN_GOOD = 0.15 # rad

IK_POSITION_TOL_M = 0.0002
IK_ROTATION_TOL_RAD = np.deg2rad(0.1)

OBJECT_PLACEMENT_MIN_GOOD_DOWN_M = 0.080

# ============================================================
# Sweep configuration
# ============================================================

SWEEP_STEP_M = 0.005
MAX_SWEEP_DISTANCE_M = 0.50

SWEEP_DIRECTION_ORDER = (
    "OUTWARD",
    "INWARD",
    "LEFT",
    "RIGHT",
    "UP",
    "DOWN",
)

# ============================================================
# Tabletop workspace grid configuration
# ============================================================

TABLETOP_GRID_STEP_M = 0.020

# ============================================================
# Basic helpers
# ============================================================

def normalize(vector: np.ndarray) -> np.ndarray:
    """
    Return a normalized 3D vector.

    Raises
    ------
    ValueError
        If the input does not have shape (3,), contains non-finite
        values, or has near-zero norm.
    """
    vector = np.asarray(
        vector,
        dtype=np.float64,
    )

    if vector.shape != (3,):
        raise ValueError(f"vector must have shape (3,), got {vector.shape}")

    if not np.all(np.isfinite(vector)):
        raise ValueError("vector must contain finite values")
    
    norm = float(np.linalg.norm(vector))

    if norm < 1e-12:
        raise ValueError("cannot normalize a near-zero vector")

    return vector / norm

def joint_limit_margin(
    q_arm: np.ndarray,
) -> tuple[float, int]:
    """
    Return the minimum RM65 joint-limit margin.

    The margin of joint i is:

        min(
            q[i] - lower[i],
            upper[i] - q[i],
        )

    Returns
    -------
    min_margin : float
        Smallest joint-limit margin in radians.

        Positive:
            configuration is inside the limits.

        Zero:
            one joint is exactly on a limit.

        Negative:
            at least one joint violates its limit.

    limiting_joint_index : int
        Zero-based RM65 joint index associated with min_margin.
    """
    q_arm = np.asarray(
        q_arm,
        dtype=np.float64,
    )

    if q_arm.shape != (ARM_DOF,):
        raise ValueError(f"q_arm must have shape ({ARM_DOF},), got {q_arm.shape}")
    
    if not np.all(np.isfinite(q_arm)):
        raise ValueError("q_arm contain non-finite values")
    
    lower_margin = (q_arm - RM65_JOINT_LOWER)

    upper_margin = (RM65_JOINT_UPPER - q_arm)

    margins = np.minimum(lower_margin, upper_margin)

    limiting_joint_index = int(np.argmin(margins))

    min_margin = float(margins[limiting_joint_index])

    return (min_margin, limiting_joint_index,)

def classify_solution(
    *,
    converged: bool,
    sigma_min: float,
    condition_number: float,
    joint_margin: float,
    position_error_m: float,
    rotation_error_rad: float,
) -> tuple[str, list[str]]:
    """
    Classify one Cartesian IK waypoint as GOOD, EDGE, or FAIL.

    FAIL
        - IK did not converge
        - diagnostic values are non-finite
        - joint limits are violated
        - final pose error exceeds the required tolerance

    EDGE
        IK converged and the pose is valid, but the configuration
        is close to a singularity or joint limit.

    GOOD
        IK converged and all operational-health thresholds are met.
    """
    if not converged:
        return ("FAIL", ["IK did not converge"],)
    
    diagonostics = np.array(
        [
            sigma_min,
            condition_number,
            joint_margin,
            position_error_m,
            rotation_error_rad,
        ],
        dtype=np.float64,
    )

    if not np.all(np.isfinite(diagonostics)):
        return ("FAIL", ["non-finite diagnostic values"],)

    fail_reasons: list[str] = []

    if joint_margin < 0.0:
        fail_reasons.append(
            f"joint-limit violation "
            f"({joint_margin:.6f} rad)"
        )

    if position_error_m > IK_POSITION_TOL_M:
        fail_reasons.append(
            f"position error "
            f"{position_error_m * 1000.0:.4f} mm "
            f"> {IK_POSITION_TOL_M * 1000.0:.4f} mm"
        )

    if rotation_error_rad > IK_ROTATION_TOL_RAD:
        fail_reasons.append(
            f"rotation error "
            f"{np.rad2deg(rotation_error_rad):.5f} deg "
            f"> "
            f"{np.rad2deg(IK_ROTATION_TOL_RAD):.5f} deg"
        )

    if fail_reasons:
        return (
            "FAIL",
            fail_reasons,
        )

    edge_reasons: list[str] = []

    edge_reasons: list[str] = []

    if sigma_min < SIGMA_MIN_GOOD:
        edge_reasons.append(
            f"sigma_min={sigma_min:.5f}"
            f"<{SIGMA_MIN_GOOD:.5f}"
        )

    if condition_number > CONDITION_MAX_GOOD:
        edge_reasons.append(
            f"condition={condition_number:.2f}"
            f">{CONDITION_MAX_GOOD:.2f}"
        )

    if joint_margin < JOINT_MARGIN_GOOD:
        edge_reasons.append(
            f"joint_margin={joint_margin:.5f}"
            f"<{JOINT_MARGIN_GOOD:.5f}"
        )

    if edge_reasons:
        return (
            "EDGE",
            edge_reasons,
        )

    return (
        "GOOD",
        [],
    )

    # ============================================================
# MuJoCo ready-state FK
# ============================================================


def load_ready_state() -> tuple[
    mujoco.MjModel,
    mujoco.MjData,
    np.ndarray,
    np.ndarray,
]:
    """
    Load the combined RM65 + SG100 MuJoCo model and place the RM65
    at Q_TELEOP_READY.

    Returns
    -------
    model
        Loaded combined MuJoCo model.

    data
        MuJoCo simulation data after forward kinematics.

    tcp_position
        sg100_base_link world position, shape (3,).

    tcp_rotation
        sg100_base_link world rotation matrix, shape (3, 3).
    """
    if not MODEL_PATH.exists():
        raise FileNotFoundError(
            f"combined model not found: {MODEL_PATH}"
        )

    model = mujoco.MjModel.from_xml_path(
        str(MODEL_PATH)
    )

    data = mujoco.MjData(model)

    if model.nq != TOTAL_DOF:
        raise RuntimeError(
            f"expected nq={TOTAL_DOF}, got {model.nq}"
        )

    if model.nv != TOTAL_DOF:
        raise RuntimeError(
            f"expected nv={TOTAL_DOF}, got {model.nv}"
        )

    # Start from the model's default state.
    mujoco.mj_resetData(
        model,
        data,
    )

    # RM65 occupies qpos[0:6].
    #
    # SG100 hand joints remain at the model default state.
    # Hand articulation does not change the pose of sg100_base_link.
    data.qpos[:ARM_DOF] = Q_TELEOP_READY

    data.qvel[:] = 0.0

    mujoco.mj_forward(
        model,
        data,
    )

    try:
        tcp_body = data.body(
            TCP_BODY
        )
    except KeyError as exc:
        raise RuntimeError(
            f"TCP body {TCP_BODY!r} not found in model"
        ) from exc

    tcp_position = np.array(
        tcp_body.xpos,
        dtype=np.float64,
        copy=True,
    )

    tcp_rotation = np.array(
        tcp_body.xmat,
        dtype=np.float64,
        copy=True,
    ).reshape(3, 3)

    if tcp_position.shape != (3,):
        raise RuntimeError(
            "unexpected TCP position shape: "
            f"{tcp_position.shape}"
        )

    if tcp_rotation.shape != (3, 3):
        raise RuntimeError(
            "unexpected TCP rotation shape: "
            f"{tcp_rotation.shape}"
        )

    if not np.all(
        np.isfinite(tcp_position)
    ):
        raise RuntimeError(
            "TCP position contains non-finite values"
        )

    if not np.all(
        np.isfinite(tcp_rotation)
    ):
        raise RuntimeError(
            "TCP rotation contains non-finite values"
        )

    return (
        model,
        data,
        tcp_position,
        tcp_rotation,
    )


# ============================================================
# Operational workspace frame
# ============================================================


def build_workspace_frame(
    tcp_position: np.ndarray,
) -> dict[str, np.ndarray]:
    """
    Construct the operational workspace directions around the RM65 base.

    The RM65 base is assumed to be located at the MuJoCo world origin.

    Definitions
    -----------
    outward
        Horizontal direction from the RM65 base toward the current TCP.

    inward
        Opposite of outward, toward the base.

    left
        Horizontal direction 90 degrees counter-clockwise from outward
        in the world XY plane.

    right
        Opposite of left.

    up
        World +Z.

    down
        World -Z.
    """
    tcp_position = np.asarray(
        tcp_position,
        dtype=np.float64,
    )

    if tcp_position.shape != (3,):
        raise ValueError(
            "tcp_position must have shape (3,), "
            f"got {tcp_position.shape}"
        )

    radial_horizontal = np.array(
        [
            tcp_position[0],
            tcp_position[1],
            0.0,
        ],
        dtype=np.float64,
    )

    outward = normalize(
        radial_horizontal
    )

    left = normalize(
        np.array(
            [
                -outward[1],
                 outward[0],
                 0.0,
            ],
            dtype=np.float64,
        )
    )

    up = np.array(
        [0.0, 0.0, 1.0],
        dtype=np.float64,
    )

    directions = {
        "OUTWARD": outward,
        "INWARD": -outward,
        "LEFT": left,
        "RIGHT": -left,
        "UP": up,
        "DOWN": -up,
    }

    return directions

# ============================================================
# Workspace IK waypoint solving
# ============================================================


def make_ik_solver(
    model: mujoco.MjModel,
) -> RM65IKSolver:
    """
    Construct the validated RM65 IK solver for the combined model.

    Solver parameters intentionally use the defaults already validated
    by the existing RM65 IK / teleoperation regression tests.
    """
    return RM65IKSolver(
        model,
        tcp_body_name=TCP_BODY,
    )


def solve_workspace_waypoint(
    *,
    solver: RM65IKSolver,
    q_seed: np.ndarray,
    target_position: np.ndarray,
    target_rotation: np.ndarray,
) -> dict[str, object]:
    """
    Solve one RM65 Cartesian workspace waypoint.

    Parameters
    ----------
    solver
        Existing RM65 IK solver.

    q_seed
        Previous successful RM65 joint configuration.
        Shape: (6,).

    target_position
        Desired sg100_base_link world position.
        Shape: (3,).

    target_rotation
        Desired sg100_base_link world rotation.
        Shape: (3, 3).

    Returns
    -------
    result : dict
        Diagnostic information for this waypoint.

        Important fields:

            classification
            reasons
            q_solution
            q_step_max
            converged
            iterations
            position_error_m
            rotation_error_rad
            sigma_min
            condition_number
            damping
            joint_margin
            limiting_joint_index
            solver_reason
    """
    q_seed = np.asarray(
        q_seed,
        dtype=np.float64,
    )

    target_position = np.asarray(
        target_position,
        dtype=np.float64,
    )

    target_rotation = np.asarray(
        target_rotation,
        dtype=np.float64,
    )

    if q_seed.shape != (ARM_DOF,):
        raise ValueError(
            f"q_seed must have shape ({ARM_DOF},), "
            f"got {q_seed.shape}"
        )

    if target_position.shape != (3,):
        raise ValueError(
            "target_position must have shape (3,), "
            f"got {target_position.shape}"
        )

    if target_rotation.shape != (3, 3):
        raise ValueError(
            "target_rotation must have shape (3,3), "
            f"got {target_rotation.shape}"
        )

    if not np.all(np.isfinite(q_seed)):
        raise ValueError(
            "q_seed contains non-finite values"
        )

    if not np.all(np.isfinite(target_position)):
        raise ValueError(
            "target_position contains non-finite values"
        )

    if not np.all(np.isfinite(target_rotation)):
        raise ValueError(
            "target_rotation contains non-finite values"
        )

    q_solution, ik_result = solver.solve(
        q_seed=q_seed,
        target_position=target_position,
        target_rotation=target_rotation,
    )

    q_solution = np.asarray(
        q_solution,
        dtype=np.float64,
    )

    if q_solution.shape != (ARM_DOF,):
        raise RuntimeError(
            "IK solver returned unexpected q shape: "
            f"{q_solution.shape}"
        )

    if np.all(np.isfinite(q_solution)):
        (
            min_joint_margin,
            limiting_joint_index,
        ) = joint_limit_margin(
            q_solution
        )

        q_step_max = float(
            np.max(
                np.abs(
                    q_solution - q_seed
                )
            )
        )
    else:
        min_joint_margin = float("nan")
        limiting_joint_index = -1
        q_step_max = float("nan")

    classification, reasons = classify_solution(
        converged=bool(
            ik_result.converged
        ),
        sigma_min=float(
            ik_result.sigma_min
        ),
        condition_number=float(
            ik_result.condition_number
        ),
        joint_margin=min_joint_margin,
        position_error_m=float(
            ik_result.position_error
        ),
        rotation_error_rad=float(
            ik_result.rotation_error
        ),
    )

    return {
        "classification": classification,
        "reasons": reasons,
        "q_solution": q_solution,
        "q_step_max": q_step_max,
        "converged": bool(
            ik_result.converged
        ),
        "iterations": int(
            ik_result.iterations
        ),
        "position_error_m": float(
            ik_result.position_error
        ),
        "rotation_error_rad": float(
            ik_result.rotation_error
        ),
        "sigma_min": float(
            ik_result.sigma_min
        ),
        "condition_number": float(
            ik_result.condition_number
        ),
        "damping": float(
            ik_result.damping
        ),
        "joint_margin": min_joint_margin,
        "limiting_joint_index": (
            limiting_joint_index
        ),
        "solver_reason": str(
            ik_result.reason
        ),
    }

# ============================================================
# Continuous directional workspace sweep
# ============================================================


def sweep_direction(
    *,
    solver: RM65IKSolver,
    start_q: np.ndarray,
    start_position: np.ndarray,
    target_rotation: np.ndarray,
    direction: np.ndarray,
    direction_name: str,
    step_m: float = SWEEP_STEP_M,
    max_distance_m: float = MAX_SWEEP_DISTANCE_M,
) -> list[dict[str, object]]:
    """
    Continuously sweep the TCP away from the ready pose along one
    Cartesian direction.

    Each waypoint is solved using the previous successful IK solution
    as the seed.

    EDGE waypoints remain part of the continuous trajectory.

    The sweep stops at the first FAIL waypoint.

    Parameters
    ----------
    solver
        RM65 IK solver.

    start_q
        RM65 configuration at the sweep origin.

    start_position
        TCP world position at the sweep origin.

    target_rotation
        Fixed TCP orientation for the entire sweep.

    direction
        Cartesian sweep direction in world coordinates.

    direction_name
        Human-readable label such as OUTWARD or DOWN.

    step_m
        Distance between consecutive Cartesian waypoints.

    max_distance_m
        Maximum displacement from the sweep origin.

    Returns
    -------
    records
        One dictionary per sampled waypoint.

        The first record corresponds to +step_m, not the ready pose.
    """
    start_q = np.asarray(
        start_q,
        dtype=np.float64,
    )

    start_position = np.asarray(
        start_position,
        dtype=np.float64,
    )

    target_rotation = np.asarray(
        target_rotation,
        dtype=np.float64,
    )

    direction = normalize(
        direction
    )

    if start_q.shape != (ARM_DOF,):
        raise ValueError(
            f"start_q must have shape ({ARM_DOF},), "
            f"got {start_q.shape}"
        )

    if start_position.shape != (3,):
        raise ValueError(
            "start_position must have shape (3,), "
            f"got {start_position.shape}"
        )

    if target_rotation.shape != (3, 3):
        raise ValueError(
            "target_rotation must have shape (3,3), "
            f"got {target_rotation.shape}"
        )

    if step_m <= 0.0:
        raise ValueError(
            "step_m must be positive"
        )

    if max_distance_m < step_m:
        raise ValueError(
            "max_distance_m must be >= step_m"
        )

    if not direction_name:
        raise ValueError(
            "direction_name must not be empty"
        )

    records: list[dict[str, object]] = []

    q_seed = start_q.copy()

    num_steps = int(
        np.floor(
            max_distance_m / step_m
            + 1e-12
        )
    )

    for step_index in range(
        1,
        num_steps + 1,
    ):
        distance_m = (
            step_index * step_m
        )

        target_position = (
            start_position
            + distance_m * direction
        )

        result = solve_workspace_waypoint(
            solver=solver,
            q_seed=q_seed,
            target_position=target_position,
            target_rotation=target_rotation,
        )

        record = dict(
            result
        )

        record.update(
            {
                "direction": direction_name,
                "step_index": step_index,
                "distance_m": float(
                    distance_m
                ),
                "target_position": np.array(
                    target_position,
                    dtype=np.float64,
                    copy=True,
                ),
            }
        )

        records.append(
            record
        )

        # A converged GOOD/EDGE solution becomes the seed for the
        # next Cartesian waypoint.
        if (
            result["classification"]
            != "FAIL"
            and result["converged"]
        ):
            q_seed = np.array(
                result["q_solution"],
                dtype=np.float64,
                copy=True,
            )

        # The first FAIL defines the continuous reachability boundary.
        if result["classification"] == "FAIL":
            break

    return records

# ============================================================
# Directional sweep summary
# ============================================================

# ============================================================
# Tabletop workspace grid
# ============================================================


def build_signed_grid_axis(
    *,
    negative_extent_m: float,
    positive_extent_m: float,
    step_m: float,
) -> np.ndarray:
    """
    Build a signed workspace coordinate axis containing zero.

    Parameters
    ----------
    negative_extent_m
        Available distance in the negative direction.
        Must be non-negative.

    positive_extent_m
        Available distance in the positive direction.
        Must be non-negative.

    step_m
        Grid spacing.

    Returns
    -------
    axis
        Signed coordinates from the negative side through zero
        to the positive side.

    Notes
    -----
    The extents are rounded inward to complete grid steps.

    Example:

        negative_extent = 0.110 m
        positive_extent = 0.115 m
        step = 0.020 m

    produces:

        [-0.10, -0.08, ..., 0.0, ..., +0.08, +0.10]

    rather than sampling outside the validated GOOD envelope.
    """
    negative_extent_m = float(
        negative_extent_m
    )

    positive_extent_m = float(
        positive_extent_m
    )

    step_m = float(
        step_m
    )

    if negative_extent_m < 0.0:
        raise ValueError(
            "negative_extent_m must be non-negative"
        )

    if positive_extent_m < 0.0:
        raise ValueError(
            "positive_extent_m must be non-negative"
        )

    if step_m <= 0.0:
        raise ValueError(
            "step_m must be positive"
        )

    negative_steps = int(
        np.floor(
            negative_extent_m / step_m
            + 1e-12
        )
    )

    positive_steps = int(
        np.floor(
            positive_extent_m / step_m
            + 1e-12
        )
    )

    integer_steps = np.arange(
        -negative_steps,
        positive_steps + 1,
        dtype=np.int64,
    )

    return (
        integer_steps.astype(
            np.float64
        )
        * step_m
    )

# ============================================================
# Sweep classification segments
# ============================================================


def extract_classification_segments(
    records: list[dict[str, object]],
) -> list[dict[str, object]]:
    """
    Group consecutive sweep records with the same classification.

    Example
    -------
    A classification sequence such as:

        GOOD GOOD EDGE EDGE GOOD EDGE FAIL

    becomes:

        GOOD  [first ... last]
        EDGE  [first ... last]
        GOOD  [first ... last]
        EDGE  [first ... last]
        FAIL  [first ... last]

    This is necessary because the operational workspace does not have
    to be monotonic in GOOD / EDGE classification.
    """
    if not records:
        return []

    segments: list[
        dict[str, object]
    ] = []

    segment_start = records[0]
    previous_record = records[0]
    classification = str(
        records[0]["classification"]
    )

    for record in records[1:]:
        current_classification = str(
            record["classification"]
        )

        if current_classification != classification:
            segments.append(
                {
                    "classification": classification,
                    "start_record": segment_start,
                    "end_record": previous_record,
                    "sample_count": (
                        int(
                            previous_record["step_index"]
                        )
                        - int(
                            segment_start["step_index"]
                        )
                        + 1
                    ),
                }
            )

            segment_start = record
            classification = (
                current_classification
            )

        previous_record = record

    # Close the final open segment.
    segments.append(
        {
            "classification": classification,
            "start_record": segment_start,
            "end_record": previous_record,
            "sample_count": (
                int(
                    previous_record["step_index"]
                )
                - int(
                    segment_start["step_index"]
                )
                + 1
            ),
        }
    )

    return segments


def summarize_direction_sweep(
    records: list[dict[str, object]],
) -> dict[str, object]:
    """
    Summarize one continuous directional workspace sweep.

    The summary distinguishes:

        last_good
            Farthest waypoint still classified GOOD.

        first_edge
            First reachable waypoint classified EDGE.

        last_reachable
            Farthest converged GOOD/EDGE waypoint.

        first_fail
            First waypoint classified FAIL.

    This distinction is important because the operational workspace
    boundary and the absolute continuous reachability boundary are
    not the same thing.
    """
    if not records:
        raise ValueError(
            "records must not be empty"
        )

    direction_name = str(
        records[0]["direction"]
    )

    segments = extract_classification_segments(
        records
    )

    initial_good_segment = None

    if (
        segments
        and segments[0]["classification"] == "GOOD"
    ):
        initial_good_segment = segments[0]

    initial_good_end = (
        initial_good_segment["end_record"]
        if initial_good_segment is not None
        else None
    )

    for record in records:
        if str(record["direction"]) != direction_name:
            raise ValueError(
                "all records must belong to the same direction"
            )

    good_records = [
        record
        for record in records
        if record["classification"] == "GOOD"
    ]

    edge_records = [
        record
        for record in records
        if record["classification"] == "EDGE"
    ]

    reachable_records = [
        record
        for record in records
        if (
            record["classification"]
            in ("GOOD", "EDGE")
            and record["converged"]
        )
    ]

    fail_records = [
        record
        for record in records
        if record["classification"] == "FAIL"
    ]

    last_good = (
        good_records[-1]
        if good_records
        else None
    )

    first_edge = (
        edge_records[0]
        if edge_records
        else None
    )

    last_reachable = (
        reachable_records[-1]
        if reachable_records
        else None
    )

    first_fail = (
        fail_records[0]
        if fail_records
        else None
    )

    if reachable_records:
        min_sigma_record = min(
            reachable_records,
            key=lambda record: float(
                record["sigma_min"]
            ),
        )

        max_condition_record = max(
            reachable_records,
            key=lambda record: float(
                record["condition_number"]
            ),
        )

        min_margin_record = min(
            reachable_records,
            key=lambda record: float(
                record["joint_margin"]
            ),
        )

        max_q_step_record = max(
            reachable_records,
            key=lambda record: float(
                record["q_step_max"]
            ),
        )
    else:
        min_sigma_record = None
        max_condition_record = None
        min_margin_record = None
        max_q_step_record = None

    return {
        "direction": direction_name,
        "sample_count": len(records),
        "segments": segments,
        "initial_good_segment": initial_good_segment,
        "initial_good_end": initial_good_end,
        "last_good": last_good,
        "first_edge": first_edge,
        "last_reachable": last_reachable,
        "first_fail": first_fail,
        "min_sigma_record": min_sigma_record,
        "max_condition_record": max_condition_record,
        "min_margin_record": min_margin_record,
        "max_q_step_record": max_q_step_record,
    }

def record_distance_mm(
    record: dict[str, object] | None,
) -> float | None:
    """
    Return a sweep record distance in millimeters.
    """
    if record is None:
        return None

    return float(
        record["distance_m"]
    ) * 1000.0

# ============================================================
# Six-direction operational envelope
# ============================================================


def run_six_direction_sweeps(
    *,
    model: mujoco.MjModel,
    start_q: np.ndarray,
    start_position: np.ndarray,
    target_rotation: np.ndarray,
    directions: dict[str, np.ndarray],
) -> dict[str, dict[str, object]]:
    """
    Run six independent continuous Cartesian sweeps from the same
    validated ready configuration.

    Each direction starts again from:

        start_q
        start_position
        target_rotation

    A fresh IK solver is constructed for every direction so the
    sweeps remain completely independent.

    Returns
    -------
    results
        Dictionary keyed by direction name.

        Each entry contains:

            records
            summary
    """
    results: dict[
        str,
        dict[str, object],
    ] = {}

    for direction_name in SWEEP_DIRECTION_ORDER:
        if direction_name not in directions:
            raise KeyError(
                f"missing workspace direction: "
                f"{direction_name}"
            )

        # Intentionally create an independent solver for each sweep.
        # No state from the previous direction should affect the next.
        solver = make_ik_solver(
            model
        )

        records = sweep_direction(
            solver=solver,
            start_q=start_q,
            start_position=start_position,
            target_rotation=target_rotation,
            direction=directions[
                direction_name
            ],
            direction_name=direction_name,
        )

        summary = summarize_direction_sweep(
            records
        )

        results[direction_name] = {
            "records": records,
            "summary": summary,
        }

    return results

def build_tabletop_grid_axes(
    six_direction_results: dict[
        str,
        dict[str, object],
    ],
    *,
    step_m: float = TABLETOP_GRID_STEP_M,
) -> tuple[np.ndarray, np.ndarray]:
    """
    Build radial and lateral tabletop grid coordinates from the
    initial continuous GOOD envelope.

    Coordinate convention
    ---------------------
    radial > 0
        OUTWARD

    radial < 0
        INWARD

    lateral > 0
        LEFT

    lateral < 0
        RIGHT
    """
    required_directions = (
        "OUTWARD",
        "INWARD",
        "LEFT",
        "RIGHT",
    )

    for direction_name in required_directions:
        if direction_name not in six_direction_results:
            raise KeyError(
                "missing six-direction result: "
                f"{direction_name}"
            )

    def initial_good_extent(
        direction_name: str,
    ) -> float:
        summary = six_direction_results[
            direction_name
        ]["summary"]

        record = summary[
            "initial_good_end"
        ]

        if record is None:
            raise RuntimeError(
                f"{direction_name} has no initial "
                "continuous GOOD segment"
            )

        return float(
            record["distance_m"]
        )

    outward_extent = (
        initial_good_extent(
            "OUTWARD"
        )
    )

    inward_extent = (
        initial_good_extent(
            "INWARD"
        )
    )

    left_extent = (
        initial_good_extent(
            "LEFT"
        )
    )

    right_extent = (
        initial_good_extent(
            "RIGHT"
        )
    )

    radial_axis = build_signed_grid_axis(
        negative_extent_m=inward_extent,
        positive_extent_m=outward_extent,
        step_m=step_m,
    )

    lateral_axis = build_signed_grid_axis(
        negative_extent_m=right_extent,
        positive_extent_m=left_extent,
        step_m=step_m,
    )

    return (
        radial_axis,
        lateral_axis,
    )

def tabletop_grid_point_world(
    *,
    ready_position: np.ndarray,
    directions: dict[str, np.ndarray],
    radial_m: float,
    lateral_m: float,
    vertical_m: float = 0.0,
) -> np.ndarray:
    """
    Convert tabletop workspace coordinates into a MuJoCo world
    TCP position.

    radial:
        positive = OUTWARD
        negative = INWARD

    lateral:
        positive = LEFT
        negative = RIGHT

    vertical:
        positive = UP
        negative = DOWN
    """
    ready_position = np.asarray(
        ready_position,
        dtype=np.float64,
    )

    if ready_position.shape != (3,):
        raise ValueError(
            "ready_position must have shape (3,)"
        )

    required = (
        "OUTWARD",
        "LEFT",
        "UP",
    )

    for direction_name in required:
        if direction_name not in directions:
            raise KeyError(
                f"missing direction: {direction_name}"
            )

    return (
        ready_position
        + float(radial_m)
        * directions["OUTWARD"]
        + float(lateral_m)
        * directions["LEFT"]
        + float(vertical_m)
        * directions["UP"]
    )

# ============================================================
# Straight Cartesian workspace path
# ============================================================


def solve_straight_workspace_path(
    *,
    solver: RM65IKSolver,
    start_q: np.ndarray,
    start_position: np.ndarray,
    target_position: np.ndarray,
    target_rotation: np.ndarray,
    step_m: float = SWEEP_STEP_M,
) -> dict[str, object]:
    """
    Solve a straight Cartesian TCP path from the ready pose to one
    workspace target.

    The TCP orientation remains fixed for the entire path.

    Each successful waypoint becomes the seed for the next waypoint.

    Path classification
    -------------------
    GOOD
        Every sampled waypoint is GOOD.

    EDGE
        The complete path is reachable, but at least one waypoint
        is classified EDGE.

    FAIL
        At least one waypoint is classified FAIL. The path stops
        immediately at the first FAIL.

    Returns
    -------
    result
        Dictionary containing:

            classification
            records
            completed
            path_length_m
            requested_waypoints
            completed_waypoints
            first_edge
            first_fail
            endpoint_record
    """
    start_q = np.asarray(
        start_q,
        dtype=np.float64,
    )

    start_position = np.asarray(
        start_position,
        dtype=np.float64,
    )

    target_position = np.asarray(
        target_position,
        dtype=np.float64,
    )

    target_rotation = np.asarray(
        target_rotation,
        dtype=np.float64,
    )

    if start_q.shape != (ARM_DOF,):
        raise ValueError(
            f"start_q must have shape ({ARM_DOF},), "
            f"got {start_q.shape}"
        )

    if start_position.shape != (3,):
        raise ValueError(
            "start_position must have shape (3,), "
            f"got {start_position.shape}"
        )

    if target_position.shape != (3,):
        raise ValueError(
            "target_position must have shape (3,), "
            f"got {target_position.shape}"
        )

    if target_rotation.shape != (3, 3):
        raise ValueError(
            "target_rotation must have shape (3,3), "
            f"got {target_rotation.shape}"
        )

    if step_m <= 0.0:
        raise ValueError(
            "step_m must be positive"
        )

    displacement = (
        target_position
        - start_position
    )

    path_length_m = float(
        np.linalg.norm(displacement)
    )

    # Zero-motion target: solve the current pose once so the returned
    # structure has the same semantics as every other grid point.
    if path_length_m < 1e-12:
        waypoint = solve_workspace_waypoint(
            solver=solver,
            q_seed=start_q,
            target_position=start_position,
            target_rotation=target_rotation,
        )

        waypoint = dict(
            waypoint
        )

        waypoint.update(
            {
                "waypoint_index": 0,
                "path_fraction": 1.0,
                "path_distance_m": 0.0,
                "target_position": np.array(
                    start_position,
                    dtype=np.float64,
                    copy=True,
                ),
            }
        )

        return {
            "classification": waypoint[
                "classification"
            ],
            "records": [waypoint],
            "completed": (
                waypoint["classification"]
                != "FAIL"
            ),
            "path_length_m": 0.0,
            "requested_waypoints": 1,
            "completed_waypoints": (
                1
                if waypoint["classification"] != "FAIL"
                else 0
            ),
            "first_edge": (
                waypoint
                if waypoint["classification"] == "EDGE"
                else None
            ),
            "first_fail": (
                waypoint
                if waypoint["classification"] == "FAIL"
                else None
            ),
            "endpoint_record": (
                waypoint
                if waypoint["classification"] != "FAIL"
                else None
            ),
        }

    num_waypoints = int(
        np.ceil(
            path_length_m / step_m
        )
    )

    records: list[
        dict[str, object]
    ] = []

    q_seed = start_q.copy()

    first_edge = None
    first_fail = None

    for waypoint_index in range(
        1,
        num_waypoints + 1,
    ):
        path_fraction = (
            waypoint_index
            / num_waypoints
        )

        waypoint_position = (
            start_position
            + path_fraction * displacement
        )

        waypoint = solve_workspace_waypoint(
            solver=solver,
            q_seed=q_seed,
            target_position=waypoint_position,
            target_rotation=target_rotation,
        )

        waypoint = dict(
            waypoint
        )

        waypoint.update(
            {
                "waypoint_index": waypoint_index,
                "path_fraction": float(
                    path_fraction
                ),
                "path_distance_m": float(
                    path_fraction
                    * path_length_m
                ),
                "target_position": np.array(
                    waypoint_position,
                    dtype=np.float64,
                    copy=True,
                ),
            }
        )

        records.append(
            waypoint
        )

        if (
            first_edge is None
            and waypoint["classification"] == "EDGE"
        ):
            first_edge = waypoint

        if waypoint["classification"] == "FAIL":
            first_fail = waypoint
            break

        q_seed = np.array(
            waypoint["q_solution"],
            dtype=np.float64,
            copy=True,
        )

    completed = (
        first_fail is None
        and len(records) == num_waypoints
    )

    if first_fail is not None:
        classification = "FAIL"
    elif first_edge is not None:
        classification = "EDGE"
    else:
        classification = "GOOD"

    endpoint_record = (
        records[-1]
        if completed
        else None
    )

    return {
        "classification": classification,
        "records": records,
        "completed": completed,
        "path_length_m": path_length_m,
        "requested_waypoints": num_waypoints,
        "completed_waypoints": (
            len(records)
            if completed
            else len(records) - 1
        ),
        "first_edge": first_edge,
        "first_fail": first_fail,
        "endpoint_record": endpoint_record,
    }

# ============================================================
# Tabletop grid evaluation
# ============================================================


def evaluate_tabletop_grid_point(
    *,
    model: mujoco.MjModel,
    ready_q: np.ndarray,
    ready_position: np.ndarray,
    ready_rotation: np.ndarray,
    directions: dict[str, np.ndarray],
    radial_m: float,
    lateral_m: float,
) -> dict[str, object]:
    """
    Evaluate one tabletop grid point.

    Every grid point is evaluated independently from the same
    teleoperation-ready configuration.

    The TCP follows a straight Cartesian path from READY to the
    requested radial/lateral target while keeping R_ready fixed.
    """
    target_position = tabletop_grid_point_world(
        ready_position=ready_position,
        directions=directions,
        radial_m=radial_m,
        lateral_m=lateral_m,
        vertical_m=0.0,
    )

    # Each tabletop target gets a fresh solver so that no state from
    # a previously evaluated grid point can influence this result.
    solver = make_ik_solver(
        model
    )

    path_result = solve_straight_workspace_path(
        solver=solver,
        start_q=ready_q,
        start_position=ready_position,
        target_position=target_position,
        target_rotation=ready_rotation,
    )

    reachable_records = [
        row
        for row in path_result["records"]
        if (
            row["classification"] != "FAIL"
            and row["converged"]
        )
    ]

    if reachable_records:
        min_sigma_record = min(
            reachable_records,
            key=lambda row: float(
                row["sigma_min"]
            ),
        )

        max_condition_record = max(
            reachable_records,
            key=lambda row: float(
                row["condition_number"]
            ),
        )

        min_margin_record = min(
            reachable_records,
            key=lambda row: float(
                row["joint_margin"]
            ),
        )

        max_q_step_record = max(
            reachable_records,
            key=lambda row: float(
                row["q_step_max"]
            ),
        )

        min_sigma = float(
            min_sigma_record["sigma_min"]
        )

        max_condition = float(
            max_condition_record[
                "condition_number"
            ]
        )

        min_joint_margin = float(
            min_margin_record[
                "joint_margin"
            ]
        )

        limiting_joint_index = int(
            min_margin_record[
                "limiting_joint_index"
            ]
        )

        max_q_step = float(
            max_q_step_record[
                "q_step_max"
            ]
        )
    else:
        min_sigma = float("nan")
        max_condition = float("nan")
        min_joint_margin = float("nan")
        limiting_joint_index = -1
        max_q_step = float("nan")

    first_edge = path_result[
        "first_edge"
    ]

    first_fail = path_result[
        "first_fail"
    ]

    endpoint = path_result[
        "endpoint_record"
    ]

    return {
        "radial_m": float(radial_m),
        "lateral_m": float(lateral_m),
        "target_position": np.array(
            target_position,
            dtype=np.float64,
            copy=True,
        ),
        "classification": path_result[
            "classification"
        ],
        "completed": bool(
            path_result["completed"]
        ),
        "path_length_m": float(
            path_result["path_length_m"]
        ),
        "requested_waypoints": int(
            path_result[
                "requested_waypoints"
            ]
        ),
        "completed_waypoints": int(
            path_result[
                "completed_waypoints"
            ]
        ),
        "first_edge": first_edge,
        "first_fail": first_fail,
        "endpoint_record": endpoint,
        "min_sigma": min_sigma,
        "max_condition": max_condition,
        "min_joint_margin": min_joint_margin,
        "limiting_joint_index": (
            limiting_joint_index
        ),
        "max_q_step": max_q_step,
        "path_result": path_result,
    }

def evaluate_tabletop_grid(
    *,
    model: mujoco.MjModel,
    ready_q: np.ndarray,
    ready_position: np.ndarray,
    ready_rotation: np.ndarray,
    directions: dict[str, np.ndarray],
    radial_axis: np.ndarray,
    lateral_axis: np.ndarray,
) -> list[dict[str, object]]:
    """
    Evaluate all radial × lateral tabletop targets independently.

    Returns one flat record list.

    Ordering:
        radial outer loop
        lateral inner loop
    """
    radial_axis = np.asarray(
        radial_axis,
        dtype=np.float64,
    )

    lateral_axis = np.asarray(
        lateral_axis,
        dtype=np.float64,
    )

    if radial_axis.ndim != 1:
        raise ValueError(
            "radial_axis must be 1D"
        )

    if lateral_axis.ndim != 1:
        raise ValueError(
            "lateral_axis must be 1D"
        )

    if radial_axis.size == 0:
        raise ValueError(
            "radial_axis must not be empty"
        )

    if lateral_axis.size == 0:
        raise ValueError(
            "lateral_axis must not be empty"
        )

    records: list[
        dict[str, object]
    ] = []

    for radial_m in radial_axis:
        for lateral_m in lateral_axis:
            record = evaluate_tabletop_grid_point(
                model=model,
                ready_q=ready_q,
                ready_position=ready_position,
                ready_rotation=ready_rotation,
                directions=directions,
                radial_m=float(radial_m),
                lateral_m=float(lateral_m),
            )

            records.append(
                record
            )

    return records

def find_tabletop_grid_record(
    records: list[dict[str, object]],
    *,
    radial_m: float,
    lateral_m: float,
    atol: float = 1e-9,
) -> dict[str, object]:
    """
    Find one evaluated tabletop record by signed workspace coordinates.
    """
    matches = [
        record
        for record in records
        if (
            abs(
                float(record["radial_m"])
                - float(radial_m)
            )
            <= atol
            and abs(
                float(record["lateral_m"])
                - float(lateral_m)
            )
            <= atol
        )
    ]

    if len(matches) != 1:
        raise RuntimeError(
            "expected exactly one grid record for "
            f"radial={radial_m}, "
            f"lateral={lateral_m}; "
            f"found {len(matches)}"
        )

    return matches[0]

# ============================================================
# Tabletop grid summary / visualization
# ============================================================


def summarize_tabletop_grid(
    records: list[dict[str, object]],
) -> dict[str, object]:
    """
    Summarize a complete tabletop workspace grid.

    The classification of every grid point represents the complete
    straight Cartesian path from Q_TELEOP_READY to that target.

    GOOD
        Entire path remained GOOD.

    EDGE
        Entire path was reachable, but at least one waypoint entered
        EDGE.

    FAIL
        Straight path encountered at least one failed waypoint.
    """
    if not records:
        raise ValueError(
            "records must not be empty"
        )

    good_records = [
        row
        for row in records
        if row["classification"] == "GOOD"
    ]

    edge_records = [
        row
        for row in records
        if row["classification"] == "EDGE"
    ]

    fail_records = [
        row
        for row in records
        if row["classification"] == "FAIL"
    ]

    reachable_records = (
        good_records
        + edge_records
    )

    if reachable_records:
        min_sigma_record = min(
            reachable_records,
            key=lambda row: float(
                row["min_sigma"]
            ),
        )

        max_condition_record = max(
            reachable_records,
            key=lambda row: float(
                row["max_condition"]
            ),
        )

        min_margin_record = min(
            reachable_records,
            key=lambda row: float(
                row["min_joint_margin"]
            ),
        )

        max_q_step_record = max(
            reachable_records,
            key=lambda row: float(
                row["max_q_step"]
            ),
        )
    else:
        min_sigma_record = None
        max_condition_record = None
        min_margin_record = None
        max_q_step_record = None

    return {
        "total_count": len(records),
        "good_count": len(good_records),
        "edge_count": len(edge_records),
        "fail_count": len(fail_records),
        "good_records": good_records,
        "edge_records": edge_records,
        "fail_records": fail_records,
        "min_sigma_record": min_sigma_record,
        "max_condition_record": max_condition_record,
        "min_margin_record": min_margin_record,
        "max_q_step_record": max_q_step_record,
    }

def print_tabletop_ascii_map(
    *,
    records: list[dict[str, object]],
    radial_axis: np.ndarray,
    lateral_axis: np.ndarray,
) -> None:
    """
    Print a top-view tabletop operational workspace map.

    Display convention:

                         OUTWARD
                            ↑

            LEFT  ← workspace →  RIGHT

                            ↓
                          INWARD

    Symbols
    -------
    R
        Q_TELEOP_READY origin.

    G
        Complete straight path GOOD.

    E
        Reachable but path enters EDGE.

    X
        Straight path FAIL.
    """
    radial_axis = np.asarray(
        radial_axis,
        dtype=np.float64,
    )

    lateral_axis = np.asarray(
        lateral_axis,
        dtype=np.float64,
    )

    symbols = {
        "GOOD": "G",
        "EDGE": "E",
        "FAIL": "X",
    }

    print(
        "===== TABLETOP OPERATIONAL WORKSPACE MAP ====="
    )

    print()
    print(
        "                         OUTWARD +radial"
    )
    print(
        "                                ↑"
    )

    print()
    print(
        "          LEFT +lateral                    "
        "-lateral RIGHT"
    )
    print()

    # Positive radial is displayed at the top.
    for radial_m in radial_axis[::-1]:
        cells: list[str] = []

        # Positive lateral is displayed on the left.
        for lateral_m in lateral_axis[::-1]:
            row = find_tabletop_grid_record(
                records,
                radial_m=float(
                    radial_m
                ),
                lateral_m=float(
                    lateral_m
                ),
            )

            symbol = symbols[
                row["classification"]
            ]

            if (
                abs(radial_m) < 1e-12
                and abs(lateral_m) < 1e-12
            ):
                symbol = "R"

            cells.append(
                symbol
            )

        print(
            f'{radial_m * 1000.0:+6.0f} mm  '
            + " ".join(cells)
        )

    print()

    lateral_labels = (
        lateral_axis[::-1]
        * 1000.0
    )

    print(
        "lateral [mm], left → right:"
    )

    print(
        " ".join(
            f"{value:+.0f}"
            for value in lateral_labels
        )
    )

    print()
    print("Legend:")
    print("  R = READY")
    print("  G = GOOD")
    print("  E = EDGE")
    print("  X = FAIL")

def find_largest_good_rectangle_containing_ready(
    *,
    records: list[dict[str, object]],
    radial_axis: np.ndarray,
    lateral_axis: np.ndarray,
) -> dict[str, object]:
    """
    Find the largest axis-aligned sampled rectangle that:

        1. contains the READY coordinate (0, 0), and
        2. contains only GOOD grid points.

    The search operates on the evaluated discrete tabletop grid.

    Returns
    -------
    result
        Dictionary containing:

            radial_min_m
            radial_max_m
            lateral_min_m
            lateral_max_m

            radial_span_m
            lateral_span_m
            area_m2

            radial_sample_count
            lateral_sample_count
            total_sample_count
    """
    radial_axis = np.asarray(
        radial_axis,
        dtype=np.float64,
    )

    lateral_axis = np.asarray(
        lateral_axis,
        dtype=np.float64,
    )

    if radial_axis.ndim != 1:
        raise ValueError(
            "radial_axis must be 1D"
        )

    if lateral_axis.ndim != 1:
        raise ValueError(
            "lateral_axis must be 1D"
        )

    if not np.any(
        np.isclose(
            radial_axis,
            0.0,
        )
    ):
        raise ValueError(
            "radial_axis must contain zero"
        )

    if not np.any(
        np.isclose(
            lateral_axis,
            0.0,
        )
    ):
        raise ValueError(
            "lateral_axis must contain zero"
        )

    radial_values = np.sort(
        radial_axis
    )

    lateral_values = np.sort(
        lateral_axis
    )

    best: dict[str, object] | None = None

    for radial_start_index in range(
        len(radial_values)
    ):
        radial_min = float(
            radial_values[
                radial_start_index
            ]
        )

        if radial_min > 0.0:
            continue

        for radial_end_index in range(
            radial_start_index,
            len(radial_values),
        ):
            radial_max = float(
                radial_values[
                    radial_end_index
                ]
            )

            if radial_max < 0.0:
                continue

            for lateral_start_index in range(
                len(lateral_values)
            ):
                lateral_min = float(
                    lateral_values[
                        lateral_start_index
                    ]
                )

                if lateral_min > 0.0:
                    continue

                for lateral_end_index in range(
                    lateral_start_index,
                    len(lateral_values),
                ):
                    lateral_max = float(
                        lateral_values[
                            lateral_end_index
                        ]
                    )

                    if lateral_max < 0.0:
                        continue

                    rectangle_is_good = True

                    for radial_m in radial_values[
                        radial_start_index:
                        radial_end_index + 1
                    ]:
                        for lateral_m in lateral_values[
                            lateral_start_index:
                            lateral_end_index + 1
                        ]:
                            record = (
                                find_tabletop_grid_record(
                                    records,
                                    radial_m=float(
                                        radial_m
                                    ),
                                    lateral_m=float(
                                        lateral_m
                                    ),
                                )
                            )

                            if (
                                record["classification"]
                                != "GOOD"
                            ):
                                rectangle_is_good = False
                                break

                        if not rectangle_is_good:
                            break

                    if not rectangle_is_good:
                        continue

                    radial_span = (
                        radial_max
                        - radial_min
                    )

                    lateral_span = (
                        lateral_max
                        - lateral_min
                    )

                    area_m2 = (
                        radial_span
                        * lateral_span
                    )

                    radial_sample_count = (
                        radial_end_index
                        - radial_start_index
                        + 1
                    )

                    lateral_sample_count = (
                        lateral_end_index
                        - lateral_start_index
                        + 1
                    )

                    candidate = {
                        "radial_min_m": radial_min,
                        "radial_max_m": radial_max,
                        "lateral_min_m": lateral_min,
                        "lateral_max_m": lateral_max,
                        "radial_span_m": radial_span,
                        "lateral_span_m": lateral_span,
                        "area_m2": area_m2,
                        "radial_sample_count": (
                            radial_sample_count
                        ),
                        "lateral_sample_count": (
                            lateral_sample_count
                        ),
                        "total_sample_count": (
                            radial_sample_count
                            * lateral_sample_count
                        ),
                    }

                    if best is None:
                        best = candidate
                        continue

                    candidate_area = float(
                        candidate["area_m2"]
                    )

                    best_area = float(
                        best["area_m2"]
                    )

                    if candidate_area > (
                        best_area + 1e-12
                    ):
                        best = candidate

    if best is None:
        raise RuntimeError(
            "no all-GOOD rectangle containing READY found"
        )

    return best

# ============================================================
# Vertical-clearance anchor points
# ============================================================


def build_vertical_clearance_anchors(
    rectangle: dict[str, object],
) -> dict[str, tuple[float, float]]:
    """
    Build representative radial/lateral anchor points inside the
    all-GOOD tabletop rectangle.

    The READY workspace origin (0, 0) is used as the central anchor.

    Returns
    -------
    anchors
        Mapping:

            name -> (radial_m, lateral_m)

        containing:

            READY
            OUTWARD_MID
            INWARD_MID
            LEFT_MID
            RIGHT_MID
            OUTWARD_LEFT
            OUTWARD_RIGHT
            INWARD_LEFT
            INWARD_RIGHT
    """
    radial_min = float(
        rectangle["radial_min_m"]
    )

    radial_max = float(
        rectangle["radial_max_m"]
    )

    lateral_min = float(
        rectangle["lateral_min_m"]
    )

    lateral_max = float(
        rectangle["lateral_max_m"]
    )

    if not (
        radial_min <= 0.0 <= radial_max
    ):
        raise ValueError(
            "rectangle must contain radial zero"
        )

    if not (
        lateral_min <= 0.0 <= lateral_max
    ):
        raise ValueError(
            "rectangle must contain lateral zero"
        )

    return {
        "READY": (
            0.0,
            0.0,
        ),
        "OUTWARD_MID": (
            radial_max,
            0.0,
        ),
        "INWARD_MID": (
            radial_min,
            0.0,
        ),
        "LEFT_MID": (
            0.0,
            lateral_max,
        ),
        "RIGHT_MID": (
            0.0,
            lateral_min,
        ),
        "OUTWARD_LEFT": (
            radial_max,
            lateral_max,
        ),
        "OUTWARD_RIGHT": (
            radial_max,
            lateral_min,
        ),
        "INWARD_LEFT": (
            radial_min,
            lateral_max,
        ),
        "INWARD_RIGHT": (
            radial_min,
            lateral_min,
        ),
    }

# ============================================================
# Vertical-clearance evaluation
# ============================================================


def evaluate_vertical_clearance_anchor(
    *,
    model: mujoco.MjModel,
    ready_q: np.ndarray,
    ready_position: np.ndarray,
    ready_rotation: np.ndarray,
    directions: dict[str, np.ndarray],
    anchor_name: str,
    radial_m: float,
    lateral_m: float,
    step_m: float = SWEEP_STEP_M,
    max_down_m: float = MAX_SWEEP_DISTANCE_M,
) -> dict[str, object]:
    """
    Evaluate downward continuous workspace clearance at one tabletop
    anchor.

    Procedure
    ---------
    1. Start from Q_TELEOP_READY.
    2. Follow a straight horizontal Cartesian path to the anchor.
    3. Require that the complete horizontal path is GOOD.
    4. Use the anchor endpoint joint configuration as the seed.
    5. Sweep downward in 5 mm Cartesian increments while keeping the
       original ready TCP orientation fixed.

    Returns
    -------
    result
        Dictionary containing the horizontal path and the complete
        downward sweep diagnostics.
    """
    if not anchor_name:
        raise ValueError(
            "anchor_name must not be empty"
        )

    anchor_position = tabletop_grid_point_world(
        ready_position=ready_position,
        directions=directions,
        radial_m=radial_m,
        lateral_m=lateral_m,
        vertical_m=0.0,
    )

    # --------------------------------------------------------
    # Phase 1: READY -> horizontal anchor
    # --------------------------------------------------------

    horizontal_solver = make_ik_solver(
        model
    )

    horizontal_path = solve_straight_workspace_path(
        solver=horizontal_solver,
        start_q=ready_q,
        start_position=ready_position,
        target_position=anchor_position,
        target_rotation=ready_rotation,
        step_m=step_m,
    )

    if not horizontal_path["completed"]:
        raise RuntimeError(
            f"{anchor_name}: horizontal path did not complete"
        )

    if horizontal_path["classification"] != "GOOD":
        raise RuntimeError(
            f"{anchor_name}: expected horizontal GOOD path, "
            f'got {horizontal_path["classification"]}'
        )

    horizontal_endpoint = (
        horizontal_path["endpoint_record"]
    )

    if horizontal_endpoint is None:
        raise RuntimeError(
            f"{anchor_name}: missing horizontal endpoint"
        )

    q_anchor = np.array(
        horizontal_endpoint["q_solution"],
        dtype=np.float64,
        copy=True,
    )

    # --------------------------------------------------------
    # Phase 2: anchor -> DOWN
    # --------------------------------------------------------

    down_solver = make_ik_solver(
        model
    )

    down_records = sweep_direction(
        solver=down_solver,
        start_q=q_anchor,
        start_position=anchor_position,
        target_rotation=ready_rotation,
        direction=directions["DOWN"],
        direction_name=f"{anchor_name}_DOWN",
        step_m=step_m,
        max_distance_m=max_down_m,
    )

    down_summary = summarize_direction_sweep(
        down_records
    )

    return {
        "anchor_name": anchor_name,
        "radial_m": float(radial_m),
        "lateral_m": float(lateral_m),
        "anchor_position": np.array(
            anchor_position,
            dtype=np.float64,
            copy=True,
        ),
        "q_anchor": q_anchor,
        "horizontal_path": horizontal_path,
        "horizontal_path_length_m": float(
            horizontal_path["path_length_m"]
        ),
        "down_records": down_records,
        "down_summary": down_summary,
    }

def evaluate_vertical_clearance_anchors(
    *,
    model: mujoco.MjModel,
    ready_q: np.ndarray,
    ready_position: np.ndarray,
    ready_rotation: np.ndarray,
    directions: dict[str, np.ndarray],
    anchors: dict[str, tuple[float, float]],
    step_m: float = SWEEP_STEP_M,
    max_down_m: float = MAX_SWEEP_DISTANCE_M,
) -> dict[str, dict[str, object]]:
    """
    Evaluate downward continuous workspace clearance at all selected
    tabletop anchors.

    Every anchor is evaluated independently from Q_TELEOP_READY.

    Returns
    -------
    results
        Mapping:

            anchor_name -> vertical-clearance result
    """
    if not anchors:
        raise ValueError(
            "anchors must not be empty"
        )

    results: dict[
        str,
        dict[str, object],
    ] = {}

    for anchor_name, (
        radial_m,
        lateral_m,
    ) in anchors.items():
        result = evaluate_vertical_clearance_anchor(
            model=model,
            ready_q=ready_q,
            ready_position=ready_position,
            ready_rotation=ready_rotation,
            directions=directions,
            anchor_name=anchor_name,
            radial_m=radial_m,
            lateral_m=lateral_m,
            step_m=step_m,
            max_down_m=max_down_m,
        )

        results[
            anchor_name
        ] = result

    return results

def record_limiting_joint_name(
    record: dict[str, object] | None,
) -> str:
    """
    Return the human-readable limiting RM65 joint name for a record.
    """
    if record is None:
        return "--"

    joint_index = int(
        record["limiting_joint_index"]
    )

    if joint_index < 0:
        return "--"

    return f"J{joint_index + 1}"

# ============================================================
# Initial continuous-GOOD downward clearance
# ============================================================


def measure_initial_good_down_clearance(
    *,
    model: mujoco.MjModel,
    ready_q: np.ndarray,
    ready_position: np.ndarray,
    ready_rotation: np.ndarray,
    directions: dict[str, np.ndarray],
    radial_m: float,
    lateral_m: float,
    step_m: float = SWEEP_STEP_M,
    max_down_m: float = MAX_SWEEP_DISTANCE_M,
) -> dict[str, object]:
    """
    Measure the initial continuous GOOD downward clearance at one
    radial/lateral tabletop point.

    Procedure
    ---------
    1. Move from Q_TELEOP_READY to the XY anchor using the validated
       straight Cartesian horizontal path.
    2. Require the complete horizontal path to be GOOD.
    3. Starting from the resulting q_anchor, move downward in Cartesian
       increments while keeping R_ready fixed.
    4. Stop immediately at the first EDGE or FAIL waypoint.

    The returned GOOD clearance is therefore the maximum sampled DOWN
    distance for which the complete motion from READY remains GOOD.
    """
    anchor_position = tabletop_grid_point_world(
        ready_position=ready_position,
        directions=directions,
        radial_m=radial_m,
        lateral_m=lateral_m,
        vertical_m=0.0,
    )

    # --------------------------------------------------------
    # READY -> XY anchor
    # --------------------------------------------------------

    horizontal_solver = make_ik_solver(
        model
    )

    horizontal_path = solve_straight_workspace_path(
        solver=horizontal_solver,
        start_q=ready_q,
        start_position=ready_position,
        target_position=anchor_position,
        target_rotation=ready_rotation,
        step_m=step_m,
    )

    if not horizontal_path["completed"]:
        raise RuntimeError(
            "horizontal path did not complete for "
            f"radial={radial_m}, lateral={lateral_m}"
        )

    if horizontal_path["classification"] != "GOOD":
        raise RuntimeError(
            "horizontal path is not GOOD for "
            f"radial={radial_m}, lateral={lateral_m}: "
            f'{horizontal_path["classification"]}'
        )

    endpoint = horizontal_path[
        "endpoint_record"
    ]

    if endpoint is None:
        raise RuntimeError(
            "horizontal endpoint is missing"
        )

    q_seed = np.array(
        endpoint["q_solution"],
        dtype=np.float64,
        copy=True,
    )

    # --------------------------------------------------------
    # XY anchor -> DOWN
    # --------------------------------------------------------

    down_solver = make_ik_solver(
        model
    )

    num_steps = int(
        np.floor(
            max_down_m / step_m
            + 1e-12
        )
    )

    records: list[
        dict[str, object]
    ] = []

    last_good = None
    first_non_good = None

    for step_index in range(
        1,
        num_steps + 1,
    ):
        distance_m = (
            step_index * step_m
        )

        target_position = (
            anchor_position
            + distance_m
            * directions["DOWN"]
        )

        result = solve_workspace_waypoint(
            solver=down_solver,
            q_seed=q_seed,
            target_position=target_position,
            target_rotation=ready_rotation,
        )

        record = dict(
            result
        )

        record.update(
            {
                "step_index": step_index,
                "distance_m": float(
                    distance_m
                ),
                "target_position": np.array(
                    target_position,
                    dtype=np.float64,
                    copy=True,
                ),
            }
        )

        records.append(
            record
        )

        if record["classification"] == "GOOD":
            last_good = record

            q_seed = np.array(
                record["q_solution"],
                dtype=np.float64,
                copy=True,
            )

            continue

        first_non_good = record
        break

    good_clearance_m = (
        float(
            last_good["distance_m"]
        )
        if last_good is not None
        else 0.0
    )

    return {
        "radial_m": float(radial_m),
        "lateral_m": float(lateral_m),
        "anchor_position": np.array(
            anchor_position,
            dtype=np.float64,
            copy=True,
        ),
        "q_anchor": np.array(
            endpoint["q_solution"],
            dtype=np.float64,
            copy=True,
        ),
        "horizontal_path": horizontal_path,
        "good_clearance_m": good_clearance_m,
        "last_good": last_good,
        "first_non_good": first_non_good,
        "down_records": records,
    }

def evaluate_vertical_good_clearance_map(
    *,
    model: mujoco.MjModel,
    ready_q: np.ndarray,
    ready_position: np.ndarray,
    ready_rotation: np.ndarray,
    directions: dict[str, np.ndarray],
    tabletop_records: list[dict[str, object]],
    radial_axis: np.ndarray,
    lateral_axis: np.ndarray,
    radial_min_m: float,
    radial_max_m: float,
    lateral_min_m: float,
    lateral_max_m: float,
) -> list[dict[str, object]]:
    """
    Measure initial continuous GOOD downward clearance at every GOOD
    XY grid point inside the selected tabletop rectangle.
    """
    results: list[
        dict[str, object]
    ] = []

    for radial_m in radial_axis:
        radial_m = float(
            radial_m
        )

        if (
            radial_m < radial_min_m - 1e-12
            or radial_m > radial_max_m + 1e-12
        ):
            continue

        for lateral_m in lateral_axis:
            lateral_m = float(
                lateral_m
            )

            if (
                lateral_m < lateral_min_m - 1e-12
                or lateral_m > lateral_max_m + 1e-12
            ):
                continue

            tabletop_record = (
                find_tabletop_grid_record(
                    tabletop_records,
                    radial_m=radial_m,
                    lateral_m=lateral_m,
                )
            )

            if (
                tabletop_record["classification"]
                != "GOOD"
            ):
                raise RuntimeError(
                    "selected vertical-clearance rectangle "
                    "contains a non-GOOD horizontal point"
                )

            result = (
                measure_initial_good_down_clearance(
                    model=model,
                    ready_q=ready_q,
                    ready_position=ready_position,
                    ready_rotation=ready_rotation,
                    directions=directions,
                    radial_m=radial_m,
                    lateral_m=lateral_m,
                )
            )

            results.append(
                result
            )

    return results

def find_vertical_clearance_record(
    records: list[dict[str, object]],
    *,
    radial_m: float,
    lateral_m: float,
    atol: float = 1e-9,
) -> dict[str, object]:
    """
    Find one vertical-clearance map record.
    """
    matches = [
        record
        for record in records
        if (
            abs(
                float(record["radial_m"])
                - float(radial_m)
            )
            <= atol
            and abs(
                float(record["lateral_m"])
                - float(lateral_m)
            )
            <= atol
        )
    ]

    if len(matches) != 1:
        raise RuntimeError(
            "expected exactly one vertical-clearance record; "
            f"found {len(matches)}"
        )

    return matches[0]

def print_vertical_good_clearance_map(
    *,
    records: list[dict[str, object]],
    radial_values: np.ndarray,
    lateral_values: np.ndarray,
) -> None:
    """
    Print continuous GOOD downward clearance in millimeters.

    Positive radial is displayed upward.
    Positive lateral is displayed on the left.
    """
    print(
        "===== CONTINUOUS GOOD DOWN CLEARANCE MAP [mm] ====="
    )

    print()
    print(
        "                         OUTWARD"
    )
    print(
        "                            ↑"
    )
    print()

    for radial_m in radial_values[::-1]:

        cells: list[str] = []

        for lateral_m in lateral_values[::-1]:

            record = find_vertical_clearance_record(
                records,
                radial_m=float(radial_m),
                lateral_m=float(lateral_m),
            )

            clearance_mm = (
                float(
                    record["good_clearance_m"]
                )
                * 1000.0
            )

            cells.append(
                f"{clearance_mm:4.0f}"
            )

        print(
            f'{radial_m * 1000.0:+5.0f}  '
            + " ".join(cells)
        )

    print()
    print(
        "lateral [mm], LEFT → RIGHT:"
    )

    print(
        " ".join(
            f"{value * 1000.0:+.0f}"
            for value in lateral_values[::-1]
        )
    )

def find_largest_vertical_clearance_rectangle(
    *,
    records: list[dict[str, object]],
    radial_values: np.ndarray,
    lateral_values: np.ndarray,
    minimum_clearance_m: float,
) -> dict[str, object]:
    """
    Find the largest sampled axis-aligned rectangle in which every
    XY point has at least the requested initial continuous GOOD
    downward clearance.

    The rectangle does NOT need to geometrically contain READY.

    Every included XY point has already been independently validated
    from Q_TELEOP_READY by the vertical-clearance map evaluation.
    """
    radial_values = np.sort(
        np.asarray(
            radial_values,
            dtype=np.float64,
        )
    )

    lateral_values = np.sort(
        np.asarray(
            lateral_values,
            dtype=np.float64,
        )
    )

    if radial_values.ndim != 1:
        raise ValueError(
            "radial_values must be 1D"
        )

    if lateral_values.ndim != 1:
        raise ValueError(
            "lateral_values must be 1D"
        )

    if minimum_clearance_m < 0.0:
        raise ValueError(
            "minimum_clearance_m must be non-negative"
        )

    best = None

    for radial_start_index in range(
        len(radial_values)
    ):
        for radial_end_index in range(
            radial_start_index,
            len(radial_values),
        ):
            radial_min = float(
                radial_values[
                    radial_start_index
                ]
            )

            radial_max = float(
                radial_values[
                    radial_end_index
                ]
            )

            for lateral_start_index in range(
                len(lateral_values)
            ):
                for lateral_end_index in range(
                    lateral_start_index,
                    len(lateral_values),
                ):
                    lateral_min = float(
                        lateral_values[
                            lateral_start_index
                        ]
                    )

                    lateral_max = float(
                        lateral_values[
                            lateral_end_index
                        ]
                    )

                    rectangle_records = []

                    valid = True

                    for radial_m in radial_values[
                        radial_start_index:
                        radial_end_index + 1
                    ]:
                        for lateral_m in lateral_values[
                            lateral_start_index:
                            lateral_end_index + 1
                        ]:
                            record = (
                                find_vertical_clearance_record(
                                    records,
                                    radial_m=float(
                                        radial_m
                                    ),
                                    lateral_m=float(
                                        lateral_m
                                    ),
                                )
                            )

                            if (
                                float(
                                    record[
                                        "good_clearance_m"
                                    ]
                                )
                                + 1e-12
                                < minimum_clearance_m
                            ):
                                valid = False
                                break

                            rectangle_records.append(
                                record
                            )

                        if not valid:
                            break

                    if not valid:
                        continue

                    radial_span_m = (
                        radial_max
                        - radial_min
                    )

                    lateral_span_m = (
                        lateral_max
                        - lateral_min
                    )

                    area_m2 = (
                        radial_span_m
                        * lateral_span_m
                    )

                    radial_sample_count = (
                        radial_end_index
                        - radial_start_index
                        + 1
                    )

                    lateral_sample_count = (
                        lateral_end_index
                        - lateral_start_index
                        + 1
                    )

                    actual_min_clearance_m = min(
                        float(
                            row[
                                "good_clearance_m"
                            ]
                        )
                        for row in rectangle_records
                    )

                    candidate = {
                        "minimum_required_clearance_m": float(
                            minimum_clearance_m
                        ),
                        "actual_min_clearance_m": (
                            actual_min_clearance_m
                        ),
                        "radial_min_m": radial_min,
                        "radial_max_m": radial_max,
                        "lateral_min_m": lateral_min,
                        "lateral_max_m": lateral_max,
                        "radial_span_m": radial_span_m,
                        "lateral_span_m": lateral_span_m,
                        "area_m2": area_m2,
                        "radial_sample_count": (
                            radial_sample_count
                        ),
                        "lateral_sample_count": (
                            lateral_sample_count
                        ),
                        "total_sample_count": (
                            radial_sample_count
                            * lateral_sample_count
                        ),
                    }

                    if best is None:
                        best = candidate
                        continue

                    candidate_area = float(
                        candidate["area_m2"]
                    )

                    best_area = float(
                        best["area_m2"]
                    )

                    if candidate_area > (
                        best_area + 1e-12
                    ):
                        best = candidate
                        continue

                    # Deterministic tie-break:
                    # prefer the rectangle containing more sampled
                    # validated points when physical area is equal.
                    if abs(
                        candidate_area
                        - best_area
                    ) <= 1e-12:
                        if (
                            candidate["total_sample_count"]
                            > best["total_sample_count"]
                        ):
                            best = candidate

    if best is None:
        raise RuntimeError(
            "no rectangle satisfies minimum vertical clearance "
            f"{minimum_clearance_m * 1000.0:.1f} mm"
        )

    return best

# ============================================================
# Object-placement validity mask
# ============================================================


def build_object_placement_validity_mask(
    *,
    records: list[dict[str, object]],
    radial_values: np.ndarray,
    lateral_values: np.ndarray,
    minimum_clearance_m: float = (
        OBJECT_PLACEMENT_MIN_GOOD_DOWN_M
    ),
) -> dict[str, object]:
    """
    Build the base object-placement validity mask.

    A sampled XY point is VALID when its initial continuous GOOD
    downward clearance is at least minimum_clearance_m.

    Important
    ---------
    This mask constrains the grasp-relevant placement region.

    It does NOT mean that every point of the complete object geometry
    must remain inside the VALID region. Non-grasp-critical parts of
    an object may later be allowed to overlap EXCLUDED cells.
    """
    radial_values = np.asarray(
        radial_values,
        dtype=np.float64,
    )

    lateral_values = np.asarray(
        lateral_values,
        dtype=np.float64,
    )

    if radial_values.ndim != 1:
        raise ValueError(
            "radial_values must be 1D"
        )

    if lateral_values.ndim != 1:
        raise ValueError(
            "lateral_values must be 1D"
        )

    if radial_values.size == 0:
        raise ValueError(
            "radial_values must not be empty"
        )

    if lateral_values.size == 0:
        raise ValueError(
            "lateral_values must not be empty"
        )

    if minimum_clearance_m < 0.0:
        raise ValueError(
            "minimum_clearance_m must be non-negative"
        )

    clearance_map = np.empty(
        (
            len(radial_values),
            len(lateral_values),
        ),
        dtype=np.float64,
    )

    validity_mask = np.zeros(
        clearance_map.shape,
        dtype=np.bool_,
    )

    for radial_index, radial_m in enumerate(
        radial_values
    ):
        for lateral_index, lateral_m in enumerate(
            lateral_values
        ):
            record = find_vertical_clearance_record(
                records,
                radial_m=float(radial_m),
                lateral_m=float(lateral_m),
            )

            clearance_m = float(
                record["good_clearance_m"]
            )

            clearance_map[
                radial_index,
                lateral_index,
            ] = clearance_m

            validity_mask[
                radial_index,
                lateral_index,
            ] = (
                clearance_m
                + 1e-12
                >= minimum_clearance_m
            )

    return {
        "radial_values_m": np.array(
            radial_values,
            dtype=np.float64,
            copy=True,
        ),
        "lateral_values_m": np.array(
            lateral_values,
            dtype=np.float64,
            copy=True,
        ),
        "clearance_map_m": clearance_map,
        "validity_mask": validity_mask,
        "minimum_clearance_m": float(
            minimum_clearance_m
        ),
        "valid_count": int(
            np.count_nonzero(
                validity_mask
            )
        ),
        "excluded_count": int(
            validity_mask.size
            - np.count_nonzero(
                validity_mask
            )
        ),
        "total_count": int(
            validity_mask.size
        ),
    }

def print_object_placement_validity_mask(
    mask_data: dict[str, object],
) -> None:
    """
    Print the sampled object-placement validity mask.

    Display convention:

                         OUTWARD
                            ↑

             LEFT  ← workspace →  RIGHT

                            ↓
                          INWARD

    Symbols
    -------
    O
        VALID for normal grasp-relevant placement.

    X
        EXCLUDED for normal grasp-relevant placement.

    R
        READY workspace origin.
    """
    radial_values = np.asarray(
        mask_data["radial_values_m"],
        dtype=np.float64,
    )

    lateral_values = np.asarray(
        mask_data["lateral_values_m"],
        dtype=np.float64,
    )

    validity_mask = np.asarray(
        mask_data["validity_mask"],
        dtype=np.bool_,
    )

    print(
        "===== OBJECT PLACEMENT VALIDITY MASK ====="
    )

    print()
    print(
        "minimum GOOD-down clearance:",
        float(
            mask_data[
                "minimum_clearance_m"
            ]
        ) * 1000.0,
        "mm",
    )

    print()
    print(
        "                            OUTWARD"
    )
    print(
        "                               ↑"
    )
    print()

    for radial_index in range(
        len(radial_values) - 1,
        -1,
        -1,
    ):
        radial_m = float(
            radial_values[
                radial_index
            ]
        )

        cells: list[str] = []

        for lateral_index in range(
            len(lateral_values) - 1,
            -1,
            -1,
        ):
            lateral_m = float(
                lateral_values[
                    lateral_index
                ]
            )

            valid = bool(
                validity_mask[
                    radial_index,
                    lateral_index,
                ]
            )

            symbol = (
                "O"
                if valid
                else "X"
            )

            if (
                abs(radial_m) < 1e-12
                and abs(lateral_m) < 1e-12
            ):
                symbol = "R"

            cells.append(
                symbol
            )

        print(
            f'{radial_m * 1000.0:+5.0f} mm  '
            + " ".join(cells)
        )

    print()
    print(
        "lateral [mm], LEFT -> RIGHT:"
    )

    print(
        " ".join(
            f"{value * 1000.0:+.0f}"
            for value in lateral_values[::-1]
        )
    )

    print()
    print("Legend:")
    print("  O = normal placement VALID")
    print("  X = EXCLUDED")
    print("  R = READY")

"""
python - <<'PY'
from sg100_teleop.tests.test_rm65_sg100_workspace_sweep import (
    Q_TELEOP_READY,
    load_ready_state,
    build_workspace_frame,
    run_six_direction_sweeps,
    build_tabletop_grid_axes,
    evaluate_tabletop_grid,
    find_largest_good_rectangle_containing_ready,
    evaluate_vertical_good_clearance_map,
    build_object_placement_validity_mask,
    print_object_placement_validity_mask,
)

model, _, p_ready, R_ready = load_ready_state()

directions = build_workspace_frame(
    p_ready
)

six_direction_results = run_six_direction_sweeps(
    model=model,
    start_q=Q_TELEOP_READY,
    start_position=p_ready,
    target_rotation=R_ready,
    directions=directions,
)

radial_axis, lateral_axis = build_tabletop_grid_axes(
    six_direction_results
)

tabletop_records = evaluate_tabletop_grid(
    model=model,
    ready_q=Q_TELEOP_READY,
    ready_position=p_ready,
    ready_rotation=R_ready,
    directions=directions,
    radial_axis=radial_axis,
    lateral_axis=lateral_axis,
)

rectangle = find_largest_good_rectangle_containing_ready(
    records=tabletop_records,
    radial_axis=radial_axis,
    lateral_axis=lateral_axis,
)

radial_values = radial_axis[
    (radial_axis >= rectangle["radial_min_m"] - 1e-12)
    &
    (radial_axis <= rectangle["radial_max_m"] + 1e-12)
]

lateral_values = lateral_axis[
    (lateral_axis >= rectangle["lateral_min_m"] - 1e-12)
    &
    (lateral_axis <= rectangle["lateral_max_m"] + 1e-12)
]

records = evaluate_vertical_good_clearance_map(
    model=model,
    ready_q=Q_TELEOP_READY,
    ready_position=p_ready,
    ready_rotation=R_ready,
    directions=directions,
    tabletop_records=tabletop_records,
    radial_axis=radial_axis,
    lateral_axis=lateral_axis,
    radial_min_m=float(
        rectangle["radial_min_m"]
    ),
    radial_max_m=float(
        rectangle["radial_max_m"]
    ),
    lateral_min_m=float(
        rectangle["lateral_min_m"]
    ),
    lateral_max_m=float(
        rectangle["lateral_max_m"]
    ),
)

mask_data = build_object_placement_validity_mask(
    records=records,
    radial_values=radial_values,
    lateral_values=lateral_values,
)

print_object_placement_validity_mask(
    mask_data
)

print()
print("===== MASK SUMMARY =====")

print(
    "total:",
    mask_data["total_count"],
)

print(
    "valid:",
    mask_data["valid_count"],
)

print(
    "excluded:",
    mask_data["excluded_count"],
)
PY
"""

"""
===== OBJECT PLACEMENT VALIDITY MASK =====

minimum GOOD-down clearance: 80.0 mm

                            OUTWARD
                               ↑

  +60 mm  O O O O O O O O O O O O O O O O
  +40 mm  O O O O O O O O O O O O O O O O
  +20 mm  O O O O O O O O O O O O O O O O
   +0 mm  O O O O O O O O R O O O O O O O
  -20 mm  O O O O O X X X X O O O O O O O
  -40 mm  O O O O O X X X X X O O O O O O
  -60 mm  O O O O O X X X X X O O O O O O

lateral [mm], LEFT -> RIGHT:
+160 +140 +120 +100 +80 +60 +40 +20 +0 -20 -40 -60 -80 -100 -120 -140

Legend:
  O = normal placement VALID
  X = EXCLUDED
  R = READY

===== MASK SUMMARY =====
total: 112
valid: 98
excluded: 14
"""

"""
python - <<'PY'
from pathlib import Path
import numpy as np

from sg100_teleop.tests.test_rm65_sg100_workspace_sweep import (
    Q_TELEOP_READY,
    OBJECT_PLACEMENT_MIN_GOOD_DOWN_M,
    load_ready_state,
    build_workspace_frame,
    run_six_direction_sweeps,
    build_tabletop_grid_axes,
    evaluate_tabletop_grid,
    find_largest_good_rectangle_containing_ready,
    evaluate_vertical_good_clearance_map,
    build_object_placement_validity_mask,
)

output_path = Path(
    "sg100_teleop/assets/workspace/"
    "object_placement_mask_ready_v1.npz"
)

# ============================================================
# Rebuild validated workspace data
# ============================================================

model, _, p_ready, R_ready = load_ready_state()

directions = build_workspace_frame(
    p_ready
)

six_direction_results = run_six_direction_sweeps(
    model=model,
    start_q=Q_TELEOP_READY,
    start_position=p_ready,
    target_rotation=R_ready,
    directions=directions,
)

radial_axis, lateral_axis = build_tabletop_grid_axes(
    six_direction_results
)

tabletop_records = evaluate_tabletop_grid(
    model=model,
    ready_q=Q_TELEOP_READY,
    ready_position=p_ready,
    ready_rotation=R_ready,
    directions=directions,
    radial_axis=radial_axis,
    lateral_axis=lateral_axis,
)

rectangle = find_largest_good_rectangle_containing_ready(
    records=tabletop_records,
    radial_axis=radial_axis,
    lateral_axis=lateral_axis,
)

radial_values = radial_axis[
    (radial_axis >= rectangle["radial_min_m"] - 1e-12)
    &
    (radial_axis <= rectangle["radial_max_m"] + 1e-12)
]

lateral_values = lateral_axis[
    (lateral_axis >= rectangle["lateral_min_m"] - 1e-12)
    &
    (lateral_axis <= rectangle["lateral_max_m"] + 1e-12)
]

vertical_records = evaluate_vertical_good_clearance_map(
    model=model,
    ready_q=Q_TELEOP_READY,
    ready_position=p_ready,
    ready_rotation=R_ready,
    directions=directions,
    tabletop_records=tabletop_records,
    radial_axis=radial_axis,
    lateral_axis=lateral_axis,
    radial_min_m=float(
        rectangle["radial_min_m"]
    ),
    radial_max_m=float(
        rectangle["radial_max_m"]
    ),
    lateral_min_m=float(
        rectangle["lateral_min_m"]
    ),
    lateral_max_m=float(
        rectangle["lateral_max_m"]
    ),
)

mask_data = build_object_placement_validity_mask(
    records=vertical_records,
    radial_values=radial_values,
    lateral_values=lateral_values,
)

# ============================================================
# Regression guards before saving
# ============================================================

assert mask_data["total_count"] == 112
assert mask_data["valid_count"] == 98
assert mask_data["excluded_count"] == 14

assert abs(
    float(mask_data["minimum_clearance_m"])
    - OBJECT_PLACEMENT_MIN_GOOD_DOWN_M
) < 1e-12

# ============================================================
# Save runtime artifact
# ============================================================

np.savez_compressed(
    output_path,

    artifact_version=np.array(
        "object_placement_mask_v1"
    ),

    radial_values_m=np.asarray(
        mask_data["radial_values_m"],
        dtype=np.float64,
    ),

    lateral_values_m=np.asarray(
        mask_data["lateral_values_m"],
        dtype=np.float64,
    ),

    clearance_map_m=np.asarray(
        mask_data["clearance_map_m"],
        dtype=np.float64,
    ),

    validity_mask=np.asarray(
        mask_data["validity_mask"],
        dtype=np.bool_,
    ),

    minimum_clearance_m=np.array(
        mask_data["minimum_clearance_m"],
        dtype=np.float64,
    ),

    q_teleop_ready=np.asarray(
        Q_TELEOP_READY,
        dtype=np.float64,
    ),

    ready_position_world_m=np.asarray(
        p_ready,
        dtype=np.float64,
    ),

    ready_rotation_world=np.asarray(
        R_ready,
        dtype=np.float64,
    ),

    outward_world=np.asarray(
        directions["OUTWARD"],
        dtype=np.float64,
    ),

    left_world=np.asarray(
        directions["LEFT"],
        dtype=np.float64,
    ),

    up_world=np.asarray(
        directions["UP"],
        dtype=np.float64,
    ),
)

print("===== OBJECT PLACEMENT MASK ARTIFACT SAVED =====")
print()
print("path:", output_path)
print("threshold:", mask_data["minimum_clearance_m"] * 1000.0, "mm")
print("shape:", mask_data["validity_mask"].shape)
print("total:", mask_data["total_count"])
print("valid:", mask_data["valid_count"])
print("excluded:", mask_data["excluded_count"])
PY
"""