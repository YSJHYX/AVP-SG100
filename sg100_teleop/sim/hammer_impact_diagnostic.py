"""Read-only MuJoCo diagnostics for the Hammer/Peg demo.

Nothing in this module changes controller or contact parameters.  The runtime
recorder samples compiled-model state after each physics step, detects only the
named hammer-head/peg-cap pair, and writes one bounded trace per impact.
"""

from __future__ import annotations

import csv
from collections import deque
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Iterable

import mujoco
import numpy as np

from sg100_teleop.sim.combined_robot_layout import (
    ARM_DOF,
    RM65_JOINT_ORDER,
    resolve_robot_layout,
)


HAMMER_BODY = "demo_hammer"
HAMMER_HEAD_GEOM = "hammer_head_collision"
HAMMER_VISUAL_GEOM = "hammer_visual"
PEG_BODY = "demo_peg"
PEG_GEOM = "peg_strike_cap"
PEG_JOINT = "demo_peg_slide"
TCP_BODY = "sg100_base_link"


def _require_id(model, objtype, name: str) -> int:
    object_id = mujoco.mj_name2id(model, objtype, name)
    if object_id < 0:
        raise RuntimeError(f"Required MuJoCo object not found: {name}")
    return int(object_id)


def _name(model, objtype, object_id: int) -> str:
    return mujoco.mj_id2name(model, objtype, int(object_id)) or f"id:{object_id}"


def contact_matches_pair(contact, geom_a: int, geom_b: int) -> bool:
    """Return true only for an unordered pair of the two requested geoms."""

    pair = {int(contact.geom1), int(contact.geom2)}
    return len(pair) == 2 and pair == {int(geom_a), int(geom_b)}


def object_velocity(
    model,
    data,
    objtype,
    object_id: int,
) -> tuple[np.ndarray, np.ndarray]:
    """Return world-frame (linear, angular) velocity for an object frame.

    MuJoCo documents ``mj_objectVelocity`` ordering as ``rot:lin``.  Keeping
    the split here avoids relying on ``body COM`` velocity for the offset
    hammer-head collision geom.
    """

    spatial = np.zeros(6, dtype=np.float64)
    mujoco.mj_objectVelocity(
        model,
        data,
        objtype,
        int(object_id),
        spatial,
        0,
    )
    return spatial[3:].copy(), spatial[:3].copy()


def body_point_velocity(model, data, body_id: int, point_world) -> np.ndarray:
    """World linear velocity of a point rigidly attached to a body."""

    linear, angular = object_velocity(
        model, data, mujoco.mjtObj.mjOBJ_BODY, body_id
    )
    # For mjOBJ_BODY MuJoCo centers the returned spatial velocity at the
    # body's inertial-frame origin (COM), exposed as data.xipos, rather than
    # the authored kinematic body origin data.xpos.
    offset = np.asarray(point_world, dtype=np.float64) - data.xipos[body_id]
    return linear + np.cross(angular, offset)


def peg_displacement(qpos_before: float, qpos_after: float) -> tuple[float, float]:
    """Peg insertion displacement in its authored slide coordinate."""

    displacement_m = float(qpos_after) - float(qpos_before)
    return displacement_m, displacement_m * 1000.0


def integrate_normal_impulse(normal_forces: Iterable[float], dt: float) -> float:
    """Rectangular discrete integral of normal force over sample intervals."""

    values = np.asarray(tuple(normal_forces), dtype=np.float64)
    if not np.isfinite(dt) or dt <= 0.0:
        raise ValueError("dt must be finite and > 0")
    if not np.all(np.isfinite(values)):
        raise ValueError("normal forces must be finite")
    return float(np.sum(values) * dt)


@dataclass(frozen=True)
class HammerModelIds:
    hammer_body: int
    hammer_head_geom: int
    hammer_visual_geom: int
    peg_body: int
    peg_geom: int
    peg_joint: int
    peg_qposadr: int
    peg_dofadr: int
    tcp_body: int

    @classmethod
    def resolve(cls, model) -> "HammerModelIds":
        peg_joint = _require_id(model, mujoco.mjtObj.mjOBJ_JOINT, PEG_JOINT)
        return cls(
            hammer_body=_require_id(model, mujoco.mjtObj.mjOBJ_BODY, HAMMER_BODY),
            hammer_head_geom=_require_id(
                model, mujoco.mjtObj.mjOBJ_GEOM, HAMMER_HEAD_GEOM
            ),
            hammer_visual_geom=_require_id(
                model, mujoco.mjtObj.mjOBJ_GEOM, HAMMER_VISUAL_GEOM
            ),
            peg_body=_require_id(model, mujoco.mjtObj.mjOBJ_BODY, PEG_BODY),
            peg_geom=_require_id(model, mujoco.mjtObj.mjOBJ_GEOM, PEG_GEOM),
            peg_joint=peg_joint,
            peg_qposadr=int(model.jnt_qposadr[peg_joint]),
            peg_dofadr=int(model.jnt_dofadr[peg_joint]),
            tcp_body=_require_id(model, mujoco.mjtObj.mjOBJ_BODY, TCP_BODY),
        )


def geom_effective_parameters(model, geom_id: int) -> dict[str, Any]:
    """Compiled, effective geom values (including inherited defaults)."""

    geom_type = mujoco.mjtGeom(int(model.geom_type[geom_id])).name
    return {
        "name": _name(model, mujoco.mjtObj.mjOBJ_GEOM, geom_id),
        "type": geom_type.removeprefix("mjGEOM_").lower(),
        "size": model.geom_size[geom_id].copy(),
        "local_pos": model.geom_pos[geom_id].copy(),
        "local_quat": model.geom_quat[geom_id].copy(),
        "friction": model.geom_friction[geom_id].copy(),
        "solref": model.geom_solref[geom_id].copy(),
        "solimp": model.geom_solimp[geom_id].copy(),
        "condim": int(model.geom_condim[geom_id]),
        "margin": float(model.geom_margin[geom_id]),
        "gap": float(model.geom_gap[geom_id]),
        "contype": int(model.geom_contype[geom_id]),
        "conaffinity": int(model.geom_conaffinity[geom_id]),
    }


def _mesh_body_local_aabb(model, geom_id: int) -> tuple[np.ndarray, np.ndarray] | None:
    if int(model.geom_type[geom_id]) != int(mujoco.mjtGeom.mjGEOM_MESH):
        return None
    mesh_id = int(model.geom_dataid[geom_id])
    start = int(model.mesh_vertadr[mesh_id])
    count = int(model.mesh_vertnum[mesh_id])
    vertices = model.mesh_vert[start : start + count]
    rotation = np.zeros(9, dtype=np.float64)
    mujoco.mju_quat2Mat(rotation, model.geom_quat[geom_id])
    transformed = vertices @ rotation.reshape(3, 3).T + model.geom_pos[geom_id]
    return transformed.min(axis=0), transformed.max(axis=0)


def _cylinder_body_local_aabb(model, geom_id: int) -> tuple[np.ndarray, np.ndarray] | None:
    if int(model.geom_type[geom_id]) != int(mujoco.mjtGeom.mjGEOM_CYLINDER):
        return None
    radius, half_length = model.geom_size[geom_id, :2]
    corners = np.array(
        [
            [x, y, z]
            for x in (-radius, radius)
            for y in (-radius, radius)
            for z in (-half_length, half_length)
        ],
        dtype=np.float64,
    )
    rotation = np.zeros(9, dtype=np.float64)
    mujoco.mju_quat2Mat(rotation, model.geom_quat[geom_id])
    transformed = corners @ rotation.reshape(3, 3).T + model.geom_pos[geom_id]
    return transformed.min(axis=0), transformed.max(axis=0)


def _joint_audit(model, joint_id: int) -> dict[str, Any]:
    dof = int(model.jnt_dofadr[joint_id])
    joint_type = mujoco.mjtJoint(int(model.jnt_type[joint_id])).name
    return {
        "name": _name(model, mujoco.mjtObj.mjOBJ_JOINT, joint_id),
        "type": joint_type.removeprefix("mjJNT_").lower(),
        "axis": model.jnt_axis[joint_id].copy(),
        "range": model.jnt_range[joint_id].copy(),
        "limited": bool(model.jnt_limited[joint_id]),
        "damping": float(model.dof_damping[dof]),
        "frictionloss": float(model.dof_frictionloss[dof]),
        "armature": float(model.dof_armature[dof]),
        "actuator_force_range": model.jnt_actfrcrange[joint_id].copy(),
        "actuator_force_limited": bool(model.jnt_actfrclimited[joint_id]),
    }


def _actuator_type(model, actuator_id: int) -> str:
    gain = model.actuator_gainprm[actuator_id]
    bias = model.actuator_biasprm[actuator_id]
    if np.isclose(bias[1], -gain[0]) and bias[1] != 0.0:
        return "position"
    return (
        f"gain={mujoco.mjtGain(int(model.actuator_gaintype[actuator_id])).name},"
        f" bias={mujoco.mjtBias(int(model.actuator_biastype[actuator_id])).name}"
    )


def _actuator_audit(model, actuator_id: int) -> dict[str, Any]:
    joint_id = int(model.actuator_trnid[actuator_id, 0])
    dof = int(model.jnt_dofadr[joint_id])
    forcerange = model.actuator_forcerange[actuator_id].copy()
    force_limited = bool(model.actuator_forcelimited[actuator_id])
    joint_force_range = model.jnt_actfrcrange[joint_id].copy()
    joint_force_limited = bool(model.jnt_actfrclimited[joint_id])
    effective_range = forcerange if force_limited else joint_force_range
    effective_limited = force_limited or joint_force_limited
    return {
        "name": _name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, actuator_id),
        "joint": _name(model, mujoco.mjtObj.mjOBJ_JOINT, joint_id),
        "type": _actuator_type(model, actuator_id),
        "kp": float(model.actuator_gainprm[actuator_id, 0]),
        "kv_effective": float(-model.actuator_biasprm[actuator_id, 2]),
        "ctrlrange": model.actuator_ctrlrange[actuator_id].copy(),
        "ctrl_limited": bool(model.actuator_ctrllimited[actuator_id]),
        "forcerange": forcerange,
        "force_limited": force_limited,
        "joint_force_range": joint_force_range,
        "joint_force_limited": joint_force_limited,
        "effective_force_range": effective_range,
        "effective_force_limited": effective_limited,
        "gear": model.actuator_gear[actuator_id].copy(),
        "joint_damping": float(model.dof_damping[dof]),
        "joint_armature": float(model.dof_armature[dof]),
        "joint_range": model.jnt_range[joint_id].copy(),
    }


def audit_hammer_model(model) -> dict[str, Any]:
    """Return a compiled-model static audit without mutating the model."""

    ids = HammerModelIds.resolve(model)
    layout = resolve_robot_layout(model)
    hammer_visual_aabb = _mesh_body_local_aabb(model, ids.hammer_visual_geom)
    hammer_head_aabb = _cylinder_body_local_aabb(model, ids.hammer_head_geom)
    hammer = {
        "body_name": HAMMER_BODY,
        "head_collision": geom_effective_parameters(model, ids.hammer_head_geom),
        "visual": geom_effective_parameters(model, ids.hammer_visual_geom),
        "mass": float(model.body_mass[ids.hammer_body]),
        "com_local": model.body_ipos[ids.hammer_body].copy(),
        "inertia_diagonal_principal": model.body_inertia[ids.hammer_body].copy(),
        "inertial_quat": model.body_iquat[ids.hammer_body].copy(),
        "visual_body_local_aabb": hammer_visual_aabb,
        "head_collision_body_local_aabb": hammer_head_aabb,
    }
    peg = {
        "body_name": PEG_BODY,
        "collision": geom_effective_parameters(model, ids.peg_geom),
        "joint": _joint_audit(model, ids.peg_joint),
        "mass": float(model.body_mass[ids.peg_body]),
        "com_local": model.body_ipos[ids.peg_body].copy(),
        "inertia_diagonal_principal": model.body_inertia[ids.peg_body].copy(),
        "inertial_quat": model.body_iquat[ids.peg_body].copy(),
    }
    actuators = [
        _actuator_audit(model, int(actuator_id))
        for actuator_id in layout.arm_actuator_ids
    ]
    return {
        "timestep": float(model.opt.timestep),
        "hammer": hammer,
        "peg": peg,
        "rm65_actuators": actuators,
        "authoritative_source": "MuJoCo compiled model effective values",
    }


def _array_text(value) -> str:
    return np.array2string(
        np.asarray(value), precision=8, separator=", ", suppress_small=False
    )


def format_static_audit(audit: dict[str, Any]) -> str:
    """Human-readable static physics and actuator audit."""

    hammer = audit["hammer"]
    peg = audit["peg"]
    head = hammer["head_collision"]
    cap = peg["collision"]
    joint = peg["joint"]
    lines = [
        "HAMMER IMPACT STATIC PHYSICS AUDIT",
        f"authoritative source: {audit['authoritative_source']}",
        f"simulation dt: {audit['timestep']:.6f} s",
        "",
        "HAMMER",
        f"body: {hammer['body_name']}",
        f"mass: {hammer['mass']:.9g} kg",
        f"COM local: {_array_text(hammer['com_local'])} m",
        f"inertia diagonal (principal): {_array_text(hammer['inertia_diagonal_principal'])} kg*m^2",
        f"head collision geom: {head['name']} ({head['type']})",
        f"head collision size: {_array_text(head['size'])} m",
        f"head collision local offset: {_array_text(head['local_pos'])} m",
        f"head friction: {_array_text(head['friction'])}",
        f"head solref: {_array_text(head['solref'])}",
        f"head solimp: {_array_text(head['solimp'])}",
        f"head condim/margin/gap: {head['condim']} / {head['margin']} / {head['gap']}",
    ]
    if hammer["visual_body_local_aabb"] is not None:
        lower, upper = hammer["visual_body_local_aabb"]
        lines.extend(
            [
                f"visual geom/mesh: {hammer['visual']['name']}",
                f"visual body-local AABB min: {_array_text(lower)} m",
                f"visual body-local AABB max: {_array_text(upper)} m",
            ]
        )
    if hammer["head_collision_body_local_aabb"] is not None:
        lower, upper = hammer["head_collision_body_local_aabb"]
        lines.extend(
            [
                f"head collision body-local AABB min: {_array_text(lower)} m",
                f"head collision body-local AABB max: {_array_text(upper)} m",
                "visual/collision alignment: proxy bounds overlap the scanned head reference; runtime debug flag provides contact confirmation",
            ]
        )
    lines.extend(
        [
            "",
            "PEG",
            f"body: {peg['body_name']}",
            f"mass: {peg['mass']:.9g} kg",
            f"COM local: {_array_text(peg['com_local'])} m",
            f"inertia diagonal (principal): {_array_text(peg['inertia_diagonal_principal'])} kg*m^2",
            f"collision geom: {cap['name']} ({cap['type']})",
            f"collision size/local offset: {_array_text(cap['size'])} / {_array_text(cap['local_pos'])} m",
            f"contact friction: {_array_text(cap['friction'])}",
            f"contact solref: {_array_text(cap['solref'])}",
            f"contact solimp: {_array_text(cap['solimp'])}",
            f"contact condim/margin/gap: {cap['condim']} / {cap['margin']} / {cap['gap']}",
            f"joint: {joint['name']} ({joint['type']}) axis={_array_text(joint['axis'])}",
            f"joint range: {_array_text(joint['range'])} m",
            f"joint damping/frictionloss/armature: {joint['damping']} / {joint['frictionloss']} / {joint['armature']}",
            "",
            "RM65 ACTUATORS",
        ]
    )
    for actuator in audit["rm65_actuators"]:
        lines.extend(
            [
                f"{actuator['joint']} / {actuator['name']}: type={actuator['type']}",
                f"  kp={actuator['kp']:.9g}, kv_effective={actuator['kv_effective']:.9g}",
                f"  ctrlrange={_array_text(actuator['ctrlrange'])}, gear={_array_text(actuator['gear'])}",
                f"  actuator forcerange={_array_text(actuator['forcerange'])}, limited={actuator['force_limited']}",
                f"  joint actuatorfrcrange={_array_text(actuator['joint_force_range'])}, limited={actuator['joint_force_limited']}",
                f"  EFFECTIVE force limit={_array_text(actuator['effective_force_range'])}",
                f"  joint damping/armature/range={actuator['joint_damping']} / {actuator['joint_armature']} / {_array_text(actuator['joint_range'])}",
            ]
        )
    lines.extend(
        [
            "",
            "CONTACT PARAMETER PROVENANCE",
            "solref/friction/condim: direct named geom settings in production object XML",
            "solimp/margin/gap: compiled MuJoCo defaults (not directly authored on the named geoms)",
            "all numeric values above: compiled effective values",
        ]
    )
    return "\n".join(lines)


def effective_arm_force_limits(model, layout) -> np.ndarray:
    limits = np.full(ARM_DOF, np.inf, dtype=np.float64)
    for index, actuator_id in enumerate(layout.arm_actuator_ids):
        actuator_id = int(actuator_id)
        if model.actuator_forcelimited[actuator_id]:
            limits[index] = float(np.max(np.abs(model.actuator_forcerange[actuator_id])))
            continue
        joint_id = int(model.actuator_trnid[actuator_id, 0])
        if model.jnt_actfrclimited[joint_id]:
            limits[index] = float(np.max(np.abs(model.jnt_actfrcrange[joint_id])))
    return limits


def _flatten_trace_row(sample: dict[str, Any]) -> dict[str, Any]:
    row: dict[str, Any] = {}
    for key, value in sample.items():
        array = np.asarray(value) if isinstance(value, (np.ndarray, list, tuple)) else None
        if array is None or array.ndim == 0:
            row[key] = value
            continue
        flat = array.reshape(-1)
        suffixes = ("x", "y", "z", "w") if len(flat) <= 4 else None
        for index, item in enumerate(flat):
            suffix = suffixes[index] if suffixes is not None else str(index + 1)
            row[f"{key}_{suffix}"] = item
    return row


class HammerImpactDiagnostic:
    """Bounded impact-event recorder for the production teleop loop."""

    def __init__(
        self,
        model,
        output_dir: str | Path,
        *,
        pre_frames: int = 25,
        post_frames: int = 5,
        print_events: bool = True,
    ):
        if pre_frames < 1 or post_frames < 1:
            raise ValueError("pre_frames and post_frames must be >= 1")
        self.model = model
        self.ids = HammerModelIds.resolve(model)
        self.layout = resolve_robot_layout(model)
        self.dt = float(model.opt.timestep)
        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.pre_frames = int(pre_frames)
        self.post_frames = int(post_frames)
        self.print_events = bool(print_events)
        self._pre = deque(maxlen=self.pre_frames)
        self._event: list[dict[str, Any]] | None = None
        self._contact_seen = False
        self._post_remaining = 0
        self._event_count = 0
        self._previous_goal: np.ndarray | None = None
        self._previous_goal_change_time: float | None = None
        self._target_qvel_estimate = np.zeros(ARM_DOF, dtype=np.float64)
        self._arm_force_limits = effective_arm_force_limits(model, self.layout)
        self._unexpected_pairs_warned: set[str] = set()
        self.events: list[dict[str, Any]] = []

    @property
    def event_count(self) -> int:
        return self._event_count

    def _contact_observation(self, data) -> dict[str, Any]:
        target_indices: list[int] = []
        target_names: list[str] = []
        unexpected_strike_pairs: list[str] = []
        all_pairs: list[str] = []
        for index in range(int(data.ncon)):
            contact = data.contact[index]
            name1 = _name(self.model, mujoco.mjtObj.mjOBJ_GEOM, contact.geom1)
            name2 = _name(self.model, mujoco.mjtObj.mjOBJ_GEOM, contact.geom2)
            all_pairs.append(f"{name1}<->{name2}")
            if contact_matches_pair(
                contact, self.ids.hammer_head_geom, self.ids.peg_geom
            ):
                target_indices.append(index)
                target_names.append(f"{name1}<->{name2}")
            else:
                body1 = int(self.model.geom_bodyid[int(contact.geom1)])
                body2 = int(self.model.geom_bodyid[int(contact.geom2)])
                hammer_peg_body_pair = {body1, body2} == {
                    self.ids.hammer_body,
                    self.ids.peg_body,
                }
                peg_cap_involved = self.ids.peg_geom in {
                    int(contact.geom1),
                    int(contact.geom2),
                }
                if hammer_peg_body_pair or peg_cap_involved:
                    unexpected_strike_pairs.append(f"{name1}<->{name2}")

        normal_force = 0.0
        position = np.full(3, np.nan)
        normal = np.full(3, np.nan)
        distance = np.nan
        closing_speed = np.nan
        if target_indices:
            position_accum = np.zeros(3)
            for index in target_indices:
                force = np.zeros(6, dtype=np.float64)
                mujoco.mj_contactForce(self.model, data, index, force)
                normal_force += max(0.0, float(force[0]))
                position_accum += data.contact[index].pos
            first = data.contact[target_indices[0]]
            position = position_accum / len(target_indices)
            normal = np.asarray(first.frame[:3], dtype=np.float64).copy()
            if int(first.geom1) != self.ids.hammer_head_geom:
                normal *= -1.0
            distance = float(min(data.contact[index].dist for index in target_indices))
            hammer_point_velocity = body_point_velocity(
                self.model, data, self.ids.hammer_body, first.pos
            )
            peg_point_velocity = body_point_velocity(
                self.model, data, self.ids.peg_body, first.pos
            )
            closing_speed = float(
                np.dot(hammer_point_velocity - peg_point_velocity, normal)
            )

        return {
            "active": bool(target_indices),
            "count": len(target_indices),
            "normal_force": normal_force,
            "position": position,
            "normal": normal,
            "distance": distance,
            "closing_speed": closing_speed,
            "target_pairs": "|".join(target_names),
            "unexpected_strike_pairs": "|".join(unexpected_strike_pairs),
            "all_pairs": "|".join(all_pairs),
        }

    def _sample(
        self,
        data,
        q_arm_goal: np.ndarray,
        target_tcp: np.ndarray | None,
        raw_target_tcp: np.ndarray | None,
    ) -> dict[str, Any]:
        sim_time = float(data.time)
        head_linear, head_angular = object_velocity(
            self.model,
            data,
            mujoco.mjtObj.mjOBJ_GEOM,
            self.ids.hammer_head_geom,
        )
        hammer_com_velocity = body_point_velocity(
            self.model,
            data,
            self.ids.hammer_body,
            data.xipos[self.ids.hammer_body],
        )
        inertia_rotation = data.ximat[self.ids.hammer_body].reshape(3, 3)
        inertia_world = (
            inertia_rotation
            @ np.diag(self.model.body_inertia[self.ids.hammer_body])
            @ inertia_rotation.T
        )
        hammer_linear_energy = 0.5 * self.model.body_mass[
            self.ids.hammer_body
        ] * float(np.dot(hammer_com_velocity, hammer_com_velocity))
        hammer_rotational_energy = 0.5 * float(
            head_angular @ inertia_world @ head_angular
        )
        tcp_linear, tcp_angular = object_velocity(
            self.model,
            data,
            mujoco.mjtObj.mjOBJ_BODY,
            self.ids.tcp_body,
        )
        q_goal = np.asarray(q_arm_goal, dtype=np.float64).copy()
        if q_goal.shape != (ARM_DOF,):
            raise ValueError(f"q_arm_goal must have shape ({ARM_DOF},)")
        if self._previous_goal is None:
            self._previous_goal = q_goal.copy()
            self._previous_goal_change_time = sim_time
        elif not np.array_equal(q_goal, self._previous_goal):
            goal_dt = sim_time - float(self._previous_goal_change_time)
            if goal_dt > 0.0:
                self._target_qvel_estimate = (
                    q_goal - self._previous_goal
                ) / goal_dt
            self._previous_goal = q_goal.copy()
            self._previous_goal_change_time = sim_time
        target_qvel = self._target_qvel_estimate.copy()
        target_qvel_age = sim_time - float(self._previous_goal_change_time)

        contact = self._contact_observation(data)
        target_tcp_array = np.full((4, 4), np.nan)
        if target_tcp is not None:
            candidate = np.asarray(target_tcp, dtype=np.float64)
            if candidate.shape == (4, 4):
                target_tcp_array = candidate.copy()
        raw_target_tcp_array = np.full((4, 4), np.nan)
        if raw_target_tcp is not None:
            candidate = np.asarray(raw_target_tcp, dtype=np.float64)
            if candidate.shape == (4, 4):
                raw_target_tcp_array = candidate.copy()

        arm_actuator_force = data.actuator_force[
            self.layout.arm_actuator_ids
        ].copy()
        saturation_fraction = np.divide(
            np.abs(arm_actuator_force),
            self._arm_force_limits,
            out=np.zeros(ARM_DOF, dtype=np.float64),
            where=np.isfinite(self._arm_force_limits) & (self._arm_force_limits > 0),
        )
        return {
            "sim_time": sim_time,
            "dt": self.dt,
            "hammer_head_pos_xyz": data.geom_xpos[self.ids.hammer_head_geom].copy(),
            "hammer_head_vel_xyz": head_linear,
            "hammer_head_omega_xyz": head_angular,
            "hammer_com_vel_xyz": hammer_com_velocity,
            "hammer_linear_com_energy_J": hammer_linear_energy,
            "hammer_rotational_energy_J": hammer_rotational_energy,
            "hammer_total_rigid_body_energy_J": (
                hammer_linear_energy + hammer_rotational_energy
            ),
            "tcp_pos_xyz": data.xpos[self.ids.tcp_body].copy(),
            "tcp_rotation_matrix": data.xmat[self.ids.tcp_body].copy(),
            "tcp_vel_xyz": tcp_linear,
            "tcp_omega_xyz": tcp_angular,
            "target_tcp_matrix": target_tcp_array.reshape(-1),
            "raw_target_tcp_matrix": raw_target_tcp_array.reshape(-1),
            "arm_qpos_6": data.qpos[self.layout.arm_qpos_indices].copy(),
            "arm_qvel_6": data.qvel[self.layout.dof_indices[:ARM_DOF]].copy(),
            "arm_qgoal_6": q_goal,
            "arm_tracking_error_6": (
                q_goal - data.qpos[self.layout.arm_qpos_indices]
            ),
            "target_joint_velocity_estimate_6": target_qvel,
            "target_joint_velocity_estimate_age_s": target_qvel_age,
            "arm_ctrl_6": data.ctrl[self.layout.arm_actuator_ids].copy(),
            "arm_actuator_force_6": arm_actuator_force,
            "arm_effective_joint_actuator_force_6": data.qfrc_actuator[
                self.layout.dof_indices[:ARM_DOF]
            ].copy(),
            "arm_actuator_force_limit_6": self._arm_force_limits.copy(),
            "arm_actuator_saturation_fraction_6": saturation_fraction,
            "peg_joint_qpos": float(data.qpos[self.ids.peg_qposadr]),
            "peg_joint_qvel": float(data.qvel[self.ids.peg_dofadr]),
            "peg_world_pos_xyz": data.xpos[self.ids.peg_body].copy(),
            "contact_active": int(contact["active"]),
            "contact_count": int(contact["count"]),
            "contact_position_xyz": contact["position"],
            "contact_normal_xyz": contact["normal"],
            "contact_dist": contact["distance"],
            "contact_normal_force": contact["normal_force"],
            "contact_normal_closing_speed": contact["closing_speed"],
            "contact_target_geom_pairs": contact["target_pairs"],
            "contact_target_geom_ids": (
                f"{self.ids.hammer_head_geom}<->{self.ids.peg_geom}"
            ),
            "contact_unexpected_strike_pairs": contact[
                "unexpected_strike_pairs"
            ],
            "contact_all_geom_pairs": contact["all_pairs"],
        }

    def sample(
        self,
        data,
        q_arm_goal: np.ndarray,
        *,
        target_tcp: np.ndarray | None = None,
        raw_target_tcp: np.ndarray | None = None,
    ) -> dict[str, Any] | None:
        """Observe one post-step state; return a summary when an event closes."""

        sample = self._sample(data, q_arm_goal, target_tcp, raw_target_tcp)
        unexpected = str(sample["contact_unexpected_strike_pairs"])
        if unexpected:
            for pair in unexpected.split("|"):
                if pair not in self._unexpected_pairs_warned:
                    print(
                        "[HAMMER IMPACT DIAGNOSTIC] unexpected strike contact: "
                        f"{pair}; expected {HAMMER_HEAD_GEOM}<->{PEG_GEOM}"
                    )
                    self._unexpected_pairs_warned.add(pair)
        active = bool(sample["contact_active"])
        if self._event is None:
            if not active:
                self._pre.append(sample)
                return None
            self._event = list(self._pre)
            self._event.append(sample)
            self._contact_seen = True
            self._post_remaining = self.post_frames
            return None

        self._event.append(sample)
        if active:
            self._contact_seen = True
            self._post_remaining = self.post_frames
            return None

        self._post_remaining -= 1
        if self._post_remaining > 0:
            return None
        return self._finalize_event()

    def _summarize(self, samples: list[dict[str, Any]]) -> dict[str, Any]:
        contact_indices = [
            index for index, sample in enumerate(samples) if sample["contact_active"]
        ]
        first_contact = contact_indices[0]
        last_contact = contact_indices[-1]
        pre_index = max(0, first_contact - 1)
        pre = samples[pre_index]
        post = samples[-1]
        force_samples = [
            float(sample["contact_normal_force"])
            for sample in samples[first_contact : last_contact + 1]
        ]
        peg_m, peg_mm = peg_displacement(
            pre["peg_joint_qpos"], post["peg_joint_qpos"]
        )
        peg_world_delta = (
            np.asarray(post["peg_world_pos_xyz"])
            - np.asarray(pre["peg_world_pos_xyz"])
        )
        force_stack = np.stack(
            [np.abs(sample["arm_actuator_force_6"]) for sample in samples]
        )
        peak_force = force_stack.max(axis=0)
        saturation_fraction = np.divide(
            peak_force,
            self._arm_force_limits,
            out=np.zeros(ARM_DOF, dtype=np.float64),
            where=np.isfinite(self._arm_force_limits) & (self._arm_force_limits > 0),
        )
        pre_speed = float(np.linalg.norm(pre["hammer_head_vel_xyz"]))
        post_speed = float(np.linalg.norm(post["hammer_head_vel_xyz"]))
        actual_qvel = np.asarray(pre["arm_qvel_6"])
        target_qvel = np.asarray(pre["target_joint_velocity_estimate_6"])
        target_norm = float(np.linalg.norm(target_qvel))
        tracking_ratio = (
            float(np.linalg.norm(actual_qvel) / target_norm)
            if target_norm > 1e-9
            else np.nan
        )
        saturated = bool(np.any(saturation_fraction >= 0.95))
        if saturated:
            suspect = "ACTUATOR_SATURATION"
            evidence = "At least one RM65 joint reached >=95% of its effective force limit in the impact window."
        elif pre_speed < 0.30:
            suspect = "CONTROL_VELOCITY_TOO_LOW"
            evidence = "Diagnostic heuristic: measured pre-impact head speed was below 0.30 m/s."
        else:
            suspect = "INCONCLUSIVE"
            evidence = "A synthetic speed sweep and additional real strikes are required before tuning."
        return {
            "event_number": self._event_count + 1,
            "impact_timestamp": float(samples[first_contact]["sim_time"]),
            "pre_impact_head_speed": pre_speed,
            "pre_impact_head_velocity": np.asarray(pre["hammer_head_vel_xyz"]),
            "head_normal_impact_velocity": float(
                samples[first_contact]["contact_normal_closing_speed"]
            ),
            "pre_impact_angular_velocity": np.asarray(pre["hammer_head_omega_xyz"]),
            "pre_impact_linear_com_energy_J": float(
                pre["hammer_linear_com_energy_J"]
            ),
            "pre_impact_rotational_energy_J": float(
                pre["hammer_rotational_energy_J"]
            ),
            "pre_impact_total_rigid_body_energy_J": float(
                pre["hammer_total_rigid_body_energy_J"]
            ),
            "pre_impact_tcp_speed": float(np.linalg.norm(pre["tcp_vel_xyz"])),
            "pre_impact_tcp_velocity": np.asarray(pre["tcp_vel_xyz"]),
            "arm_qvel": actual_qvel,
            "target_q_velocity_estimate": target_qvel,
            "target_q_velocity_estimate_age_s": float(
                pre["target_joint_velocity_estimate_age_s"]
            ),
            "tracking_ratio": tracking_ratio,
            "contact_count_peak": max(int(s["contact_count"]) for s in samples),
            "contact_position": np.asarray(samples[first_contact]["contact_position_xyz"]),
            "contact_normal": np.asarray(samples[first_contact]["contact_normal_xyz"]),
            "minimum_contact_dist": float(
                min(samples[index]["contact_dist"] for index in contact_indices)
            ),
            "peak_normal_force": max(force_samples),
            "contact_duration_s": len(contact_indices) * self.dt,
            "estimated_normal_impulse_Ns": integrate_normal_impulse(
                force_samples, self.dt
            ),
            "peg_qpos_before": float(pre["peg_joint_qpos"]),
            "peg_qpos_after": float(post["peg_joint_qpos"]),
            "peg_displacement_m": peg_m,
            "peg_displacement_mm": peg_mm,
            "peg_world_displacement": peg_world_delta,
            "post_impact_head_speed": post_speed,
            "head_speed_loss": pre_speed - post_speed,
            "arm_peak_force": peak_force,
            "arm_force_limit": self._arm_force_limits.copy(),
            "arm_saturation_fraction": saturation_fraction,
            "arm_saturated": saturation_fraction >= 0.95,
            "primary_suspect": suspect,
            "evidence": evidence,
        }

    def _write_trace(self, samples: list[dict[str, Any]], number: int) -> tuple[Path, Path]:
        stem = f"impact_{number:03d}"
        npz_path = self.output_dir / f"{stem}.npz"
        csv_path = self.output_dir / f"{stem}.csv"
        keys = samples[0].keys()
        arrays: dict[str, np.ndarray] = {}
        for key in keys:
            values = [sample[key] for sample in samples]
            try:
                arrays[key] = np.asarray(values)
            except ValueError:
                arrays[key] = np.asarray(values, dtype=str)
        np.savez_compressed(npz_path, **arrays)

        rows = [_flatten_trace_row(sample) for sample in samples]
        fieldnames: list[str] = []
        for row in rows:
            for field in row:
                if field not in fieldnames:
                    fieldnames.append(field)
        with csv_path.open("w", newline="", encoding="utf-8") as handle:
            writer = csv.DictWriter(handle, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(rows)
        return csv_path, npz_path

    def _finalize_event(self) -> dict[str, Any]:
        assert self._event is not None
        samples = self._event
        if not any(sample["contact_active"] for sample in samples):
            self._event = None
            self._contact_seen = False
            return {}
        summary = self._summarize(samples)
        self._event_count += 1
        csv_path, npz_path = self._write_trace(samples, self._event_count)
        summary["trace_csv"] = csv_path
        summary["trace_npz"] = npz_path
        self.events.append(summary)
        self._pre.clear()
        for sample in samples[-self.pre_frames :]:
            self._pre.append(sample)
        self._event = None
        self._contact_seen = False
        self._post_remaining = 0
        if self.print_events:
            print(format_impact_summary(summary))
        return summary

    def close(self) -> dict[str, Any] | None:
        """Flush a partial contact episode safely; no-contact is a no-op."""

        if self._event is None:
            return None
        if not any(sample["contact_active"] for sample in self._event):
            self._event = None
            return None
        return self._finalize_event()


def format_impact_summary(summary: dict[str, Any]) -> str:
    lines = [
        "",
        "=" * 50,
        f"HAMMER IMPACT EVENT #{summary['event_number']}",
        "=" * 50,
        f"sim_time: {summary['impact_timestamp']:.6f} s",
        "",
        "HAMMER",
        f"head speed pre-impact: {summary['pre_impact_head_speed']:.6f} m/s",
        f"head normal impact velocity: {summary['head_normal_impact_velocity']:.6f} m/s",
        f"angular velocity: {_array_text(summary['pre_impact_angular_velocity'])} rad/s",
        f"linear COM kinetic energy: {summary['pre_impact_linear_com_energy_J']:.9g} J",
        f"rotational kinetic energy: {summary['pre_impact_rotational_energy_J']:.9g} J",
        f"total rigid-body kinetic energy: {summary['pre_impact_total_rigid_body_energy_J']:.9g} J",
        "",
        "TCP",
        f"linear speed: {summary['pre_impact_tcp_speed']:.6f} m/s",
        "",
        "ARM",
        f"qvel: {_array_text(summary['arm_qvel'])} rad/s",
        "target q velocity estimate (TARGET DERIVATIVE ESTIMATE):",
        f"  {_array_text(summary['target_q_velocity_estimate'])} rad/s",
        f"target derivative estimate age: {summary['target_q_velocity_estimate_age_s']:.6f} s",
        f"tracking ratio (norm actual/target estimate): {summary['tracking_ratio']:.6g}",
        "",
        "CONTACT",
        f"peak contact count: {summary['contact_count_peak']}",
        f"contact position: {_array_text(summary['contact_position'])} m",
        f"contact normal: {_array_text(summary['contact_normal'])}",
        f"minimum dist: {summary['minimum_contact_dist']:.9g} m",
        f"peak normal force: {summary['peak_normal_force']:.6f} N (FORCE)",
        f"contact duration: {summary['contact_duration_s'] * 1000.0:.3f} ms",
        "estimated normal impulse: "
        f"{summary['estimated_normal_impulse_Ns']:.9g} N*s "
        "(ESTIMATED FROM DISCRETE CONTACT FORCE)",
        "",
        "PEG",
        f"qpos before: {summary['peg_qpos_before']:.9g} m",
        f"qpos after: {summary['peg_qpos_after']:.9g} m",
        f"displacement: {summary['peg_displacement_mm']:.6f} mm",
        "",
        "POST IMPACT",
        f"hammer head speed: {summary['post_impact_head_speed']:.6f} m/s",
        f"velocity loss: {summary['head_speed_loss']:.6f} m/s",
        "",
        "RM65 ACTUATOR FORCE SATURATION",
    ]
    for index, joint_name in enumerate(RM65_JOINT_ORDER):
        lines.append(
            f"{joint_name}: peak={summary['arm_peak_force'][index]:.6g}, "
            f"limit={summary['arm_force_limit'][index]:.6g}, "
            f"fraction={summary['arm_saturation_fraction'][index] * 100.0:.2f}%, "
            f"saturation={'YES' if summary['arm_saturated'][index] else 'NO'}"
        )
    lines.extend(
        [
            "",
            f"PRIMARY SUSPECT: {summary['primary_suspect']}",
            f"evidence: {summary['evidence']}",
            f"trace CSV: {summary['trace_csv']}",
            f"trace NPZ: {summary['trace_npz']}",
            "=" * 50,
        ]
    )
    return "\n".join(lines)


def enable_collision_debug_visualization(model, viewer) -> None:
    """Diagnostic-only collision proxy/contact-point overlay."""

    ids = HammerModelIds.resolve(model)
    model.geom_rgba[ids.hammer_head_geom] = np.array([1.0, 0.1, 0.1, 0.45])
    model.geom_rgba[ids.peg_geom] = np.array([0.1, 0.4, 1.0, 0.45])
    viewer.opt.geomgroup[int(model.geom_group[ids.hammer_head_geom])] = 1
    viewer.opt.geomgroup[int(model.geom_group[ids.peg_geom])] = 1
    viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTPOINT] = 1
