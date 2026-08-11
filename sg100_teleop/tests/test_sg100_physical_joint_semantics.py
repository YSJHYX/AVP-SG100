from dataclasses import dataclass
from pathlib import Path

import mujoco
import numpy as np
from scipy.spatial.transform import Rotation

from sg100_teleop.hand.canonical_hand import CanonicalHandState
from sg100_teleop.retargeting.sg100_retargeter import SG100Retargeter
from sg100_teleop.sim.combined_robot_layout import SG100_JOINT_ORDER


ROOT = Path(__file__).resolve().parents[2]
MODEL_PATH = ROOT / "sg100_teleop/assets/sg100/mjcf/sg100.xml"

# Mathematical q=0 is a valid, open configuration in the authoritative model.
# This is a measurement reference only; it does not define production neutral.
TEST_KINEMATIC_REFERENCE = np.zeros(11, dtype=np.float64)

EXPECTED_AXES = {
    "thumb_j1": (0.0, 0.0, -1.0),
    "thumb_j2": (0.0, 0.0, 1.0),
    "thumb_j3": (0.0, 0.0, 1.0),
    "index_j1": (0.0, 0.0, -1.0),
    "index_j2": (0.0, 0.0, 1.0),
    "index_j3": (0.0, 0.0, 1.0),
    "middle_j1": (0.0, 0.0, -1.0),
    "middle_j2": (0.0, 0.0, 1.0),
    "little_j1": (0.0, 0.0, -1.0),
    "little_j2": (0.0, 0.0, 1.0),
    "little_j3": (0.0, 0.0, 1.0),
}


@dataclass(frozen=True)
class SemanticSpec:
    joint: str
    feature: str
    semantic: str
    distal_body: str
    samples: tuple[float, float, float]
    metric: str
    monotonic: str


SPECS = (
    SemanticSpec(
        "thumb_j1",
        "thumb_opposition",
        "opposition",
        "thumb_link3",
        (0.0, 0.5, 1.0),
        "thumb_to_opposition_distance",
        "decreasing",
    ),
    SemanticSpec(
        "thumb_j2",
        "thumb_abduction",
        "abduction/outward-down",
        "thumb_link3",
        (0.0, 0.5, 1.0),
        "base_z",
        "decreasing",
    ),
    SemanticSpec(
        "thumb_j3",
        "thumb_distal",
        "distal signed",
        "thumb_link3",
        (-1.0, 0.0, 1.0),
        "thumb_to_opposition_distance",
        "decreasing",
    ),
    SemanticSpec(
        "index_j1",
        "index_adduction",
        "adduction toward thumb",
        "index_link3",
        (0.0, 0.5, 1.0),
        "index_to_thumb_distance",
        "decreasing",
    ),
    SemanticSpec(
        "index_j2",
        "index_flexion",
        "flexion",
        "index_link3",
        (0.0, 0.5, 1.0),
        "base_z",
        "decreasing",
    ),
    SemanticSpec(
        "index_j3",
        "index_distal",
        "distal signed",
        "index_link3",
        (-1.0, 0.0, 1.0),
        "base_y",
        "decreasing",
    ),
    SemanticSpec(
        "middle_j1",
        "middle_flexion",
        "flexion",
        "middle_link2",
        (0.0, 0.5, 1.0),
        "base_z",
        "decreasing",
    ),
    SemanticSpec(
        "middle_j2",
        "middle_distal",
        "distal signed",
        "middle_link2",
        (-1.0, 0.0, 1.0),
        "base_y",
        "decreasing",
    ),
    SemanticSpec(
        "little_j1",
        "little_outward",
        "outward/down",
        "little_linik3",
        (0.0, 0.5, 1.0),
        "little_to_middle_distance",
        "increasing",
    ),
    SemanticSpec(
        "little_j2",
        "little_flexion",
        "flexion",
        "little_linik3",
        (0.0, 0.5, 1.0),
        "base_z",
        "decreasing",
    ),
    SemanticSpec(
        "little_j3",
        "little_distal",
        "distal signed",
        "little_linik3",
        (-1.0, 0.0, 1.0),
        "base_y",
        "decreasing",
    ),
)


class PhysicalSemanticHarness:
    def __init__(self):
        self.model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
        self.data = mujoco.MjData(self.model)
        self.retargeter = SG100Retargeter()

        assert tuple(self.retargeter.joint_order) == SG100_JOINT_ORDER
        assert tuple(spec.joint for spec in SPECS) == SG100_JOINT_ORDER

        self.joint_ids = {
            name: mujoco.mj_name2id(
                self.model,
                mujoco.mjtObj.mjOBJ_JOINT,
                name,
            )
            for name in SG100_JOINT_ORDER
        }
        self.body_ids = {
            spec.distal_body: mujoco.mj_name2id(
                self.model,
                mujoco.mjtObj.mjOBJ_BODY,
                spec.distal_body,
            )
            for spec in SPECS
        }
        assert all(joint_id >= 0 for joint_id in self.joint_ids.values())
        assert all(body_id >= 0 for body_id in self.body_ids.values())

        for spec in SPECS:
            joint_id = self.joint_ids[spec.joint]
            np.testing.assert_allclose(
                self.model.jnt_axis[joint_id],
                EXPECTED_AXES[spec.joint],
            )
            np.testing.assert_allclose(
                self.model.jnt_range[joint_id],
                self.retargeter.joints[spec.joint]["range"],
            )
            assert self.retargeter.FEATURE_MAPPING[spec.joint][0] == spec.feature

        self.set_q(TEST_KINEMATIC_REFERENCE)
        self.thumb_reference = self.body_point("thumb_link3")
        self.middle_reference = self.body_point("middle_link2")
        self.opposition_reference = 0.5 * (
            self.body_point("index_link3")
            + self.body_point("middle_link2")
        )

    def set_q(self, q: np.ndarray) -> None:
        q = np.asarray(q, dtype=np.float64)
        assert q.shape == (11,)
        self.data.qpos[:] = q
        self.data.qvel[:] = 0.0
        mujoco.mj_forward(self.model, self.data)

    def body_point(self, body_name: str) -> np.ndarray:
        """Return the distal body's physical COM in SG100 base-local frame."""
        return self.data.xipos[self.body_ids[body_name]].copy()

    def relative_body_rotation(self, joint_name: str) -> np.ndarray:
        joint_id = self.joint_ids[joint_name]
        body_id = int(self.model.jnt_bodyid[joint_id])
        parent_id = int(self.model.body_parentid[body_id])
        body_rotation = self.data.xmat[body_id].reshape(3, 3)
        parent_rotation = self.data.xmat[parent_id].reshape(3, 3)
        return parent_rotation.T @ body_rotation

    def canonical_q(self, feature: str, value: float) -> np.ndarray:
        state = CanonicalHandState(**{feature: value})
        return self.retargeter.retarget(state)

    def metric(self, spec: SemanticSpec) -> float:
        point = self.body_point(spec.distal_body)
        if spec.metric == "base_z":
            return float(point[2])
        if spec.metric == "base_y":
            return float(point[1])
        if spec.metric == "thumb_to_opposition_distance":
            return float(np.linalg.norm(point - self.opposition_reference))
        if spec.metric == "index_to_thumb_distance":
            return float(np.linalg.norm(point - self.thumb_reference))
        if spec.metric == "little_to_middle_distance":
            return float(np.linalg.norm(point - self.middle_reference))
        raise AssertionError(f"unknown physical metric: {spec.metric}")


def rotation_vector(rotation: np.ndarray) -> np.ndarray:
    return Rotation.from_matrix(rotation).as_rotvec()


def assert_strict_monotonic(values, direction: str, tolerance=1e-6) -> None:
    differences = np.diff(np.asarray(values, dtype=np.float64))
    if direction == "increasing":
        assert np.all(differences > tolerance), differences
    elif direction == "decreasing":
        assert np.all(differences < -tolerance), differences
    else:
        raise AssertionError(f"unknown monotonic direction: {direction}")


def direct_joint_results(harness: PhysicalSemanticHarness):
    results = {}
    for spec in SPECS:
        joint_id = harness.joint_ids[spec.joint]
        qpos_address = int(harness.model.jnt_qposadr[joint_id])
        lower, upper = harness.model.jnt_range[joint_id]
        direct_samples = (float(lower), float(0.5 * (lower + upper)), float(upper))
        axis = harness.model.jnt_axis[joint_id].copy()

        harness.set_q(TEST_KINEMATIC_REFERENCE)
        reference_rotation = harness.relative_body_rotation(spec.joint)
        measured_angles = []
        distal_points = []
        for value in direct_samples:
            q = TEST_KINEMATIC_REFERENCE.copy()
            q[qpos_address] = value
            harness.set_q(q)
            current_rotation = harness.relative_body_rotation(spec.joint)
            delta = reference_rotation.T @ current_rotation
            rotvec = rotation_vector(delta)
            projected_angle = float(np.dot(rotvec, axis))
            residual = np.linalg.norm(rotvec - projected_angle * axis)
            assert abs(projected_angle - value) < 1e-6
            assert residual < 1e-7
            measured_angles.append(projected_angle)
            distal_points.append(harness.body_point(spec.distal_body))

        assert_strict_monotonic(measured_angles, "increasing")
        assert np.linalg.norm(distal_points[0] - distal_points[-1]) > 1e-3
        results[spec.joint] = {
            "range": (float(lower), float(upper)),
            "axis": axis,
            "angles": tuple(measured_angles),
        }
    return results


def canonical_physical_results(harness: PhysicalSemanticHarness):
    results = {}
    neutral_q = harness.retargeter.retarget(CanonicalHandState.neutral())
    np.testing.assert_allclose(neutral_q, TEST_KINEMATIC_REFERENCE)

    for spec in SPECS:
        joint_index = SG100_JOINT_ORDER.index(spec.joint)
        joint_id = harness.joint_ids[spec.joint]
        axis = harness.model.jnt_axis[joint_id].copy()

        harness.set_q(TEST_KINEMATIC_REFERENCE)
        reference_rotation = harness.relative_body_rotation(spec.joint)
        q_values = []
        physical_metrics = []
        geometry_angles = []

        for semantic_value in spec.samples:
            q = harness.canonical_q(spec.feature, semantic_value)
            changed = np.flatnonzero(np.abs(q - neutral_q) > 1e-12)
            if semantic_value == 0.0:
                assert changed.size == 0
            else:
                np.testing.assert_array_equal(changed, [joint_index])

            harness.set_q(q)
            q_values.append(float(q[joint_index]))
            physical_metrics.append(harness.metric(spec))

            current_rotation = harness.relative_body_rotation(spec.joint)
            delta = reference_rotation.T @ current_rotation
            signed_angle = float(np.dot(rotation_vector(delta), axis))
            assert abs(signed_angle - q[joint_index]) < 1e-6
            geometry_angles.append(signed_angle)

        assert_strict_monotonic(physical_metrics, spec.monotonic)

        # Signed distal channels must traverse lower -> zero -> upper.
        if "distal" in spec.feature:
            lower, upper = harness.model.jnt_range[joint_id]
            np.testing.assert_allclose(q_values, [lower, 0.0, upper])
            assert_strict_monotonic(geometry_angles, "increasing")

        results[spec.joint] = {
            "feature": spec.feature,
            "semantic": spec.semantic,
            "q": tuple(q_values),
            "metric": spec.metric,
            "metric_values": tuple(physical_metrics),
            "geometry_angles": tuple(geometry_angles),
        }
    return results


def run_regression():
    harness = PhysicalSemanticHarness()
    direct = direct_joint_results(harness)
    canonical = canonical_physical_results(harness)
    return direct, canonical


def test_sg100_physical_joint_semantics():
    run_regression()


def main():
    direct, canonical = run_regression()
    print("SG100 PHYSICAL JOINT SEMANTICS: PASS")
    print()
    print("JOINT       SEMANTIC                 DIRECT  CANONICAL")
    for spec in SPECS:
        assert spec.joint in direct
        assert spec.joint in canonical
        print(f"{spec.joint:12s}{spec.semantic:25s}PASS    PASS")
    print()
    print("CANONICAL PHYSICAL METRICS (low, mid, high)")
    for spec in SPECS:
        result = canonical[spec.joint]
        values = ", ".join(f"{value:+.6f}" for value in result["metric_values"])
        print(f"{spec.joint:12s}{result['metric']:34s}{values}")


if __name__ == "__main__":
    main()
