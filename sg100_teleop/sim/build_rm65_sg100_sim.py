from __future__ import annotations

from copy import deepcopy
from pathlib import Path
import xml.etree.ElementTree as ET

import mujoco


REPO_ROOT = Path(__file__).resolve().parents[2]

RM65_XML = (
    REPO_ROOT
    / "sg100_teleop"
    / "assets"
    / "RM65-B"
    / "mjcf"
    / "rm65_sim.xml"
)

SG100_XML = (
    REPO_ROOT
    / "sg100_teleop"
    / "assets"
    / "sg100"
    / "mjcf"
    / "sg100_sim.xml"
)

OUTPUT = (
    REPO_ROOT
    / "sg100_teleop"
    / "assets"
    / "rm65_sg100"
    / "mjcf"
    / "rm65_sg100_sim.xml"
)


RM65_JOINTS = [
    "joint_1",
    "joint_2",
    "joint_3",
    "joint_4",
    "joint_5",
    "joint_6",
]

SG100_JOINTS = [
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
]

EXPECTED_JOINT_ORDER = RM65_JOINTS + SG100_JOINTS


def find_body(root: ET.Element, name: str) -> ET.Element:
    for body in root.iter("body"):
        if body.attrib.get("name") == name:
            return body

    raise RuntimeError(f"Body not found: {name}")


def rewrite_rm65_mesh_paths(root: ET.Element) -> None:
    """
    rm65_sim.xml originally lives inside RM65-B/mjcf.

    The combined XML lives inside rm65_sg100/mjcf, so the
    relative asset paths must be rewritten.
    """
    asset = root.find("asset")

    if asset is None:
        raise RuntimeError("RM65 asset block not found")

    for mesh in asset.findall("mesh"):
        file_name = Path(mesh.attrib["file"]).name

        mesh.set(
            "file",
            f"../../RM65-B/meshes/{file_name}",
        )


def import_sg100_assets(
    combined_root: ET.Element,
    sg_root: ET.Element,
) -> dict[str, str]:
    """
    Copy all SG100 mesh assets into the combined model.

    Prefix every SG100 mesh name so it can never collide with
    RM65 asset names such as 'base_link'.
    """
    combined_asset = combined_root.find("asset")

    if combined_asset is None:
        combined_asset = ET.Element("asset")
        combined_root.insert(1, combined_asset)

    sg_asset = sg_root.find("asset")

    if sg_asset is None:
        raise RuntimeError("SG100 asset block not found")

    mesh_name_map: dict[str, str] = {}

    for mesh in sg_asset.findall("mesh"):
        old_name = mesh.attrib["name"]
        new_name = f"sg100_{old_name}"

        mesh_name_map[old_name] = new_name

        copied = deepcopy(mesh)
        copied.set("name", new_name)

        file_name = Path(mesh.attrib["file"]).name

        copied.set(
            "file",
            f"../../sg100/meshes_mujoco/{file_name}",
        )

        combined_asset.append(copied)

    return mesh_name_map


def rewrite_mesh_references(
    element: ET.Element,
    mesh_name_map: dict[str, str],
) -> None:
    for geom in element.iter("geom"):
        mesh_name = geom.attrib.get("mesh")

        if mesh_name in mesh_name_map:
            geom.set(
                "mesh",
                mesh_name_map[mesh_name],
            )


def attach_sg100_worldbody(
    combined_root: ET.Element,
    sg_root: ET.Element,
    mesh_name_map: dict[str, str],
) -> None:
    """
    Recover the SG100 base_link frame as a fixed child of
    RM65 link_6.

    The SG100 URDF root was previously fixed to world, which is
    why its base geom and four finger chains appear directly
    under SG100 worldbody.
    """
    rm_link6 = find_body(
        combined_root,
        "link_6",
    )

    sg_worldbody = sg_root.find("worldbody")

    if sg_worldbody is None:
        raise RuntimeError("SG100 worldbody not found")

    sg_base = ET.SubElement(
        rm_link6,
        "body",
        {
            "name": "sg100_base_link",

            # First combined-kinematics assumption:
            #
            # RM65 link_6 mounting frame == SG100 base_link
            # mounting frame.
            #
            # SG100 base_link CAD already includes the adapter.
            "pos": "0 0 0",
            "quat": "1 0 0 0",
        },
    )

    # Copy SG100 base/root geoms.
    for geom in sg_worldbody.findall("geom"):
        copied_geom = deepcopy(geom)

        if copied_geom.attrib.get("name") == "base_link_visual":
            copied_geom.set(
                "name",
                "sg100_base_link_visual",
            )

        # SG100 base was previously fixed to world, so its mass
        # did not affect standalone dynamics. For this first
        # gravity-free kinematic validation, keep the base CAD
        # visual-only and massless.
        copied_geom.set("mass", "0")

        rewrite_mesh_references(
            copied_geom,
            mesh_name_map,
        )

        sg_base.append(copied_geom)

    # Copy the four complete SG100 finger subtrees.
    for body in sg_worldbody.findall("body"):
        copied_body = deepcopy(body)

        rewrite_mesh_references(
            copied_body,
            mesh_name_map,
        )

        sg_base.append(copied_body)


def import_sg100_actuators(
    combined_root: ET.Element,
    sg_root: ET.Element,
) -> None:
    combined_actuator = combined_root.find("actuator")

    if combined_actuator is None:
        combined_actuator = ET.SubElement(
            combined_root,
            "actuator",
        )

    sg_actuator = sg_root.find("actuator")

    if sg_actuator is None:
        raise RuntimeError("SG100 actuator block not found")

    for actuator in sg_actuator:
        combined_actuator.append(
            deepcopy(actuator)
        )


def validate_model(model: mujoco.MjModel) -> None:
    print("\n===== COMBINED MODEL =====")
    print("nq    =", model.nq)
    print("nv    =", model.nv)
    print("nu    =", model.nu)
    print("njnt  =", model.njnt)
    print("nbody =", model.nbody)
    print("ngeom =", model.ngeom)

    if model.nq != 17:
        raise RuntimeError(
            f"Expected nq=17, got {model.nq}"
        )

    if model.nv != 17:
        raise RuntimeError(
            f"Expected nv=17, got {model.nv}"
        )

    if model.nu != 17:
        raise RuntimeError(
            f"Expected nu=17, got {model.nu}"
        )

    if model.njnt != 17:
        raise RuntimeError(
            f"Expected njnt=17, got {model.njnt}"
        )

    actual_joint_order = []

    print("\nJOINT ORDER:")

    for jid in range(model.njnt):
        name = mujoco.mj_id2name(
            model,
            mujoco.mjtObj.mjOBJ_JOINT,
            jid,
        )

        actual_joint_order.append(name)

        print(
            f"[{jid:02d}] "
            f"{name:<12} "
            f"qposadr={model.jnt_qposadr[jid]} "
            f"range={model.jnt_range[jid].tolist()}"
        )

    if actual_joint_order != EXPECTED_JOINT_ORDER:
        raise RuntimeError(
            "Unexpected combined joint order.\n"
            f"Expected: {EXPECTED_JOINT_ORDER}\n"
            f"Actual:   {actual_joint_order}"
        )

    print("\nACTUATOR ORDER:")

    for aid in range(model.nu):
        name = mujoco.mj_id2name(
            model,
            mujoco.mjtObj.mjOBJ_ACTUATOR,
            aid,
        )

        print(
            f"[{aid:02d}] "
            f"{name:<18} "
            f"ctrlrange="
            f"{model.actuator_ctrlrange[aid].tolist()}"
        )

    base_id = mujoco.mj_name2id(
        model,
        mujoco.mjtObj.mjOBJ_BODY,
        "sg100_base_link",
    )

    if base_id < 0:
        raise RuntimeError(
            "sg100_base_link body not found"
        )

    print("\nSG100 BASE BODY:")
    print("body id =", base_id)

    print("\nVALIDATION: PASS")


def build() -> None:
    print("===== BUILD RM65 + SG100 =====")
    print("RM65 :", RM65_XML)
    print("SG100:", SG100_XML)
    print("OUT  :", OUTPUT)

    rm_tree = ET.parse(RM65_XML)
    sg_tree = ET.parse(SG100_XML)

    combined_root = deepcopy(
        rm_tree.getroot()
    )

    sg_root = sg_tree.getroot()

    combined_root.set(
        "model",
        "RM65-SG100-sim",
    )

    # --------------------------------------------------------
    # First combined validation is deliberately gravity-free.
    # We are validating:
    #
    #   hierarchy
    #   joint order
    #   actuator order
    #   mounting orientation
    #   17DoF control
    #
    # not real arm-hand dynamics.
    # --------------------------------------------------------
    option = combined_root.find("option")

    if option is None:
        option = ET.Element("option")
        combined_root.insert(0, option)

    option.set("timestep", "0.002")
    option.set("integrator", "implicitfast")
    option.set("gravity", "0 0 0")

    rewrite_rm65_mesh_paths(
        combined_root
    )

    mesh_name_map = import_sg100_assets(
        combined_root,
        sg_root,
    )

    attach_sg100_worldbody(
        combined_root,
        sg_root,
        mesh_name_map,
    )

    import_sg100_actuators(
        combined_root,
        sg_root,
    )

    OUTPUT.parent.mkdir(
        parents=True,
        exist_ok=True,
    )

    tree = ET.ElementTree(
        combined_root
    )

    ET.indent(
        tree,
        space="  ",
    )

    tree.write(
        OUTPUT,
        encoding="utf-8",
        xml_declaration=True,
    )

    print("\nXML WRITE: PASS")

    model = mujoco.MjModel.from_xml_path(
        str(OUTPUT)
    )

    print("MUJOCO COMPILE: PASS")

    validate_model(model)


if __name__ == "__main__":
    build()
