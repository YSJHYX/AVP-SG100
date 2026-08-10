from pathlib import Path
import xml.etree.ElementTree as ET

import mujoco


REPO_ROOT = Path(__file__).resolve().parents[2]

SOURCE = (
    REPO_ROOT
    / "sg100_teleop"
    / "assets"
    / "RM65-B"
    / "mjcf"
    / "rm65.xml"
)

OUTPUT = (
    REPO_ROOT
    / "sg100_teleop"
    / "assets"
    / "RM65-B"
    / "mjcf"
    / "rm65_sim.xml"
)

JOINT_NAMES = [
    "joint_1",
    "joint_2",
    "joint_3",
    "joint_4",
    "joint_5",
    "joint_6",
]


def build() -> None:
    tree = ET.parse(SOURCE)
    root = tree.getroot()

    root.set("model", "RM65-B-sim")

    # ------------------------------------------------------------
    # Simulation options
    #
    # Stage B2 is only for kinematics/control validation.
    # Gravity is intentionally disabled for now.
    # ------------------------------------------------------------
    option = root.find("option")
    if option is None:
        option = ET.Element("option")
        root.insert(0, option)

    option.set("timestep", "0.002")
    option.set("integrator", "implicitfast")
    option.set("gravity", "0 0 0")

    # ------------------------------------------------------------
    # Joint numerical stabilization.
    #
    # These are validation parameters, NOT physical RM65 motor
    # parameters.
    # ------------------------------------------------------------
    joints = {
        joint.attrib["name"]: joint
        for joint in root.iter("joint")
        if "name" in joint.attrib
    }

    missing = [name for name in JOINT_NAMES if name not in joints]
    if missing:
        raise RuntimeError(f"Missing RM65 joints: {missing}")

    for name in JOINT_NAMES:
        joint = joints[name]
        joint.set("damping", "0.1")
        joint.set("armature", "0.01")

    # ------------------------------------------------------------
    # Imported CAD meshes are visual-only during validation.
    #
    # Simplified collision geometry will be introduced later.
    # ------------------------------------------------------------
    for index, geom in enumerate(root.iter("geom")):
        if not geom.get("name"):
            geom.set("name", f"rm65_visual_{index}")

        geom.set("group", "1")
        geom.set("contype", "0")
        geom.set("conaffinity", "0")

    # ------------------------------------------------------------
    # Rebuild actuator block.
    # ------------------------------------------------------------
    old_actuator = root.find("actuator")
    if old_actuator is not None:
        root.remove(old_actuator)

    actuator = ET.SubElement(root, "actuator")

    for name in JOINT_NAMES:
        ET.SubElement(
            actuator,
            "position",
            {
                "name": f"{name}_pos",
                "joint": name,
                "kp": "50",
                "dampratio": "1",
                "inheritrange": "1",
            },
        )

    ET.indent(tree, space="  ")
    tree.write(
        OUTPUT,
        encoding="utf-8",
        xml_declaration=True,
    )

    print("===== RM65 SIM BUILD =====")
    print("SOURCE:", SOURCE)
    print("OUTPUT:", OUTPUT)

    model = mujoco.MjModel.from_xml_path(str(OUTPUT))

    print("\nCOMPILE: PASS")
    print("nq    =", model.nq)
    print("nv    =", model.nv)
    print("nu    =", model.nu)
    print("njnt  =", model.njnt)
    print("nbody =", model.nbody)
    print("ngeom =", model.ngeom)

    print("\nACTUATORS:")
    for aid in range(model.nu):
        actuator_name = mujoco.mj_id2name(
            model,
            mujoco.mjtObj.mjOBJ_ACTUATOR,
            aid,
        )

        print(
            f"[{aid:02d}] "
            f"{actuator_name:<16} "
            f"ctrlrange={model.actuator_ctrlrange[aid].tolist()}"
        )


if __name__ == "__main__":
    build()
