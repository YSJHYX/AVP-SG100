from pathlib import Path
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[2]

SRC = ROOT / "sg100_teleop/assets/sg100/mjcf/sg100.xml"
DST = ROOT / "sg100_teleop/assets/sg100/mjcf/sg100_sim.xml"


# 第一版仿真参数：
# 目的不是拟合真实 SG100 motor dynamics，
# 而是建立稳定的 position-control validation backend。
TIMESTEP = 0.002       # 500 Hz physics
JOINT_DAMPING = 0.002
JOINT_ARMATURE = 0.001
POSITION_KP = 2.0
POSITION_DAMP_RATIO = 1.0


tree = ET.parse(SRC)
root = tree.getroot()

root.set("model", "SG100V3_sim")


# -------------------------------------------------------------------
# Simulation options
# -------------------------------------------------------------------

option = root.find("option")

if option is None:
    option = ET.Element("option")
    compiler = root.find("compiler")

    children = list(root)
    insert_idx = children.index(compiler) + 1 if compiler is not None else 0
    root.insert(insert_idx, option)

option.set("timestep", str(TIMESTEP))
option.set("integrator", "implicitfast")
option.set("gravity", "0 0 -9.81")


# -------------------------------------------------------------------
# Joints
# -------------------------------------------------------------------

joints = root.findall(".//joint")

if len(joints) != 11:
    raise RuntimeError(
        f"Expected 11 SG100 joints, found {len(joints)}"
    )

joint_names = []

for joint in joints:
    name = joint.attrib["name"]
    joint_names.append(name)

    joint.set("damping", str(JOINT_DAMPING))
    joint.set("armature", str(JOINT_ARMATURE))


# -------------------------------------------------------------------
# Visual geometry
#
# The imported CAD meshes are visualization only.
#
# contype=0 / conaffinity=0 means they participate in no contacts.
#
# Later we will add dedicated simplified collision geoms separately.
# -------------------------------------------------------------------

geoms = root.findall(".//geom")

for i, geom in enumerate(geoms):
    mesh_name = geom.attrib.get("mesh", f"geom_{i}")

    geom.set("name", f"{mesh_name}_visual")
    geom.set("group", "1")

    geom.set("contype", "0")
    geom.set("conaffinity", "0")


# -------------------------------------------------------------------
# Position actuators
# -------------------------------------------------------------------

old_actuator = root.find("actuator")

if old_actuator is not None:
    root.remove(old_actuator)

actuator = ET.SubElement(root, "actuator")

for joint_name in joint_names:
    ET.SubElement(
        actuator,
        "position",
        {
            "name": f"{joint_name}_pos",
            "joint": joint_name,

            # Position servo
            "kp": str(POSITION_KP),

            # Approximately critically damped response.
            "dampratio": str(POSITION_DAMP_RATIO),

            # Automatically inherit joint range as control range.
            "inheritrange": "1",
        },
    )


# -------------------------------------------------------------------
# Save
# -------------------------------------------------------------------

ET.indent(tree, space="  ")
tree.write(
    DST,
    encoding="unicode",
    xml_declaration=False,
)

print("===== SG100 SIM MJCF GENERATED =====")
print("source :", SRC)
print("output :", DST)

print()
print("joints    :", len(joints))
print("visuals   :", len(geoms))
print("actuators :", len(joint_names))

print()
print("physics timestep :", TIMESTEP)
print("joint damping    :", JOINT_DAMPING)
print("joint armature   :", JOINT_ARMATURE)
print("position kp      :", POSITION_KP)
print("position damping :", POSITION_DAMP_RATIO)

print()
print("Joint order:")

for i, name in enumerate(joint_names):
    print(f"  [{i:02d}] {name}")
