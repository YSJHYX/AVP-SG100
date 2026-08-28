# SG100 Demo Scene Suite (V8-derived, V2)

Branch: `feature/v8-demo-scene-suite` (from tag `demo_display_version`,
commit `dc380e7fca7fce948e4dd04288d37f3dd5a813c9`)

Three demo scenes derived from the frozen V8 production scene
(`rm65_sg100_avp_grasp_scene.xml`).  They add **tool use,
articulated-environment manipulation, and heterogeneous grasping**
on top of pick-and-place, while keeping every V8 contract frozen:

| frozen contract | value |
|---|---|
| robot include | `rm65_sg100_demo_robot.xml` - identical to `rm65_sg100_avp_grasp_robot.xml` EXCEPT hand-contact geom friction (see below); joints/mass/actuators/cameras byte-identical (`test_demo_sg100_contact_friction`) |
| controllable DoF | nu = 17 (RM65 joint_1..6 + SG100 11 hand joints) |
| grasp-ready pose | V8 runtime startup IK solution, baked as each scene's `demo_ready` keyframe (arm `[0.348188, -1.361485, -2.286026, -0.102464, 2.074488, -0.049589]`, hand all-zero open).  Joint `ref` values untouched - zero semantics preserved. |
| cameras | `workstation_front` (world, fovy 50, VLA key `base_0_rgb`), `wrist_camera` (on `sg100_base_link`, fovy 75, key `right_wrist_0_rgb`, sensor-only - no housing/bracket/collision), `demo_camera` (world, fovy 60, presentation only) |
| light | single `workstation_overhead`, directional, castshadow off, headlight off |
| workstation | `grasp_table` 900 x 1000 x 40 mm, top face z = 0; charcoal visual floor |
| timestep / integrator | 0.002 s implicitfast, noslip 3 |

**V2 change - SG100 hand friction (PART A)**: production leaves the
three distal capsules (thumb/index/little link3) at the MuJoCo
default 1.0 / priority 0 and the five contact pads at 1.2, which
let grasped tools creep in the first AVP test.  The demo robot
gives all eight hand-contact geoms `friction="1.4 0.01 0.001"
priority="1"` (spec target ~1.2, ceiling 1.5; torsional/rolling
kept at production values).  Production files are untouched.

**V2 change - DEMO_INITIAL_STATE_EXPORT_CONTRACT (PART E)**: every
free task object's XML default qpos equals its `demo_ready`
keyframe entry, so the passthrough converter (which builds from
`qpos0`) and the keyframe/USDZ export path place objects
identically - this was the root cause of the V1 AVP hammer pose
mismatch.  Verified by `test_initial_state_qpos0_matches_keyframe`;
per-scene world-pose references are written to
`recording/demo_scene_previews/<demo>/initial_pose_manifest.json`,
and the full MJCF-vs-USDZ body table (worst error 0.000 mm /
0.000 deg) is `recording/demo_scene_previews/usdz_transform_table.md`.

Runner usage - `run_avp_arm_hand_sim.py` already exposes overrides,
no runner changes were needed.  **Always pass `--usdz`**: without
it the streamer rebuilds the stage through its built-in XML
converter (nested articulated bodies - the V1 AVP drawer
scatter).  See `AVP_DEMO_V2_CHECKLIST.md` for the field-test
procedure.

```bash
python -m sg100_teleop.sim.run_avp_arm_hand_sim --ip <AVP_IP> \
    --model sg100_teleop/assets/rm65_sg100/mjcf/<demo_scene>.xml \
    --usdz  sg100_teleop/assets/rm65_sg100/usdz/<demo>_compat.usdz \
    --wrist-roll-ik
```

Validation entry point (success metrics + waypoint IK audit):

```bash
conda run -n mujoco-learn python -m sg100_teleop.sim.demo_task_validation \
    --demo 01 --ik-audit     # 02 / 03 likewise
```

Shared technical notes:
* Task objects use the V8 collision classes (object 2/11, fixture
  4/3) so finger / table / arm / object / fixture contacts all stay
  enabled exactly as in V8; visual meshes never collide.
* All task bodies are **flat worldbody children** (AVP USDZ
  contract forbids nested runtime bodies).
* Every scene carries exactly one keyframe, `demo_ready`; USDZ
  exports use `--initial-keyframe demo_ready`, so the USDZ opens in
  the V8 grasp-ready posture (V2 acceptance 1 mm / 0.5 deg; actual
  0.000 mm / 0.000 deg for every streamed body).
* AVP dynamic props: the Tracking Streamer streams the world pose of
  **every named body**, so the hammer, drawer body, peg, and kit
  objects all animate on AVP out of the box (no runtime change).

---

## DEMO 01 - Hammer Peg Assembly

*"Use a whole-hand power grasp to pick up a hammer, orient the tool,
and drive a large assembly peg into a fixture through repeated
controlled impacts."*

**PURPOSE / SG100 CAPABILITY**: tool use, whole-hand power grasp,
thumb opposition + four-finger wrap + palm support, grasp stability
under impact, wrist orientation, arm-hand coordination.  This is
GRASP TOOL -> ORIENT TOOL -> USE TOOL -> CONTACT INTERACTION ->
PLACE TOOL - not a move-A-to-B task.

**V2 change - LOW TOOL CRADLE (PART B)**: the flat tool rest is
replaced by a two-pier cradle (head support 40 mm + handle-end
support 44.5 mm) at (0.40, -0.12).  The hammer settles on it with
the grip span mid-air: 50.7-64.8 mm of clear space under the open
grip zone (spec >= 30 mm), i.e. the hammer sits ~32 mm higher than
V1 - fingers close from the sides without table contact, the
failure mode behind "hammer hard to grasp" in the first AVP test.
3 s settle from the keyframe: 0.008 mm / 0.06 deg.

| item | value |
|---|---|
| SCENE XML | `rm65_sg100_demo_hammer_peg_assembly_scene.xml` |
| OBJECT XML | `rm65_sg100_demo_hammer_peg_assembly_objects.xml` |
| USDZ | `../usdz/rm65_sg100_demo_hammer_peg_assembly_compat.usdz` |
| SCANNED OBJECTS USED | `Cole_Hardware_Hammer_Black` (CC-BY 4.0) |
| hammer dimensions | 292 x 60 x 96 mm; handle 200 x ~25 mm oval; head Ø58 x 96 (cylinder, strike faces at local +-Z) |
| hammer mass | 0.40 kg (light mallet class) |
| hammer collision | handle capsule + head cylinder proxies, visual = scan |
| hammer rest pose | settled on the cradle at (0.340, -0.120, 0.0385), strike axis vertical; body pos == keyframe == USDZ |
| fixture | primitive base plate + split pedestal with 30 mm open slot (peg visible its whole travel), static |
| peg | shaft Ø24 x 60 + strike cap Ø48 x 12; slide joint on -Z, qpos = insertion depth 0..40 mm; frictionloss 6 N, damping 5 |
| peg diameter / travel | Ø24 shaft / 40 mm travel (exposed 45 -> 5 mm) |
| GOAL | 2-3 controlled mallet taps drive the peg to >= 75 % insertion, then hammer back on the cradle |
| SUCCESS CRITERIA | `demo_peg_slide >= 0.030` AND hammer still on the table (computed in `demo_task_validation.evaluate_demo01`) |
| KEY WAYPOINTS | hammer_pregrasp (z 0.12) / hammer_grasp (z 0.077, raised-contact height) / hammer_lift / hammer_prestrike / hammer_strike / hammer_return (see `demo_task_validation.DEMO_WAYPOINTS["01"]`) |
| IK REACHABILITY | 6/6 REACHABLE (position-only = wrist-roll teleop task); min joint margin 0.204 rad |
| CAMERA CONTRACT | identical to V8 (byte-equal definitions; `test_demo_scene_camera_contract`) |
| READY POSE REGRESSION | 17-D qpos max error 0 vs V8 runtime startup; link_6 / sg100_base_link FK error 0 (`test_demo_scene_robot_contract`) |
| PHYSICS VALIDATION | 3 s cradle settle 0.066 mm / < 0.1 deg (spec < 2 mm / < 2 deg); 0.5 m/s tap -> ~19 mm/strike, 2 strikes to >= 75 %; no NaN/explosions (`test_demo_scene_physics`) |
| USDZ VALIDATION | package self-contained, robot prims = V8 layout, initial transforms = keyframe within 1 mm / 0.5 deg incl. handle-axis and head-axis semantic check, mesh AABB matches MuJoCo (`test_demo_scene_usdz_contract`) |
| AVP DYNAMIC PROP STATUS | ROBOT supported; FREE OBJECTS (hammer) supported; PEG (slide joint) supported |
| PREVIEW DIRECTORY | `recording/demo_scene_previews/hammer_peg_assembly/` |

---

## DEMO 02 - Drawer Loading & Closing

*"Open the drawer, take the item from the table, load it into the
drawer, and close the drawer."*

**PURPOSE / SG100 CAPABILITY**: hook grasp on a horizontal bar
handle, articulated-environment interaction, release-and-regrasp,
grasp-mode transition (hook -> power grasp -> hook), sequential
multi-stage manipulation.  (V1's retrieve-from-inside task is
retired - files renamed to
`rm65_sg100_demo_drawer_loading_and_closing_*`.)

**V2 changes (PART C)**:
* Handle is now a **horizontal industrial bar** - Ø30 x 130 mm,
  axis along Y, world z = 0.1365, mounted on two end posts on the
  drawer front (0 deg off horizontal).  The V1 low vertical loop
  collided with the frozen V8 ready hand; the raised horizontal
  bar leaves 11 mm+ clearance and reads as a proper pull handle.
* The drawer is ONE rigid moving body: a single slide joint
  (`demo_drawer_slide`, 0..150 mm, damping 8, frictionloss 4 N)
  owns all eight moving geoms (bottom, 2 sides, back, front,
  handle bar, 2 posts) - one USDZ Xform moves the whole assembly,
  which is what stops the AVP scatter.
* The tape now starts ON THE TABLE at (0.30, -0.18); the goal
  region is the drawer interior itself (drawer-local frame), so
  success is state-independent of how far the drawer is open.

| item | value |
|---|---|
| SCENE XML | `rm65_sg100_demo_drawer_loading_and_closing_scene.xml` |
| OBJECT XML | `rm65_sg100_demo_drawer_loading_and_closing_objects.xml` |
| USDZ | `../usdz/rm65_sg100_demo_drawer_loading_and_closing_compat.usdz` |
| SCANNED OBJECTS USED | `SNAIL_MEASURING_TAPE` (CC-BY 4.0) |
| fixture | open-top cabinet at (0.62, 0.18) 250 x 320 x 150 mm (back + sides + base, open top so cameras see in) |
| drawer travel | 150 mm on -X (qpos = open depth); damping 8, frictionloss 4 N |
| handle | horizontal bar Ø30 x 130 mm at world z = 0.1365, two end posts on the drawer front; hook grasp from above |
| task object | tape measure 64 x 195 x 66 mm, 0.25 kg, collision = one rounded-box proxy; starts ON THE TABLE at (0.30, -0.18) |
| GOAL | PHASE 1 drawer >= 80 % open; PHASE 2 tape inside the drawer cavity (drawer-local goal region); PHASE 3 drawer closed with the tape inside |
| SUCCESS CRITERIA | `demo_drawer_slide >= 0.120` AND tape in the drawer-local region AND `demo_drawer_slide <= 0.030` at the end (`evaluate_demo02`) |
| KEY WAYPOINTS | handle_pregrasp / handle_grasp / drawer_open / table_object_pregrasp / table_object_grasp / table_object_lift / drawer_load_prepose / drawer_load / handle_regrasp / drawer_close (10 waypoints) |
| IK REACHABILITY | 10/10 REACHABLE; min joint margin 0.213 rad |
| CAMERA CONTRACT | identical to V8 |
| READY POSE REGRESSION | 0 error (as DEMO 01) |
| PHYSICS VALIDATION | 25 N pull opens to 150.6 mm with the arm parked clear (IK park at (0.30, -0.10, 0.25)); release holds 150.2 mm (no spring-back); tape placed at the cavity centre closes the loop - push closes to < 1 mm with the tape riding inside, drawer-local tape position (0.03, 0.00) inside the goal region (`test_demo_scene_physics.test_demo02_open_load_close_success`) |
| USDZ VALIDATION | as DEMO 01 plus every drawer geom's world pose checked at closed state and rigid-assembly check at half/full open (`test_drawer_usdz_parts_match_mujoco`) |
| AVP DYNAMIC PROP STATUS | ROBOT supported; FREE OBJECTS (tape) supported; DRAWER ROOT (slide joint) supported - required so the whole drawer follows on AVP |
| PREVIEW DIRECTORY | `recording/demo_scene_previews/drawer_loading_and_closing/` |

Teleop-usability notes: handle reachable at hook height; pull/push
direction (-X / +X) intuitive; during the pull the operator
retracts the hand with the bar, and during close the handle
travels +X away from the parked arm - no arm-drawer collision at
any waypoint.

---

## DEMO 03 - Maintenance Kit Packing

*"Use different grasp strategies to pick heterogeneous maintenance
items and pack them into designated tray locations with coarse pose
requirements."*

**PURPOSE / SG100 CAPABILITY**: the same SG100 hand switches two
visibly different grasp morphologies in one sequence:

* **OBJECT A (box) -> multi-finger pinch / lateral grasp** (thumb
  vs index+middle across the 49 mm faces).  Selective finger
  activation - little finger stays out of the way - is exactly
  what 11-DoF hands buy over 1-DoF grippers.
* **OBJECT B (screwdriver) -> tripod / elongated-object grasp**
  (thumb + index + middle around the 30 mm cushion grip) plus a
  **carry rotation**: the tool starts along Y and must land along
  the slot axis X.

**V2 change (PART D)**: the syrup bottle (OBJECT C) is **removed
from the demo** (assets deleted) - three crowded targets made the
scene hard to read and the round bottle offered no grasp contrast
beyond the box.  Two objects now sit on opposite sides of the
workspace with a clear approach corridor between them, and at
READY the hand has >= 56 mm surface clearance to every task
object (spec >= 40 mm, `mj_geomDistance`-measured: box 56.1 mm,
screwdriver grip 96.9 mm / shaft 138.8 mm).

| item | value |
|---|---|
| SCENE XML | `rm65_sg100_demo_maintenance_kit_packing_scene.xml` |
| OBJECT XML | `rm65_sg100_demo_maintenance_kit_packing_objects.xml` |
| USDZ | `../usdz/rm65_sg100_demo_maintenance_kit_packing_compat.usdz` |
| SCANNED OBJECTS USED | `ReadytoUse_Rolled_Fondant_Pure_White_24_oz_box`, `Craftsman_Grip_Screwdriver_Phillips_Cushion` (CC-BY 4.0) |
| OBJECT A | rectangular box 122 x 49 x 169 mm, 0.55 kg, box proxy, standing at (0.44, 0.34) - coarse orientation |
| OBJECT B | screwdriver 260 x 36 x 38 mm, 0.13 kg (grip 29-36 x 112 mm), grip + shaft capsule proxies, lying along -Y at (0.54, 0.13), grip toward +X |
| tray | two-zone industrial tray 440 x 330 mm at (0.42, -0.19): zone A rectangle 180 x 120 (centre (0.32, -0.12), blue), zone B slot 320 x 88 along X (centre (0.42, -0.295), green); dividers are 8 mm low strips; paints visual-only - no closed pockets, no invisible walls |
| target clearance | A: 30 / 58.5 mm vs box; B: 25 / 30 mm vs screwdriver |
| elongated yaw tolerance | slot axis = X; success requires tool-axis yaw within 20 deg |
| GOAL | box in zone A, screwdriver in zone B with yaw <= 20 deg, both stable |
| SUCCESS CRITERIA | `evaluate_demo03` two-zone membership + yaw |
| KEY WAYPOINTS | box_pregrasp / box_grasp / box_goal + screwdriver_pregrasp / screwdriver_grasp / screwdriver_lift / screwdriver_reorient / screwdriver_goal (8 waypoints) |
| IK REACHABILITY | 8/8 REACHABLE; min joint margin 0.204 rad; all waypoints inside radius 0.57 m - the wrist-roll position-only workspace from the V8 seed is ~0.59 m at z = 0.12, which drove the layout |
| CAMERA CONTRACT | identical to V8 |
| READY POSE REGRESSION | 0 error (as DEMO 01) |
| PHYSICS VALIDATION | 3 s rest: both objects stable (no drift); gentle zone placement satisfies both zone memberships + yaw; no object-object or tray interference |
| USDZ VALIDATION | as DEMO 01 |
| AVP DYNAMIC PROP STATUS | ROBOT supported; FREE OBJECTS (A/B) supported; DRAWER/PEG n/a |
| PREVIEW DIRECTORY | `recording/demo_scene_previews/maintenance_kit_packing/` |

No sub-centimetre insertion anywhere; every target zone is a marked
area, not a pocket.

---

## Previews

`recording/demo_scene_previews/<demo>/` contains, per scene:
workstation_front / wrist / demo PNGs for READY and the task beats
(Demo 01: ready / hammer_grasp / prestrike / strike / peg
before-after; Demo 02: drawer_closed_ready / handle_grasp / open /
table_object_grasp / loading / loaded / closed_final; Demo 03:
initial / box_grasp / screwdriver_grasp / reorient / complete),
plus `smoke_<camera>.mp4` camera reels.  All MP4s for one demo are
rendered from a single recorded state timeline
(`mujoco_recording.npz`, replayed - physics is never re-run per
camera).  The smoke reels are kinematic puppets (arm interpolating
audited IK waypoints, grasped objects welded to the palm frame) -
camera checks, not policy replays.  Composition was verified by
segmentation rendering (every cabinet/drawer/handle/object geom
confirmed visible in all three cameras).

## Tests

| file | env | covers |
|---|---|---|
| `sg100_teleop/tests/test_demo_scene_camera_contract.py` | mujoco-learn | camera freeze vs V8 |
| `sg100_teleop/tests/test_demo_scene_robot_contract.py` | mujoco-learn | nu=17, joints/actuators, keyframe = V8 ready, FK |
| `sg100_teleop/tests/test_demo_sg100_contact_friction.py` | mujoco-learn | V2 PART A: production friction unchanged; demo robot differs in friction ONLY |
| `sg100_teleop/tests/test_demo_scene_v2_contracts.py` | mujoco-learn | V2: qpos0==keyframe, cradle clearance+stability, horizontal handle, single-rigid drawer, tape-on-table, goal-inside-drawer, exactly-two-objects, READY clearance, two-zone success |
| `sg100_teleop/tests/test_demo_scene_physics.py` | mujoco-learn | statics + per-demo task physics incl. open-load-close cycle |
| `sg100_teleop/tests/test_demo_scene_usdz_contract.py` | .venv_usdz | USDZ package/naming/pose (1 mm / 0.5 deg)/drawer rigid assembly/AABB |

## Known limitations

* The smoke MP4s use scripted kinematic puppets, not closed-loop
  grasping; the physical strike/insertion and drawer-cycle numbers
  come from the scripted-force physics tests, not from the reels.
* The drawer open/close smoke reel drives the slide joint
  kinematically; the physical 25 N pull / push cycle (full travel,
  holds position, tape rides along) is the authoritative check.
* Demo-suite tip friction 1.4 is a demonstration-only tuning;
  production keeps 1.2/1.0 (the production behaviour is frozen).
* The V8 production test
  `test_v8_object_collisions.py::test_objects_remain_stable_on_table_for_three_seconds`
  fails on this machine **on the pristine baseline tag as well**
  (pre-existing, unrelated to this suite; reported, not fixed).
* A concurrent human edit to `run_avp_arm_hand_sim.DEFAULT_MODEL`
  (scene-switch comments) exists in the working tree; it was not
  made or reverted by this task.
