# SG100 Demo Scene Suite (V8-derived)

Branch: `feature/v8-demo-scene-suite` (from tag `demo_display_version`,
commit `dc380e7fca7fce948e4dd04288d37f3dd5a813c9`)

Three demo scenes derived from the frozen V8 production scene
(`rm65_sg100_avp_grasp_scene.xml`).  They add **tool use,
articulated-environment manipulation, and heterogeneous grasping**
on top of pick-and-place, while keeping every V8 contract frozen:

| frozen contract | value |
|---|---|
| robot include | `rm65_sg100_avp_grasp_robot.xml` (unchanged) |
| controllable DoF | nu = 17 (RM65 joint_1..6 + SG100 11 hand joints) |
| grasp-ready pose | V8 runtime startup IK solution, baked as each scene's `demo_ready` keyframe (arm `[0.348188, -1.361485, -2.286026, -0.102464, 2.074488, -0.049589]`, hand all-zero open).  Joint `ref` values untouched - zero semantics preserved. |
| cameras | `workstation_front` (world, fovy 50, VLA key `base_0_rgb`), `wrist_camera` (on `sg100_base_link`, fovy 75, key `right_wrist_0_rgb`, sensor-only - no housing/bracket/collision), `demo_camera` (world, fovy 60, presentation only) |
| light | single `workstation_overhead`, directional, castshadow off, headlight off |
| workstation | `grasp_table` 900 x 1000 x 40 mm, top face z = 0; charcoal visual floor |
| timestep / integrator | 0.002 s implicitfast, noslip 3 |

Runner usage - `run_avp_arm_hand_sim.py` already exposes overrides,
no runner changes were needed:

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
  the V8 grasp-ready posture (verified to 2e-6 m).
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

| item | value |
|---|---|
| SCENE XML | `rm65_sg100_demo_hammer_peg_assembly_scene.xml` |
| OBJECT XML | `rm65_sg100_demo_hammer_peg_assembly_objects.xml` |
| USDZ | `../usdz/rm65_sg100_demo_hammer_peg_assembly_compat.usdz` |
| SCANNED OBJECTS USED | `Cole_Hardware_Hammer_Black` (CC-BY 4.0) |
| hammer dimensions | 292 x 60 x 96 mm; handle 200 x ~25 mm oval; head Ø58 x 96 (cylinder, strike faces at local +-Z) |
| hammer mass | 0.40 kg (light mallet class) |
| hammer collision | handle capsule + head cylinder proxies, visual = scan |
| fixture | primitive base plate + split pedestal with 30 mm open slot (peg visible its whole travel), static |
| peg | shaft Ø24 x 60 + strike cap Ø48 x 12; slide joint on -Z, qpos = insertion depth 0..40 mm; frictionloss 6 N, damping 5 |
| peg diameter / travel | Ø24 shaft / 40 mm travel (exposed 45 -> 5 mm) |
| initial poses | hammer at tool rest (0.34, -0.12) strike-face-down; peg exposed 45 mm at fixture (0.52, 0.20) |
| GOAL | 2-3 controlled mallet taps drive the peg to >= 75 % insertion, then hammer back on the rest |
| SUCCESS CRITERIA | `demo_peg_slide >= 0.030` AND hammer still on the table (computed in `demo_task_validation.evaluate_demo01`) |
| KEY WAYPOINTS | hammer_pregrasp / hammer_grasp / hammer_lift / hammer_prestrike / hammer_strike / hammer_return (see `demo_task_validation.DEMO_WAYPOINTS["01"]`) |
| IK REACHABILITY | 6/6 REACHABLE (position-only = wrist-roll teleop task); min joint margin 0.224 rad (hammer_lift); tightest joints J2/J3 with >= 0.22 rad everywhere |
| CAMERA CONTRACT | identical to V8 (byte-equal definitions; `test_demo_scene_camera_contract`) |
| READY POSE REGRESSION | 17-D qpos max error 0 vs V8 runtime startup; link_6 / sg100_base_link FK error 0 (`test_demo_scene_robot_contract`) |
| PHYSICS VALIDATION | 3 s static drift ~0; peg no-contact drift 0.0000 mm (< 1 mm required); 0.5 m/s tap -> ~19 mm/strike, 2 strikes to >= 75 %, 3 strikes to full insertion; no NaN/explosions (`test_demo_scene_physics`) |
| USDZ VALIDATION | package self-contained, robot prims = V8 layout, initial transforms = keyframe (2e-6), mesh AABB matches MuJoCo (1e-6) (`test_demo_scene_usdz_contract`) |
| AVP DYNAMIC PROP STATUS | ROBOT supported; FREE OBJECTS (hammer) supported; PEG (slide joint) supported |
| PREVIEW DIRECTORY | `recording/demo_scene_previews/hammer_peg_assembly/` |

---

## DEMO 02 - Drawer Retrieval & Tray Loading

*"Open an articulated drawer with a large bar handle, release the
handle, retrieve an item from inside, and load it into a target
tray."*

**PURPOSE / SG100 CAPABILITY**: hook/power grasp on the bar handle,
articulated-environment interaction, release-and-regrasp, grasp-mode
transition (hook -> power grasp), sequential manipulation.

| item | value |
|---|---|
| SCENE XML | `rm65_sg100_demo_drawer_retrieval_and_loading_scene.xml` |
| OBJECT XML | `rm65_sg100_demo_drawer_retrieval_and_loading_objects.xml` |
| USDZ | `../usdz/rm65_sg100_demo_drawer_retrieval_and_loading_compat.usdz` |
| SCANNED OBJECTS USED | `SNAIL_MEASURING_TAPE` (CC-BY 4.0) |
| fixture | open-top cabinet at (0.62, 0.18) 250 x 320 x 150 mm (back + sides + base, open top so cameras see in) |
| drawer travel | 150 mm on -X (qpos = open depth); damping 8, frictionloss 4 N |
| handle | horizontal bar Ø30 x 140 mm, 48 mm clear of the drawer front, mounted low (world z = 0.0415) so the frozen V8 ready hand clears it by ~21 mm |
| retrieval object | tape measure 64 x 195 x 66 mm, 0.25 kg, collision = one rounded-box proxy |
| object placement | on the drawer bottom toward the front third (0.60, 0.18); after full open the body sits at (0.45, 0.18) with 53 mm finger clearance to the drawer front |
| GOAL | STAGE 1 drawer >= 80 % open; STAGE 2 tape COM inside the goal tray; FINAL tape stable in tray |
| SUCCESS CRITERIA | `demo_drawer_slide >= 0.120` AND tape COM in tray interior (`evaluate_demo02`) |
| tray | open shallow industrial tray 230 x 170 mm, orange target mark, no lid/pocket, at (0.33, -0.22) |
| KEY WAYPOINTS | handle_pregrasp / handle_grasp / drawer_open / object_pregrasp / object_grasp / tray_goal |
| IK REACHABILITY | 6/6 REACHABLE; min joint margin 0.486 rad - the most relaxed scene |
| CAMERA CONTRACT | identical to V8 |
| READY POSE REGRESSION | 0 error (as DEMO 01) |
| PHYSICS VALIDATION | closed drawer drift 0.0000 mm in 3 s (no self-open); 25 N pull opens to 150.6 mm with the arm parked; release holds 150.2 mm (no spring-back); tape rides the drawer, settles stable in the tray |
| USDZ VALIDATION | as DEMO 01 (all three contract tests pass) |
| AVP DYNAMIC PROP STATUS | ROBOT supported; FREE OBJECTS (tape) supported; DRAWER (prismatic) supported |
| PREVIEW DIRECTORY | `recording/demo_scene_previews/drawer_retrieval_and_loading/` |

Teleop-usability notes (PART 41): handle reachable; pull direction
(-X, toward the operator) intuitive; no arm-table collision at any
waypoint; no extreme wrist orientation needed (position-only IK
solves every waypoint); the interior object is visible from both
workstation_front (open cabinet top) and wrist_camera.  During the
pull the operator naturally retracts the hand with the handle - the
frozen V8 ready-pose hand pocket overlaps the drawer-front sweep
path, which is why the scripted test parks the arm upright first.

---

## DEMO 03 - Maintenance Kit Packing

*"Use different grasp strategies to pick heterogeneous maintenance
items and pack them into designated tray locations with coarse pose
requirements."*

**PURPOSE / SG100 CAPABILITY**: the point is **not** A/B/C sorting -
the same SG100 hand switches three visibly different grasp
morphologies in one sequence:

* **OBJECT A -> whole-hand / power grasp** (four-finger wrap +
  thumb opposition around a d58 round body).  A parallel gripper
  squeezes such a bottle off-centre or drops it; the anthropomorphic
  wrap self-centres it.
* **OBJECT B -> multi-finger pinch / lateral grasp** (thumb vs
  index+middle across 49 mm faces).  Selective finger activation -
  little finger stays out of the way - is exactly what 11-DoF
  hands buy over 1-DoF grippers.
* **OBJECT C -> tripod / elongated-object grasp** (thumb + index +
  middle around the 30 mm cushion grip) plus a **carry rotation**:
  the tool starts along Y and must land along the slot axis X.

| item | value |
|---|---|
| SCENE XML | `rm65_sg100_demo_maintenance_kit_packing_scene.xml` |
| OBJECT XML | `rm65_sg100_demo_maintenance_kit_packing_objects.xml` |
| USDZ | `../usdz/rm65_sg100_demo_maintenance_kit_packing_compat.usdz` |
| SCANNED OBJECTS USED | `Black_Elderberry_Syrup_54_oz_Gaia_Herbs`, `ReadytoUse_Rolled_Fondant_Pure_White_24_oz_box`, `Craftsman_Grip_Screwdriver_Phillips_Cushion` (all CC-BY 4.0) |
| OBJECT A | round bottle 58 x 59 x 145 mm, 0.42 kg, cylinder proxy, standing at (0.31, 0.30) - orientation free |
| OBJECT B | rectangular box 122 x 49 x 169 mm, 0.55 kg, box proxy, standing at (0.41, 0.32) - coarse orientation |
| OBJECT C | screwdriver 260 x 36 x 38 mm, 0.13 kg (grip 29-36 x 112 mm), grip + shaft capsule proxies, lying along -Y at (0.50, 0.24), grip toward the operator |
| tray | open two-row industrial tray 440 x 330 mm at (0.42, -0.19): zone A circle Ø115, zone B 160 x 110, zone C slot 320 x 88 along X; dividers are 8 mm low strips; paints visual-only - no closed pockets, no invisible walls |
| target clearance | A: (115-58)/2 = 28.5 mm; B: 19 / 30.5 mm; C: 30 / 25 mm (spec 15-30 mm) |
| elongated yaw tolerance | slot axis = X; success requires tool-axis yaw within 20 deg |
| GOAL | A in zone A, B in zone B, C in zone C with yaw <= 20 deg, all stable |
| SUCCESS CRITERIA | `evaluate_demo03` zone membership + yaw |
| KEY WAYPOINTS | A/B/C grasp + goal pairs (six waypoints) |
| IK REACHABILITY | 6/6 REACHABLE; min joint margin 0.244 rad (zone A approach); all waypoints inside radius 0.57 m - the wrist-roll position-only workspace from the V8 seed is ~0.59 m at z = 0.12, which drove the final layout |
| CAMERA CONTRACT | identical to V8 |
| READY POSE REGRESSION | 0 error (as DEMO 01) |
| PHYSICS VALIDATION | 3 s rest motion <= 0.6 mm per object; gentle zone placement satisfies all three zone memberships + yaw; no object-object or tray interference |
| USDZ VALIDATION | as DEMO 01 (all three contract tests pass) |
| AVP DYNAMIC PROP STATUS | ROBOT supported; FREE OBJECTS (A/B/C) supported; DRAWER/PEG n/a |
| PREVIEW DIRECTORY | `recording/demo_scene_previews/maintenance_kit_packing/` |

No sub-centimetre insertion anywhere; every target zone is a marked
area, not a pocket.

---

## Previews

`recording/demo_scene_previews/<demo>/` contains, per scene:
workstation_front / wrist / demo PNGs for READY and the task beats
(hammer grasp/prestrike/strike/peg before-after; drawer
closed/open/object-grasp/tray-goal; kit initial / A / B / C grasps /
complete), plus `smoke_<camera>.mp4` camera reels.  All MP4s for one
demo are rendered from a single recorded state timeline
(`mujoco_recording.npz`, replayed - physics is never re-run per
camera).  The smoke reels are kinematic puppets (arm interpolating
audited IK waypoints, grasped objects welded to the palm frame) -
camera checks, not policy replays.

## Tests

| file | env | covers |
|---|---|---|
| `sg100_teleop/tests/test_demo_scene_camera_contract.py` | mujoco-learn | PART 10 camera freeze vs V8 |
| `sg100_teleop/tests/test_demo_scene_robot_contract.py` | mujoco-learn | nu=17, joints/actuators, keyframe = V8 ready, FK |
| `sg100_teleop/tests/test_demo_scene_physics.py` | mujoco-learn | statics + per-demo task physics |
| `sg100_teleop/tests/test_demo_scene_usdz_contract.py` | .venv_usdz | USDZ package/naming/pose/AABB |

## Known limitations

* The smoke MP4s use scripted kinematic puppets, not closed-loop
  grasping; the physical strike/insertion numbers come from the
  scripted-impulse physics tests, not from the reels.
* The drawer-open smoke reel drives the slide joint kinematically;
  the physical pull test (25 N, full travel, holds position) is the
  authoritative check.
* Object B (box) is grasped standing; a very timid release could tip
  it - the 122 x 49 mm base keeps this rare.
* The V8 production test
  `test_v8_object_collisions.py::test_objects_remain_stable_on_table_for_three_seconds`
  fails on this machine **on the pristine baseline tag as well**
  (pre-existing, unrelated to this suite; reported, not fixed).
* A concurrent human edit to `run_avp_arm_hand_sim.DEFAULT_MODEL`
  (scene-switch comments) exists in the working tree; it was not
  made or reverted by this task.
