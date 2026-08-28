# AVP Demo Suite V2 - Field-Test Checklist

Branch `feature/v8-demo-scene-suite`, V2 revision (2026-08-28).
V2 fixes the three issues found in the first AVP field test:
hammer pose mismatch in the AVP view, the drawer scattering on
AVP, and the hammer being hard to grasp.

## 0. Launch (every demo) - THE critical fix

```bash
python -m sg100_teleop.sim.run_avp_arm_hand_sim --ip <AVP_IP> \
    --model sg100_teleop/assets/rm65_sg100/mjcf/<demo_scene>.xml \
    --usdz  sg100_teleop/assets/rm65_sg100/usdz/<demo>_compat.usdz \
    --wrist-roll-ik
```

- [ ] **`--usdz` IS PASSED.**  Running without `--usdz` falls back
  to the streamer's built-in XML converter, which rebuilds the
  stage from `qpos0` into nested articulated bodies - this was the
  root cause of the V1 hammer pose mismatch and the drawer
  scattering.  V2 also makes every XML default pose equal the
  `demo_ready` keyframe (`DEMO_INITIAL_STATE_EXPORT_CONTRACT`), so
  both paths now agree numerically - but the prebuilt USDZ remains
  the required path (flat single-body Xforms, verified 0.000 mm /
  0.000 deg against MuJoCo).
- [ ] The AVP scene opens with the robot in the V8 ready pose,
  hand open, and the task objects at the positions below
  (compare against `recording/demo_scene_previews/<demo>/
  workstation_front_*.png`).
- [ ] Free objects, the drawer, and the peg all follow the sim
  (streamed dynamic props; no frozen props).

## 1. DEMO 01 - Hammer Peg Assembly

What changed in V2: the hammer now rests raised on a **two-pier
cradle** (head support + handle-end support, piers 40 / 44.5 mm
tall).  The open grip span between the piers has 51-64 mm of
clearance under the handle - fingers close from the sides without
touching anything, and the SG100 fingertip friction is raised
(1.0 -> 1.4, priority 1) to stop grasp slip.

Initial state (must match the AVP view):
- [ ] Hammer ON the cradle at (0.34, -0.12), handle raised, head
  strike axis vertical, grip span mid-air.
- [ ] Peg exposed ~45 mm in the fixture at (0.52, 0.20).

Teleop sequence and checks:
- [ ] Power grasp: hand descends vertically onto the raised
  handle, fingers wrap, hammer lifts straight up off the piers
  (no snags, no need to slide it off).
- [ ] Carry to the peg; 2-3 controlled taps drive the peg in
  (>= 75 % = success; MuJoCo physics: ~19 mm/strike).
- [ ] Hammer returns to the cradle and stays put when released.
- [ ] Grasp slip: if the hammer slides in the hand during
  lift/strike, note the phase - friction was raised from 1.0 to
  1.4 exactly for this; anything worse than a slow creep is a
  finding.

## 2. DEMO 02 - Drawer Loading & Closing (NEW TASK)

The V1 retrieval-and-tray task is retired.  New sequence:
**open the drawer -> take the tape from the table -> load it into
the drawer -> close the drawer.**

What changed in V2: the handle is now a **horizontal industrial
bar** (Ø30 x 130 mm, world z = 0.1365, two end posts on the
drawer front) instead of the low vertical loop, and the drawer is
one rigid moving body (single slide joint owning all 8 moving
geoms) - that plus the prebuilt USDZ is what stops the AVP
scattering.

Initial state (must match the AVP view):
- [ ] Drawer closed, horizontal bar handle on the drawer front,
  nothing floating or detached.
- [ ] Tape measure lying ON THE TABLE at (0.30, -0.18) - NOT
  inside the drawer.

Teleop sequence and checks:
- [ ] Hook grasp over the horizontal bar (palm over the bar,
  fingers curl down behind it), pull -X: drawer opens its full
  150 mm and stays open when released.
- [ ] Release the handle, power-grasp the tape on the table,
  lift, carry over the open cavity, lower and release - tape
  ends up INSIDE the drawer cavity (goal region is the drawer
  interior, it rides along with the drawer).
- [ ] Re-hook the bar (it has travelled -X with the open drawer)
  and push +X until the drawer fully closes with the tape inside.
- [ ] During the pull/push the handle must stay one piece with
  the drawer (no gap opening between posts and front panel).

## 3. DEMO 03 - Maintenance Kit Packing (two objects)

What changed in V2: the syrup bottle is **removed**.  Exactly two
task objects remain, placed on opposite sides of the workspace
with a clear approach corridor between them; at READY the hand
has >= 56 mm surface clearance to every object.

Initial state (must match the AVP view):
- [ ] Exactly TWO objects: white box standing at (0.44, 0.34)
  and screwdriver lying at (0.54, 0.13).  No bottle anywhere.
- [ ] Hand nowhere near either object at READY.

Teleop sequence and checks:
- [ ] Pinch-grasp the box mid-height, pack it in zone A (blue
  plane, centre (0.32, -0.12)).
- [ ] Tripod-grasp the screwdriver cushion grip, carry, land it
  along zone B (green strip, centre (0.42, -0.295), long axis
  roughly along X; yaw within 20 deg = success).
- [ ] Both objects stay put after release.

## 4. Result log (fill per run)

| demo | pass / fail | phases failed | notes (slip, lag, pose mismatch, scatter) |
|---|---|---|---|
| 01 | | | |
| 02 | | | |
| 03 | | | |

Report any pose mismatch with: demo, camera view (AVP vs
workstation_front PNG), which body, approximate offset - the
reference manifests are
`recording/demo_scene_previews/<demo>/initial_pose_manifest.json`
and the full body table is
`recording/demo_scene_previews/usdz_transform_table.md`
(all errors 0.000 mm / 0.000 deg at export time).
