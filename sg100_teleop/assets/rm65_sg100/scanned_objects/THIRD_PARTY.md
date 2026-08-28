# Third-Party Scanned-Object Assets

Source repository: https://github.com/kevinzakka/mujoco_scanned_objects
Upstream commit: `6ff8d275cebfd5b47e49685e3cfbe64b20e49a3c` (main)

The 3D assets (obj + png) originate from Google's Scanned Objects
Dataset and are licensed **CC-BY 4.0**
(https://creativecommons.org/licenses/by/4.0/).  The repository's
MJCF XML files are MIT licensed.  Use, modification, and
redistribution of the assets are permitted with attribution; this
file is that attribution record.

Reference citation for the dataset:

```bibtex
@misc{downs2022scannedobjects,
  url = {https://arxiv.org/abs/2204.11918},
  author = {Downs, Laura and Francis, Anthony and Koenig, Nate and
            Kinman, Brandon and Hickman, Ryan and Reymann, Krista and
            McHugh, Thomas B. and Vanhoucke, Vincent},
  title = {Google Scanned Objects: A High-Quality Dataset of 3D
           Scanned Household Items},
  publisher = {arXiv},
  year = {2022},
}
```

Only `model.obj` (visual mesh) and `texture.png` were copied per
selected model.  The upstream 32-piece V-HACD collision meshes were
NOT copied - every demo object uses primitive collision proxies for
MuJoCo contact stability (see each demo objects XML header).

## Per-object record

### OBJECT 1 - demo_hammer (DEMO 01, Hammer Peg Assembly)

| field | value |
|---|---|
| task alias | `demo_hammer` |
| original name | `Cole_Hardware_Hammer_Black` |
| upstream path | `models/Cole_Hardware_Hammer_Black/` |
| upstream commit | `6ff8d275cebfd5b47e49685e3cfbe64b20e49a3c` |
| license | CC-BY 4.0 (obj/png) |
| files copied | `model.obj`, `texture.png` |
| raw AABB | 291.7 x 60.0 x 96.1 mm (x/y/z, metres raw) |
| raw unit | metres (verified against a real mallet) |
| applied scale | 1.0 (none) |
| final AABB in scene | 291.7 x 60.0 x 96.1 mm |
| mesh transform in body | identity (body pose carries placement) |
| collision proxy | handle capsule + head cylinder (Ø58 x 96, axis local Z) |
| mass / density change | explicit inertial 0.40 kg, hand-tuned diaginertia |

### OBJECT 2 - demo_drawer_item (DEMO 02, Drawer Loading & Closing)

| field | value |
|---|---|
| task alias | `demo_drawer_item` |
| original name | `SNAIL_MEASURING_TAPE` |
| upstream path | `models/SNAIL_MEASURING_TAPE/` |
| upstream commit | `6ff8d275cebfd5b47e49685e3cfbe64b20e49a3c` |
| license | CC-BY 4.0 (obj/png) |
| files copied | `model.obj`, `texture.png` |
| raw AABB | 63.9 x 194.5 x 65.7 mm |
| raw unit | metres |
| applied scale | 1.0 (none) |
| final AABB in scene | 63.9 x 194.5 x 65.7 mm |
| mesh transform in body | identity |
| collision proxy | single box 64 x 195 x 66 mm |
| mass / density change | explicit inertial 0.25 kg |

### OBJECT 3 - syrup bottle - REMOVED FROM DEMO V2

`Black_Elderberry_Syrup_54_oz_Gaia_Herbs` (CC-BY 4.0,
`models/Black_Elderberry_Syrup_54_oz_Gaia_Herbs/` @
`6ff8d275cebfd5b47e49685e3cfbe64b20e49a3c`) was used as DEMO 03's
round bottle (V1 task alias `kit_object_a`, cylinder proxy d58,
0.42 kg).  **Demo V2 removes it** - the scene was reduced to two
task objects (box + screwdriver) for grasp contrast and workspace
legibility, and the asset directory was deleted from the repo.
No syrup-bottle files ship in V2; this entry remains as the
attribution/dispatch record only.

### OBJECT 4 - kit_object_a (DEMO 03)

| field | value |
|---|---|
| task alias | `kit_object_a` (rectangular box, pinch grasp; renamed from V1 `kit_object_b`) |
| original name | `ReadytoUse_Rolled_Fondant_Pure_White_24_oz_box` |
| upstream path | `models/ReadytoUse_Rolled_Fondant_Pure_White_24_oz_box/` |
| upstream commit | `6ff8d275cebfd5b47e49685e3cfbe64b20e49a3c` |
| license | CC-BY 4.0 (obj/png) |
| files copied | `model.obj`, `texture.png` |
| raw AABB | 121.7 x 49.1 x 168.7 mm |
| raw unit | metres |
| applied scale | 1.0 (none) |
| final AABB in scene | 121.7 x 49.1 x 168.7 mm |
| mesh transform in body | identity |
| collision proxy | box 122 x 49 x 169 mm |
| mass / density change | explicit inertial 0.55 kg |

### OBJECT 5 - kit_object_b (DEMO 03)

| field | value |
|---|---|
| task alias | `kit_object_b` (elongated tool, tripod grasp; renamed from V1 `kit_object_c`) |
| original name | `Craftsman_Grip_Screwdriver_Phillips_Cushion` |
| upstream path | `models/Craftsman_Grip_Screwdriver_Phillips_Cushion/` |
| upstream commit | `6ff8d275cebfd5b47e49685e3cfbe64b20e49a3c` |
| license | CC-BY 4.0 (obj/png) |
| files copied | `model.obj`, `texture.png` |
| raw AABB | 260.4 x 36.3 x 37.9 mm |
| raw unit | metres |
| applied scale | 1.0 (none) |
| final AABB in scene | 260.4 x 36.3 x 37.9 mm |
| mesh transform in body | body quat = settled rest pose (local +X along world -Y, small rest tilt); body pos == keyframe == USDZ |
| collision proxy | grip capsule (d35 x 94) + shaft capsule (d12) |
| mass / density change | explicit inertial 0.13 kg |
