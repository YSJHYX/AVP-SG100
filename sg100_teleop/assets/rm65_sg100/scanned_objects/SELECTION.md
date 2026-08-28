# Scanned-Object Candidate Inventory & Selection

Upstream: kevinzakka/mujoco_scanned_objects @
`6ff8d275cebfd5b47e49685e3cfbe64b20e49a3c`
(1030 models from Google Scanned Objects, CC-BY 4.0)

Selection criteria (task spec PART 15-16): SG100-actually-graspable,
teleop-forgiving, grasp-strategy-legible, real-task semantics, no
millimetre alignment, visible in workstation_front + wrist_camera,
stable primitive collision proxy, sane mass/size.  Geometry rules:
grasp width >= 25-30 mm, handle length >= 100 mm, no ring handles /
thin paper / transparent / deformable / sub-10 mm precision features.

Sixteen candidates were measured (raw AABB from `model.obj`, metres).
Five were selected; totals stay inside the 4-6 model budget.

## Measured inventory

| model | raw AABB (mm) | class | verdict |
|---|---|---|---|
| Cole_Hardware_Hammer_Black | 292 x 60 x 96 | HAND_TOOL / HAMMER_OR_MALLET | **SELECTED** (DEMO 01 mallet: handle 200 mm x ~25 mm oval, head Ø58 x 96, faces +-Z) |
| SNAIL_MEASURING_TAPE | 64 x 195 x 66 | COMPACT_OBJECT | **SELECTED** (DEMO 02 drawer item: 64 mm grasp width, flat bottom, no roll) |
| Black_Elderberry_Syrup_54_oz_Gaia_Herbs | 60 x 59 x 145 | CYLINDRICAL | **REMOVED FROM DEMO V2** (was DEMO 03 A; the bottle was dropped when Demo 03 went to two objects - assets deleted from the repo) |
| ReadytoUse_Rolled_Fondant_Pure_White_24_oz_box | 122 x 49 x 169 | RECTANGULAR | **SELECTED** (DEMO 03 A since V2: 49 mm pinch faces, plain pack - parts-box look) |
| Craftsman_Grip_Screwdriver_Phillips_Cushion | 260 x 36 x 38 | ELONGATED / HAND_TOOL | **SELECTED** (DEMO 03 B: cushion grip 29-36 mm x 112 mm, shaft never grasped) |
| HeavyDuty_Flashlight | 78 x 78 x 229 | ELONGATED | rejected: 44-76 mm body too chunky vs screwdriver's clean 30 mm grip |
| Don_Franciscos_Gourmet_Coffee (12 oz) | 103 x 103 x 141 | CYLINDRICAL | rejected: Ø103 too wide for SG100 wrap (palm ~90 mm) |
| Nescafe_Tasters_Choice (7 oz) | 97 x 79 x 186 | CYLINDRICAL | rejected: oval 97 x 79, tippy standing |
| Phillips_Milk_of_Magnesia | 73 x 57 x 174 | CYLINDRICAL | rejected: oval cross-section weakens wrap demo |
| Black_Elderberry alt. Krill_Oil | 55 x 54 x 98 | CYLINDRICAL | rejected: shorter twin of the selected syrup; no added value |
| Crunch_Girl_Scouts candy box | 139 x 63 x 164 | RECTANGULAR | rejected: near-duplicate of selected fondant box, busier texture |
| Pony_C_Clamp_1440 | 202 x 173 x 27 | HAND_TOOL | rejected: flat clamp, awkward grasp, no clean strategy story |
| HAMMER_BALL | 284 x 185 x 121 | OTHER | rejected: "Arm & Hammer"-brand product, not a tool |
| HAMMER_PEG | 252 x 186 x 89 | OTHER | rejected: same - brand name, not hardware |
| Arm_Hammer_Diaper_Pail_Refills | (brand product) | OTHER | rejected: not a hammer |
| Shurtape / 3M tape rolls | ~ 60-190 wide | OTHER | rejected: tape rolls roll; spec discourages rolling objects |

## HAMMER SCANNED MODEL: FOUND

`Cole_Hardware_Hammer_Black` is a genuine rubber mallet (cylindrical
head, axis local +Z, Ø58 x 96 mm, flat strike faces at local z = 0
and z = 0.096; handle along +X, oval ~20 x 28 mm, effective grasp
length ~200 mm).  Verified by mesh slicing plus rendered visual
inspection; numbers recorded in the DEMO 01 objects XML header and
THIRD_PARTY.md.
