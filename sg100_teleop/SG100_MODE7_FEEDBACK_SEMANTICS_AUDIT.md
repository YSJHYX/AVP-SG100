# SG100 Mode-7 Feedback Semantics Audit

**Date:** 2026-09-03 (rev 7: J6 active PASS; J7 software ready)
**Method:** static SDK/protocol/binary audit plus read-only review of the
preserved real artifact `recording/sg100_mode7_probe_20260903_163543`.  
**Hardware execution by this Agent:** none. The user-produced J6 active
artifact was reviewed read-only; no homing/zeroing or new hardware command was
performed.
**Decision:** J6 active hold and +0.5° identity command **PASS — REAL
HARDWARE**; J7 sign flip **SOFTWARE READY / HARDWARE PENDING**; exact MODE7
feedback-to-canonical map `F_fb` **PENDING**.

本轮方向已改变：人工掰动的
`sg100_mode7_mapping_characterization.py` 只保留为历史工具，不再是下一条
正式硬件路径，也不得由 Agent 执行。J6 的
`sg100_mode7_active_micro_motion.py` 真机流程已由用户完成并通过复核，不得重复。
J7 复用同一工具，但 active seed 是已知 MODE7 zero-output SDK command target
0°，而不是 raw feedback；canonical −0.5° 经一次 F_cmd 变为 SDK +0.5°。

`F_cmd` 与 `F_fb` 仍完全分离：N3 只为 identity-command candidate 和 MODE7
delta channel/sign 产生待复核证据，绝不从 `F_cmd` 逆推 `F_fb`，也不自动更新
production map。`feedback_map_valid=False`、`canonical_valid=False`、dataset
`QUARANTINED` 保持不变。

## J6 ACTIVE EVIDENCE（REAL HARDWARE）

- active hold command `85346273598375 ns`；J6 baseline/hold feedback
  `-0.659181535°`，settled/peak excursion 0°。
- micro command `85362800446342 ns`；首个 fresh RX 在 4.255 ms 后，J6
  settled delta `+0.252689660°`、peak `0.367402673°`。
- dominant J6；second J10=0°（其余十轴同为 0°）；dominance ratio 因第二
  通道为零而为工具 epsilon 下的 `2.5269e8`，即有效无限大。
- active hold/micro 全 11 RX mode 持续为 7；phase 内 sequence 连续，无
  nonfinite、stall、excessive excursion、fault。
- 五次 SetHandControl 顺序与工具 contract 完全一致，cleanup MODE_NULL
  submission accepted。用户确认 hold 无可见运动，micro 仅 J6 朝掌心轻微屈曲。

因此 J6 identity F_cmd channel、physical direction 和 MODE7 feedback response
均为 SUPPORTED/PASS — REAL HARDWARE；此结论不外推到其余十轴或 F_fb。

## CRC TRANSIENT

`[RS485Adapter] CRC error` 来自主机 RX parser。CRC 不等时控制流不会调用
上层 callback，而是移除坏 header 后继续找下一帧；没有 TX 失败，也没有显式
retry。J6 关键采样窗口全连续且 fault/mode/position 正常，分类为
`CRC_TRANSIENT_RECOVERED`。旧 artifact 未记录控制台时间戳，因此不能声称与
某一 CSV 行精确对齐；新 wrapper 仅监听 SDK 自有输出并记录 count/timestamps，
没有自定义 RS485 实现。

The vendor guidance for this audit is **CONFIRMED** (user-vendor
communication, 2026-09-03):

- while a joint is in `MODE_JOINT_POSITION = 7`, returned position should be
  the correct joint angle;
- in other joint modes, returned position may be raw/accumulated data;
- the mode-7 entry contract is id+mode only: populate the 11 joint IDs and
  use mode 7 for every joint (rev 3).

The real artifact proves the mode-dependent change on this unit: MODE7 values
are finite, stable, bounded joint-angle-scale values instead of the MODE_NULL
multi-turn coordinate, and the transition produced no visible movement.
That does not establish each channel's exact physical identity, sign, zero, or
range convention. Those four properties are the separate pending `F_fb` map.

## PALM REALTIME VS JOINT MODE

`SwitchToRealtimeMode()` changes the **palm transport/runtime state**. In the
audited SDK it sends the palm resume command (`0x25`), waits for its ACK, and
restarts the 10 ms communication scheduler. It does not mean that any joint is
in joint control mode 7.

`MODE_JOINT_POSITION = 7` is a **per-joint control mode byte** carried in each
`Joint_Control_t` from PC to palm to joint board. A hand can therefore have:

- palm realtime active while every joint reports `MODE_NULL = 0` (the N0/N1
  passive setup, enforced on the outbound control frame);
- palm realtime active with mode 7 on some or all joint boards; or
- mixed returned joint modes while a command transition propagates.

The existing vendor logs contain 96,657 valid realtime feedback frames with
mixed per-joint modes, confirming that palm runtime mode and joint control mode
cannot be represented by one flag.

## MODE FIELD

**Enum values:**

- `MODE_NULL = 0`
- `MODE_POSITION = 1`
- `MODE_JOINT_POSITION = 7`

**`Joint_Status.mode` source:** vendor protocol types define
`MotorUp_t` as the `Joint -> PC` joint-board status structure, with byte 1 named
the current control mode. `Joint_Status_t`, used in the palm's whole-hand
response, has the same 12-byte layout and the same `id`, `mode`, `position`,
velocity, and current fields. The palm aggregates those per-joint records into
`Hand_Status_t`.

The current host binary's `ProtocolHandler::UpdateHandStatus` first copies the
complete 132-byte payload into its status cache. It then visits only the
position float at offset 2 of each 12-byte joint record to apply the injected
position decoder. It does not rewrite the byte at offset 1. Thus the SDK does
not synthesize `Joint_Status.mode`; it preserves the mode delivered in the
status payload.

The registered SDK callback receives the preserved `Hand_Status_t`. The C
wrapper also copies `status.joints[i].mode` by returned joint id into its
11-byte `mode_out` array.

**Callback preserves mode: YES** through the public C++ callback and the C
wrapper. The Python driver's public feedback object now preserves it too
(`SG100HandFeedback.joint_modes`, rev 2); see CURRENT DRIVER.

**The returned mode is real joint-board state, not a command echo** (rev 2
evidence): in the vendor logs the TX stream stays mode 7 continuously while
the RX side flips to mode 0 for 1–4 frames and back — e.g. 2026-08-06
15:42:00.453→.493 (`J5: 7,8.92° → 0,42604.57° → 7,8.93°`) with all 11 joints
flipping together for a single frame. No host command caused those mode-0
frames; the joint/palm firmware produced them.

The protocol/header descriptions establish that the value is intended to be
the joint board's **current joint control mode**. The release contains no
firmware source with which to audit the board-side assignment statement.

## MODE-DEPENDENT FEEDBACK

The host receive decoder is **not mode-dependent**. In both the deployed
library and newer V3.1 library it is called with only `(joint_index,
wire_position)`, not the mode byte, and applies:

```text
callback_deg = (wire_position_rad * 180/pi - b[joint]) / a[joint]
```

There is no host branch for modes 0, 1, or 7. Therefore any mode-dependent
meaning or conversion already exists in the status position emitted by the
joint/palm firmware.

**MODE_NULL position:** raw/accumulated-like angular coordinate. This is
**REAL-HARDWARE PROVEN for the MODE_NULL-commanded N1 session**: approximately
15–30 degrees of manual physical J1 motion produced a continuous 0→1854.147
callback-degree signal, or 32.360978 wire radians (5.150410 turns), with no
wrap. The tool proved its only outbound whole-hand frame used mode 0, but the
saved N1 CSV does not contain returned mode columns because the Python driver
discarded them. Therefore the artifact cannot independently attest every
callback's returned `Joint_Status.mode`; that omission is part of the current
driver/audit gap.

**MODE_POSITION position:** not safely physical joint angle. Vendor logs
contain mode-1 values up to 9055.37 callback degrees and four joint samples
above 360 degrees. No mode-1 physical conversion is documented.

**MODE_JOINT_POSITION position:** **strongly supported as a distinct,
zero-referenced/reduced joint coordinate**, consistent with the new vendor
statement, but physical correctness on the current unit remains
**hardware-unverified**. Mode-7 log values are overwhelmingly joint-angle-sized
and never exceed 360 degrees in the audited logs. Rare J5 values reach 311.26
degrees and values do not universally remain in the currently deployed map,
so logs alone cannot certify physical correctness, calibration validity, or
the active historical angle map.

**Mode-dependent firmware conversion: STRONGLY SUPPORTED.** The strongest
offline evidence is the coordinate changing with the returned mode in adjacent
10 ms frames while the host decoder remains identical:

```text
J5 mode 7:      8.92 deg
J5 mode 0:  42604.57 deg   (+10 ms)
J5 mode 7:      8.93 deg   (+10 ms)
```

The physical joint cannot traverse that distance and return in 20 ms. Across
the full log set, 1,676 adjacent mode-0→mode-7 joint transitions (within 100
ms) changed from a pre-transition range of `[-21243.11, 42604.57]` to a
post-transition range of `[-164.92, 110.76]`. There were 1,502 observed
mode-7→mode-0 transitions with the inverse bounded-to-accumulated pattern.

This evidence proves a mode-correlated board/palm payload change, not the exact
source of the mode-7 value. Firmware could select a joint encoder, rebase and
scale an accumulated motor encoder, or use another logical estimator.

## MODE 7 ZERO / SCALE

The only public semantic statement in `sg100_types.h` is that mode 7 is
“joint position control (absolute position based on `joint_zero_rad`).”
`ParamCalibrationGroup_t.joint_zero_rad` and `JointZeroResult_t.joint_zero_rad`
are floats in radians described as the joint mechanical zero angle.

**`joint_zero_rad` involvement:** **DOCUMENTED qualitatively**, but its exact
coordinate domain is still unknown. No public artifact states whether it is
stored in accumulated motor radians, gearbox-input radians, output-joint
radians, or another logical coordinate.

**Transmission conversion:** a mode-7 reduction or alternate joint-angle
sensor is **STRONGLY SUGGESTED** by the immediate mode transitions, but an
explicit transmission division is **UNKNOWN**. No host ratio exists, and the
firmware source/control equations are absent.

**Equation: UNKNOWN.** Neither of these candidates can be selected from the
available evidence:

```text
joint_position = accumulated_position - joint_zero_rad
joint_position = (accumulated_position - joint_zero_rad) / ratio
```

Nor can the audit exclude a direct joint-encoder/logic-angle selection. The
logs may be used as mode-semantic evidence, but not to fit a production ratio
or zero.

## VENDOR LOG EVIDENCE

The parser scanned all three files:

- `sg100_control.log`
- `sg100_control.1.log`
- `sg100_control.2.log`

Together they contain 222,648 recognizable realtime RX records. Nineteen
malformed/truncated records were excluded. Valid returned joint observations
were grouped by each joint's own mode byte, including mixed-mode frames.

| Returned joint mode | Joint observations | Position range (callback deg) | Outside current runtime map | `abs(position)>180` | `abs(position)>360` | Assessment |
|---|---:|---:|---:|---:|---:|---|
| mode 0 | 102,531 | `-21243.11…42604.57` | 3,716 (3.624%) | 1,637 (1.597%) | 1,248 (1.217%) | Multi-turn/raw values occur. |
| mode 1 | 311 | `-213.75…9055.37` | 44 (14.148%) | 14 (4.502%) | 8 (2.572%) | Sparse sample, but multi-turn/raw values occur. |
| mode 7 | 2,346,077 | `-164.92…311.26` | 198,385 (8.456%) | 3,101 (0.132%) | 0 | Strongly bounded relative to modes 0/1; not universally within current limits. |

Uniform whole-hand frames numbered 505 for mode 0, 24 for mode 1, and 125,443
for mode 7. Another 96,657 valid frames were mixed-mode, mainly during
per-joint/asynchronous transitions.

Mode-7 per-joint observed ranges were:

| Joint | Range (callback deg) | Joint | Range (callback deg) |
|---|---:|---|---:|
| J1 | `-2.38…80.91` | J7 | `-3.43…110.23` |
| J2 | `-119.54…0.49` | J8 | `-45.21…5.97` |
| J3 | `-51.51…38.51` | J9 | `-13.74…0.35` |
| J4 | `-1.46…44.02` | J10 | `-0.78…108.42` |
| J5 | `-164.92…311.26` | J11 | `-43.30…0.61` |
| J6 | `-43.91…17.56` |  |  |

“Outside current runtime map” is only a comparison against the currently
deployed `joint_angle_map.json`; the exact map loaded when the historical logs
were recorded is not proven. It is not treated as a calibration verdict.
Nevertheless, the J5 mode-7 excursions and absence of metrological ground
truth prevent upgrading mode-7 physical semantics from strongly supported to
proven.

## CURRENT DRIVER

**Mode-aware since rev 2 (2026-09-03).** The previously discarded mode bytes
are now preserved end to end, and the data path enforces the mode boundary:

- `SG100HandFeedback` carries `joint_modes` (11 hardware mode bytes),
  `feedback_domain`, and `canonical_valid`
  (`sg100_teleop/robot/sg100_driver.py`).
- `SG100SDKDriver.read_feedback()` classifies every sample:
  all-11 == mode 7 → `MODE7_JOINT_POSITION_CANDIDATE`; anything else with
  mode information → `NON_CANONICAL_DIAGNOSTIC`. `canonical_valid` is
  additionally gated on the historical `mode7_hardware_validated` flag. The
  flag now specifically means the exact inbound canonical map `F_fb` has
  passed authorized hardware validation; it defaults to **False**. The real
  angle-domain milestone alone does not flip it.
- `ArmHandDataCollector.observe_hand_feedback()` rejects any sample that
  carries mode information but is not canonical-valid with
  `HAND_FEEDBACK_NON_CANONICAL_MODE` — a MODE_NULL accumulated value (e.g.
  the N1 1854-deg trace) can no longer reach `measured_hand_q`. Samples
  without mode information (mock/legacy drivers) keep the previous
  manifest-level quarantine path. Raw audit events now record
  `joint_modes`/`feedback_domain`/`canonical_valid` per sample.
- Mode-0, mode-1, mixed, and unvalidated mode-7 positions remain available
  only as explicitly labeled diagnostics (`q_rad` numeric meaning stays
  returned-mode-dependent).

The remaining gap is intentionally retained: `q_rad` still exposes the
hardware-map conversion for API compatibility even for non-canonical
samples; consumers must check `canonical_valid` before treating it as
physical state (the collector already does).

## DATA CONTRACT

Measured SG100 data remains **NOT TRAINING READY**. The existing
`SG100_PHYSICAL_SCALE_UNCHARACTERIZED` quarantine, empty training-eligible
indices, and `BLOCKED` canonical verdict remain required. Dataset dimensions
remain unchanged at 17 (`arm 0:6`, `hand 6:17`).

The future acceptance rule is conjunctive:

```text
all 11 returned joint modes == MODE_JOINT_POSITION (7)
AND mode-7 physical output-joint semantics are hardware verified
AND the canonical sign/order/zero mapping is verified
```

MODE_NULL feedback must never be silently labeled or consumed as canonical
measured hand q. Until the future driver gate is implemented, quarantine is
the fail-safe barrier; it is not evidence that the stored hand component is
already physical.

## SAFE MODE7 ENTRY

**Rev 3 (2026-09-03, vendor contract): the entry blocker model is corrected.**
The previous single gate assumed entering mode 7 necessarily requires
deriving a physical target from MODE_NULL feedback — that assumption is
**retired as non-authoritative**. The vendor's entry contract is:

> populate the 11 joint IDs (`id = 1..11`) and use
> `MODE_JOINT_POSITION = 7` for every joint; in mode 7 the returned
> position is expected to be the correct joint angle.

Three INDEPENDENT states now govern the stack
(`tools/sg100_mode7_feedback_probe.py`):

- `MODE7_ENTRY_CONTRACT_CONFIRMED = True` — the id+mode contract above is
  sufficient for the **full-hand mode7 FEEDBACK probe** (one `Hand_Control_t`
  through one `SetHandControl`, all 11 joints, zero-initialized remaining
  fields: position=0.0, p=i=d=output_limit=0). This zero-output configuration
  is intended to produce no active drive; no-motion remains an explicit
  hardware acceptance result, not a software guarantee.
- `MODE7_JOINT_ANGLE_SCALE_HARDWARE_SUPPORTED = True` — supported by the
  preserved real run: finite/non-multiturn, stable all-mode7 values, mode
  rebasing, fault 0, and no visible entry movement.
- `MODE7_CANONICAL_MAPPING_HARDWARE_VALIDATED = False` — exact inbound
  identity/sign/zero/range mapping remains pending. Therefore canonical
  measured state, dataset training, active movement, AVP, and H2 remain
  blocked. `canonical_valid` remains false.

Historical context (rev 2, retained as evidence, superseded as a blocker):
the teaching example replays MODE_NULL callback values as mode-7 targets, and
the real N1 proved this unit's MODE_NULL feedback can be accumulated
(~1854 deg) — which is exactly why **MODE_NULL status.position must NEVER be
copied into a mode-7 target** (`MODE_NULL_TARGET_REUSE = FORBIDDEN`; no
startup-offset derivation, no clamping workaround). The feedback probe needs
no such target: its mode-7 frame is zero-gain/zero-target.

**Vendor-recommended mode-7 gains (documented, with provenance):**

- production logs (every mode-7 TX frame, ~125 k frames): p=328, i=0, d=0,
  output_limit=1638 → **kp=0.1, ki=0.0, kd=0.0, output_limit=0.5** — identical
  to `InitHandCtrl()` in `realtime_joint_control_example.cpp`;
- teaching example: kp=0.5, ki=0.0, kd=0.05, output_limit=1.0.

The FEEDBACK probe deliberately uses neither set: its frame is the fixed
zero-output configuration above. The production-log set remains the documented
choice for any future active-control work.

## NEXT HARDWARE TEST

**First real run 2026-09-03 (recorded) + transition state machine fixed.**
Phase A PASSED on hardware (99.9918 Hz, fault 0, RX modes MODE_NULL ×11,
finite small MODE_NULL positions).  The mode7 send was accepted through the
native path (`send_positions_rad` → `sg100_set_hand_control` →
`SG100Device::SetHandControl`; one call updates `cached_control_`, the vendor
CommScheduler re-transmits it).  The FIRST returned frame reported mode0 ×11
and the probe (v2) wrongly hard-failed at frame 0 — that is the known
asynchronous transient (this audit's log evidence: TX mode7 while RX flips
to mode0 for 1–4 frames), i.e. a **probe state-machine bug, NOT mode7
rejection**.  Classification of that run:

- MODE7 command submitted: **YES** (native SetHandControl path, operator YES)
- MODE7 actually reached: **UNKNOWN** (observation aborted at frame 0)
- MODE7 feedback physical: **NOT TESTED**
- visible movement: **NOT CONFIRMED** (the prompt was never reached)

Probe v4 (`sg100_teleop/tools/sg100_mode7_feedback_probe.py`) independently
corrects the probe-only send abstraction and replaces the
frame-0 requirement with an explicit bounded transition state machine
`MODE7_COMMAND_SUBMITTED → WAITING_FOR_MODE7_RX → MODE7_STABLE /
MODE7_SEMANTIC_REJECTED / TIMEOUT`:

- `send_mode7_feedback_probe()` bypasses canonical mapping, clamping, unit
  conversion, and production gains; it submits exactly one SDK-domain native
  update with all positions 0 degrees, all modes 7, and p=i=d=output_limit=0;
- 500 ms monotonic transition window, sequence+timestamp deduplicated; cached
  pre-command RX is excluded and frame count never ends the timeout early;
- the fifth consecutive fresh all-11-mode7 post-submit callback immediately
  marks `MODE7_STABLE` and fixes the stable latency; this milestone is not the
  end of observation;
- after stability the probe remains feedback-only and collects until at least
  20 total fresh post-command frames exist (bounded by a separate 500 ms
  post-stable tail); there is no additional `SetHandControl` call and no target
  change;
- mode0 frames are allowed before stability (their 1854/42604-like accumulated
  positions are recorded but never position-checked); losing all-mode7 after
  stability rejects the observation;
- immediate hard fail only for fault callback, transport stall, or non-finite
  data; unexpected modes and a >±360-deg position on a joint whose own RX mode
  is 7 are semantic rejections with normal safe cleanup;
- timeout without stability = FAIL result with the full transition history
  printed and preserved (frame / dt_ms / J1..J11 modes, first-all-mode7
  frame, per-frame mode7 joint counts);
- position-domain evaluation uses the full contiguous all-mode7 sample window
  from the start of the run that established stability through observation
  completion; the operator movement prompt follows (`y`
  or `yes` means movement and FAIL; `n`, `no`, or Enter means none observed);
  a software fault before the prompt reports
  `NO_VISIBLE_ENTRY_MOTION: NOT_CONFIRMED`;
- artifacts under `recording/sg100_mode7_probe_<ts>/`
  (metadata.json, transition_samples.csv containing all fresh post-submit rows
  through the terminal state, summary.json
  distinguishing COMMAND_SUBMITTED / FIRST_RX / FIRST_MODE7_ANY /
  FIRST_MODE7_ALL / MODE7_STABLE / TIMEOUT) plus TX/cache audit counters
  (SetHandControl calls, cached requested modes, scheduler alive, RX
  first-mode7 / first-all-11 / stable latencies).

Every exit after submission now attempts MODE_NULL restoration before the
outer `stop()` and `disconnect()`, and artifacts survive timeout, semantic
rejection, operator movement, EOF/interrupt, fault, and unexpected exception.

### Preserved real v4 artifact (read-only review)

During this software-review round, a new preserved working-tree artifact
appeared at `recording/sg100_mode7_probe_20260903_163543/`. This reviewer did
not execute hardware. Read-only inspection shows:

- Phase A: 99.9946 Hz, fault 0, MODE_NULL ×11.
- Exactly one mode7 application update; requested positions/gains/output limit
  all zero; MODE_NULL restoration and stop/disconnect succeeded.
- Frames 0–2 were MODE_NULL at 9.407/19.402/29.395 ms.
- First-any-mode7 and first-all-mode7 were both 39.396 ms.
- The fifth consecutive all-11-mode7 frame marked `MODE7_STABLE` at 79.401 ms.
- Observation continued to 20 total fresh post-command frames (17 contiguous
  stable mode7 samples) through 199.638 ms, with no mode loss or multi-turn
  mode7 value. The operator result records no visible movement.
- MODE7 control/status entry is **PASS** and the bounded joint-angle-scale
  feedback domain is **SUPPORTED**. SDK command-range equality is not a
  prerequisite for that verdict.
- SDK command-range compatibility is **PARTIAL** (7/11). Canonical-range
  compatibility is separately **PARTIAL** (8/11).
- J7 `-6.9324°` is outside its SDK command interval `[0,125]` but strictly
  inside canonical `middle_j1` `[-2.18,0] rad` (about `[-124.9,0]°`). The old
  J7 range rejection compared different coordinate domains and was wrong.
- J2 `+18.0833°`, J5 `-0.4285°`, and J10 `-3.9554°` remain strict canonical
  range mismatches. J5 is a near-boundary offset candidate; J2 and J10 remain
  explicit zero/sign/identity questions. None rejects the angle domain.

The output model is now independent: `MODE7_CONTROL_STATE=PASS`,
`MODE7_ENTRY_NO_VISIBLE_MOTION=PASS`,
`MODE7_JOINT_ANGLE_SCALE_DOMAIN=SUPPORTED`, command/canonical range
compatibility `PARTIAL`, `MODE7_CANONICAL_MAPPING=PENDING`, and
`MODE7_PHYSICAL_FEEDBACK_HARDWARE_VALIDATED=PARTIAL_ANGLE_DOMAIN_SUPPORTED`.
`canonical_valid` remains false and the dataset remains quarantined.

Command and feedback transforms are independent:

```text
q_sdk_command = F_cmd(q_canonical)
q_canonical   = F_fb(q_mode7_feedback)
```

An outbound sign candidate therefore cannot be copied automatically into
`F_fb`. The strict range review below uses the SDK command limits captured in
the preserved real artifact and the current canonical config limits, displayed
rounded in degrees. This keeps the historical result reproducible if a later
machine or checkout selects a different vendor `joint_angle_map.json`.
`distance` is zero for an in-range value. No tolerance is used to turn a
mismatch into a pass.

| Joint | Canonical name | Real MODE7 baseline ° | SDK command range ° | Cmd compatible | Canonical range ° | Canonical compatible | Distance outside canonical ° | Interpretation |
|---|---|---:|---:|:---:|---:|:---:|---:|---|
| J1 | thumb_j1 | +10.22895 | [0,125] | YES | [0,125] | YES | 0 | strict range-compatible; mapping pending |
| J2 | thumb_j2 | +18.08332 | [-150,0] | NO | [-149.5,0] | NO | 18.08332 | sign/zero/identity unresolved |
| J3 | thumb_j3 | +7.22935 | [-60,90] | YES | [-60,90] | YES | 0 | strict range-compatible; mapping pending |
| J4 | index_j1 | +0.13200 | [0,90] | YES | [0,90] | YES | 0 | strict range-compatible; mapping pending |
| J5 | index_j2 | -0.42847 | [0,125] | NO | [0,125] | NO | 0.42847 | near-boundary zero/calibration candidate |
| J6 | index_j3 | -9.28344 | [-60,90] | YES | [-60,90] | YES | 0 | identity command channel/direction supported by later active test; F_fb pending |
| J7 | middle_j1 | -6.93240 | [0,125] | NO | [-125,0] | YES | 0 | previous command-range rejection corrected |
| J8 | middle_j2 | -0.03279 | [-60,90] | YES | [-60,90] | YES | 0 | strict range-compatible; mapping pending |
| J9 | little_j1 | -0.31862 | [-180,0] | YES | [-180,0] | YES | 0 | strict range-compatible; mapping pending |
| J10 | little_j2 | -3.95540 | [0,125] | NO | [0,125] | NO | 3.95540 | zero/sign/calibration unresolved |
| J11 | little_j3 | -8.81108 | [-60,90] | YES | [-60,90] | YES | 0 | strict range-compatible; mapping pending |

The zero-gain zero-target MODE7 frame is unchanged. Angle-domain acceptance
uses returned MODE7 status, finite/stable/non-multiturn values, bounded angular
magnitude, mode-dependent rebasing, fault 0, and no visible movement. The
no-motion stable-run classifier permits at most 1 degree peak-to-peak per
joint; this is a stability criterion, not a range tolerance. SDK command ranges
are diagnostic only. Canonical range checks are separate and strict: no
tolerance converts an out-of-range baseline into a pass.

The next hardware stage is the same two-stage tool restricted to **J7 only**;
J6 has completed. J7 is software-prepared but has not been executed. The old
manual characterization tool remains preserved but is retired as the
primary next step. N3 selects only an identity-range candidate from J3/J6/J8/
J11, re-evaluates live headroom, enables one position-producing joint, and
keeps the other ten in MODE7 with zero PID/output. It never homes, zeros,
clamps an invalid target, sends a return motion, derives offsets, or modifies
`sg100_hardware_map.yaml`.
