# SG100 Position Domain / Transmission Ratio Final Audit

**Date:** 2026-09-03  
**Safety status:** Static/offline audit only. No hardware command or additional sign characterization was run.  
**Final decision (revised 2026-09-03):** MODE7 bounded joint-angle-scale
feedback **SUPPORTED — REAL HARDWARE**; exact canonical map **PENDING.**  
**Canonical mapping:** **BLOCKED.**  
**H2:** **BLOCKED.**

This report is superseded on mode-dependent feedback by
[SG100_MODE7_FEEDBACK_SEMANTICS_AUDIT.md](SG100_MODE7_FEEDBACK_SEMANTICS_AUDIT.md).
The vendor has **CONFIRMED** (user-vendor communication, 2026-09-03) that
mode-7 feedback should be the correct joint angle, while other modes may
expose raw/accumulated data. The preserved real MODE7 artifact now verifies a
finite, stable, rebased joint-angle-scale domain on this unit. It does not yet
verify per-joint physical identity, sign, zero, or canonical range.

The original audit covered the checked-out vendor release repository
`/home/heymandex2025/SG100_SDK_Publish` at commit
`0fa5e00a953a9f11fb380d51a37ba1ab71c97381` (`V1.5_NO_TACTILE`), all of its
Git branches and history, the newer `origin/V3.1` release at `f924b44`, the
backup repository, headers, documentation, examples, configuration, large SDK
logs, protocol/calibration material, product/technical documents, and both the
current and V3.1 Linux x86-64 shared libraries. The release repository has
never contained joint-board or palm firmware source. The vendor's current
[public website](https://www.heymanbot.com/) was also checked; it publishes no
SG100 transmission data. Search results for similarly named motors and other
dexterous hands are unrelated and were excluded.

Evidence labels used below:

- **PROVEN:** directly established by a checked artifact, disassembly, or the
  saved real trace.
- **DOCUMENTED:** stated by vendor material, but not enough to establish the
  underlying physical domain.
- **INFERRED:** consistent with evidence, but not vendor-confirmed.
- **UNKNOWN:** required information is absent.

## FEEDBACK POSITION

**FEEDBACK DOMAIN:** **mode-dependent at the firmware boundary.** Its wire unit
is radians and the host SDK always presents it as degrees after only an affine
angle-map inversion. MODE_NULL is proven raw/accumulated-like. MODE_POSITION is
not validated as physical. MODE_JOINT_POSITION feedback is
real-hardware-supported as a reduced, zero-referenced joint-angle-scale
coordinate. Exact physical/canonical mapping remains pending.

| Item | Audit result |
|---|---|
| `MotorUp_t.position` | Vendor header: “current accumulated position,” raw `float`, rad. It is the joint-board status field returned by `CMD_GET_STATUS` (`0x02`), but logs show its numeric semantics change with returned joint mode. |
| `Joint_Status_t.position` | Same 12-byte palm-protocol status layout as `MotorUp_t.position`. The host preserves `mode` and decodes position without consulting it; mode-dependent semantics must therefore originate in firmware. |
| SDK callback value | `user_deg = (wire_rad * 180/pi - b_i) / a_i`. This is **PROVEN** in the current binary and V3.1 binary. |
| Physical quantity | MODE_NULL source: **UNKNOWN raw/accumulated-like**. MODE7: **SUPPORTED bounded joint-angle-scale domain on real hardware**; exact output-joint identity/sign/zero/scale mapping remains pending. |
| Reference frame | MODE_NULL origin: **UNKNOWN**. MODE7 is documented as based on `joint_zero_rad`, but the exact zero equation is **UNKNOWN**. |
| Multi-turn behavior | **YES, PROVEN only for the observed MODE_NULL J1 trace.** The trace reached 32.360978 wire rad (5.150410 rotations) continuously, without wrap or discontinuity. Audited mode-7 logs contain no value beyond 360 callback degrees. |
| Motor-side vs joint-side | MODE_NULL is empirically motor/actuator-side-like. MODE7 is real-hardware-supported as joint-angle-scale; the exact joint-side canonical map is not yet verified. |
| Transmission already applied | **NO at the host layer.** Some firmware-side scale/rebase in mode 7 is strongly supported; whether it is a ratio calculation or alternate sensor selection is **UNKNOWN**. |

The header exposes separate `motor_raw_count`, `joint_raw_count`,
`encoder_angle`, `joint_angle`, `logic_angle`, and distinct motor-encoder and
joint-encoder faults. It never identifies which of these feeds
`MotorUp_t.position`. The existence of a joint encoder therefore does not prove
that realtime `position` is joint-encoder angle.

The real evidence is saved in
`recording/sg100_n1_sign_20260903_135111/`. Its offline analysis used the first
saved row per unique callback timestamp (3210 callbacks at 99.968 Hz), avoiding
polling duplicates. J1 was continuous and predominantly monotonic. This proves
the J1 channel and positive direction for the instructed canonical-positive
manual movement, but not its physical scale.

Vendor `sg100_control*.log` files show that large accumulated feedback occurs
in MODE_NULL and MODE_POSITION, while MODE_JOINT_POSITION is overwhelmingly
joint-angle-sized. Adjacent mode transitions change the same joint between
these domains in 10 ms (for example J5: mode7 `8.92`, mode0 `42604.57`, mode7
`8.93`). This strongly supports firmware mode dependence, but does not reveal
the conversion formula or feedback sensor source.

## COMMAND POSITION

**COMMAND DOMAIN:** the caller supplies SDK “user degrees.” The host clamps and
affine-maps those values, then places radians on the wire. What the firmware
interprets those wire radians to mean is mode-dependent and incompletely
documented.

| Item | Audit result |
|---|---|
| `MotorDown_t.position` | Vendor header: target position, raw `float`, rad. The physical/reference domain is not stated. |
| `Joint_Control_t.position` | Same position payload for mode 1 and mode 7, raw `float`, rad. |
| Host send conversion | `wire_rad = (a_i * clamp(user_deg) + b_i) * pi/180`. **PROVEN** by current and V3.1 disassembly. |
| `MODE_POSITION = 1` | Firmware “position-loop control.” The host applies the formula above. The firmware target coordinate, zero, and sensor source are **UNKNOWN**. Large historical mode-1 feedback values show that mode 1 cannot be assumed to be physical output-joint control. |
| `MODE_JOINT_POSITION = 7` | Documented as “joint position control (absolute position based on `joint_zero_rad`).” The host uses exactly the same encoder and payload path as mode 1 and passes mode byte 7 unchanged. Therefore any mode-7 zero/transmission behavior must occur in firmware. Its equation is **UNKNOWN**. |
| Same domain as feedback | **UNKNOWN.** There is one status field and it carries the current mode, but no source or protocol document defines command/status coordinate equivalence. Historical mode-7 tracking is evidence of compatibility in some configurations, not proof for this unit or MODE_NULL feedback. |

The current binary's `SG100Device::Impl::SetHandControl` jump table routes modes
1 and 7 to the same host-side position encoder. Both receive the configured
`a/b` map, clamp, and `pi/180`; only the mode byte distinguishes them when sent
to firmware. The receive path always performs `180/pi` followed by inverse
`a/b`, regardless of whether the callback is consumed by the teaching example
or production code.

The SDK document `Position_Transform_Calculation.md` calls the affine-mapped
value a “hand-side calibrated physical angle.” **Rev 2 correction (do not
over-generalize):** that claim is contradicted only for the **MODE_NULL**
case — the real J1 trace was recorded while every joint was commanded
MODE_NULL, and MODE_NULL is the mode the vendor says may expose
raw/accumulated data. The vendor statement does NOT contradict the document
for mode 7: in MODE_JOINT_POSITION the vendor confirms the returned position
should be the correct joint angle, and the historical logs' mode-7 values
(joint-angle-sized, never beyond 360 deg) are consistent with the
"calibrated physical angle" description there. The release documentation
remains internally stale in other ways: the user guide lists an obsolete
mode number, and transformation-switch methods described in the transform
document are absent from the checked-out public header/binary. These
comments cannot establish physical-domain semantics by themselves.

## TRANSMISSION

**Per-joint ratios found:** **NONE.** No SG100 ratio table or single ratio was
found in current files, backup/reference files, any Git revision/branch,
technical/product documents, examples, configuration, logs, or official public
web material.

**Host SDK applies ratio:** **NO.** Explicit binary result:

> **NO MOTOR→OUTPUT JOINT SCALE EXISTS IN HOST SDK.**

The current x86-64 binary decodes feedback as raw wire radians times
`57.29578`, then subtracts `b` and divides by `a`. Its send path applies the
configured affine map and multiplies by `0.017453292`. Disassembly found no
additional multiply/divide, per-joint transmission lookup, or plausible ratio
constant in the control/status paths. Ratio-range constant candidates were
formatter/string bytes or unrelated values (including OTA 100 percent), with
no relevant references. The independently inspected V3.1 binary has the same
result.

**Firmware applies ratio:** **UNKNOWN.** The release package contains no
firmware source and no protocol/calibration document defining such a scale.
Mode-7 logs strongly support a firmware-side scale/rebase or alternate
joint-angle sensor, but “based on `joint_zero_rad`” does not establish a
transmission-ratio equation.

No authoritative **SG100** actuator ratio in the J1-implied 61.8–123.6 range
was found. Values such as 90:1 or 100:1 found for unrelated products are not
SG100 evidence and must not be adopted.

## ZERO

`ParamCalibrationGroup_t.joint_zero_rad` and
`JointZeroResult_t.joint_zero_rad` are documented as a saved “joint mechanical
zero angle” in radians.

**`joint_zero_rad` domain:** **UNKNOWN.** The word “joint” and unit `rad` do not
distinguish motor accumulated angle, gearbox input, gearbox output, physical
joint angle, or another internal logical coordinate. The nearby calibration
structure separately stores electrical zero, which rules out treating the two
fields as synonyms, but it does not identify the mechanical scale.

**Mode-7 use:** only the qualitative statement “absolute position based on
`joint_zero_rad`” is available. No audited public artifact provides an equation
such as `motor_target = zero + ratio * joint_target`, its sign convention, or
whether feedback is rebased in mode 7. Exact mathematical use is therefore
**UNKNOWN**. It is reasonable—but only an inference—that the value anchors an
internal accumulated coordinate used by the joint controller.

## TEACHING EXAMPLE

**Classification: CONFIG-DEPENDENT (user option B).** On the audited unit and
runtime map, the published example is unsafe and coordinate-domain invalid.

The example:

1. enters `MODE_NULL`;
2. copies `status.joints[i].position` directly into `positions_deg`;
3. records those values without normalization, zero handling, wrapping, or a
   transmission conversion;
4. reuses them directly as mode-7 `Joint_Control_t.position` values; and
5. compares callback values to the stored values with a fixed 2-degree
   tolerance.

For the real J1 callback value 1854.147 and deployed J1 user range `[0,125]`,
the SDK send path clamps replay to 125 before converting to wire radians. The
recorded frame is therefore not replayed as recorded.

This is not classified as **MISSING-CONVERSION** because exhaustive host binary
inspection found no conversion the audit had overlooked. It is not classified
as an unconditional, all-firmware **BUG** because vendor historical mode-7 logs
show degree-sized targets and feedback tracking on some prior homed/configured
sessions; mode 7 may own an undocumented firmware conversion or rebase. The
example fails to state or validate that prerequisite, so “CONFIG-DEPENDENT”
does not make it production-safe. The example was introduced at commit
`0fa5e00` and is byte-for-byte unchanged in newer V3.1; no Git-history fix or
clarification was found.

## REAL J1

| Measurement | Result |
|---|---:|
| Callback delta/peak from baseline | `+1854.1474609375 deg` |
| Wire accumulated position | `32.3609780109 rad` |
| Wire accumulated rotations | `5.1504096137` |
| Approximate physical manual delta | `15–30 deg` |
| Implied rough ratio at 30 deg | `1854.1474609375 / 30 = 61.8049:1` |
| Implied rough ratio at 15 deg | `1854.1474609375 / 15 = 123.6098:1` |
| Supported rough interval | `61.8–123.6:1` |

**Exact calibration: NOT POSSIBLE FROM THIS TRIAL.** The operator estimate was
not metrologically measured. It supports channel identity, positive direction,
and order-of-magnitude scale only. It must not select a convenient catalog
ratio or populate production configuration.

## PRODUCTION DECISION

**MODE-7 SEMANTICS PENDING.**

The new vendor statement and log evidence reopen firmware-owned physical joint
mapping as the leading hypothesis. Production still requires a safe no-jump
mode-7 entry procedure and direct hardware verification that returned mode-7
position equals independently measured physical output-joint angle. Host-owned
ratio mapping must not be implemented from the N1 estimate.

No position commands should be sent through the canonical production path
while this decision remains in force. Do not resume J2–J11 sign
characterization: signs are downstream of a proven scale and domain.

## CANONICAL MAPPING

**BLOCKED.** The mapping `feedback → canonical physical radians` is not known,
and command-domain equivalence is not proven. J1 channel identity and positive
direction remain useful partial evidence; J1 scale is `UNCHARACTERIZED`, and
J2–J11 remain `NOT_TESTED`.

Real measured SG100 datasets remain **NOT TRAINING READY**. The existing
quarantine reason `SG100_PHYSICAL_SCALE_UNCHARACTERIZED`, empty
`training_eligible_indices`, and `hand_canonical_mapping_verdict: BLOCKED` must
remain for measured-feedback recordings. Dataset shape is unchanged:
`rm65_sg100_joint17_v1`, 17 dimensions, arm `0:6`, hand `6:17`.

The prior sign-tool safety fixes remain in place: partial-session artifact
saving, explicit `NOT_TESTED`, EOF-safe quit, unique-callback deduplication, no
scale verdict, and offline artifacts. The tool is not expanded by this audit.

## H2

**BLOCKED.** H2 must not be implemented until the vendor resolves the command
and feedback physical domains and a canonical mapping can be established
without inferred transmission constants.

## VENDOR INFORMATION STILL NEEDED

The vendor must answer these questions for the exact SG100 hardware revision,
joint firmware version, palm firmware version, and calibration/configuration on
this unit:

1. What physical sensor or estimator produces `MotorUp_t.position` /
   `Joint_Status_t.position` in every control mode: motor encoder, accumulated
   rotor position, integrated velocity, gearbox input, gearbox output, joint
   encoder, or another logical quantity?
2. Is that status field unwrapped/multi-turn? What event establishes its zero,
   can it wrap or reset, and can changing mode or homing rebase its coordinate?
3. What coordinate does `MotorDown_t.position` expect in `MODE_POSITION=1`?
   Give the exact unit, reference, sign, and equation.
4. What coordinate does it expect in `MODE_JOINT_POSITION=7`? Give the exact
   firmware equation from requested physical joint angle to actuator target.
5. Does mode 7 apply a per-joint motor-to-output transmission ratio? If so,
   provide the authoritative ratios for J1–J11, including all gearing,
   belt/cable/worm/linkage/coupling factors and signs.
6. What exact coordinate is stored in `joint_zero_rad`, how is it measured, and
   how is it used mathematically by mode 7?
7. When mode 7 is active, is realtime `Joint_Status_t.position` returned in
   zero-referenced physical output-joint radians, or does it remain an
   accumulated motor/internal coordinate? If it remains internal, what public
   API returns physical joint feedback?
8. Which homing/calibration flags and procedures are mandatory before mode 7,
   and how can software verify that all required calibration is valid without
   commanding motion?
9. Which `joint_angle_map.json` values and transform-enable behavior are
   required for this hardware/firmware pairing? Are `a/b/min/max` merely API
   polarity/range mapping, or do any encode transmission?
10. Under what documented firmware/configuration preconditions is
    `teaching_mode_example.cpp` valid when it records MODE_NULL feedback and
    replays it through mode 7? How should an application reject a callback such
    as `1854.147 deg` before the `[0,125]` SDK clamp?
11. Provide matching firmware source or a versioned protocol/control document
    that defines these equations, plus a safe read-only method to identify the
    installed joint/palm firmware and calibration schema.
