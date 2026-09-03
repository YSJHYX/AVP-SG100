# SG100 Latest SDK Integration (V1.5_NO_TACTILE)

Status: **MODE7 ENTRY PASS / ANGLE-SCALE DOMAIN SUPPORTED ON REAL HARDWARE;
ACTIVE MICRO-MOTION TOOL PREPARED / HARDWARE PENDING.**
N0 established the link and feedback cadence.  Real N1 artifact
`sg100_n1_sign_20260903_135111` tested only thumb_j1 while all joints were
commanded MODE_NULL and exposed that mode's uncharacterized
accumulated/raw-like scale.  The vendor CONFIRMED (2026-09-03) that mode-7
feedback is the intended correct joint angle AND supplied the full-hand entry
contract (ids 1..11 + mode 7 on every joint), so the zero-gain full-hand
mode7 FEEDBACK probe has since passed on real hardware with all 11 RX modes=7,
fault 0 and no visible entry motion. It supports a finite, stable,
non-multiturn joint-angle-scale domain, but exact `F_fb` remains pending and
the driver/collector still enforce `canonical_valid=False`. The next stage is
the prepared two-stage single-joint ACTIVE HOLD + one micro delta harness; it
has not been executed. See
[SG100_MODE7_FEEDBACK_SEMANTICS_AUDIT.md](SG100_MODE7_FEEDBACK_SEMANTICS_AUDIT.md).

## SDK version

| item | value |
|---|---|
| path | `~/SG100_SDK_Publish` |
| git | branch `V1.5_NO_TACTILE`, commit `0fa5e00a953a9f11fb380d51a37ba1ab71c97381` ("feat(publish): add teaching mode example") |
| library | `lib/linux-x86_64/libsg100_sdk.so`, 1 015 632 B, md5 `fd4f724ed5a92a6465476d7172aca5c7` |
| transport | RS485 UART active, 1 000 000 baud, vendor `CommScheduler` 10 ms cycle |
| docs | `docs/SG100_SDK_User_Guide.md`, `docs/Realtime_Performance_Analysis.md`, `docs/Architecture_Analysis.md`, `docs/Homing_Calibration_Flow.md`, `docs/Position_Transform_Calculation.md`, `docs/ERROR_CODE_GUIDE.md` |
| reference example | `examples/teaching_mode_example.cpp` (production pattern this integration follows) |

The previous backup checkout (`~/SG100_SDK_Publish_backup_wrong_branch`, a
14-joint pre-V1.5 branch) is NOT supported.  Old-SDK reverse-engineered
facts were revalidated against this version (binary re-audited) or dropped.

## Production API

Command path (upper layers see only canonical radians):

```
AVP -> SG100Retargeter -> pinch/power-grasp/limits/filter
    -> canonical q[11] rad
    -> SG100HardwareMap (canonical rad <-> SDK user rad; single authority)
    -> rad2deg  (SG100SDKDriver)
    -> Hand_Control_t {id=1..11, mode, position[deg], p,i,d,output_limit}
    -> SG100Device::SetHandControl        (vendor, cache update)
    -> CommScheduler 10 ms: clamp + a*x+b + deg2rad -> 0x05 TX @100 Hz
    -> SG100
```

- Input unit of `SetHandControl`: **SDK user-angle DEGREES** (vendor
  documented, Realtime_Performance_Analysis §2.2; re-confirmed in the new
  binary: the cache at Impl+0x79 stores the caller struct verbatim and the
  scheduler's encode lambda applies `clamp(min,max) -> *a+b -> *pi/180`).
- Default control mode `MODE_JOINT_POSITION` (7), gains from the vendor
  teaching example (Kp 0.5, Ki 0, Kd 0.05, output_limit 1.0, quantized with
  `SG100_Quant` semantics range 10).
- `SetJointPositions` is NOT used by the production path (its forced
  `MODE_POSITION` + inherit-cache-PID behavior and the non-round-trip clamp
  make it unsuitable as the authoritative mapping; the migration removed
  its wrapper entry point entirely).
- Out-of-range guard: the driver loads `joint_angle_map.json` user ranges
  and REJECTS targets outside them instead of letting the vendor clamp
  silently — the recorded action always equals the encoded action.
- N3 does not use the general 11-axis command helper. Its narrow diagnostic
  API builds one `Hand_Control_t` with per-joint fields and calls the same
  `sg100_set_hand_control -> SG100Device::SetHandControl` path exactly once per
  stage. Only the selected joint has `p=0.1, i=0, d=0,
  output_limit=0.5`; the other ten remain MODE7 with zero output. Those active
  values are evidenced by the current SDK realtime example and vendor MODE7
  production log (quantized 328/0/0/1638).

## Feedback API

- `RegisterHandStatusCallback` delivers `Hand_Status_t` on the vendor
  communication thread at ~100 Hz while realtime runs.  The SDK computes
  `(wire_position * 180/pi - b) / a` with no receive clamp.  This is a
  caller-facing *numeric degree conversion*, not proof of physical joint
  degrees: real J1 motion of roughly 15–30 physical degrees produced a
  continuous 0→1854.147 callback excursion.  The wrapper copies each sample
  under a mutex; Python dedupes by callback sequence.  No device timestamp
  exists; the boundary uses host monotonic receive time.
- `RequestHandStatus` (0x26) remains available for explicit queries; in
  RS485 active mode the scheduler delivers it.  Caveat carried over from
  RUN-A evidence: in NONRT_PAUSED the palm answer reflects its status
  cache and can be stale — the authoritative measured state for decisions
  is feedback sampled AFTER realtime entry.
- The wire/status field is documented as **accumulated position in radians**
  and aliases `MotorUp_t`, but its returned numeric semantics are
  mode-dependent. MODE_NULL raw/accumulated-like behavior is real-hardware
  proven; the vendor CONFIRMED mode-7 (`MODE_JOINT_POSITION`) position is the
  intended correct joint angle. The preserved 2026-09-03 real run now supports
  a finite, stable, reduced/rebased joint-angle-scale MODE7 domain on this
  unit; exact channel identity/sign/zero/range mapping remains unproven. The
  exact firmware zero/scale equation is absent. **The Python driver now
  preserves the returned mode bytes**: `SG100HandFeedback.joint_modes` (11
  hardware `Joint_Status.mode` bytes — real joint-board state, not a command
  echo), `feedback_domain` (`MODE7_JOINT_POSITION_CANDIDATE` when all 11
  report mode 7, else `NON_CANONICAL_DIAGNOSTIC`), and `canonical_valid`
  (False until the historical `mode7_hardware_validated` flag is explicitly
  flipped after exact `F_fb` canonical mapping validation). `q_rad` remains provisional:
  its numeric meaning is returned-mode-dependent, so consumers must check
  `canonical_valid` before treating it as physical state (the measured
  collector rejects non-canonical samples with
  `HAND_FEEDBACK_NON_CANONICAL_MODE`).

## Joint order

`thumb_j1..thumb_j3, index_j1..index_j3, middle_j1..middle_j2,
little_j1..little_j3` == SDK joint ids 1..11 (identity hardware map).
Single conversion authority: `sg100_hardware_map.yaml` +
`SG100HardwareMap`; dataset boundary is canonical radians.

## Struct ABI (sg100_types.h @ this version)

`Joint_Control_t` 16 B (id, mode, f32 position @2, p/i/d/outlim u16
@6/8/10/12, reserved 2) · `Joint_Status_t` 12 B · `Hand_Control_t` 176 B ·
`Hand_Status_t` 132 B · `MotorDown_t` 14 B · `MotorUp_t` 12 B.  The driver
asserts these at load (`sg100_test_struct_sizes`) and fails loudly on any
future layout change.

## Realtime state machine

```
connect()            vendor SetConfig+Init+Start (scheduler runs; palm
                     usually NONRT_PAUSED after power-up)
enter_realtime()     1. SetHandControl(MODE_NULL x11)   — cache carries no
                        drive output, so the later resume cannot move the
                        hand regardless of physical pose
                     2. wait >=1 fresh status callback (cache content has
                        reached the palm)
                     3. vendor SwitchToRealtimeMode()    — 0x25 -> 0xA5 ACK
                        -> scheduler restart (vendor internals)
                     4. wait fresh status flow            -> realtime_active
WAIT C               send_positions_rad() fail-closed until realtime entry
                     AND the runner teleop gate opens; the hand sits in the
                     MODE_NULL passive hold; feedback is raw diagnostic,
                     not eligible to drive canonical MuJoCo hand pose
C                    runner captures the AVP anchor, teleop targets flow
stop()               MODE_NULL seed again (passive hold, link alive)
disconnect()         vendor Stop() (scheduler join + port close)
```

The old custom raw 0x25 / raw 0x27 / audited 0x05 sender / CANFD-passive
dual transport are deleted.  The vendor scheduler owns TX; this
integration never builds protocol frames.

## Zero semantics (BLOCKED pending hardware validation)

- Authoritative zero per vendor: each joint board's homing state —
  `CMD_HOMING_ZERO (0x07)` runtime zeroing, `CMD_SET_JOINT_ZERO (0x25
  joint-side)` storing `joint_zero_rad`, `MODE_JOINT_POSITION` targets
  referenced to it.  Vendor SOP: `HomingZeroMasked(ALL)` then control.
- **N0 evidence (2026-09-03)**: with the hand physically at the mechanical
  zero pose and all joints in MODE_NULL, every callback read ~0 deg
  (±0.055°). This proves only a coincident raw-coordinate reading at that pose;
  it does **not** prove mode-7 zero alignment or `offset_deg = 0` in the
  physical-joint domain.
- Startup pose is NEVER redefined as zero.  `assess_zero_pose()` only
  REPORTS measured deviation from canonical zero.
- Controlled return-to-zero: NOT IMPLEMENTED (blocked until the canonical
  zero <-> vendor zero alignment is confirmed for arbitrary startup
  poses).

## C / AVP semantics

Before C: no AVP hand commands reach the SG100 (driver fail-closed +
runner gate); hand is under MODE_NULL hold. MODE_NULL position is diagnostic
raw feedback and must not be rendered or consumed as canonical physical hand
pose. The returned-mode gate now exists in the driver/collector (mode-aware
`canonical_valid` + `HAND_FEEDBACK_NON_CANONICAL_MODE` rejection), but
real-mode anchor and teleop stay blocked until mode-7 physical semantics pass
the authorized hardware validation. Data contract shape is unchanged: `state_hand` = measured feedback,
`action_hand` = actually-sent command, `next_state_hand` = post-action
measurement. SG100 components remain provisional, quarantined callback
coordinates rather than validated canonical physical radians.

## Known open blockers

1. **Exact MODE7 feedback-to-canonical mapping (`F_fb`)** — the full-hand
   entry probe is complete, not the next experiment. Its zero-output frame
   and RX state machine remain the regression reference.
   Five consecutive fresh all-11-mode7 callbacks timestamp `MODE7_STABLE`;
   feedback-only observation then continues, without another send or target
   change, until at least 20 total fresh post-command frames are captured.
   The preserved v4 artifact (`recording/sg100_mode7_probe_20260903_163543`;
   externally produced, read-only reviewed here) reached all-11 mode7: first
   all7 at 39.396 ms, stable milestone at 79.401 ms, then 20 total fresh
   frames/17 stable-mode7 samples, fault 0, no visible movement, and successful
   passive cleanup. These facts are **PASS — REAL HARDWARE** for MODE7 entry,
   all-11 status, and no-motion entry, and **SUPPORTED — REAL HARDWARE** for
   the bounded joint-angle-scale feedback domain.
   SDK command-range compatibility is only diagnostic and is PARTIAL (7/11).
   Strict canonical-range compatibility is independently PARTIAL (8/11): J7
   `-6.9324°` is canonical-compatible despite being SDK-command-incompatible;
   J2 `+18.0833°`, J5 `-0.4285°`, and J10 `-3.9554°` remain unresolved.
   Exact identity/sign/zero/range mapping is PENDING, so canonical_valid stays
   False and AVP/full-hand movement/dataset remain BLOCKED. MODE_NULL status.position is FORBIDDEN as
   a mode-7 target (accumulated: ~1854/42604 deg observed).
   History: the joint_angle_map.json divergence itself is closed — the
   The N0 probe proved the runtime file is the on-disk 11-entry map
   (md5 8a0c1de4a7d60a47a016dd453fb49ee3) via strace openat tracing; the
   branch-HEAD variant is not what runs.
   The prepared next tool is
   `tools/sg100_mode7_active_micro_motion.py`: full-hand zero-output MODE7,
   then active hold at fresh MODE7 current position, then one independently
   confirmed micro delta. It has not been executed. The manual-move tool is
   preserved but retired as the primary path.
2. Vendor send clamp semantics hardware-unverified for position targets
   (no position command has ever been sent through the new backend).
3. Active MODE_JOINT_POSITION target behavior and gain suitability remain
   hardware-unverified. This does not undo the passive MODE7 entry/angle-scale
   milestone.

## Current N-stage gates

| Gate | Status |
|---|---|
| N0 MODE_NULL realtime | PASS — REAL HARDWARE |
| N1 full-hand MODE7 safe entry | PASS — REAL HARDWARE |
| N2 MODE7 joint-angle-scale feedback | SUPPORTED — REAL HARDWARE |
| N3 single-joint active hold + micro-motion | TOOL PREP / HARDWARE PENDING |
| N4 per-joint F_cmd/F_fb mapping | PENDING |
| N5 full-hand active command integration | BLOCKED |
| N6 AVP integration | BLOCKED |
| N7 dataset enable | BLOCKED / QUARANTINED |

## N1 — canonical <-> SDK user-angle mapping (2026-09-03 audit)

### Real N1 stop result (supersedes the pre-run assumptions below)

Only thumb_j1 was physically tested.  Its callback trace was continuous and
predominantly monotonic, so SDK channel J1 and positive callback direction are
supported for that instructed motion.  Physical scale is
`UNCHARACTERIZED`; direction sign is not a gain.  The remaining ten joints
are `NOT_TESTED`, every production gain candidate is not hardware confirmed,
and the canonical mapping is `BLOCKED`.

The historical CSV has 15,071 polling rows but only 3,210 unique callback
timestamps (~99.97 Hz).  After deduplication J1 rises gradually from 0 to
1854.147 callback units; the largest 10 ms step is 99.975 units.  It crosses
360/720/1080/1440/1800 without wrapping.  The return trace continuously falls
from 1836.199 to about 528.3–533.0 and never returns to baseline.  Thus the
magnitude is not a phase/bookkeeping artifact or a one-sample jump.

For the observed **MODE_NULL** J1 status, the SDK applies rad→degree and
inverse `a,b`; for J1 (`a=1,b=0`) the 1854.147 callback value corresponds
exactly to 32.361 wire radians (5.150 turns). The naming (`MotorUp_t`) and
observed scale strongly support a raw motor/actuator-side-like accumulated
coordinate in MODE_NULL. This conclusion must not be generalized to mode 7:
vendor guidance and log transitions strongly support different mode-7
semantics, pending physical verification.

**Evidence base**: N0 real-hardware PASS (MODE_NULL only, realtime PASS,
100.0 Hz feedback, 11D finite, fault 0, no visible movement, static
mechanical-zero pose reads ~0 deg ± 0.055° on every SDK joint) plus the
strace-proven runtime json.

- `joint_angle_map.json` a/b describe **SDK user -> wire**, NOT
  canonical -> SDK user; they are not used to derive the sign.
- offset_deg = 0 remains a provisional configuration value, not a mode-7 fact.
  N0 observed MODE_NULL callback zero at the mechanical zero pose (±0.055°),
  which cannot establish mode-7 physical zero alignment. No startup-derived
  offsets are permitted.
- Direction candidates (+1 on ten joints, -1 on middle_j1) came only from
  range morphology.  They are offline hypotheses, not gains and not
  hardware confirmation.  Numeric range similarity cannot establish scale.
- The candidate is **NOT applied** to `sg100_hardware_map.yaml` (still
  identity): a wrong sign would command a joint the OPPOSITE direction
  inside its range, which no range guard can catch.  Until the passive
  sign characterization confirms it, middle_j1 outbound targets remain
  range-REJECTED by the driver (fail-closed).  No axis is presently proven
  safe for position control from these candidates.

### Teaching mode (MODE_NULL) audit — manual sign characterization

`teaching_mode_example.cpp` has NO dedicated teaching API: teaching =
whole-hand `SetHandControl(MODE_NULL x11)` in realtime mode.  MODE_NULL
is "空控制 (停机)" (no control output): motors produce no active
position/torque output, the hand is hand-moveable by design (the
example's entire purpose is human posing + record), and the SDK keeps
streaming the 10 ms control cycle so feedback stays ~100 Hz (N0 proved
this on hardware).  Entering NULL after a position hold releases the
hold (gravity relaxation possible — N0 saw none at the zero pose);
exiting teaching into position targets WOULD move the hand and is out of
scope for characterization.  Position/homing entry points do not exist
in our backend wrapper, so the characterization tool cannot
accidentally command motion.

The vendor example records `status.joints[i].position` directly into
`TeachingFrame.positions_deg`, without normalization, zero subtraction,
ratio conversion, wrap handling, or `joint_zero_rad` processing.  Replay
copies the same values directly into `MODE_JOINT_POSITION`.  The SDK send
path then clamps them to `joint_angle_map.json` before `a*x+b` and degree→rad.
For the real J1 frame, ~1854 would clamp to the configured maximum 125, so
it cannot reproduce the recorded 32.361-rad accumulated coordinate.  Its
reached check also compares current callback data against the unclamped
recorded value, allowing an immediate false success at the just-recorded
pose or an eventual timeout.  The example does not replay this case safely.

**SAFE FOR FURTHER MANUAL SIGN CHARACTERIZATION: NO — BLOCKED.**

### Passive sign characterization tool (hardware execution disabled)

Direction semantics (2026-09-03 correction): each joint carries an
EXPLICIT `test_canonical_delta_sign` (+1/-1) and physical instruction
(thumb_j2/middle_j1/little_j1 test the NEGATIVE canonical direction —
their usable motion from zero is toward the LOWER limit; a generic
"toward the upper limit" instruction is wrong as a sign source).  The
expected SDK delta sign is `sign(candidate_direction * test_delta_sign)`:
e.g. middle_j1 (canonical NEGATIVE test x direction -1)
expects an SDK POSITIVE delta.  Before every movement the tool prints
the full semantics block (joint / physical instruction / test canonical
delta / candidate direction / expected SDK channel / expected SDK delta
sign).  Command-side JSON ranges are not used as passive feedback headroom
or physical return limits.

`sg100_teleop/tools/sg100_sign_characterization.py` now refuses `--execute`
before SDK loading or transport access.  Its retained session contract is:
pre-flight serializes the outgoing
`Hand_Control_t` and ABORTS unless mode[0:11] == MODE_NULL; the only
outgoing control is the NULL hold (position/homing entry points do not
exist in the backend wrapper — position-producing /
SetJointPositions / homing counters are structurally 0).  Per joint
(canonical order): ~1.5 s baseline -> operator instruction built from
sg100_joint_semantics.yaml canonical directions -> operator manually
moves the single DOF while the tool records opaque SDK callback positions.
Analysis separates INDEX and DIRECTION from SCALE.  Scale is always
`UNCHARACTERIZED`; return error is reported only in callback units and
requires operator confirmation rather than a fixed physical-degree
tolerance.  Missing joints are explicitly `NOT_TESTED`; selftest evidence
cannot populate hardware state.  No offsets are derived and no mapping file
is touched.  Artifacts:
`recording/sg100_n1_sign_<timestamp>/{metadata.json,samples.csv,
summary.json}` (metadata carries SDK commit, lib + runtime
joint_angle_map hashes, joint order, candidate direction signs).  Offline
scripted session: `--selftest` (no hardware).  Offline tests:
`test_sg100_sign_characterization.py`.

### Read-only joint zero / calibration status (implemented, no hardware call)

- `RuntimeStatusFrame_t` via `GetRuntimeStatus` (0x17):
  `runtime_state` (JointState_e, incl. JOINT_STATE_HOMING), `foc_ready`,
  `runtime_fault`, `calibration_flags`, target_*.
- `ParamCalibrationGroup_t` via `ReadParamCalib` (0x14):
  **`joint_zero_rad`** ("关节机械零点角度"), `electrical_angle_zero`,
  `calibration_flags`.
- Driver surface: `read_runtime_status/cached_runtime_status/
  read_param_calib/cached_param_calib` (decode to dicts).  CAVEAT: both
  are joint-level PASSTHROUGH queries — the vendor scheduler delivers
  them only in WORK_MODE_PASSTHROUGH, so a manual pause(0x24)/resume
  (0x25) cycle is required around the query on hardware; the driver
  never switches modes by itself and nothing in production calls these.

### Already-REALTIME startup

`enter_realtime` is valid in BOTH startup states: NONRT_PAUSED (0x25
transitions) and ALREADY-REALTIME (the idempotent 0x25 re-resume ACKs;
the NULL seed precedes the switch either way, so the palm never applies
a position target).  No redundant state machine is needed.

### Feedback domain naming

`SG100HandFeedback.raw_positions` is the SDK callback number converted back by
`pi/180` for storage. Its physical meaning depends on the now-preserved
returned joint mode (`joint_modes`): MODE_NULL is raw/accumulated-like;
mode 7 is real-hardware-supported as bounded joint-angle-scale feedback
(`feedback_domain` = MODE7 candidate), but its exact canonical map is pending
(`canonical_valid` = False). Neither `raw_positions` nor `q_rad` may be
described as validated canonical state while `canonical_valid` is False.

## Historical hardware baseline (OLD architecture — not transferable)

The deleted CANFD-passive integration demonstrated on 2026-09-02: ~100 Hz
command / ~100 Hz feedback whole-hand hold, CRC-clean, stable at measured
poses, plus the RUN A/B feedback-semantics evidence (0x26 NONRT staleness;
feedback outside mode 7 is raw/accumulated-like with joint-board-dependent
zeroes).  Those results validate the PHYSICAL LINK and protocol capacity
only; they are NOT a validation of this new backend.  New-SDK hardware
status: N0 link/cadence validated; N1 stopped after thumb_j1; physical
canonical mapping **BLOCKED**.

## Hardware validation commands (pending, do not run without authorization)

1. `python -m sg100_teleop.tools.sg100_hardware_probe --port /dev/ttyUSB0
   --execute` — the FIRST hardware contact of the new backend.  Tool
   guarantees:
   - MODE_NULL only: the sole control frame is the whole-hand NULL hold;
     position/homing entry points do not exist in the backend wrapper, so
     position-producing / SetJointPositions / homing counters are
     structurally 0.  Before the mode switch the tool serializes the
     exact Hand_Control_t through the wrapper's builder and verifies the
     mode byte is MODE_NULL for all 11 joints (printed).
   - Runtime mapping proof: the run re-executes itself under
     `strace -f -e trace=openat` and reports the exact
     `joint_angle_map.json` the running libsg100_sdk.so opens — absolute
     path, md5/sha256, and the parsed J1..J11 a/b/min/max rows.  Nothing
     is inferred from git HEAD, cwd, or file names; if no open is
     observed (SDK default in-process map), the runtime map is reported
     UNPROVEN.
   - Feedback is reported in SDK USER DEGREES only — no canonical
     conversion, no morphology claims until the runtime map/sign issue is
     resolved.  Raw wire positions are not exposed by the public
     callback and are reported unavailable rather than inferred.
   - `enter_realtime` pre-switch status wait is best-effort (a paused
     palm may stay silent on scheduler frames; the NULL cache — not a
     pre-switch response — is what makes the switch motionless).  The
     post-switch status flow is mandatory.
2. Zero-state assessment + json/gain reconciliation (blocker 1).
3. Only then: WAIT-C teleop smoke via
   `run_avp_arm_hand_sim.py --hand-backend real`.
