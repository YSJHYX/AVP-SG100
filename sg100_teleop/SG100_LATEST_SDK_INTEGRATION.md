# SG100 Latest SDK Integration (V1.5_NO_TACTILE)

Status: software migration complete; **new-SDK hardware validation NOT YET
PERFORMED**.  Every hardware-facing behavior below is offline-verified only
(headers/docs/examples/binary audit + compiled wrapper + mocked tests) until
a hardware session says otherwise.

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

## Feedback API

- `RegisterHandStatusCallback` delivers `Hand_Status_t` on the vendor
  communication thread at ~100 Hz while realtime runs; positions are the
  same user-degree domain (inverse map `(wire*180/pi - b)/a`, no receive
  clamp).  The wrapper copies each sample under a mutex; Python polls it
  and dedupes by a monotonically increasing sequence (same sequence ==
  cache not refreshed).  No device timestamp exists; the boundary uses
  host monotonic receive time (`host_monotonic_receive`).
- `RequestHandStatus` (0x26) remains available for explicit queries; in
  RS485 active mode the scheduler delivers it.  Caveat carried over from
  RUN-A evidence: in NONRT_PAUSED the palm answer reflects its status
  cache and can be stale — the authoritative measured state for decisions
  is feedback sampled AFTER realtime entry.
- `measured_hand_q[11]`: canonical radians, `SG100HandFeedback.q_rad`,
  software joint order below.  Domain facts: the wire field is the joint
  board accumulated coordinate (multi-turn, zero = joint-board reference),
  so canonical zero alignment is meaningful only relative to the joint
  boards' homing state — see "Zero semantics".

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
                     MODE_NULL passive hold; MuJoCo shows measured feedback
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
- Startup pose is NEVER redefined as zero.  `assess_zero_pose()` only
  REPORTS measured deviation from canonical zero.
- Controlled return-to-zero: NOT IMPLEMENTED (blocked until the canonical
  zero <-> vendor zero alignment is proven on hardware).

## C / AVP semantics

Before C: no AVP hand commands reach the SG100 (driver fail-closed +
runner gate); hand under MODE_NULL hold; MuJoCo real-hand pose renders
measured feedback, never the command.  At C: AVP anchor captured, teleop
enabled.  Data contract unchanged: `state_hand` = measured feedback,
`action_hand` = actually-sent command, `next_state_hand` = post-action
measurement, all canonical radians.

## Known open blockers

1. **joint_angle_map.json divergence** — the on-disk file (11 entries,
   mixed signs; all our 2026-09 hardware evidence was collected with it)
   differs from the branch HEAD version (J2..J11 all `a=-1`, different
   ranges).  Which mapping matches our physical hand is decidable only in
   a hardware session; until then the driver uses the on-disk file and
   rejects out-of-range targets.  With the identity hardware map +
   on-disk json, canonical `middle_j1` (software range [-2.18, 0] rad)
   maps to NEGATIVE user degrees outside the json [0, 125] range — every
   natural middle_j1 flexion target is currently REJECTED.  Resolving this
   (hardware map gain sign or json choice) is the first hardware task.
2. Vendor send clamp + sign semantics hardware-unverified on the new SDK.
3. MODE_JOINT_POSITION behavior and gain suitability hardware-unverified.

## Historical hardware baseline (OLD architecture — not transferable)

The deleted CANFD-passive integration demonstrated on 2026-09-02: ~100 Hz
command / ~100 Hz feedback whole-hand hold, CRC-clean, stable at measured
poses, plus the RUN A/B feedback-semantics evidence (0x26 NONRT staleness;
wire positions are accumulated coordinates with joint-board-dependent
zeroes).  Those results validate the PHYSICAL LINK and protocol capacity
only; they are NOT a validation of this new backend.  New-SDK hardware
status: **NOT YET VALIDATED**.

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
