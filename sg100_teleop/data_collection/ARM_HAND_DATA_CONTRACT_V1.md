# RM65-SG100 Arm-Hand Data Contract V1

## Canonical schema

- Schema: `rm65_sg100_joint17_v1`
- Physical dimension: `17`
- Master clock: `time.monotonic_ns()`
- Representation: absolute joint position
- RM65 unit: radians
- SG100 unit: radians
- Stored canonical data is never normalized in place.

The authoritative order is:

```text
0:6    RM65 q1..q6 absolute joint positions [rad]
6:17   SG100 q0..q10 absolute joint positions [rad]
```

This is a canonical RM65-SG100 platform representation. It is not an
embodiment-independent representation because the first six coordinates are
specific to RM65 morphology.

## Transition semantics

Two provenance modes share the same 17D layout and schema name.  The
mode is recorded in the manifest (`hand_state_source` +
`writer_version`); the layout never changes.

### Measured mode (writer_version 2, SG100 hardware present)

For one accepted command bundle:

```text
state_t = [measured_arm_q_before[6] |
           measured_hand_q_before[11]]        # both strictly before action

action_t = [actual_arm_qcmd_sent_after_limiter[6] |
            hand_hardware_sent_qcmd[11]]      # final post-lock,
                                               # post-filter, transport-accepted

next_state_t = [measured_arm_q_after_action[6] |
                first_measured_hand_q_after_action[11]]
```

Sources are fixed:

- `arm_state_source = "rm65_measured_joint_feedback"`
- `arm_action_source = "real_joint_command_sent"`
- `hand_state_source = "sg100_measured_joint_feedback"`
- `hand_initial_state_source = "first_measured_feedback_replay_seed"`
- `hand_action_source = "final_hardware_sent_qcmd"`
- `hand_hardware_present = true`
- `hand_feedback_available = true`
- `hand_feedback_valid = true` (finite/transport-valid only; it does not mean
  canonical-valid—MODE7 angle-scale support and exact `F_fb` mapping are
  separate)
- `hand_feedback_semantics = "sg100_driver_reported_joint_feedback"`
  (the SDK 0x05 response is a measurement, not a command echo; position
  semantics depend on returned joint mode: MODE_NULL is proven
  raw/accumulated-like, while mode 7 is real-hardware-supported as bounded
  joint-angle-scale feedback but not yet mapped exactly to canonical state)
- `hand_feedback_physical_scale = "ANGLE_SCALE_SUPPORTED_MAPPING_PENDING"`
- `hand_canonical_mapping_verdict = "BLOCKED"`
- `hand_feedback_timestamp_source = "host_monotonic_receive"` (the
  vendor SDK provides no device timestamp; none is fabricated)
- `hand_action_hardware_applied = false` — the SDK can only prove the
  frame was queued (`hand_action_hardware_sent = true`); only
  measured feedback proves execution

Hand causal rules (mirroring the arm standard):

- `state_hand` is the newest measured SG100 feedback strictly before
  the action-effective time (max(arm_sent, hand_sent));
- `next_state_hand` is the FIRST measured SG100 feedback strictly
  after it;
- if any other hand command was successfully sent between the action
  and that post-action feedback, the bundle is hard-rejected as
  `INTERVENING_HAND_COMMAND` — a two-command transition is never
  relabelled as single-action;
- the canonical hand action is the final post-retarget, post-pinch,
  post-power-grasp-lock, post-clamp, post-filter qcmd that the
  transport accepted.  The raw retarget value (pre-lock, e.g. thumb_j1
  60 deg before the +40 deg lock bias) stays in the raw audit stream
  only.

Measured-mode hand rejections:

- `NO_PRE_ACTION_HAND_FEEDBACK`, `NO_POST_ACTION_HAND_FEEDBACK`
- `STALE_PRE_HAND_STATE`, `STALE_POST_HAND_STATE`
- `HAND_FEEDBACK_INVALID` (NaN/non-finite/deduped-invalid)
- `HAND_COMMAND_NOT_SENT`, `HAND_SENT_WITHOUT_DECISION`
- `INTERVENING_HAND_COMMAND`

The measured replay seed: the first valid measured sample is recorded
as the `hand_measured_baseline` raw event for replay/audit.  It is
NOT required to equal `state[0, 6:17]` (that V1 proxy equality was
the frozen baseline-replay bug; measured state comes from feedback,
not from a baseline).

### Proxy mode (writer_version 1, legacy / no hand hardware)

```text
state_t = [measured_arm_q_before[6] |
           previous_hand_qcmd_proxy[11]]

action_t = [actual_arm_qcmd_sent_after_limiter[6] |
            final_teleop_hand_qcmd[11]]

next_state_t = [measured_arm_q_after_action[6] |
                latest_hand_qcmd_proxy_after_action[11]]
```

- `hand_state_source = "previous_command_proxy"`
- `hand_initial_state_source = "serialized_command_proxy_baseline"`
- `hand_action_source = "final_teleop_qcmd"`
- `hand_hardware_present = false`
- `hand_action_hardware_applied = false`
- `hand_feedback_available = false`
- `hand_feedback_valid = false`

Legacy episodes stay readable exactly as recorded; the inspector
labels them `LEGACY HAND PROXY EPISODE` and never reinterprets the
proxy values as measured feedback.

An AVP target, TCP target, IK goal, producer target, or pre-limiter joint goal
must never be stored as canonical arm state/action. They belong only to the
raw audit stream.

## Successful RM65 send boundary

`RM65RealBackend` emits `ArmCommandSent` only after all of the following:

```text
latest IK/producer target
  -> real JointTrajectorySafetyLimiter
  -> rm_movej_canfd(...)
  -> SDK success return
  -> ArmCommandSent(q_rad, sent_monotonic_ns, command_bundle_id)
```

The event contains both limiter input and actual limiter output. Failed SDK
sends do not emit canonical action events.

## Command bundle clock

Every eligible control decision receives a monotonically increasing
`command_bundle_id` and records:

- `decision_timestamp_ns`
- `arm_command_sent_timestamp_ns`
- `hand_command_applied_timestamp_ns`
- `action_effective_timestamp_ns = max(arm sent, hand applied)`

Timestamp honesty: both backend SDK calls return at HOST send time; no
field claims to be a device actuation timestamp.  In measured mode the
stored `hand_command_applied_timestamp_ns` is the HAND HARDWARE SEND
time (host send, field name kept for schema compatibility); in proxy
mode it is the software-command time.

Every canonical transition must satisfy:

```text
state_timestamp_ns
  < action_effective_timestamp_ns
  < next_state_timestamp_ns
```

The state is the newest valid RM65 feedback strictly before action effective
time. The next state is a newly received valid RM65 feedback strictly after
that time. The collector never implements transitions as `state[:-1]`,
`action[:-1]`, `state[1:]`.

## First action and pre-roll

When the recorder is armed it immediately starts ingesting RM65 feedback,
including while real-arm streaming is disabled. A first actual command is
accepted only if a valid pre-action feedback sample already exists. Otherwise
the bundle is rejected as `NO_PRE_ACTION_FEEDBACK`.

## Hand initial proxy baseline (proxy mode only)

When the collector enters `RECORDER_ARMED` in PROXY mode, it serializes the
current valid final teleoperation hand qcmd as a standalone
`hand_proxy_baseline` raw event. The event contains a sequence ID, monotonic
timestamp, 11D qcmd, source, and explicit `measured=false`,
`hardware_feedback=false`, and `hardware_applied=false` provenance.

The first hand state is defined as:

```text
state_hand_0 = serialized current final qcmd baseline
baseline_timestamp_ns < first_hand_action_timestamp_ns
```

The baseline is a command proxy only. It is neither SG100 encoder feedback nor
confirmation that SG100 hardware moved. If no valid current final qcmd exists,
the collector waits: it never substitutes zeros or a nominal home pose, and it
cannot build a full 17D transition until an explicit baseline is established.
The inspector independently compares `state[0, 6:17]` with the serialized raw
baseline using the same strict raw/canonical tolerance policy.

In MEASURED mode the baseline role is replaced by the first measured feedback
sample (`hand_measured_baseline` raw event, replay seed / audit only). MODE_NULL
samples are raw/accumulated-like SDK callback coordinates and must never be
silently interpreted as physical/canonical hand state. Canonical eligibility
requires all expected joints to return mode 7 **and** hardware validation of
the exact feedback-to-canonical identity/sign/zero/range map. The preserved
2026-09-03 real run supports MODE7 as a bounded, stable joint-angle-scale
domain, but that evidence alone does not establish the canonical map.
**The mode gate is now implemented** (2026-09-03): the driver preserves
`joint_modes`/`feedback_domain`/`canonical_valid` per sample, and the
collector rejects any sample that carries mode information but is not
canonical-valid with `HAND_FEEDBACK_NON_CANONICAL_MODE` — a MODE_NULL
accumulated value (e.g. the N1 ~1854-deg trace) can no longer enter
`measured_hand_q`. Until exact `F_fb` mapping validation passes
(`canonical_valid` becomes attainable), every measured SG100 episode remains
quarantined and non-training-eligible.

## Raw audit stream

`arm_hand_raw_events.jsonl` retains physical, timestamped events including:

- raw AVP wrist pose
- mapped relative wrist target
- filtered TCP target
- IK candidate and final joint goal
- IK convergence/acceptance
- continuity-gate status
- latest producer target
- real limiter input and output
- successful `q_cmd_sent`
- measured RM65 feedback and qvel when available
- controller TCP when available
- final teleoperation hand software qcmd
- tracking and safety status
- hard rejection events
- serialized initial hand command-proxy baseline

Measured mode additionally retains the full hand provenance chain:

- `hand_raw_retarget` / `hand_coordinated_command` /
  `hand_final_command` — the three retargeter stages (base mapping,
  post-pinch, post-power-grasp-lock)
- `hand_hardware_sent` — commands actually accepted by the SG100
  transport (host send timestamp, `transmission_confirmed` =
  successfully_transmitted)
- `hand_feedback` — every new measured SG100 sample (measured=true,
  host receive timestamp)
- `hand_measured_baseline` — the first measured replay seed

The current runner records `continuity_gate_status =
"not_applied_in_current_runtime"` because the current production runner does
not invoke `IKContinuityGate`. This is an honest audit value, not a fabricated
acceptance result. The backend limiter and watchdog remain active.

## Episode files

With `--collect-arm-hand-data`, the visual recording session contains:

```text
arm_hand_data/
  arm_hand_joint17_v1.npz
  arm_hand_raw_events.jsonl
  arm_hand_manifest.json
```

The NPZ contains `float64` arrays.  RM65 coordinates are physical; SG100
measured coordinates are quarantined as unscaled while the N1 blocker is
open:

- `state [N,17]`
- `action [N,17]`
- `next_state [N,17]`
- command bundle IDs and all causal timestamps
- `arm_state_measured [N] = true`
- `hand_state_measured [N] = false` (proxy mode) / `true` (measured mode)
- `hand_action_hardware_applied [N] = false`
- `hand_action_hardware_sent [N]` (measured mode: transport-accepted)

The manifest records sources, units, rejection counts, camera presence,
Markov diagnostic, quarantine status, and training-eligible indices.

## Hard causal rejection

Rejected bundles do not enter canonical training arrays. Supported reasons:

- `NON_MONOTONIC_TIMESTAMP`
- `STATE_AFTER_ACTION`
- `NEXT_STATE_BEFORE_ACTION`
- `NO_PRE_ACTION_FEEDBACK`
- `NO_POST_ACTION_FEEDBACK`
- `STALE_PRE_STATE`
- `STALE_POST_STATE`
- `ARM_FEEDBACK_INVALID`
- `ARM_COMMAND_NOT_SENT`
- `HAND_COMMAND_INVALID`
- `IK_REJECTED`
- `SAFETY_FAULT`
- `TRACKING_LOST`
- `STALE_TARGET`
- `NONFINITE_STATE`
- `NONFINITE_ACTION`
- `OUT_OF_RANGE`
- `CLOCK_DISCONTINUITY`
- `MISSING_HAND_PROXY_BASELINE`
- `INVALID_HAND_PROXY_BASELINE`
- `HAND_BASELINE_AFTER_ACTION`
- `HAND_BASELINE_STATE_MISMATCH`
- measured mode only: `NO_PRE_ACTION_HAND_FEEDBACK`,
  `NO_POST_ACTION_HAND_FEEDBACK`, `STALE_PRE_HAND_STATE`,
  `STALE_POST_HAND_STATE`, `HAND_FEEDBACK_INVALID`,
  `HAND_COMMAND_NOT_SENT`, `HAND_SENT_WITHOUT_DECISION`,
  `INTERVENING_HAND_COMMAND`

Default feedback freshness limits are 100 ms pre-action and 250 ms
post-action and can be configured without changing the schema.

## Markov diagnostic is not a proof

The 17D model state intentionally contains positions only. Joint velocity,
controller/motor internal state, contact force, latency, friction, and other
hidden variables can affect dynamics. Therefore:

- hard causal validation determines transition eligibility;
- `MARKOV_OK`, `MARKOV_SUSPECT`, and `INSUFFICIENT_DATA` are statistical
  diagnostics only;
- `MARKOV_SUSPECT` quarantines canonical transitions by default;
- raw events and episodes are never physically deleted by the diagnostic.

Auxiliary fields include arm qvel when available, feedback dt, previous
canonical action, tracking/safety status, command latency, controller TCP,
and a hand-qcmd velocity proxy. They are not part of the 17D model state.

## Cameras

No real camera is connected in V1:

- `head_camera_present = false`
- `wrist_camera_present = false`
- `multimodal_training_ready = false`

`NullCameraProvider.latest()` returns `None`; it never creates a fake black
image. `RealSenseCameraProvider` is a future interface and raises until a real
implementation is connected.

The calibration/synchronization schema reserves:

- `T_base_head_camera` or generic parent-to-head transform
- `T_parent_wrist_camera`
- intrinsics, distortion, depth scale, calibration ID
- sensor timestamp, host receive timestamp, master timestamp, frame ID
- a shared monotonic master-clock model

## Normalization and future adapters

Normalization belongs to dataset build/training adapters. It must be computed
from the training split (for example q01/q99) and must not destructively
replace the hardware recorder's radians.

`ArmHandDatasetAdapter` and `load_physical_episode` form the stable seam for a
future `export_arm_hand_to_openpi.py` or `export_arm_hand_to_lerobot.py`.
Padding to 32D, normalization, and camera mapping occur there after the future
AHFE architecture is finalized. The hardware recorder does not depend on
AHFE, Pi0.5 heads, checkpoint format, or model action dimension.

## Runtime activation

Canonical collection is opt-in and real-feedback-only:

```bash
python -m sg100_teleop.sim.run_avp_arm_hand_sim \
  --arm-backend real \
  --arm-ip <RM65_IP> \
  --collect-arm-hand-data
```

The runner rejects `--collect-arm-hand-data` in simulation mode rather than
mislabeling MuJoCo qpos as measured RM65 feedback.

## Offline episode inspection

After collection, validate the saved canonical arrays against their manifest
and raw event stream without modifying the episode:

```bash
python -m sg100_teleop.data_collection.inspect_episode \
  recording/<session>/arm_hand_data
```

The inspector recomputes strict causality, exact command-bundle association,
effective timestamps, RM65 raw/canonical command and feedback matches, SG100
command-proxy consistency, finite-value checks, and stored rejection
consistency. See `REAL_DATA_V1_VALIDATION.md` for the first hardware run and
acceptance procedure.
