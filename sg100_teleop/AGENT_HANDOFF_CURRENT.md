# AGENT HANDOFF — MODE7 DOMAIN SEPARATION (2026-09-03, round 6)

**Session result:** second-reviewer software audit and repair only. No hardware
execution, no commit, and no push.

## Hardware evidence

- N0 palm realtime + MODE_NULL ×11 passed at ~100 Hz, fault 0, with no visible
  movement. MODE_NULL feedback is not canonical joint angle.
- The first real MODE7 submission used all joints mode 7 and zero position,
  gains, and output limit. Its first RX frame was mode0 ×11 and probe v2
  stopped before the movement prompt. This remains **EXPECTED RX TRANSIENT +
  OLD PROBE BUG**, not evidence that mode7 was rejected.
- That first run therefore left MODE7 reached **UNKNOWN**, physical feedback
  **NOT TESTED**, and movement **NOT CONFIRMED**.
- A new working-tree artifact subsequently appeared at
  `recording/sg100_mode7_probe_20260903_163543/`. This reviewer did not execute
  hardware and inspected it read-only. It records: first RX mode0 at 9.407 ms;
  three mode0 frames; first all-mode7 at 39.396 ms; `MODE7_STABLE` at the fifth
  consecutive all7 frame at 79.401 ms; 20 total frames/17 stable mode7 samples;
  fault 0; no visible movement; one application mode7 update; successful
  MODE_NULL restoration and disconnect.
- The artifact is a **real-hardware PASS** for MODE7 entry/control, all-11
  MODE7 status, and no-visible-motion entry. It also **SUPPORTS** a finite,
  stable, non-multiturn joint-angle-scale feedback domain. The artifact's old
  `MODE7_POSITION_DOMAIN=AMBIGUOUS` conclusion conflated SDK command ranges
  with feedback coordinates and is retired.
- Range diagnostics are now separate: SDK command compatibility is PARTIAL
  (7/11); canonical range compatibility is PARTIAL (8/11). J7 `-6.9324°` is
  outside SDK `[0,125]` but inside canonical `middle_j1` about `[-124.9,0]`.
  J2 `+18.0833°`, J5 `-0.4285°`, and J10 `-3.9554°` remain exact canonical
  mapping questions. `F_fb` identity/sign/zero/range is not yet validated.

## Independent-review findings and fixes (probe v5)

The previous implementation was accepted only in part. Its core transition
idea was sound, but the review found material defects:

- `send_positions_rad(np.zeros(11))` used the production canonical mapping,
  clipping, unit conversion, and configured-gain abstraction. The probe now
  calls narrow `SG100SDKDriver.send_mode7_feedback_probe()`, which submits one
  exact native SDK-domain update: ids 1..11, mode7 ×11, position 0 degrees,
  p=i=d=output_limit=0. It still uses `sg100_set_hand_control` and
  `SG100Device::SetHandControl`; the vendor scheduler owns ~100 Hz cached
  retransmission.
- A cached Phase-A callback could be counted as post-command frame 0 and yield
  false 0.0 ms latency. Only callbacks whose receive timestamp is strictly
  after `SetHandControl` returned are now admitted. Sequence and callback
  timestamp both participate in deduplication.
- The fifth consecutive fresh all-11-mode7 frame now marks `MODE7_STABLE`
  immediately and fixes the exact latency, but does not end observation. The
  same cached target remains untouched while feedback-only collection
  continues to at least 20 total fresh post-command frames. There is still
  exactly one application `SetHandControl` update.
- Position classification uses the full contiguous all-mode7 sample window
  from the run that established stability through observation completion;
  earlier isolated mode7 and transitional mode0 samples remain excluded.
- The angle-scale verdict no longer accepts/rejects by SDK command range.
  `MODE7_JOINT_ANGLE_SCALE_DOMAIN`, SDK command compatibility, canonical
  range compatibility, and exact canonical mapping are independent fields.
  Strict range checks have no pass-forcing tolerance; a sub-degree diagnostic
  label may identify a near-boundary offset candidate without changing FAIL.
- Loss of all-mode7, fault, multi-turn mode7 data, non-finite data, or a fresh
  RX stall during the post-stable tail still fails the probe.
- A mode7 value beyond ±360 degrees is a preserved semantic rejection, not a
  transport exception. Unexpected returned modes are likewise rejected.
  Fault, fresh-RX stall, and non-finite data remain hard failures.
- Every post-submit exit attempts MODE_NULL restoration. The outer lifecycle
  then attempts passive hold, `stop()`, and `disconnect()` independently.
- Partial metadata/summary/CSV artifacts survive timeout, semantic rejection,
  operator movement, EOF, KeyboardInterrupt, fault, and unexpected exception.
  CSV rows are fresh post-submit callbacks only and include sequence,
  timestamp, relative time, 11 modes, and 11 positions.
- Movement answers are explicit: `y`/`yes` = movement and FAIL;
  `n`/`no`/Enter = no visible movement observed; anything else is not silently
  accepted.

The all-zero control fields are a **zero-output configuration intended to
produce no active drive**. They do not prove firmware behavior; no-motion
remains a real-hardware acceptance criterion.

## Current offline evidence

- Focused MODE7 suite: **66 passed**.
- Full `test_sg100_*.py` collection under Windows/UTF-8 mode: **166 passed,
  4 failed**. Two failures are Windows-only test assumptions (`grep` and a
  POSIX path suffix); two are the existing N1 guard correctly detecting that
  the uncommitted Windows SDK-directory selection exposes a different J2
  command interval than the preserved hardware-run map. None changes the
  MODE7 feedback-domain result.
- Selected MODE7 + arm/hand data-safety suite: **85 passed, 2 failed**. The
  remaining failures are a Windows temporary-NPZ file lock and an existing
  causal-chain assertion, outside this correction.
- `compileall`, passive mapping dry-run, feedback-probe explain mode, and
  `git diff --check`: passed.
- No hardware was executed.

## Gates

- `MODE7_JOINT_ANGLE_SCALE_HARDWARE_SUPPORTED = True`.
- `MODE7_CANONICAL_MAPPING_HARDWARE_VALIDATED = False` and the composite
  verdict is `PARTIAL_ANGLE_DOMAIN_SUPPORTED`.
- Driver `canonical_valid` remains false by default.
- Dataset remains quarantined and not training ready; 17D schema unchanged.
- Active motion, homing, AVP/H2, and dataset training remain blocked.

## Next step

Do not run more hardware from this handoff. The prepared next stage is
`tools/sg100_mode7_mapping_characterization.py`, one physical DOF per run. It
submits the proven zero-output MODE7 configuration once, then observes only
raw MODE7 feedback while the operator moves the joint in a canonical
direction and returns it. It sends no later target, performs no homing/zeroing
or startup-offset derivation, changes no production map, fails on fault/mode
loss, and restores MODE_NULL. It was not executed this round. No gate change,
active motion, AVP/H2, or dataset training is allowed.
