# AGENT HANDOFF — INDEPENDENT MODE7 PROBE REVIEW (2026-09-03, round 5)

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
- That artifact is **not a physical-domain PASS**. It reports
  `MODE7_POSITION_DOMAIN=AMBIGUOUS`: J2 is +18.083° against configured
  `[-150,0]`, and J5/J7/J10 are negative against nonnegative configured
  ranges. MODE7 entry/stability is now observed, but canonical physical
  angle/range semantics remain unvalidated.

## Independent-review findings and fixes (probe v4)

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

- Focused mode7 suite: 62 passed.
- Full `test_sg100_*.py` suite: 166 passed.
- Selected SG100 driver + arm/hand data-safety suite: 107 passed.
- `compileall`, dry-run/explain, and `git diff --check`: passed.

## Gates

- `MODE7_PHYSICAL_FEEDBACK_HARDWARE_VALIDATED = False`.
- Driver `canonical_valid` remains false by default.
- Dataset remains quarantined and not training ready; 17D schema unchanged.
- Active motion, homing, AVP/H2, and dataset training remain blocked.

## Next step

Do not run more hardware from this handoff. First resolve the mode7 callback
sign/range disagreement using the preserved v4 artifact and vendor semantics.
No gate change, active motion, homing, AVP/H2, or dataset training is allowed.
