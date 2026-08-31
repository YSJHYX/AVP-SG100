# DATA CONTRACT V1 Compatibility Audit

Audit date: 2026-08-29

## Scope and read-only rule

OpenPI was inspected only. No files under `~/vla_ws/openpi` were modified by
this task. The audited checkout was:

- branch: `feature/arm-hand-ahfe-pytorch`
- commit: `0d0affe1efee863d15b817217239ad61152fa76a`

The checkout already contained unrelated local AHFE/model changes at audit
time. They were left untouched.

## Confirmed dimensions

The current OpenPI arm-hand policy declares:

| Layer | Dimension |
|---|---:|
| RM65 physical arm | 6 |
| SG100 physical hand | 11 |
| Physical state/action | 17 |
| Pi0/Pi0.5 model action dimension | 32 |

Evidence inspected:

- `src/openpi/policies/arm_hand_policy.py`
  - `ARM_DIM = 6`
  - `HAND_DIM = 11`
  - `ROBOT_ACTION_DIM = 17`
  - `MODEL_ACTION_DIM = 32`
- `src/openpi/policies/arm_hand_policy_test.py`
  - validates physical 17D input/output and `[0:6] | [6:17]` split
  - validates cropping model 32D output back to physical 17D
- `src/openpi/transforms.py`
  - `PadStatesAndActions` pads physical state/action to model dimension
- `src/openpi/models/pi0_config.py`
  - model `action_dim` default remains 32
- `src/openpi/transforms.py`, `src/openpi/training/data_loader.py`
  - normalization uses checkpoint/data-config `norm_stats`
  - quantile normalization is supported

## Compatibility result

`rm65_sg100_joint17_v1` is dimensionally compatible with the current policy
adapter:

```text
physical recorder [RM65 joint6 | SG100 joint11] = 17D radians
  -> dataset adapter / training transforms
  -> training-split normalization
  -> padding to model 32D
  -> Pi0.5 / future finalized AHFE
```

The collector deliberately does not import OpenPI, read model configuration,
pad to 32D, normalize, synthesize camera images, or depend on AHFE internals.

## Architecture boundary

The supported direction of dependency is:

```text
REAL ROBOT
  -> timestamped raw event stream
  -> causal canonical rm65_sg100_joint17_v1 episode
  -> ArmHandDatasetAdapter
  -> future finalized OpenPI/AHFE or LeRobot export
```

The unfinished AHFE architecture must not determine hardware recording
format. This task did not change:

- `ArmHandFeatureFactorizer`
- action heads or auxiliary heads
- Pi0.5 backbone
- pretrained projections
- `MODEL_ACTION_DIM`
- checkpoint restoration
- normalization implementation

## Current camera limitation

The present policy adapter expects head imagery and supports optional wrist
imagery, but V1 hardware collection has neither real camera connected. The
episode is correctly marked `multimodal_training_ready = false`; it is not yet
valid for multimodal training. Future camera providers must deliver real
frames and calibration on the shared monotonic clock before that flag can be
set true.

## Current runtime limitation

The production teleop runner currently does not call `IKContinuityGate`, even
though the component exists elsewhere in this repository. Raw audit records
that fact as `not_applied_in_current_runtime`. This does not change the
canonical action boundary: only the successful, real-limiter output sent to
RM65 becomes arm action. Introducing the gate later does not require a schema
change; its accepted/rejected status already has a stable raw-audit field.
