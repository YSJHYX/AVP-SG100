"""Real-robot backend layer for RM65 sim-to-real migration (v3 §B2).

This subpackage owns every RealMan SDK interaction.  Upstream teleop
code (tracking, mapper, TCP filter, IK) must not import the SDK
directly; it publishes joint targets through an ``ArmBackend``.

Module map (sg100_teleop/RM65_真机迁移方案_v3.md §B2.1):

- ``arm_state``: ``ArmTarget`` / ``ArmState`` dataclasses
- ``latest_target_buffer``: thread-safe single-slot target buffer
- ``joint_trajectory_safety``: q / qdot / qddot limiter (measured dt)
- ``ik_continuity_gate``: glitch-frame guard (failure classification)
- ``safety_monitor``: age/follow-error watchdogs + stop sequence
- ``arm_backend``: backend abstraction
- ``rm65_sim``: sim twin backend (writes MuJoCo ctrl)
- ``rm65_real``: real RM65 backend (CANFD sender + feedback, SDK lazy)
"""
