"""Connected-session return cycle. No SDK wait or automatic AVP anchoring."""
from __future__ import annotations

import time

import numpy as np


class ReturnHomeCycle:
    def __init__(self, arm, hand, arm_target, hand_target, *, speed_percent=25,
                 timeout_s=30., tolerance_rad=np.deg2rad(1.), stable_s=.2):
        self.arm, self.hand = arm, hand
        self.arm_target = np.asarray(arm_target, dtype=float).copy()
        self.hand_target = np.asarray(hand_target, dtype=float).copy()
        self.speed_percent = speed_percent
        self.timeout_ns = int(timeout_s * 1e9)
        self.tolerance_rad = tolerance_rad
        self.stable_ns = int(stable_s * 1e9)
        self.stage = "IDLE"
        self.message = ""
        self.started_ns = 0
        self._stable_since = None
        self._last_sequence = None

    @property
    def moving(self):
        return self.stage == "MOVING"

    @property
    def allows_arm(self):
        return self.stage in ("IDLE", "READY")

    @property
    def allows_calibration(self):
        return self.stage in ("IDLE", "READY", "WAITING_NEUTRAL", "CANCELLED")

    def snapshot(self):
        return {"stage": self.stage, "message": self.message}

    def _feedback(self, now):
        state = self.arm.joint_state()
        if (state is None or not state.valid or state.controller_error not in (None, 0)
                or not 0 <= now - state.received_monotonic_ns <= 250_000_000
                or np.asarray(state.q_rad).shape != (6,) or not np.all(np.isfinite(state.q_rad))):
            raise RuntimeError("归位时机械臂反馈过期、无效或控制器报错")
        return state

    def _hand_transport(self, now):
        if getattr(self.hand, "fault_active", False) or not getattr(self.hand, "serial_open", True):
            raise RuntimeError("归位时手部故障或串口断开")
        if hasattr(self.hand, "read_feedback"):
            feedback = self.hand.read_feedback()
            # Passive -> MODE7 is asynchronous. After the initial hand command,
            # allow its mode switch to appear in the callback, not indefinitely.
            if now - self.started_ns < 200_000_000:
                return
            if (feedback is None or not feedback.valid
                    or not 0 <= now - feedback.received_monotonic_ns <= 100_000_000
                    or tuple(feedback.joint_modes) != (7,) * 11):
                raise RuntimeError("归位时手部反馈过期、无效或没有进入 MODE7")

    def start(self):
        if self.stage in ("MOVING", "ANCHORING"):
            raise RuntimeError("归位或标定正在进行")
        if self.arm is None or self.hand is None or self.arm.enabled:
            raise RuntimeError("归位要求双真机已连接且遥操已停止")
        self._feedback(time.monotonic_ns())
        self.stage = "MOVING"
        self.started_ns = time.monotonic_ns()
        self._stable_since = self._last_sequence = None
        self.message = "正在执行预设归位，可随时点击停止遥操中止"
        try:
            if self.hand.send_positions_rad(self.hand_target.copy()) is None:
                raise RuntimeError("手部初始姿态指令被拒绝")
            self.arm.begin_runtime_return(self.arm_target.copy(), speed_percent=self.speed_percent)
        except Exception as exc:
            self._stop("FAILED", str(exc))
            raise

    def _stop(self, stage, message):
        errors = []
        for name, stop in (("RM65", self.arm.stop_smooth), ("SG100", self.hand.stop)):
            try:
                result = stop()
                if result not in (None, 0):
                    raise RuntimeError(f"stop returned {result}")
            except Exception as exc:
                errors.append(f"{name}: {exc}")
        self.stage = "FAILED" if errors else stage
        self.message = message + ("；停止未确认：" + "; ".join(errors) if errors else "")
        return errors

    def cancel(self):
        if self.stage in ("MOVING", "ANCHORING", "FAILED"):
            errors = self._stop("CANCELLED", "归位已中止，请重新归位或重新建立锚点")
            if errors:
                raise RuntimeError(self.message)

    def poll(self):
        """True exactly once when fresh, stationary feedback confirms arrival."""
        if not self.moving:
            return False
        now = time.monotonic_ns()
        try:
            if now - self.started_ns > self.timeout_ns:
                raise RuntimeError("归位等待到位超时")
            state = self._feedback(now)
            self._hand_transport(now)
            if state.sequence == self._last_sequence or state.received_monotonic_ns <= self.started_ns:
                return False
            self._last_sequence = state.sequence
            position_ok = np.max(np.abs(state.q_rad - self.arm_target)) <= self.tolerance_rad
            stationary = state.qd_rad_s is not None and np.all(np.isfinite(state.qd_rad_s)) and np.max(np.abs(state.qd_rad_s)) <= .05
            if position_ok and stationary:
                if self._stable_since is None:
                    self._stable_since = state.received_monotonic_ns
                elif state.received_monotonic_ns - self._stable_since >= self.stable_ns:
                    self.arm.complete_runtime_return(state.q_rad)
                    self.stage = "WAITING_NEUTRAL"
                    self.message = "机械臂已到位，手部初始指令已发送。请确认手部姿态并保持中立，点击确认建立新锚点"
                    return True
            else:
                self._stable_since = None
        except Exception as exc:
            self._stop("FAILED", str(exc))
        return False

    def anchor_requested(self):
        if not self.allows_calibration:
            raise RuntimeError("当前归位状态不能建立锚点")
        if self.stage != "IDLE":
            self.stage = "ANCHORING"
            self.message = "等待确认之后的新有效 AVP 帧建立锚点"

    def anchor_complete(self):
        if self.stage == "ANCHORING":
            self.stage = "READY"
            self.message = "新锚点已建立，等待启用遥操"

    def anchor_lost(self):
        if self.stage in ("ANCHORING", "READY"):
            self.stage = "WAITING_NEUTRAL"
            self.message = "AVP 锚点失效，请恢复跟踪并重新确认中立姿态"
