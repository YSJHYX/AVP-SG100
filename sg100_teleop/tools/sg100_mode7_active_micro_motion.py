#!/usr/bin/env python3
"""SG100 MODE7 单关节主动保持 + 单次微动安全验证工具。

默认是纯离线 DRY RUN，绝不构造 driver 或打开串口。只有 ``--execute``
才进入真机路径；真机路径仍需要 HOLD 和 MOVE 两次逐阶段显式确认。

J6 路径保留已验证的 SDK feedback seed；J7 路径从 canonical delta 出发，
经 F_cmd 恰好一次，并只使用已知的零输出 command target 作为 hold seed。
F_cmd 与 F_fb 继续独立，canonical_valid 始终为 False。
"""

from __future__ import annotations

import argparse
import csv
import dataclasses
import hashlib
import json
import subprocess
import time
from pathlib import Path
from typing import Callable, Iterable

import numpy as np

from sg100_teleop.robot.sg100_driver import (
    HAND_DOF,
    MODE_JOINT_POSITION,
    MODE_NULL,
    SG100HardwareMap,
)
from sg100_teleop.robot.sg100_sdk_driver import (
    DEFAULT_BAUDRATE,
    DEFAULT_SDK_DIR,
    SG100SDKDriver,
    load_vendor_user_ranges,
    quantize_gain,
)


ROOT = Path(__file__).resolve().parents[2]
RECORDING_ROOT = ROOT / "recording"

# 唯一有当前 SDK example + vendor production log 双重证据的低增益组。
ACTIVE_KP = 0.1
ACTIVE_KI = 0.0
ACTIVE_KD = 0.0
ACTIVE_OUTPUT_LIMIT = 0.5
GAIN_EVIDENCE = (
    "SG100_SDK_Publish/examples/realtime_joint_control_example.cpp: "
    "InitHandCtrl(0.1,0,0,0.5); vendor sg100_control.log MODE7 TX: "
    "p=328,i=0,d=0,output_limit=1638"
)

DEFAULT_DELTA_DEG = 0.5
HARD_MAX_DELTA_DEG = 1.0
LIMIT_MARGIN_DEG = 5.0
HARD_EXCURSION_DEG = 1.5
OTHER_JOINT_HARD_EXCURSION_DEG = 0.75
HOLD_HARD_EXCURSION_DEG = 0.5
MOVEMENT_DETECTION_DEG = 0.10
DOMINANCE_RATIO_MIN = 2.0
OSCILLATION_STEP_NOISE_DEG = 0.02
MAX_OSCILLATION_REVERSALS = 2
MAX_SINGLE_FRAME_STEP_DEG = 0.75
STABLE_MODE7_FRAMES = 5
BASELINE_FRAMES = 10
HOLD_OBSERVATION_S = 0.4
MICRO_OBSERVATION_S = 0.5
TRANSPORT_STALL_S = 0.1
POLL_PERIOD_S = 0.002

# 经任务安全审计的第一轴物理候选：范围约 [-60, 90]、运动简单可见。
PREFERRED_FIRST_JOINT_IDS = (3, 6, 8, 11)
UNRESOLVED_FIRST_JOINT_IDS = (2,)
PHYSICAL_DIRECTIONS = {
    3: {"+": "thumb distal flexion toward palm", "-": "thumb distal extension outward"},
    6: {"+": "index distal flexion toward palm", "-": "index distal extension toward dorsal side"},
    8: {"+": "middle distal flexion toward palm", "-": "middle distal extension toward dorsal side"},
    11: {"+": "little distal flexion toward palm", "-": "little distal extension toward dorsal side"},
    7: {"+": "middle flexion toward palm", "-": "middle extension outward"},
}


class ActiveMicroMotionAbort(RuntimeError):
    """需要立即 MODE_NULL cleanup 的 fail-closed 条件。"""


@dataclasses.dataclass
class ActiveSequenceGate:
    """两阶段单次微动门；禁止跳过 hold 或偷偷发送返回运动。"""

    hold_passed: bool = False
    move_authorized: bool = False
    micro_command_count: int = 0

    def accept_hold(self, numeric_verdict: str, operator_saw_motion: bool) -> None:
        if numeric_verdict != "PASS" or operator_saw_motion:
            raise ActiveMicroMotionAbort("ACTIVE HOLD 必须 PASS 且无可见运动")
        self.hold_passed = True

    def confirm_move(self, response: str) -> None:
        if not self.hold_passed:
            raise ActiveMicroMotionAbort("不得在 ACTIVE HOLD PASS 前进入 micro delta")
        if response.strip() != "MOVE":
            raise ActiveMicroMotionAbort("第二次确认不是 MOVE；默认 ABORT")
        self.move_authorized = True

    def record_micro_command(self) -> None:
        if not self.move_authorized:
            raise ActiveMicroMotionAbort("micro delta 未获二次确认")
        if self.micro_command_count != 0:
            raise ActiveMicroMotionAbort("第一次工具只允许一条 micro delta command")
        self.micro_command_count = 1


@dataclasses.dataclass(frozen=True)
class ActiveFrame:
    phase: str
    sequence: int
    received_monotonic_ns: int
    modes: tuple[int, ...]
    positions_deg: tuple[float, ...]
    requested_active_target_deg: float | None
    command_index: int


@dataclasses.dataclass(frozen=True)
class FirstJointChoice:
    joint_index: int
    joint_id: int
    canonical_name: str
    current_deg: float
    lower_deg: float
    upper_deg: float
    negative_headroom_deg: float
    positive_headroom_deg: float
    minimum_headroom_deg: float
    history_artifact: str


def validate_delta(delta_deg: float) -> float:
    value = float(delta_deg)
    if not np.isfinite(value) or value <= 0.0:
        raise ValueError("delta-deg 必须是有限正数")
    if value > HARD_MAX_DELTA_DEG:
        raise ValueError(
            f"delta-deg={value} 超过硬上限 {HARD_MAX_DELTA_DEG} deg"
        )
    return value


def parse_joint_selector(value: str | int) -> int:
    """接受 J7 / middle_j1 / 7，同时保留旧的整数 CLI。"""

    text = str(value).strip().lower()
    aliases = {"middle_j1": 7}
    if text in aliases:
        return aliases[text]
    if text.startswith("j"):
        text = text[1:]
    try:
        joint_id = int(text)
    except ValueError as exc:
        raise argparse.ArgumentTypeError(f"未知关节选择: {value}") from exc
    if not 1 <= joint_id <= HAND_DOF:
        raise argparse.ArgumentTypeError("关节必须为 J1..J11")
    return joint_id


def resolve_canonical_delta_deg(
    joint_id: int, legacy_delta_deg: float,
    canonical_delta_deg: float | None,
) -> float:
    """返回带符号 canonical delta；J7 只接受朝掌心的负向微动。"""

    if canonical_delta_deg is None:
        return -validate_delta(legacy_delta_deg) if joint_id == 7 else validate_delta(
            legacy_delta_deg
        )
    value = float(canonical_delta_deg)
    if not np.isfinite(value) or value == 0.0:
        raise ValueError("canonical-delta-deg 必须是有限非零值")
    validate_delta(abs(value))
    if joint_id == 7 and value >= 0.0:
        raise ValueError("J7 首次 sign-flip 测试只允许 canonical 负向 flexion")
    if joint_id != 7 and value <= 0.0:
        raise ValueError("identity 轴首次测试只允许 canonical 正向 delta")
    return value


def apply_f_cmd_once_deg(
    hardware_map: SG100HardwareMap, canonical_deg: Iterable[float]
) -> np.ndarray:
    """无 clamp 地应用一次 canonical -> SDK F_cmd，供审计与回归。"""

    canonical = np.asarray(tuple(canonical_deg), dtype=np.float64)
    if canonical.shape != (HAND_DOF,) or not np.all(np.isfinite(canonical)):
        raise ValueError("canonical_deg must be finite 11D")
    lower = np.rad2deg(hardware_map.lower_rad)
    upper = np.rad2deg(hardware_map.upper_rad)
    if np.any(canonical < lower) or np.any(canonical > upper):
        raise ValueError("canonical command outside range; no clamp")
    return hardware_map.gain * canonical + np.rad2deg(hardware_map.offset_rad)


def plan_j7_sign_flip(
    hardware_map: SG100HardwareMap,
    vendor_ranges: list[tuple[float, float]],
    canonical_delta_deg: float,
) -> dict:
    """从已知 canonical=0 / SDK=0 command seed 规划 J7，一次 F_cmd。"""

    index = hardware_map.software_index_of["middle_j1"]
    if hardware_map.hardware_joint_ids[index] != 7:
        raise ActiveMicroMotionAbort("middle_j1 hardware channel 不是 J7")
    delta = resolve_canonical_delta_deg(7, abs(canonical_delta_deg), canonical_delta_deg)
    gain = float(hardware_map.gain[index])
    offset_deg = float(np.rad2deg(hardware_map.offset_rad[index]))
    if gain != -1.0 or offset_deg != 0.0:
        raise ActiveMicroMotionAbort("J7 F_cmd 必须为 gain=-1, offset=0")
    canonical_current = 0.0
    canonical_final = canonical_current + delta
    canonical_low = float(np.rad2deg(hardware_map.lower_rad[index]))
    canonical_high = float(np.rad2deg(hardware_map.upper_rad[index]))
    if not canonical_low <= canonical_final <= canonical_high:
        raise ActiveMicroMotionAbort("J7 canonical target 越界；拒绝且不 clamp")
    # F_cmd exactly once.  不调用会 silent clip 的 canonical_to_actual。
    seed_vector = np.zeros(HAND_DOF)
    final_vector = np.zeros(HAND_DOF)
    final_vector[index] = canonical_final
    sdk_current = float(apply_f_cmd_once_deg(hardware_map, seed_vector)[index])
    sdk_final = float(apply_f_cmd_once_deg(hardware_map, final_vector)[index])
    sdk_delta = sdk_final - sdk_current
    sdk_low, sdk_high = map(float, vendor_ranges[index])
    if not sdk_low <= sdk_current <= sdk_high or not sdk_low <= sdk_final <= sdk_high:
        raise ActiveMicroMotionAbort("J7 SDK target 越界；拒绝且不 clamp")
    return {
        "canonical_joint": "middle_j1",
        "canonical_current_deg": canonical_current,
        "canonical_requested_delta_deg": delta,
        "canonical_final_target_deg": canonical_final,
        "F_cmd_gain": gain,
        "F_cmd_offset_deg": offset_deg,
        "F_cmd_application_count": 1,
        "sdk_channel": "J7",
        "sdk_current_target_deg": sdk_current,
        "sdk_delta_deg": sdk_delta,
        "sdk_final_target_deg": sdk_final,
        "sdk_range_deg": [sdk_low, sdk_high],
        "seed_source": "MODE7_ZERO_OUTPUT_CACHED_COMMAND_TARGET",
        "raw_feedback_used_as_command_seed": False,
        "expected_physical_motion": "middle flexion toward palm",
    }


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(65536), b""):
            digest.update(block)
    return digest.hexdigest()


def _git_head(path: Path) -> str:
    try:
        return subprocess.run(
            ["git", "-C", str(path), "rev-parse", "HEAD"],
            check=True, capture_output=True, text=True,
        ).stdout.strip()
    except (OSError, subprocess.CalledProcessError):
        return "UNAVAILABLE"


def latest_successful_mode7_artifact(
    recording_root: Path = RECORDING_ROOT,
) -> tuple[Path, dict]:
    accepted: list[tuple[Path, dict]] = []
    for path in recording_root.glob("sg100_mode7_probe_*/summary.json"):
        try:
            payload = json.loads(path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError):
            continue
        if (
            payload.get("status") == "COMPLETED_PASS_ANGLE_DOMAIN_SUPPORTED"
            and payload.get("MODE7_JOINT_ANGLE_SCALE_DOMAIN") == "SUPPORTED"
            and payload.get("canonical_valid") is False
        ):
            accepted.append((path, payload))
    if not accepted:
        raise FileNotFoundError("没有可用的成功 MODE7 angle-domain artifact")
    return max(accepted, key=lambda item: item[0].parent.name)


def eligible_first_joint_indices(
    hardware_map: SG100HardwareMap,
    vendor_ranges: list[tuple[float, float]],
    history: dict,
) -> list[int]:
    """只接受 identity F_cmd 候选和简单、已稳定记录的安全首轴。"""

    if len(vendor_ranges) != HAND_DOF:
        raise ValueError("必须有 11 个 SDK command ranges")
    by_id = {
        int(item["joint"][1:]): item
        for item in history.get("per_joint", [])
        if isinstance(item.get("joint"), str) and item["joint"].startswith("J")
    }
    result = []
    for joint_id in PREFERRED_FIRST_JOINT_IDS:
        if joint_id in UNRESOLVED_FIRST_JOINT_IDS:
            continue
        index = joint_id - 1
        command_low, command_high = map(float, vendor_ranges[index])
        canonical_low, canonical_high = np.rad2deg(
            [hardware_map.lower_rad[index], hardware_map.upper_rad[index]]
        )
        record = by_id.get(joint_id)
        simple_identity = (
            float(hardware_map.gain[index]) == 1.0
            and float(hardware_map.offset_rad[index]) == 0.0
            and abs(canonical_low - command_low) <= 0.5
            and abs(canonical_high - command_high) <= 0.5
        )
        stable_history = bool(
            record
            and record.get("multi_turn_present") is False
            and record.get("sdk_command_range_compatible") is True
            and np.isfinite(float(record.get("mode7_median_deg")))
        )
        if simple_identity and stable_history:
            result.append(index)
    return result


def select_first_test_joint(
    hardware_map: SG100HardwareMap,
    vendor_ranges: list[tuple[float, float]],
    history: dict,
    current_positions_deg: Iterable[float],
    *,
    requested_joint_id: int | None = None,
    history_artifact: str = "runtime",
) -> FirstJointChoice:
    current = np.asarray(tuple(current_positions_deg), dtype=np.float64)
    if current.shape != (HAND_DOF,) or not np.all(np.isfinite(current)):
        raise ActiveMicroMotionAbort("当前 MODE7 feedback 不是 finite 11D")
    eligible = eligible_first_joint_indices(hardware_map, vendor_ranges, history)
    if requested_joint_id is not None:
        requested = int(requested_joint_id)
        if requested in UNRESOLVED_FIRST_JOINT_IDS or requested - 1 not in eligible:
            raise ActiveMicroMotionAbort(
                f"J{requested} 不满足首个主动测试轴的 identity/safety 条件"
            )
        eligible = [requested - 1]
    if not eligible:
        raise ActiveMicroMotionAbort("没有满足首轴安全条件的候选关节")

    choices = []
    for index in eligible:
        low, high = map(float, vendor_ranges[index])
        value = float(current[index])
        if not low <= value <= high:
            continue
        negative = value - low
        positive = high - value
        minimum = min(negative, positive)
        if minimum < LIMIT_MARGIN_DEG:
            continue
        choices.append(FirstJointChoice(
            joint_index=index,
            joint_id=index + 1,
            canonical_name=hardware_map.software_joint_order[index],
            current_deg=value,
            lower_deg=low,
            upper_deg=high,
            negative_headroom_deg=negative,
            positive_headroom_deg=positive,
            minimum_headroom_deg=minimum,
            history_artifact=history_artifact,
        ))
    if not choices:
        raise ActiveMicroMotionAbort(
            f"候选轴当前值不在 SDK range 内或距边界不足 {LIMIT_MARGIN_DEG} deg"
        )
    # 自动选择最差方向余量最大的轴；joint id 仅作确定性 tie-break。
    return max(choices, key=lambda item: (item.minimum_headroom_deg, -item.joint_id))


def select_j7_command_seed(
    hardware_map: SG100HardwareMap,
    vendor_ranges: list[tuple[float, float]],
    *, history_artifact: str,
) -> FirstJointChoice:
    """J7 seed 是已知零输出 command target=0，不读取 raw F_fb。"""

    index = hardware_map.software_index_of["middle_j1"]
    low, high = map(float, vendor_ranges[index])
    seed = 0.0
    if not low <= seed <= high:
        raise ActiveMicroMotionAbort("J7 已知 command seed=0 不在 SDK range")
    return FirstJointChoice(
        joint_index=index, joint_id=7, canonical_name="middle_j1",
        current_deg=seed, lower_deg=low, upper_deg=high,
        negative_headroom_deg=seed - low, positive_headroom_deg=high - seed,
        minimum_headroom_deg=min(seed - low, high - seed),
        history_artifact=history_artifact,
    )


def plan_micro_target(choice: FirstJointChoice, delta_deg: float) -> dict:
    magnitude = validate_delta(delta_deg)
    direction = +1.0 if (
        choice.positive_headroom_deg >= choice.negative_headroom_deg
    ) else -1.0
    target = choice.current_deg + direction * magnitude
    if not (
        choice.lower_deg + LIMIT_MARGIN_DEG
        <= target
        <= choice.upper_deg - LIMIT_MARGIN_DEG
    ):
        raise ActiveMicroMotionAbort("micro target 安全余量不足；拒绝且不 clamp")
    return {
        "current_target_deg": choice.current_deg,
        "lower_limit_deg": choice.lower_deg,
        "upper_limit_deg": choice.upper_deg,
        "positive_headroom_deg": choice.positive_headroom_deg,
        "negative_headroom_deg": choice.negative_headroom_deg,
        "selected_direction": "+" if direction > 0 else "-",
        "expected_physical_motion": PHYSICAL_DIRECTIONS[choice.joint_id][
            "+" if direction > 0 else "-"
        ],
        "signed_delta_deg": direction * magnitude,
        "final_target_deg": target,
    }


def build_single_driven_control(
    joint_index: int, positions_deg: Iterable[float]
) -> dict[str, np.ndarray]:
    positions = np.asarray(tuple(positions_deg), dtype=np.float64)
    if positions.shape != (HAND_DOF,) or not np.all(np.isfinite(positions)):
        raise ValueError("需要 finite 11D positions")
    if not 0 <= int(joint_index) < HAND_DOF:
        raise ValueError("joint_index 必须在 0..10")
    modes = np.full(HAND_DOF, MODE_JOINT_POSITION, dtype=np.uint8)
    p = np.zeros(HAND_DOF, dtype=np.uint16)
    i = np.zeros(HAND_DOF, dtype=np.uint16)
    d = np.zeros(HAND_DOF, dtype=np.uint16)
    output = np.zeros(HAND_DOF, dtype=np.uint16)
    p[joint_index] = quantize_gain(ACTIVE_KP)
    output[joint_index] = quantize_gain(ACTIVE_OUTPUT_LIMIT)
    producing = np.flatnonzero((p != 0) | (i != 0) | (d != 0) | (output != 0))
    if producing.tolist() != [joint_index]:
        raise ActiveMicroMotionAbort("position-producing joints 必须恰好为 1")
    return {"positions": positions, "modes": modes, "p": p, "i": i,
            "d": d, "output_limit": output}


def _fresh_post(frames: Iterable[ActiveFrame], command_ns: int) -> list[ActiveFrame]:
    return [frame for frame in frames if frame.received_monotonic_ns > command_ns]


def analyze_motion(
    baseline_frames: list[ActiveFrame],
    post_frames: list[ActiveFrame],
    *,
    command_ns: int,
    expected_joint_index: int,
    commanded_delta_deg: float,
    require_feedback_direction: bool = True,
) -> dict:
    post = _fresh_post(post_frames, command_ns)
    if not baseline_frames or not post:
        raise ActiveMicroMotionAbort("没有 fresh pre/post-command samples")
    baseline = np.median(
        np.asarray([frame.positions_deg for frame in baseline_frames]), axis=0
    )
    values = np.asarray([frame.positions_deg for frame in post])
    if not np.all(np.isfinite(values)):
        raise ActiveMicroMotionAbort("non-finite feedback")
    if any(frame.modes != (MODE_JOINT_POSITION,) * HAND_DOF for frame in post):
        raise ActiveMicroMotionAbort("unexpected mode / MODE7 loss")
    deltas = values - baseline
    settled = np.median(deltas[-min(5, len(deltas)):], axis=0)
    peak_by_joint = np.max(np.abs(deltas), axis=0)
    order = np.argsort(np.abs(settled))[::-1]
    dominant, second = int(order[0]), int(order[1])
    second_value = float(abs(settled[second]))
    ratio = float(abs(settled[dominant]) / max(second_value, 1e-9))
    detection = next((
        frame.received_monotonic_ns for frame, delta in zip(post, deltas)
        if abs(float(delta[expected_joint_index])) >= MOVEMENT_DETECTION_DEG
    ), None)
    observed = float(settled[expected_joint_index])
    target_steps = np.diff(deltas[:, expected_joint_index])
    meaningful_signs = np.sign(target_steps[
        np.abs(target_steps) >= OSCILLATION_STEP_NOISE_DEG
    ])
    reversals = int(np.count_nonzero(
        meaningful_signs[1:] != meaningful_signs[:-1]
    )) if len(meaningful_signs) > 1 else 0
    expected_sign = int(np.sign(commanded_delta_deg))
    observed_sign = int(np.sign(observed)) if abs(observed) >= MOVEMENT_DETECTION_DEG else 0
    verdict = "PASS"
    reason = None
    if float(peak_by_joint[expected_joint_index]) > HARD_EXCURSION_DEG:
        verdict, reason = "FAIL", "EXCESSIVE_TARGET_JOINT_EXCURSION"
    elif float(np.max(np.delete(peak_by_joint, expected_joint_index))) > (
        OTHER_JOINT_HARD_EXCURSION_DEG
    ):
        verdict, reason = "FAIL", "UNEXPECTED_OTHER_JOINT_EXCURSION"
    elif reversals > MAX_OSCILLATION_REVERSALS:
        verdict, reason = "FAIL", "OSCILLATION_DETECTED"
    elif dominant != expected_joint_index:
        verdict, reason = "FAIL", "WRONG_DOMINANT_JOINT"
    elif ratio < DOMINANCE_RATIO_MIN:
        verdict, reason = "FAIL", "AMBIGUOUS_DOMINANCE"
    elif observed_sign == 0:
        verdict, reason = "PENDING", "MOVEMENT_BELOW_DETECTION_THRESHOLD"
    elif require_feedback_direction and observed_sign != expected_sign:
        verdict, reason = "FAIL", "WRONG_FEEDBACK_DIRECTION"
    return {
        "verdict": verdict,
        "reason": reason,
        "baseline_deg": baseline.tolist(),
        "settled_delta_deg": settled.tolist(),
        "peak_abs_excursion_deg": peak_by_joint.tolist(),
        "dominant_feedback_joint": f"J{dominant + 1}",
        "second_largest_joint": f"J{second + 1}",
        "dominance_ratio": ratio,
        "oscillation_direction_reversals": reversals,
        "observed_target_delta_deg": observed,
        "peak_feedback_deg": float(values[
            int(np.argmax(np.abs(deltas[:, expected_joint_index]))),
            expected_joint_index,
        ]),
        "settled_feedback_deg": float(baseline[expected_joint_index] + observed),
        "commanded_direction": "+" if expected_sign > 0 else "-",
        "observed_feedback_direction": (
            "+" if observed_sign > 0 else "-" if observed_sign < 0 else "NONE"
        ),
        "first_fresh_rx_ns": post[0].received_monotonic_ns,
        "first_movement_detection_ns": detection,
        "fresh_post_command_samples": len(post),
        "feedback_direction_required": bool(require_feedback_direction),
    }


def analyze_hold(
    before: list[ActiveFrame], after: list[ActiveFrame], *, command_ns: int,
    joint_index: int,
) -> dict:
    post = _fresh_post(after, command_ns)
    if not before or not post:
        raise ActiveMicroMotionAbort("active hold 缺少 fresh samples")
    baseline = np.median(np.asarray([f.positions_deg for f in before]), axis=0)
    values = np.asarray([f.positions_deg for f in post])
    if not np.all(np.isfinite(values)):
        raise ActiveMicroMotionAbort("active hold non-finite feedback")
    if any(f.modes != (MODE_JOINT_POSITION,) * HAND_DOF for f in post):
        raise ActiveMicroMotionAbort("active hold unexpected mode")
    delta = values - baseline
    settled = np.median(delta[-min(5, len(delta)):], axis=0)
    peak = np.max(np.abs(delta), axis=0)
    verdict = "PASS"
    reason = None
    if float(peak[joint_index]) > HOLD_HARD_EXCURSION_DEG:
        verdict, reason = "FAIL", "CURRENT_POSITION_SEED_REJECTED"
    elif float(np.max(np.delete(peak, joint_index))) > OTHER_JOINT_HARD_EXCURSION_DEG:
        verdict, reason = "FAIL", "UNEXPECTED_OTHER_JOINT_EXCURSION"
    return {
        "verdict": verdict,
        "reason": reason,
        "hold_error_deg": float(settled[joint_index]),
        "peak_excursion_deg": float(peak[joint_index]),
        "settled_excursion_deg": float(settled[joint_index]),
        "all_joint_peak_abs_excursion_deg": peak.tolist(),
        "fresh_post_command_samples": len(post),
    }


def _capture(
    driver, duration_s: float, phase: str, sink: list[ActiveFrame], *,
    command_index: int, target_deg: float | None,
    min_frames: int = 1,
    require_all_mode7: bool = False,
    guard_baseline_deg: np.ndarray | None = None,
    guard_joint_index: int | None = None,
    guard_target_excursion_deg: float | None = None,
) -> list[ActiveFrame]:
    deadline = time.monotonic() + float(duration_s)
    last_sequence = sink[-1].sequence if sink else -1
    last_fresh_at = time.monotonic()
    previous_positions: np.ndarray | None = None
    captured = []
    while time.monotonic() < deadline or len(captured) < min_frames:
        if driver.fault_active:
            raise ActiveMicroMotionAbort("fault")
        feedback = driver.read_feedback()
        if feedback is not None and feedback.sequence != last_sequence:
            last_sequence = feedback.sequence
            last_fresh_at = time.monotonic()
            positions = feedback.sdk_callback_positions_rad
            if positions is None:
                raise ActiveMicroMotionAbort("missing raw MODE7 callback")
            frame = ActiveFrame(
                phase=phase,
                sequence=int(feedback.sequence),
                received_monotonic_ns=int(feedback.received_monotonic_ns),
                modes=tuple(int(v) for v in (feedback.joint_modes or ())),
                positions_deg=tuple(float(v) for v in np.rad2deg(positions)),
                requested_active_target_deg=target_deg,
                command_index=command_index,
            )
            if len(frame.modes) != HAND_DOF or len(frame.positions_deg) != HAND_DOF:
                raise ActiveMicroMotionAbort("malformed feedback")
            if not np.all(np.isfinite(frame.positions_deg)):
                raise ActiveMicroMotionAbort("non-finite feedback")
            if require_all_mode7 and frame.modes != (MODE_JOINT_POSITION,) * HAND_DOF:
                raise ActiveMicroMotionAbort("unexpected mode / MODE7 loss")
            if guard_baseline_deg is not None:
                if guard_joint_index is None or guard_target_excursion_deg is None:
                    raise ValueError("excursion guard configuration incomplete")
                excursion = np.abs(
                    np.asarray(frame.positions_deg) - guard_baseline_deg
                )
                if excursion[guard_joint_index] > guard_target_excursion_deg:
                    raise ActiveMicroMotionAbort("excessive target-joint excursion")
                if np.max(np.delete(excursion, guard_joint_index)) > (
                    OTHER_JOINT_HARD_EXCURSION_DEG
                ):
                    raise ActiveMicroMotionAbort("unexpected other-joint excursion")
                if previous_positions is not None and abs(
                    frame.positions_deg[guard_joint_index]
                    - previous_positions[guard_joint_index]
                ) > MAX_SINGLE_FRAME_STEP_DEG:
                    raise ActiveMicroMotionAbort("rapid target-joint motion")
            previous_positions = np.asarray(frame.positions_deg)
            sink.append(frame)
            captured.append(frame)
        elif time.monotonic() - last_fresh_at > TRANSPORT_STALL_S:
            raise ActiveMicroMotionAbort("transport stall")
        time.sleep(POLL_PERIOD_S)
    return captured


def _write_artifacts(path: Path, metadata: dict, frames: list[ActiveFrame],
                     summary: dict) -> None:
    path.mkdir(parents=True, exist_ok=False)
    (path / "metadata.json").write_text(
        json.dumps(metadata, indent=2, ensure_ascii=False) + "\n", encoding="utf-8"
    )
    fields = ["timestamp_ns", "phase", "sequence", "command_index",
              "requested_active_target_deg"]
    fields += [f"J{j}_mode" for j in range(1, 12)]
    fields += [f"J{j}_feedback_deg" for j in range(1, 12)]
    with (path / "samples.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for frame in frames:
            row = {
                "timestamp_ns": frame.received_monotonic_ns,
                "phase": frame.phase,
                "sequence": frame.sequence,
                "command_index": frame.command_index,
                "requested_active_target_deg": frame.requested_active_target_deg,
            }
            row.update({f"J{i + 1}_mode": value for i, value in enumerate(frame.modes)})
            row.update({f"J{i + 1}_feedback_deg": value
                        for i, value in enumerate(frame.positions_deg)})
            writer.writerow(row)
    (path / "summary.json").write_text(
        json.dumps(summary, indent=2, ensure_ascii=False) + "\n", encoding="utf-8"
    )


def audit_active_artifact(path: Path, *, observed_crc_error_count: int = 0) -> dict:
    """只读复核 active artifact；phase 间人工等待不当作 transport gap。"""

    path = Path(path)
    metadata = json.loads((path / "metadata.json").read_text(encoding="utf-8"))
    summary = json.loads((path / "summary.json").read_text(encoding="utf-8"))
    with (path / "samples.csv").open(encoding="utf-8", newline="") as stream:
        rows = list(csv.DictReader(stream))
    if not rows:
        raise ActiveMicroMotionAbort("artifact samples.csv 为空")
    joint_id = int(str(metadata["selected_joint"]).removeprefix("J"))
    phases: dict[str, list[dict]] = {}
    for row in rows:
        phases.setdefault(row["phase"], []).append(row)
    health = {}
    all_nonfinite = False
    for phase, phase_rows in phases.items():
        seq = [int(row["sequence"]) for row in phase_rows]
        timestamps = [int(row["timestamp_ns"]) for row in phase_rows]
        gaps = [b - a for a, b in zip(seq, seq[1:]) if b - a != 1]
        time_gaps_ns = [b - a for a, b in zip(timestamps, timestamps[1:])]
        positions = np.asarray([
            [float(row[f"J{i}_feedback_deg"]) for i in range(1, 12)]
            for row in phase_rows
        ])
        all_nonfinite = all_nonfinite or not bool(np.all(np.isfinite(positions)))
        health[phase] = {
            "samples": len(phase_rows),
            "sequence_gaps": gaps,
            "max_timestamp_gap_ms": (
                max(time_gaps_ns, default=0) / 1e6
            ),
        }
    active_rows = phases.get("active_hold", []) + phases.get("micro_motion", [])
    all_active_mode7 = bool(active_rows) and all(
        all(int(row[f"J{i}_mode"]) == MODE_JOINT_POSITION for i in range(1, 12))
        for row in active_rows
    )
    no_window_gap = all(
        not item["sequence_gaps"] and item["max_timestamp_gap_ms"] < TRANSPORT_STALL_S * 1000
        for item in health.values()
    )
    command_kinds = [item.get("kind") for item in summary.get("command_log", [])]
    required_commands = [
        "realtime_mode_null_seed", "mode7_passive", "active_hold",
        "micro_motion", "cleanup_mode_null",
    ]
    evidence_pass = all((
        summary.get("active_hold_verdict") == "PASS",
        summary.get("micro_motion_verdict") == "PASS",
        summary.get("operator_hold_visible_motion") is False,
        summary.get("operator_visual_confirmation") is True,
        summary.get("fault") == 0,
        summary.get("cleanup_status") == "PASS",
        summary.get("exactly_one_micro_delta_command") is True,
        summary.get("set_hand_control_api_calls") == 5,
        command_kinds == required_commands,
        all_active_mode7,
        not all_nonfinite,
        no_window_gap,
        summary.get("dominant_feedback_joint") == f"J{joint_id}",
    ))
    crc_count = int(observed_crc_error_count)
    crc_verdict = (
        "CRC_TRANSIENT_RECOVERED"
        if crc_count > 0 and evidence_pass and no_window_gap
        else "NO_CRC_REPORTED" if crc_count == 0
        else "CRC_REVIEW_REQUIRED"
    )
    return {
        "verdict": "PASS_REAL_HARDWARE" if evidence_pass else "FAIL_EVIDENCE_AUDIT",
        "joint": f"J{joint_id}",
        "active_hold": summary.get("active_hold"),
        "micro_motion": summary.get("micro_motion"),
        "fault": summary.get("fault"),
        "cleanup_status": summary.get("cleanup_status"),
        "command_kinds": command_kinds,
        "set_hand_control_api_calls": summary.get("set_hand_control_api_calls"),
        "all_active_rx_mode7": all_active_mode7,
        "nonfinite_feedback": all_nonfinite,
        "transport_stall": not no_window_gap,
        "phase_health": health,
        "crc_error_count": crc_count,
        "crc_classification": crc_verdict,
        "canonical_valid": False,
        "dataset": "QUARANTINED",
    }


def _metadata(choice: FirstJointChoice, delta_deg: float, sdk_dir: Path) -> dict:
    map_path = ROOT / "sg100_teleop/config/sg100_hardware_map.yaml"
    semantics_path = ROOT / "sg100_teleop/config/sg100_joint_semantics.yaml"
    return {
        "sdk_commit": _git_head(sdk_dir),
        "runtime_map_hash_sha256": _sha256(sdk_dir / "joint_angle_map.json"),
        "canonical_semantics_hash_sha256": _sha256(semantics_path),
        "outbound_hardware_map_hash_sha256": _sha256(map_path),
        "selected_joint": f"J{choice.joint_id}",
        "canonical_name": choice.canonical_name,
        "sdk_command_range_deg": [choice.lower_deg, choice.upper_deg],
        "canonical_range_deg": [
            float(np.rad2deg(SG100HardwareMap().lower_rad[choice.joint_index])),
            float(np.rad2deg(SG100HardwareMap().upper_rad[choice.joint_index])),
        ],
        "selection_feedback_deg": choice.current_deg,
        "negative_headroom_deg": choice.negative_headroom_deg,
        "positive_headroom_deg": choice.positive_headroom_deg,
        "F_cmd_candidate": (
            "J7_SIGN_FLIP_SOFTWARE_READY_HARDWARE_PENDING"
            if choice.joint_id == 7
            else "IDENTITY_CANDIDATE_NOT_PRODUCTION_VALIDATED"
        ),
        "F_fb_status": "DELTA_MAPPING_PENDING",
        "feedback_map_valid": False,
        "canonical_valid": False,
        "dataset": "QUARANTINED",
        "pid": {"p": ACTIVE_KP, "i": ACTIVE_KI, "d": ACTIVE_KD},
        "output_limit": ACTIVE_OUTPUT_LIMIT,
        "gain_evidence": GAIN_EVIDENCE,
        "requested_delta_deg": delta_deg,
        "hard_max_delta_deg": HARD_MAX_DELTA_DEG,
        "hard_excursion_deg": HARD_EXCURSION_DEG,
        "max_single_frame_step_deg": MAX_SINGLE_FRAME_STEP_DEG,
        "max_oscillation_direction_reversals": MAX_OSCILLATION_REVERSALS,
        "native_path": (
            "Hand_Control_t -> sg100_set_hand_control -> "
            "SG100Device::SetHandControl"
        ),
        "no_map_auto_update": True,
    }


def _dry_run(args) -> int:
    hardware_map = SG100HardwareMap()
    ranges = load_vendor_user_ranges(Path(args.sdk_dir))
    if ranges is None:
        raise RuntimeError("SDK joint_angle_map.json ranges unavailable")
    artifact, history = latest_successful_mode7_artifact(args.recording_root)
    current = np.zeros(HAND_DOF)
    for item in history["per_joint"]:
        current[int(item["joint"][1:]) - 1] = float(item["mode7_median_deg"])
    if args.joint == 7:
        choice = select_j7_command_seed(
            hardware_map, ranges, history_artifact=str(artifact.parent)
        )
        canonical_delta = resolve_canonical_delta_deg(
            7, args.delta_deg, args.canonical_delta_deg
        )
        plan = plan_j7_sign_flip(hardware_map, ranges, canonical_delta)
    else:
        choice = select_first_test_joint(
            hardware_map, ranges, history, current,
            requested_joint_id=args.joint,
            history_artifact=str(artifact.parent),
        )
        canonical_delta = resolve_canonical_delta_deg(
            choice.joint_id, args.delta_deg, args.canonical_delta_deg
        )
        if canonical_delta <= 0.0:
            raise ActiveMicroMotionAbort("identity 轴 dry-run 只接受 canonical 正向 delta")
        plan = plan_micro_target(choice, canonical_delta)
    control = build_single_driven_control(choice.joint_index, current)
    print("DRY RUN — 未构造真机 driver、未打开串口，hardware execution: NO")
    print(f"FIRST_TEST_JOINT: J{choice.joint_id} / {choice.canonical_name}")
    print(f"选择证据: {choice.history_artifact}")
    if choice.joint_id == 7:
        print("J7 raw MODE7 feedback: 仅观测，不作为 command seed")
        print("SDK command seed: 0.000000 deg (zero-output cached target)")
    else:
        print(f"历史 MODE7 feedback: {choice.current_deg:.6f} deg")
    print(f"command range: [{choice.lower_deg:.1f}, {choice.upper_deg:.1f}] deg")
    for key, value in plan.items():
        print(f"{key}: {value}")
    print("position-producing joints: 1")
    print(f"target: J{choice.joint_id}")
    print(f"MODE7 zero-output joints: {int(np.count_nonzero(control['output_limit'] == 0))}")
    print(f"PID/output_limit: {ACTIVE_KP}/{ACTIVE_KI}/{ACTIVE_KD}/{ACTIVE_OUTPUT_LIMIT}")
    print(f"excursion hard guard: {HARD_EXCURSION_DEG} deg")
    print("F_cmd/F_fb: 分离；canonical_valid=False；dataset=QUARANTINED")
    return 0


def run_hardware(args, confirm_fn: Callable[[str], str] = input) -> int:
    """真机流程；仅供用户未来显式执行，Agent 本轮不得调用。"""

    requested_joint = args.joint
    nominal_joint = 7 if requested_joint == 7 else (requested_joint or 6)
    canonical_delta = resolve_canonical_delta_deg(
        nominal_joint, args.delta_deg, getattr(args, "canonical_delta_deg", None)
    )
    delta = abs(canonical_delta)
    hardware_map = SG100HardwareMap()
    history_path, history = latest_successful_mode7_artifact(args.recording_root)
    driver = SG100SDKDriver(
        args.port, baudrate=args.baudrate, hardware_map=hardware_map,
        sdk_dir=args.sdk_dir,
    )
    ranges = driver.vendor_user_ranges
    if ranges is None:
        print("ABORT: SDK command ranges unavailable")
        return 2
    frames: list[ActiveFrame] = []
    command_log: list[dict] = []
    summary = {
        "status": "NOT_STARTED", "active_hold_verdict": "NOT_RUN",
        "micro_motion_verdict": "NOT_RUN", "canonical_valid": False,
        "dataset": "QUARANTINED", "cleanup_status": "NOT_REQUIRED",
        "command_log": command_log, "fault": 0,
    }
    historical_positions = np.zeros(HAND_DOF)
    for item in history["per_joint"]:
        historical_positions[int(item["joint"][1:]) - 1] = float(
            item["mode7_median_deg"]
        )
    preliminary_choice = (
        select_j7_command_seed(
            hardware_map, ranges, history_artifact=str(history_path.parent)
        )
        if requested_joint == 7
        else select_first_test_joint(
            hardware_map, ranges, history, historical_positions,
            requested_joint_id=requested_joint,
            history_artifact=str(history_path.parent),
        )
    )
    stamp = time.strftime("%Y%m%d_%H%M%S")
    artifact_path: Path | None = Path(args.recording_root) / (
        f"sg100_mode7_active_{stamp}_J{preliminary_choice.joint_id}"
    )
    connected = False
    cleanup_done = False
    sequence_gate = ActiveSequenceGate()
    metadata: dict = _metadata(preliminary_choice, delta, Path(args.sdk_dir))
    initial_calls = 0
    initial_crc_errors = 0
    try:
        driver.connect()
        connected = True
        initial_calls = driver.diagnostics["hand_control_calls"]
        initial_crc_errors = int(driver.diagnostics.get("crc_error_count", 0))
        driver.start()
        driver.enter_realtime()
        command_log.append({
            "command_index": 0, "kind": "realtime_mode_null_seed",
            "timestamp_ns": time.monotonic_ns(),
            "timestamp_semantics": "after enter_realtime returned",
            "target_deg": [0.0] * HAND_DOF, "modes": [MODE_NULL] * HAND_DOF,
            "p": 0.0, "i": 0.0, "d": 0.0, "output_limit": 0.0,
            "set_hand_control_call_count": driver.diagnostics["hand_control_calls"],
        })
        passive = driver.send_mode7_feedback_probe()
        if passive is None:
            raise ActiveMicroMotionAbort("initial MODE7 zero-output rejected")
        command_log.append({
            "command_index": 1, "kind": "mode7_passive", "timestamp_ns": passive.sent_monotonic_ns,
            "target_deg": [0.0] * HAND_DOF, "modes": [7] * HAND_DOF,
            "p": 0.0, "i": 0.0, "d": 0.0, "output_limit": 0.0,
            "set_hand_control_call_count": driver.diagnostics["hand_control_calls"],
        })
        passive_frames = _capture(
            driver, 0.25, "mode7_passive", frames, command_index=1,
            target_deg=None, min_frames=STABLE_MODE7_FRAMES + BASELINE_FRAMES,
        )
        stable = passive_frames[-STABLE_MODE7_FRAMES:]
        if any(frame.modes != (MODE_JOINT_POSITION,) * HAND_DOF for frame in stable):
            raise ActiveMicroMotionAbort("未达到 5 consecutive fresh all-MODE7")
        if any(
            frame.modes != (MODE_JOINT_POSITION,) * HAND_DOF
            for frame in passive_frames[-BASELINE_FRAMES:]
        ):
            raise ActiveMicroMotionAbort("稳定 MODE7 baseline frames 不足")
        current = np.median(
            np.asarray([frame.positions_deg for frame in passive_frames[-BASELINE_FRAMES:]]), axis=0
        )
        if requested_joint == 7:
            choice = select_j7_command_seed(
                hardware_map, ranges, history_artifact=str(history_path.parent)
            )
            plan = plan_j7_sign_flip(hardware_map, ranges, canonical_delta)
        else:
            choice = select_first_test_joint(
                hardware_map, ranges, history, current,
                requested_joint_id=requested_joint,
                history_artifact=str(history_path.parent),
            )
            if canonical_delta <= 0.0:
                raise ActiveMicroMotionAbort("identity 轴只接受 canonical 正向 delta")
            plan = plan_micro_target(choice, canonical_delta)
        metadata = _metadata(choice, delta, Path(args.sdk_dir))
        metadata.update({
            "canonical_requested_delta_deg": canonical_delta,
            "command_plan": plan,
            "crc_telemetry_available": True,
        })
        artifact_path = Path(args.recording_root) / (
            f"sg100_mode7_active_{stamp}_J{choice.joint_id}"
        )
        print(json.dumps(dataclasses.asdict(choice), indent=2, ensure_ascii=False))
        print(json.dumps(plan, indent=2, ensure_ascii=False))
        print("position-producing joints: 1; MODE7 zero-output joints: 10")
        if choice.joint_id == 7 and confirm_fn(
            "确认 middle_j1 位于 canonical mechanical zero；Type ZERO: "
        ).strip() != "ZERO":
            summary["status"] = "OPERATOR_ABORT_J7_ZERO_ANCHOR"
            return 3
        hold_prompt = (
            "Type HOLD to enable known zero-target active hold: "
            if choice.joint_id == 7
            else "Type HOLD to enable current-position active hold: "
        )
        if confirm_fn(hold_prompt).strip() != "HOLD":
            summary["status"] = "OPERATOR_ABORT_BEFORE_ACTIVE_HOLD"
            return 3

        seed = np.zeros(HAND_DOF, dtype=np.float64)
        # 只有目标轴 position 字段有语义；其余十轴虽为 MODE7，但零输出。
        seed[choice.joint_index] = (
            plan["sdk_current_target_deg"] if choice.joint_id == 7
            else choice.current_deg
        )
        build_single_driven_control(choice.joint_index, seed)
        hold = driver.send_mode7_active_single_joint(choice.joint_index, seed)
        if hold is None:
            raise ActiveMicroMotionAbort("active hold submission rejected")
        command_log.append({
            "command_index": 2, "kind": "active_hold", "timestamp_ns": hold.sent_monotonic_ns,
            "target_deg": list(hold.requested_positions_deg),
            "p": ACTIVE_KP, "i": ACTIVE_KI, "d": ACTIVE_KD,
            "output_limit": ACTIVE_OUTPUT_LIMIT,
            "set_hand_control_call_count": driver.diagnostics["hand_control_calls"],
        })
        hold_frames = _capture(
            driver, HOLD_OBSERVATION_S, "active_hold", frames, command_index=2,
            target_deg=float(seed[choice.joint_index]), min_frames=20,
            require_all_mode7=True,
            guard_baseline_deg=current,
            guard_joint_index=choice.joint_index,
            guard_target_excursion_deg=HOLD_HARD_EXCURSION_DEG,
        )
        hold_result = analyze_hold(
            passive_frames[-BASELINE_FRAMES:], hold_frames,
            command_ns=hold.sent_monotonic_ns, joint_index=choice.joint_index,
        )
        summary["active_hold"] = hold_result
        summary["active_hold_verdict"] = hold_result["verdict"]
        if hold_result["verdict"] != "PASS":
            summary["status"] = "CURRENT_POSITION_SEED_REJECTED"
            raise ActiveMicroMotionAbort("active hold numeric safety gate failed")
        moved = confirm_fn(
            "启用当前位姿主动保持后，机械手是否发生可见运动？[y/N] "
        ).strip().lower()
        if moved in ("y", "yes"):
            summary["operator_hold_visible_motion"] = True
            summary["status"] = "CURRENT_POSITION_SEED_REJECTED"
            raise ActiveMicroMotionAbort("operator observed hold movement")
        if moved not in ("", "n", "no"):
            raise ActiveMicroMotionAbort("active hold operator answer invalid")
        summary["operator_hold_visible_motion"] = False
        sequence_gate.accept_hold(hold_result["verdict"], False)
        try:
            sequence_gate.confirm_move(
                confirm_fn("ACTIVE HOLD PASS. Type MOVE to continue: ")
            )
        except ActiveMicroMotionAbort:
            summary["status"] = "OPERATOR_ABORT_BEFORE_MICRO_MOTION"
            return 3

        target = seed.copy()
        target[choice.joint_index] = (
            plan["sdk_final_target_deg"] if choice.joint_id == 7
            else plan["final_target_deg"]
        )
        # 明确拒绝越界；此路径没有 clip/clamp/saturate。
        if not ranges[choice.joint_index][0] <= target[choice.joint_index] <= ranges[choice.joint_index][1]:
            raise ActiveMicroMotionAbort("micro target outside range")
        build_single_driven_control(choice.joint_index, target)
        sequence_gate.record_micro_command()
        motion = driver.send_mode7_active_single_joint(choice.joint_index, target)
        if motion is None:
            raise ActiveMicroMotionAbort("micro command rejected")
        command_log.append({
            "command_index": 3, "kind": "micro_motion", "timestamp_ns": motion.sent_monotonic_ns,
            "target_deg": list(motion.requested_positions_deg),
            "p": ACTIVE_KP, "i": ACTIVE_KI, "d": ACTIVE_KD,
            "output_limit": ACTIVE_OUTPUT_LIMIT,
            "set_hand_control_call_count": driver.diagnostics["hand_control_calls"],
        })
        motion_frames = _capture(
            driver, MICRO_OBSERVATION_S, "micro_motion", frames, command_index=3,
            target_deg=float(target[choice.joint_index]), min_frames=30,
            require_all_mode7=True,
            guard_baseline_deg=np.median(
                np.asarray([frame.positions_deg for frame in hold_frames[-BASELINE_FRAMES:]]),
                axis=0,
            ),
            guard_joint_index=choice.joint_index,
            guard_target_excursion_deg=HARD_EXCURSION_DEG,
        )
        motion_result = analyze_motion(
            hold_frames[-BASELINE_FRAMES:], motion_frames,
            command_ns=motion.sent_monotonic_ns,
            expected_joint_index=choice.joint_index,
            commanded_delta_deg=(
                plan["sdk_delta_deg"] if choice.joint_id == 7
                else plan["signed_delta_deg"]
            ),
            # J7 F_fb sign is intentionally unvalidated.  Physical direction
            # is accepted only by the operator visual gate below.
            require_feedback_direction=choice.joint_id != 7,
        )
        summary["micro_motion"] = motion_result
        summary["micro_motion_verdict"] = motion_result["verdict"]
        summary.update({
            "commanded_joint": f"J{choice.joint_id}",
            "command_target_deg": float(target[choice.joint_index]),
            "delta_deg": (
                plan["sdk_delta_deg"] if choice.joint_id == 7
                else plan["signed_delta_deg"]
            ),
            "canonical_delta_deg": canonical_delta,
            "dominant_feedback_joint": motion_result["dominant_feedback_joint"],
            "observed_delta_deg": motion_result["observed_target_delta_deg"],
            "direction": {
                "commanded": motion_result["commanded_direction"],
                "feedback": motion_result["observed_feedback_direction"],
            },
            "peak_excursion_deg": motion_result["peak_abs_excursion_deg"][
                choice.joint_index
            ],
        })
        if motion_result["verdict"] != "PASS":
            summary["status"] = "MICRO_MOTION_FAIL"
            raise ActiveMicroMotionAbort(motion_result["reason"] or "motion pending")

        # 第一次实验不发送反向目标；观测完成立即 MODE_NULL，再询问目视结果。
        cleanup_ok = driver.hold_passive()
        cleanup_done = True
        command_log.append({
            "command_index": 4, "kind": "cleanup_mode_null",
            "timestamp_ns": time.monotonic_ns(),
            "target_deg": [0.0] * HAND_DOF, "modes": [MODE_NULL] * HAND_DOF,
            "p": 0.0, "i": 0.0, "d": 0.0, "output_limit": 0.0,
            "set_hand_control_call_count": driver.diagnostics["hand_control_calls"],
        })
        summary["cleanup_status"] = "PASS" if cleanup_ok else "FAIL"
        if not cleanup_ok:
            raise ActiveMicroMotionAbort("MODE_NULL cleanup rejected")
        try:
            _capture(
                driver, 0.03, "cleanup", frames, command_index=4,
                target_deg=None, min_frames=1,
            )
        except ActiveMicroMotionAbort:
            pass  # cleanup 已下发；保留已有证据并继续断开。
        visual = confirm_fn("目标物理关节是否只发生了轻微、预期方向运动？[y/N] ").strip().lower()
        summary["operator_visual_confirmation"] = visual in ("y", "yes")
        if visual not in ("y", "yes"):
            summary["status"] = "OPERATOR_VISUAL_CONFIRMATION_FAIL"
            return 5
        summary["status"] = "PASS_HARDWARE_EVIDENCE_REVIEW_REQUIRED"
        return 0
    except (KeyboardInterrupt, EOFError) as exc:
        summary["status"] = "KEYBOARD_INTERRUPT" if isinstance(exc, KeyboardInterrupt) else "EOF_ABORT"
        return 130
    except Exception as exc:
        summary.setdefault("errors", []).append(f"{type(exc).__name__}: {exc}")
        if summary["status"] in ("NOT_STARTED", "OPERATOR_ABORT_BEFORE_ACTIVE_HOLD"):
            summary["status"] = "HARD_FAIL"
        return 6
    finally:
        if connected and not cleanup_done:
            try:
                ok = driver.hold_passive()
                cleanup_done = True
                command_log.append({
                    "command_index": len(command_log), "kind": "cleanup_mode_null",
                    "timestamp_ns": time.monotonic_ns(),
                    "target_deg": [0.0] * HAND_DOF, "modes": [MODE_NULL] * HAND_DOF,
                    "p": 0.0, "i": 0.0, "d": 0.0, "output_limit": 0.0,
                    "set_hand_control_call_count": driver.diagnostics["hand_control_calls"],
                })
                summary["cleanup_status"] = "PASS" if ok else "FAIL"
                try:
                    _capture(
                        driver, 0.03, "cleanup", frames,
                        command_index=command_log[-1]["command_index"],
                        target_deg=None, min_frames=1,
                    )
                except Exception:
                    pass
            except Exception as exc:
                summary["cleanup_status"] = f"FAIL: {type(exc).__name__}: {exc}"
        final_counter_delta = 0
        if connected:
            try:
                diagnostics = driver.diagnostics
                summary["fault"] = int(bool(driver.fault_active))
                final_counter_delta = (
                    int(diagnostics["hand_control_calls"]) - initial_calls
                )
                summary["crc_error_count"] = max(
                    0, int(diagnostics.get("crc_error_count", 0))
                    - initial_crc_errors
                )
                summary["crc_error_timestamps_ns"] = list(
                    diagnostics.get("crc_error_timestamps_ns", [])
                )
                summary["crc_classification"] = (
                    "CRC_TRANSIENT_RECOVERED"
                    if summary["crc_error_count"] > 0
                    and summary.get("fault") == 0
                    and summary.get("micro_motion_verdict") == "PASS"
                    else "NO_CRC_REPORTED"
                    if summary["crc_error_count"] == 0
                    else "CRC_REVIEW_REQUIRED"
                )
            except Exception:
                final_counter_delta = len(command_log)
            try:
                driver.disconnect()  # 不调用 stop()，避免隐藏的第二次 MODE_NULL send。
            except Exception as exc:
                summary["disconnect_status"] = f"FAIL: {exc}"
        summary["recorded_command_events"] = len(command_log)
        summary["set_hand_control_api_calls"] = final_counter_delta
        summary["set_hand_control_counter_delta"] = final_counter_delta
        summary["exactly_one_micro_delta_command"] = sum(
            item["kind"] == "micro_motion" for item in command_log
        ) == 1 and sequence_gate.micro_command_count == 1
        summary["automatic_return_motion"] = False
        summary["F_cmd_status"] = "CANDIDATE_EVIDENCE_ONLY_NO_AUTO_UPDATE"
        summary["F_fb_status"] = "DELTA_EVIDENCE_ONLY_NO_AUTO_UPDATE"
        if artifact_path is not None:
            try:
                _write_artifacts(artifact_path, metadata, frames, summary)
                print(f"artifact: {artifact_path}")
            except Exception as exc:
                print(f"ARTIFACT WRITE FAIL: {exc}")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="/dev/ttyUSB0")
    parser.add_argument("--baudrate", type=int, default=DEFAULT_BAUDRATE)
    parser.add_argument("--joint", type=parse_joint_selector, default=None,
                        help="J1..J11；J7 也可写 middle_j1")
    parser.add_argument("--delta-deg", type=float, default=DEFAULT_DELTA_DEG)
    parser.add_argument(
        "--canonical-delta-deg", type=float, default=None,
        help="带符号 canonical delta；J7 首次测试应为 -0.5",
    )
    parser.add_argument(
        "--audit-artifact", type=Path, default=None,
        help="只读审计既有 active artifact，不构造 driver",
    )
    parser.add_argument(
        "--observed-crc-error-count", type=int, default=0,
        help="兼容旧 artifact：录入终端观察到的 CRC 行数量",
    )
    parser.add_argument("--sdk-dir", type=Path, default=DEFAULT_SDK_DIR)
    parser.add_argument("--recording-root", type=Path, default=RECORDING_ROOT)
    parser.add_argument("--execute", action="store_true")
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        if args.audit_artifact is not None:
            print(json.dumps(
                audit_active_artifact(
                    args.audit_artifact,
                    observed_crc_error_count=args.observed_crc_error_count,
                ),
                indent=2, ensure_ascii=False,
            ))
            return 0
        resolve_canonical_delta_deg(
            args.joint or 6, args.delta_deg, args.canonical_delta_deg
        )
        if not args.execute:
            return _dry_run(args)
        return run_hardware(args)
    except (ValueError, ActiveMicroMotionAbort, FileNotFoundError, RuntimeError) as exc:
        print(f"ABORT: {exc}")
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
