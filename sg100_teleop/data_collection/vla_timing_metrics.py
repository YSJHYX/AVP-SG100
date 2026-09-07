"""由永久保留的 timestamp/sequence 派生 timing 指标；派生数组一律不落盘。

设计原则：dataset 只保存不可重建的 native evidence（source timestamp、
sequence），mean/p50/p95/p99/max、skew、dt、native Hz、desired/effective
差异全部在本模块运行时计算。RAW 与 Curated inspector 共用这些纯函数。
"""

from __future__ import annotations

from typing import Mapping

import numpy as np


STAT_KEYS = ("mean_ms", "p50_ms", "p95_ms", "p99_ms", "max_ms")
STAT_KEYS_RAW = ("mean", "p50", "p95", "p99", "max")


def _valid_ns(*sources: np.ndarray) -> list[np.ndarray]:
    arrays = [np.asarray(source, dtype=np.int64) for source in sources]
    size = arrays[0].size
    if any(array.size != size for array in arrays):
        raise ValueError("timing 输入数组长度不一致")
    return arrays


def skew_ns(a_timestamp_ns, b_timestamp_ns) -> np.ndarray:
    """两个 native source timestamp 的绝对偏差；任一 <=0 时为 NaN。"""
    a, b = _valid_ns(a_timestamp_ns, b_timestamp_ns)
    a64 = a.astype(np.float64)
    b64 = b.astype(np.float64)
    result = np.abs(a64 - b64)
    result[(a <= 0) | (b <= 0)] = np.nan
    return result


def delta_ns(later_timestamp_ns, earlier_timestamp_ns) -> np.ndarray:
    """later - earlier；任一 <=0 时为 NaN，可为负（如实保留时序异常）。"""
    later, earlier = _valid_ns(later_timestamp_ns, earlier_timestamp_ns)
    result = later.astype(np.float64) - earlier.astype(np.float64)
    result[(later <= 0) | (earlier <= 0)] = np.nan
    return result


def summary_stats(values) -> dict[str, float]:
    """mean/p50/p95/p99/max（保留原单位，如 rad）；无有效样本时全 NaN。"""
    array = np.asarray(values, dtype=np.float64)
    finite = array[np.isfinite(array)]
    if finite.size == 0:
        return {key: float("nan") for key in STAT_KEYS_RAW}
    return {
        "mean": float(finite.mean()),
        "p50": float(np.percentile(finite, 50)),
        "p95": float(np.percentile(finite, 95)),
        "p99": float(np.percentile(finite, 99)),
        "max": float(finite.max()),
    }


def summary_stats_ns(values) -> dict[str, float]:
    """mean/p50/p95/p99/max（单位 ms）；无有效样本时全 NaN。"""
    array = np.asarray(values, dtype=np.float64)
    finite = array[np.isfinite(array)]
    if finite.size == 0:
        return {key: float("nan") for key in STAT_KEYS}
    return {
        "mean_ms": float(finite.mean() * 1e-6),
        "p50_ms": float(np.percentile(finite, 50) * 1e-6),
        "p95_ms": float(np.percentile(finite, 95) * 1e-6),
        "p99_ms": float(np.percentile(finite, 99) * 1e-6),
        "max_ms": float(finite.max() * 1e-6),
    }


def sequence_rate_hz(sequence, timestamp_ns) -> float:
    """由 publisher 自身 sequence delta / source timestamp 估计 native Hz。

    30 Hz sampler 重复采样同一 native packet 时 sequence 重复；先去重再估计，
    与 telemetry_rate_diagnostics 的口径一致。
    """
    seq = np.asarray(sequence, dtype=np.int64)
    ts = np.asarray(timestamp_ns, dtype=np.int64)
    if seq.size != ts.size:
        raise ValueError("sequence 与 timestamp 长度不一致")
    keep = (seq >= 0) & (ts > 0)
    seq = seq[keep]
    ts = ts[keep]
    if seq.size:
        unique = np.r_[True, seq[1:] != seq[:-1]]
        seq = seq[unique]
        ts = ts[unique]
    delta_seq = np.diff(seq)
    delta_ns = np.diff(ts)
    advancing = (delta_seq > 0) & (delta_ns > 0)
    if not np.any(advancing):
        return float("nan")
    return float(
        delta_seq[advancing].sum() / (delta_ns[advancing].sum() * 1e-9)
    )


def arm_hand_state_skew_ns(
    arm_qpos_source_timestamp_ns, hand_qpos_source_timestamp_ns
) -> np.ndarray:
    """curated：abs(arm_qpos_source - hand_qpos_source)。"""
    return skew_ns(
        arm_qpos_source_timestamp_ns, hand_qpos_source_timestamp_ns
    )


def arm_hand_action_skew_ns(
    arm_qcmd_source_timestamp_ns, hand_qcmd_source_timestamp_ns
) -> np.ndarray:
    """curated：abs(arm_qcmd_source - hand_qcmd_source)。"""
    return skew_ns(
        arm_qcmd_source_timestamp_ns, hand_qcmd_source_timestamp_ns
    )


def state_action_dt_ns(
    state_timestamp_ns, action_timestamp_ns
) -> np.ndarray:
    """curated：action_timestamp - state_timestamp（<=0 表示命令早于 tick）。"""
    return delta_ns(action_timestamp_ns, state_timestamp_ns)


def action_next_state_dt_ns(
    action_timestamp_ns, next_state_timestamp_ns
) -> np.ndarray:
    """curated：next_state_timestamp - action_timestamp。"""
    return delta_ns(next_state_timestamp_ns, action_timestamp_ns)


def desired_effective_delta(
    robot_desired_qcmd_17d_rad, robot_qcmd_17d_rad
) -> tuple[np.ndarray, np.ndarray]:
    """desired - effective 的逐行 L2 与 max abs；不要求两侧同时有限。"""
    desired = np.asarray(robot_desired_qcmd_17d_rad, dtype=np.float64)
    effective = np.asarray(robot_qcmd_17d_rad, dtype=np.float64)
    if desired.shape != effective.shape:
        raise ValueError("desired/effective 形状不一致")
    both = np.all(np.isfinite(desired), axis=1) & np.all(
        np.isfinite(effective), axis=1
    )
    delta = desired - effective
    l2 = np.full(desired.shape[0], np.nan)
    max_abs = np.full(desired.shape[0], np.nan)
    if np.any(both):
        l2[both] = np.linalg.norm(delta[both], axis=1)
        max_abs[both] = np.max(np.abs(delta[both]), axis=1)
    return l2, max_abs


def desired_effective_summary(
    robot_desired_qcmd_17d_rad, robot_qcmd_17d_rad
) -> dict[str, dict[str, float]]:
    """desired/effective 的 L2 与 max abs 汇总（单位 rad）；供 inspector。"""
    l2, max_abs = desired_effective_delta(
        robot_desired_qcmd_17d_rad, robot_qcmd_17d_rad
    )
    return {
        "desired_effective_l2": summary_stats(l2),
        "desired_effective_max_abs": summary_stats(max_abs),
    }


def percentile_summary(mapping: Mapping[str, float]) -> str:
    """把 summary_stats_ns 结果压成单行字符串用于打印。"""
    return " ".join(
        f"{key}={mapping[key]:.3f}" for key in STAT_KEYS
    )
