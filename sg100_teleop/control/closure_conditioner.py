"""Signal conditioning for the shared CLAW/PINCH closure input.

The raw input is ``canonical.index_flexion``: the AVP-estimated index
MCP bend angle normalized to [0, 1].  It degrades exactly when the
human grips:

* A clenched fist self-occludes the fingers, so the AVP estimate
  regresses toward a half-open hand — a sustained (multi-frame) drop
  that previously opened the robot under a steady human grip.
* A pinch pose parks the working point mid-scale where the
  normalization slope is steepest, so joint-estimate noise maps 1:1
  into small closure oscillations.

Three composable stages address the two failure modes:

1. Median filter (``median_window``): kills short tracking spikes of
   up to ``(median_window - 1) // 2`` frames.
2. Latch hysteresis (``latch_engage`` / ``latch_release``): once the
   filtered closure reaches ``latch_engage`` the output holds its
   running maximum and ignores sagging estimates; it releases only
   when the filtered closure falls below ``latch_release``, i.e. a
   genuine hand opening.  A human opening always crosses far below
   ``latch_release``; occlusion sag (observed 0.5-0.7) does not.
3. Deadband (``deadband``): in the released tracking region, output
   changes smaller than the deadband are suppressed.

Each stage stays independently disable-able: ``median_window=1``
disables the median, ``latch_engage=1.0`` effectively disables the
latch (only exact saturation engages), ``deadband=0.0`` disables the
deadband.
"""

from __future__ import annotations

from collections import deque

import numpy as np

DEFAULT_MEDIAN_WINDOW = 5
DEFAULT_LATCH_ENGAGE = 0.85
DEFAULT_LATCH_RELEASE = 0.40
DEFAULT_DEADBAND = 0.02


class ClosureSignalConditioner:
    """Condition one scalar closure sample per tracking frame.

    ``update`` must be called exactly once per new tracking frame so
    the median window spans real frames, not control-rate repeats.
    """

    def __init__(
        self,
        *,
        median_window: int = DEFAULT_MEDIAN_WINDOW,
        latch_engage: float = DEFAULT_LATCH_ENGAGE,
        latch_release: float = DEFAULT_LATCH_RELEASE,
        deadband: float = DEFAULT_DEADBAND,
    ) -> None:
        if int(median_window) != median_window or median_window < 1:
            raise ValueError(
                f"median_window must be an integer >= 1, got {median_window!r}"
            )
        latch_engage = float(latch_engage)
        latch_release = float(latch_release)
        deadband = float(deadband)
        if not (
            np.isfinite(latch_engage)
            and np.isfinite(latch_release)
            and np.isfinite(deadband)
        ):
            raise ValueError("latch/deadband parameters must be finite")
        if not 0.0 < latch_release < latch_engage <= 1.0:
            raise ValueError(
                "latch thresholds must satisfy "
                f"0 < latch_release ({latch_release}) < latch_engage "
                f"({latch_engage}) <= 1"
            )
        if not 0.0 <= deadband < 1.0:
            raise ValueError(f"deadband must be in [0, 1), got {deadband}")
        self.median_window = int(median_window)
        self.latch_engage = latch_engage
        self.latch_release = latch_release
        self.deadband = deadband
        self._buffer: deque[float] = deque(maxlen=self.median_window)
        self._latched = False
        self._latch_value = 0.0
        self._last_output: float | None = None

    @property
    def latched(self) -> bool:
        return self._latched

    @property
    def last_output(self) -> float | None:
        return self._last_output

    def reset(self) -> None:
        """Drop all history; the next update passes its raw sample through."""
        self._buffer.clear()
        self._latched = False
        self._latch_value = 0.0
        self._last_output = None

    def update(self, closure: float) -> float:
        closure = float(closure)
        if not np.isfinite(closure):
            raise ValueError("closure must be finite")
        self._buffer.append(closure)
        filtered = float(np.median(self._buffer))

        if self._latched:
            if filtered < self.latch_release:
                # Genuine opening: fall back to tracking the estimate.
                self._latched = False
                candidate = filtered
            else:
                # Occlusion sag: keep the strongest commanded grip.
                self._latch_value = max(self._latch_value, filtered)
                candidate = self._latch_value
        else:
            candidate = filtered
            if candidate >= self.latch_engage:
                self._latched = True
                self._latch_value = candidate

        if (
            self._last_output is not None
            and abs(candidate - self._last_output) <= self.deadband
        ):
            candidate = self._last_output
        self._last_output = candidate
        return candidate
