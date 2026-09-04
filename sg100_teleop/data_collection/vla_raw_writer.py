"""Bounded media writer 与防覆盖、可验证的原子 episode 保存。"""

from __future__ import annotations

import os
from pathlib import Path
import queue
import shutil
import threading
from typing import Mapping

import numpy as np

from sg100_teleop.camera.realsense_camera import RealSenseFrame


class BoundedMediaWriter:
    def __init__(
        self,
        pending_media_dir: str | Path,
        *,
        queue_size: int = 180,
        jpeg_quality: int = 97,
    ) -> None:
        if queue_size < 1 or not 95 <= jpeg_quality <= 100:
            raise ValueError("queue_size 或 jpeg_quality 非法")
        self.root = Path(pending_media_dir)
        for role in ("head", "right_wrist"):
            (self.root / role / "rgb").mkdir(parents=True, exist_ok=False)
            (self.root / role / "depth").mkdir(parents=True, exist_ok=False)
        self.jpeg_quality = int(jpeg_quality)
        self._queue: queue.Queue[tuple[int, RealSenseFrame, RealSenseFrame]] = (
            queue.Queue(maxsize=int(queue_size))
        )
        self._shutdown = threading.Event()
        self._thread = threading.Thread(
            target=self._loop, name="vla-media-writer", daemon=True
        )
        self.drop_count = 0
        self.error_count = 0
        self.max_backlog = 0
        self.errors: list[str] = []
        self.written_ticks = 0
        self._thread.start()

    @property
    def backlog(self) -> int:
        return self._queue.qsize()

    def enqueue(
        self, index: int, head: RealSenseFrame, wrist: RealSenseFrame
    ) -> bool:
        try:
            self._queue.put_nowait((int(index), head, wrist))
            self.max_backlog = max(self.max_backlog, self._queue.qsize())
            return True
        except queue.Full:
            self.drop_count += 1
            return False

    def _write_frame(self, role: str, index: int, frame: RealSenseFrame) -> None:
        import cv2

        stem = f"{index:06d}"
        rgb_path = self.root / role / "rgb" / f"{stem}.jpg"
        depth_path = self.root / role / "depth" / f"{stem}.png"
        rgb = np.asarray(frame.color_rgb)
        depth = np.asarray(frame.depth_uint16)
        if rgb.dtype != np.uint8 or rgb.ndim != 3 or rgb.shape[2] != 3:
            raise ValueError(f"{role} RGB 必须是 uint8 HxWx3")
        if depth.dtype != np.uint16 or depth.ndim != 2:
            raise ValueError(f"{role} depth 必须是 uint16 HxW")
        # Capture domain is explicit RGB8; OpenCV encoder expects BGR.
        bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
        if not cv2.imwrite(
            str(rgb_path), bgr, [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality]
        ):
            raise OSError(f"JPEG 写入失败: {rgb_path}")
        if not cv2.imwrite(str(depth_path), depth):
            raise OSError(f"uint16 PNG 写入失败: {depth_path}")

    def _loop(self) -> None:
        while not self._shutdown.is_set() or not self._queue.empty():
            try:
                item = self._queue.get(timeout=0.05)
            except queue.Empty:
                continue
            index, head, wrist = item
            try:
                self._write_frame("head", index, head)
                self._write_frame("right_wrist", index, wrist)
                self.written_ticks += 1
            except Exception as exc:
                self.error_count += 1
                self.errors.append(f"tick {index}: {type(exc).__name__}: {exc}")
            finally:
                self._queue.task_done()

    def close(self) -> None:
        self._queue.join()
        self._shutdown.set()
        self._thread.join(timeout=5.0)
        if self._thread.is_alive():
            raise RuntimeError("media writer 无法在 5 秒内结束")


def discard_pending(path: str | Path) -> None:
    pending = Path(path)
    if pending.exists():
        shutil.rmtree(pending)


def save_episode_atomic(
    output_dir: str | Path,
    episode_index: int,
    arrays: Mapping[str, np.ndarray],
    pending_media_dir: str | Path,
) -> tuple[Path, Path]:
    """先验证 NPZ 临时文件，再让 media 和 NPZ 以 rename 落盘。

    NPZ 是 source of truth，因此最后 rename；crash 最多留下可识别的孤立 media，
    不会出现指向半写 media 的永久 NPZ，也绝不会覆盖已有 episode。
    """
    root = Path(output_dir)
    root.mkdir(parents=True, exist_ok=True)
    stem = f"episode_{int(episode_index):06d}"
    final_npz = root / f"{stem}.npz"
    temp_npz = root / f"{stem}.npz.tmp"
    final_media = root / f"{stem}_media"
    pending = Path(pending_media_dir)
    for target in (final_npz, temp_npz, final_media):
        if target.exists():
            raise FileExistsError(f"拒绝覆盖已有路径: {target}")
    try:
        with temp_npz.open("xb") as handle:
            np.savez_compressed(handle, **arrays)
            handle.flush()
            os.fsync(handle.fileno())
        with np.load(temp_npz, allow_pickle=False) as archive:
            required = {"schema_name", "timestamp_ns", "episode_index"}
            missing = required - set(archive.files)
            if missing:
                raise ValueError(f"NPZ reopen 缺少字段: {sorted(missing)}")
            expected_n = int(np.asarray(arrays["timestamp_ns"]).shape[0])
            if int(archive["timestamp_ns"].shape[0]) != expected_n:
                raise ValueError("NPZ reopen frame count 不一致")
        pending.rename(final_media)
        temp_npz.rename(final_npz)
        return final_npz, final_media
    except Exception:
        if temp_npz.exists():
            temp_npz.unlink()
        raise
