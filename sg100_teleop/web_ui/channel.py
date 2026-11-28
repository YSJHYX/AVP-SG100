"""Low-rate, same-host operator IPC. Disk I/O stays off the control loop.

Each UI command has an expiry and a unique receipt. Status snapshots are lossy
latest-only; robot commands and recorded samples never travel on this channel.
"""
from __future__ import annotations

import json
import os
from pathlib import Path
import queue
import threading
import time
import uuid


def write_json(path: Path, value: dict) -> None:
    temporary = path.with_name(path.name + ".tmp")
    temporary.write_text(json.dumps(value, ensure_ascii=False, allow_nan=False), encoding="utf-8")
    os.replace(temporary, path)


class ControlClient:
    def __init__(self, directory: str | Path):
        self.root = Path(directory)
        self.root.mkdir(parents=True, exist_ok=True, mode=0o700)

    def snapshot(self) -> dict:
        try:
            value = json.loads((self.root / "status.json").read_text())
            value["fresh"] = 0 <= time.monotonic_ns() - value["updated_ns"] < 1_000_000_000
            return value
        except (OSError, ValueError, KeyError):
            return {"phase": "STARTING", "fresh": False}

    def request(self, command: str, *, timeout: float = 5.0, **arguments) -> dict:
        identifier = uuid.uuid4().hex
        request = self.root / f"request_{identifier}.json"
        receipt = self.root / f"receipt_{identifier}.json"
        deadline = time.monotonic() + timeout
        write_json(request, {"id": identifier, "command": command,
                             "arguments": arguments,
                             "issued_ns": time.monotonic_ns(),
                             "expires_ns": time.monotonic_ns() + int(timeout * 1e9)})
        try:
            while time.monotonic() < deadline:
                if receipt.exists():
                    value = json.loads(receipt.read_text())
                    if not value["ok"]:
                        raise RuntimeError(value["error"])
                    return value.get("result", {})
                time.sleep(0.02)
            raise TimeoutError(f"控制端未确认 {command}；请查看设备状态与诊断窗口")
        finally:
            request.unlink(missing_ok=True)
            receipt.unlink(missing_ok=True)


class OperatorBridge:
    def __init__(self, directory: str | Path, *, parent_pid=None):
        self.root = Path(directory)
        self.root.mkdir(parents=True, exist_ok=True, mode=0o700)
        self._normal = queue.Queue(maxsize=32)
        self._urgent = queue.Queue(maxsize=8)
        self._receipts = queue.SimpleQueue()
        self.keys = queue.Queue(maxsize=64)
        self._lock = threading.Lock()
        self._latest = {"phase": "STARTING"}
        self._stop = threading.Event()
        self.recording = False
        self.exit_requested = False
        self._last_stop_ns = 0
        self._parent_pid = parent_pid
        self.last_error = ""
        self._thread = threading.Thread(target=self._io_loop, name="operator-ipc", daemon=True)

    def start(self):
        self._thread.start()

    def update(self, **status):
        with self._lock:
            self._latest = dict(status, updated_ns=time.monotonic_ns())

    def key(self, keycode):
        try:
            self.keys.put_nowait((time.monotonic_ns(), keycode))
        except queue.Full:
            pass

    def drain(self, handler):
        for _ in range(8):
            try:
                item = self._urgent.get_nowait()
            except queue.Empty:
                try:
                    item = self._normal.get_nowait()
                except queue.Empty:
                    break
            if time.monotonic_ns() >= item["expires_ns"]:
                result = {"ok": False, "error": "操作已过期，未执行"}
            elif item["command"] == "arm" and item["issued_ns"] <= self._last_stop_ns:
                result = {"ok": False, "error": "旧的启用请求已被停止操作取消"}
            else:
                try:
                    if item["command"] in ("disarm", "shutdown"):
                        self._last_stop_ns = time.monotonic_ns()
                    result = {"ok": True, "result": handler(item["command"], item["arguments"])}
                    if item["command"] == "disarm":
                        self.last_error = ""
                except Exception as exc:
                    self.last_error = str(exc)
                    result = {"ok": False, "error": str(exc)}
            self._receipts.put((item["id"], result))

    def drain_keys(self, handler):
        for _ in range(16):
            try:
                stamp, key = self.keys.get_nowait()
            except queue.Empty:
                break
            if key in (ord("a"), ord("A")) and stamp <= self._last_stop_ns:
                continue
            try:
                handler(key)
            except Exception as exc:
                self.last_error = str(exc)
                print(f"[OPERATOR] key rejected: {exc}", flush=True)

    def _io_loop(self):
        last_status = 0.0
        while not self._stop.is_set():
            if self._parent_pid is not None and os.getppid() != self._parent_pid:
                # Losing the workstation server must never leave an orphaned
                # active operator session. Main-loop finally stops both devices.
                self.exit_requested = True
            for path in list(self.root.glob("request_*.json"))[:64]:
                try:
                    if path.stat().st_size > 4096:
                        raise ValueError("operator request too large")
                    item = json.loads(path.read_text())
                    if path.stem != "request_" + item["id"] or len(item["id"]) != 32:
                        raise ValueError("invalid request id")
                    if not isinstance(item["arguments"], dict):
                        raise ValueError("invalid arguments")
                    if item["expires_ns"] <= time.monotonic_ns():
                        raise ValueError("操作已过期，未执行")
                    target = self._urgent if item["command"] in ("disarm", "shutdown") else self._normal
                    try:
                        target.put_nowait(item)
                    except queue.Full:
                        self._receipts.put((item["id"], {"ok": False, "error": "控制命令队列已满"}))
                except (OSError, ValueError, KeyError, TypeError) as exc:
                    identifier = path.stem.removeprefix("request_")
                    self._receipts.put((identifier, {"ok": False, "error": str(exc)}))
                finally:
                    path.unlink(missing_ok=True)
            while not self._receipts.empty():
                identifier, result = self._receipts.get()
                write_json(self.root / f"receipt_{identifier}.json", result)
            if time.monotonic() - last_status >= 0.1:
                with self._lock:
                    latest = dict(self._latest)
                write_json(self.root / "status.json", latest)
                last_status = time.monotonic()
                for path in self.root.glob("receipt_*.json"):
                    if time.time() - path.stat().st_mtime > 60:
                        path.unlink(missing_ok=True)
            self._stop.wait(0.02)

    def close(self):
        self._stop.set()
        self._thread.join(timeout=2)
        while not self._receipts.empty():
            identifier, result = self._receipts.get()
            write_json(self.root / f"receipt_{identifier}.json", result)
        with self._lock:
            latest = dict(self._latest)
        write_json(self.root / "status.json", latest)
