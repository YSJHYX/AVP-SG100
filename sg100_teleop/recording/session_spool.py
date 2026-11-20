"""Private, disk-backed session buffers; final public recording formats stay unchanged.

Pickles here are produced and consumed only within this process. These temporary
files are never accepted as input recordings or loaded from an external source.
"""

from __future__ import annotations

from collections.abc import MutableMapping, MutableSet, Sequence
from pathlib import Path
import pickle
import sqlite3
import tempfile
import threading

import numpy as np


def _key_bytes(key):
    # Preserve equality for integer/numpy-integer keys used by sequence IDs.
    def normalize(value):
        if isinstance(value, np.generic):
            value = value.item()
        if isinstance(value, tuple):
            return tuple(normalize(part) for part in value)
        if isinstance(value, (int, bool)):
            return int(value)
        if isinstance(value, float) and value.is_integer():
            return int(value)
        return value
    return pickle.dumps(normalize(key), protocol=5)


class SessionSpool:
    """Bound SQLite's page cache while retaining every session record on disk."""

    def __init__(self, directory: Path) -> None:
        directory.mkdir(parents=True, exist_ok=True)
        self._temporary = tempfile.TemporaryDirectory(prefix=".session-", dir=directory)
        self._lock = threading.RLock()
        self._db = sqlite3.connect(
            str(Path(self._temporary.name) / "session.sqlite"), check_same_thread=False
        )
        # This is scratch storage replacing RAM, not the durable output file.
        self._db.execute("PRAGMA journal_mode=OFF")
        self._db.execute("PRAGMA synchronous=OFF")
        self._db.execute("PRAGMA cache_size=-1024")
        self._db.execute("PRAGMA temp_store=FILE")
        self._db.execute(
            "CREATE TABLE entries (id INTEGER PRIMARY KEY, bucket TEXT, key BLOB, "
            "value BLOB, UNIQUE(bucket, key))"
        )

    def mapping(self, name):
        return SpoolMapping(self, name)

    def sequence(self, name):
        return SpoolSequence(self, name)

    def set(self, name):
        return SpoolSet(self, name)

    def close(self):
        with self._lock:
            if self._db is not None:
                self._db.close()
                self._db = None
                self._temporary.cleanup()

    def __del__(self):
        try:
            self.close()
        except Exception:
            pass


class SpoolMapping(MutableMapping):
    def __init__(self, store, name):
        self.store, self.name = store, name

    def __getitem__(self, key):
        with self.store._lock:
            row = self.store._db.execute(
                "SELECT value FROM entries WHERE bucket=? AND key=?",
                (self.name, _key_bytes(key)),
            ).fetchone()
        if row is None:
            raise KeyError(key)
        return pickle.loads(row[0])

    def __setitem__(self, key, value):
        with self.store._lock:
            self.store._db.execute(
                "INSERT INTO entries(bucket, key, value) VALUES (?, ?, ?) "
                "ON CONFLICT(bucket, key) DO UPDATE SET value=excluded.value",
                (self.name, _key_bytes(key), pickle.dumps(value, protocol=5)),
            )

    def __delitem__(self, key):
        with self.store._lock:
            cursor = self.store._db.execute(
                "DELETE FROM entries WHERE bucket=? AND key=?",
                (self.name, _key_bytes(key)),
            )
        if not cursor.rowcount:
            raise KeyError(key)

    def __iter__(self):
        with self.store._lock:
            cursor = self.store._db.execute(
                "SELECT key FROM entries WHERE bucket=? ORDER BY id", (self.name,)
            )
            for (key,) in cursor:
                yield pickle.loads(key)

    def __len__(self):
        with self.store._lock:
            return self.store._db.execute(
                "SELECT count(*) FROM entries WHERE bucket=?", (self.name,)
            ).fetchone()[0]


class SpoolSet(MutableSet):
    def __init__(self, store, name):
        self._values = store.mapping(name)

    def __contains__(self, key):
        return key in self._values

    def __iter__(self):
        return iter(self._values)

    def __len__(self):
        return len(self._values)

    def add(self, key):
        self._values[key] = None

    def discard(self, key):
        self._values.pop(key, None)


class SpoolSequence(Sequence):
    def __init__(self, store, name):
        self._values = store.mapping(name)
        self._count = 0

    def append(self, value):
        self._values[self._count] = value
        self._count += 1

    def __len__(self):
        return self._count

    def __getitem__(self, index):
        if isinstance(index, slice):
            return [self[i] for i in range(*index.indices(self._count))]
        if index < 0:
            index += self._count
        if not 0 <= index < self._count:
            raise IndexError(index)
        return self._values[index]

    def __iter__(self):
        for index in range(self._count):
            yield self._values[index]

    def __eq__(self, other):
        if not isinstance(other, Sequence) or len(self) != len(other):
            return False
        return all(a == b for a, b in zip(self, other))


class ArraySpool:
    """Append fixed-width rows and expose them without stacking an entire run."""

    def __init__(self, directory: Path, width: int):
        self._file = tempfile.TemporaryFile(dir=directory)
        self.width = width
        self.count = 0

    def append(self, row):
        row = np.asarray(row, dtype=np.float64)
        if row.shape != (self.width,):
            raise ValueError("session row width changed")
        self._file.write(row.tobytes())
        self.count += 1

    def view(self):
        self._file.flush()
        return np.memmap(
            self._file, mode="r", dtype=np.float64, shape=(self.count, self.width)
        )

    def close(self):
        self._file.close()
