"""Board-local compatibility hooks; never installed into the system Python."""

from __future__ import annotations

import sys
import types


try:
    import _sqlite3  # noqa: F401
except ImportError:
    # python-can imports its optional SQLite logger eagerly.  The M-Robots
    # Python image omits _sqlite3, while the Piper transport does not use it.
    sqlite_module = types.ModuleType("can.io.sqlite")

    class _UnavailableSqlite:
        def __init__(self, *args: object, **kwargs: object) -> None:
            raise RuntimeError("SQLite CAN logging is unavailable on this board image")

    sqlite_module.SqliteReader = _UnavailableSqlite
    sqlite_module.SqliteWriter = _UnavailableSqlite
    sys.modules["can.io.sqlite"] = sqlite_module
