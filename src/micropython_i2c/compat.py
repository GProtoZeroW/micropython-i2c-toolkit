"""Small shims so the package imports and runs under both MicroPython and CPython.

Board code uses MicroPython's ``time.sleep_ms`` / ``ticks_*`` family and ``_thread``
locks. CPython has neither the ``ticks`` API nor ``sleep_ms``, so host-side tests
would otherwise need monkeypatching. Everything in the package imports timing from
here instead of from ``time`` directly.
"""

import time

try:
    sleep_ms = time.sleep_ms
    sleep_us = time.sleep_us
    ticks_ms = time.ticks_ms
    ticks_add = time.ticks_add
    ticks_diff = time.ticks_diff
except AttributeError:  # CPython

    def sleep_ms(ms):
        """Sleep for ``ms`` milliseconds."""
        time.sleep(ms / 1000)

    def sleep_us(us):
        """Sleep for ``us`` microseconds."""
        time.sleep(us / 1_000_000)

    def ticks_ms():
        """Monotonic millisecond counter (does not wrap on CPython)."""
        return int(time.monotonic() * 1000)

    def ticks_add(ticks, delta):
        """Offset a tick value; plain addition since CPython ticks never wrap."""
        return ticks + delta

    def ticks_diff(a, b):
        """Signed difference ``a - b`` between two tick values."""
        return a - b


try:
    import _thread
except ImportError:  # MicroPython build without threading
    _thread = None


class NullLock:
    """No-op stand-in for a mutex on single-threaded builds.

    Implements just enough of the lock interface (``acquire``, ``release``, context
    manager) that call sites need no conditional logic. Correct whenever there is no
    concurrency to guard against.
    """

    def acquire(self, *args):
        return True

    def release(self):
        pass

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        return False


def default_lock():
    """Return a real ``_thread`` lock where available, else a ``NullLock``.

    The lock guarding a bus is shared by every connection and mux on it, so callers
    normally receive one from the manager rather than calling this themselves.
    """
    return _thread.allocate_lock() if _thread is not None else NullLock()
