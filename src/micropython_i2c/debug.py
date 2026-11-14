"""Bring-up helpers for notebooks: register display and PASS/FAIL checks.

``check`` keeps a running tally so a notebook section can end with ``check_summary()``.
"""

_tally = [0, 0]  # passed, failed


def bin_bytes(value, nbytes=2):
    """Format an int as space-separated 8-bit groups, MSB first."""
    return " ".join("{:08b}".format((value >> (8 * i)) & 0xFF) for i in reversed(range(nbytes)))


def show(value, label="", nbytes=2):
    """Print a value as hex, grouped binary and decimal, padded to ``nbytes``.

    The width is explicit because inferring it from magnitude hides a zero high byte.

    Raises:
        ValueError: If ``value`` does not fit in ``nbytes``.
    """
    if not 0 <= value < (1 << (8 * nbytes)):
        raise ValueError("{:#x} does not fit in {} byte(s)".format(value, nbytes))
    print(
        "{:<16} {:#0{}x}  {}  {}".format(
            label, value, nbytes * 2 + 2, bin_bytes(value, nbytes), value
        )
    )


def show_reg(dev, reg, label="", nbytes=None):
    """Read register ``reg`` from ``dev`` (raw, uncached) and ``show`` it."""
    nbytes = dev.REG_BYTES if nbytes is None else nbytes
    show(dev.reg_read(reg, nbytes), label or "reg {:#04x}".format(reg), nbytes)


def check(label, passed, detail=""):
    """Print a PASS/FAIL line, add it to the tally, and return ``passed``."""
    passed = bool(passed)
    _tally[0 if passed else 1] += 1
    print("  [{}] {:<36} {}".format("PASS" if passed else "FAIL", label, detail))
    return passed


def check_raises(label, exc_type, fn, *args, **kwargs):
    """Check that ``fn(*args, **kwargs)`` raises ``exc_type``. Returns the exception or None."""
    try:
        fn(*args, **kwargs)
    except exc_type as e:
        check(label, True)
        print("         ", e)
        return e
    check(label, False, "it did not raise")
    return None


def info(message):
    """Print an informational line (not counted as pass or fail)."""
    print("  [info] " + message)


def check_reset():
    """Zero the tally."""
    _tally[0] = _tally[1] = 0


def check_counts():
    """``(passed, failed)`` since the last ``check_reset()``, without printing.

    For reading the tally from the host: ``get_ipython().run_line_magic("eval",
    "check_counts()")`` in a ``%%host`` cell.
    """
    return _tally[0], _tally[1]


def check_summary():
    """Print and return ``(passed, failed)`` since the last ``check_reset()``."""
    passed, failed = _tally
    print("{} passed, {} failed".format(passed, failed))
    return passed, failed
