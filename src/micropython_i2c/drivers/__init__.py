"""Concrete drivers, and ``REGISTRY`` mapping config driver names to classes.

Drivers are imported lazily by ``registry()`` so that importing one driver on the board
does not pull every other driver into RAM.
"""

# name -> (module, class). Add new drivers here.
DRIVERS = {
    "TMP102": ("tmp102", "TMP102"),
    "TMAG5273": ("tmag5273", "TMAG5273"),
    "SHTC3": ("shtc3", "SHTC3"),
    "TCA9548A": ("tca9548a", "TCA9548A"),
    "VL53L4CD": ("vl53l4cd", "VL53L4CD"),
    "QwiicRelay": ("qwiic_relay", "QwiicRelay"),
}


def driver_class(name):
    """Import and return the driver class registered as ``name``.

    Raises:
        KeyError: If ``name`` is not in ``DRIVERS``.
    """
    module, cls = DRIVERS[name]
    mod = __import__("micropython_i2c.drivers." + module, None, None, [cls])
    return getattr(mod, cls)


def registry(names=None):
    """Return ``{name: class}`` for ``names`` (default: every registered driver)."""
    return {n: driver_class(n) for n in (DRIVERS if names is None else names)}
