"""Concrete devices by type, and ``DRIVERS`` mapping config driver names to classes.

    sensors/   TMP102, TMAG5273, SHTC3, VL53L1X, VL53L4CD
    drivers/   output drivers: QwiicRelay (SparkFun relay); MCP23017 to come
    muxes/     TCA9548A

Drivers are imported lazily by ``registry()`` so that importing one driver on the board
does not pull every other driver into RAM.
"""

# name -> (module under devices, class). Add new devices here.
DRIVERS = {
    "TMP102": ("sensors.environmental.tmp102", "TMP102"),
    "TMAG5273": ("sensors.magnetic.tmag5273", "TMAG5273"),
    "SHTC3": ("sensors.environmental.shtc3", "SHTC3"),
    "TCA9548A": ("muxes.tca9548a", "TCA9548A"),
    "VL53L4CD": ("sensors.distance.vl53l4cd", "VL53L4CD"),
    "VL53L1X": ("sensors.distance.vl53l1x", "VL53L1X"),
    "QwiicRelay": ("drivers.qwiic_relay", "QwiicRelay"),
}


def driver_class(name):
    """Import and return the driver class registered as ``name``.

    Raises:
        KeyError: If ``name`` is not in ``DRIVERS``.
    """
    module, cls = DRIVERS[name]
    mod = __import__("micropython_i2c.devices." + module, None, None, [cls])
    return getattr(mod, cls)


def registry(names=None):
    """Return ``{name: class}`` for ``names`` (default: every registered driver)."""
    return {n: driver_class(n) for n in (DRIVERS if names is None else names)}
