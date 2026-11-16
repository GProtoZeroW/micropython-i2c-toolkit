# micropython-i2c-toolkit

A PyVISA-style connection layer over MicroPython's `machine.I2C`, with drivers, a
declarative mux-aware routing table, and bring-up notebooks for the
[micropylab](../micropylab) Jupyter kernel.

You resolve a device once to a `(bus, mux path, address)` handle, and from then on the
handle is the device: addressing, mux routing and per-bus locking are its job, not the
driver's.

```python
from micropython_i2c import I2CConnection, default_lock
from micropython_i2c.boards import bus_open
from micropython_i2c.devices.sensors.environmental.tmp102 import TMP102

bus = bus_open("feathers3", 0)
tmp = TMP102(I2CConnection(bus, 0x48, path=[[0x71, 3]], lock=default_lock()))
print(tmp.temperature())
```

## Layout

```
src/micropython_i2c/      board-side package (copied to /lib/micropython_i2c)
  compat.py               ticks/sleep/lock shims so the code also runs on CPython
  connection.py           I2CConnection, path_select / path_release
  node.py                 I2CNode -> I2CDevice | I2CMux
  bits.py, crc.py         BitTools, CRCMixin
  protocol.py             RegisterI2CBase, CommandI2CBase
  routing.py              RoutingTable, ConfigError
  manager.py              I2CManager, scan_check
  boards.py               board pin maps, bus_open()
  debug.py                show / check helpers for notebooks
  devices/                lazy registry (DRIVERS) over the parts, by type:
    sensors/              by kind:
      environmental/      TMP102 (temperature), SHTC3 (humidity, temperature)
      magnetic/           TMAG5273 (3-axis Hall)
      distance/           VL53L1X, VL53L4CD (optical time of flight)
    drivers/              output drivers: QwiicRelay (SparkFun relay); MCP23017 to come
    muxes/                TCA9548A
tests/                    host-side pytest: codecs, CRC vectors, routing, fake-bus drivers
dev_notebooks/            sources of the system modules (connection, routing, ...): none yet
device_notebooks/         sources of the device modules, by type (same tree as devices/). Each
                          writes a module and its tests, runs them on the host, then checks
                          the real part (jupytext .py is the source of truth)
  sensors/environmental/tmp102_devNB
  sensors/environmental/shtc3_devNB
  sensors/magnetic/tmag5273_devNB
  sensors/distance/vl53l1x_devNB
  muxes/tca9548a_devNB
  drivers/                (relay, MCP23017 to come)
notebooks/                whole-bench hardware checks
  03_routing_harness      the whole bench: config -> validate -> scan -> build -> talk
docs/design.md            why each layer is the way it is; limits and next steps
docs/datasheets/          part datasheets
archive/                  the original single-file notebooks and the old bring-up notebooks
```

## The bench

Connected now (2026-11-14):

```
I2C 1 (scl=15 sda=16)
  SparkFun Qwiic Mux (TCA9548A) @ 0x70   (address jumpers open)
    ch 0: SparkFun TMAG5273 (B1) @ 0x22
    ch 2: TMP102                 @ 0x48
    ch 7: Adafruit VL53L1X       @ 0x29
```

The TMP102, TCA9548A, TMAG5273 and VL53L1X dev notebooks run on this wiring. The others were
validated on other wiring (written in each notebook's "Hardware and configuration" table):

```
shtc3_devNB (2026-11-14):
I2C 0 (STEMMA QT, scl=9 sda=8)
  SparkFun Qwiic SHTC3 @ 0x70             (alone on the bus)

tca9548a_devNB, tmp102_devNB, tmag5273_devNB (2026-11-13), also:
I2C 0 (STEMMA QT, scl=9 sda=8)
  VL53L4CD           @ 0x29
  Qwiic Single Relay @ 0x18
```

The SHTC3 is always at 0x70, the mux's own address, so it never shares the mux's segment.

## Setup

micropylab must be checked out next to this repo (`../micropylab`); it is installed
editable from there.

```bash
uv sync                       # host tools: JupyterLab, micropylab kernel, jupytext, ruff, pytest
uv run jupyter lab
```

Open a notebook, pick the **MicroPython (micropylab)** kernel, and run it top to bottom.
Each notebook's first cells connect (`%board connect`) and copy the package to the board
(`%files copy -r src/micropython_i2c :/lib/micropython_i2c`). Re-run those two cells
after editing the package; the import cell drops stale modules so the new files load.

To ship compiled bytecode instead, `%files compile src/micropython_i2c/<mod>.py
/lib/micropython_i2c/` per module, and delete the matching `.py` on the board: when both
exist, MicroPython imports the `.py`.

## Tests and lint

The package imports on CPython, so everything that does not need silicon is tested on the
host against fake parts (`tests/fakes.py` models mux routing the way real hardware does):

```bash
uv run pytest
uv run ruff check
```

The notebooks are the hardware tests. Each ends with a PASS/FAIL tally.

## Adding a driver

See "Writing a driver" in [docs/design.md](docs/design.md). In short: a `<Part>Regs`
facts-only class plus a driver deriving from it, codecs as classmethods with offline
tests, and an entry in `devices.DRIVERS`.
