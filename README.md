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
from micropython_i2c.drivers.tmp102 import TMP102

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
  drivers/                TMP102, TMAG5273, SHTC3, TCA9548A, VL53L4CD, QwiicRelay
                          + lazy registry
tests/                    host-side pytest: codecs, CRC vectors, routing, fake-bus drivers
notebooks/                hardware bring-up (jupytext .py is the source of truth)
  01_tmp102_bringup       RegisterI2CBase through TMP102Regs (behind the mux), then the driver
  02_tmag5273_bringup     both mixins, CRC against silicon, fast reads, ranges
  03_routing_harness      the whole bench: config -> validate -> scan -> build -> talk
docs/design.md            why each layer is the way it is; limits and next steps
docs/datasheets/          part datasheets
archive/                  the original single-file notebooks, for reference
```

## The bench

```
I2C 0 (STEMMA QT, scl=9 sda=8)
  SparkFun Qwiic Mux (TCA9548A) @ 0x71   (jumpered: 0x70 is the SHTC3's only address)
    ch 0: SHTC3    @ 0x70
    ch 2: TMP102   @ 0x48
I2C 1 (pins unconfirmed; notebook 03 section 2 finds them)
  VL53L4CD           @ 0x29
  Qwiic Single Relay @ 0x18
```

The TMAG5273 notebook (02) is for when that board is connected; it is not on this bench.

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
tests, and an entry in `drivers.DRIVERS`.
