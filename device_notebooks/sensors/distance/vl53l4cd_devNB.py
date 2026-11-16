# ---
# jupyter:
#   jupytext:
#     formats: ipynb,py:percent
#     text_representation:
#       extension: .py
#       format_name: percent
#       format_version: '1.3'
#       jupytext_version: 1.19.6
#   kernelspec:
#     display_name: MicroPython (micropylab)
#     language: python
#     name: micropylab
# ---

# %% [markdown]
# > # ⚠️ Do not run this notebook unattended
# >
# > The last section, **`# Hardware validation`**, connects to the real board, ranges with the
# > VL53L4CD's laser (Class 1, eye-safe) on I2C bus 0, and has steps where you hold your hand
# > over it. *Run All* and `jlx run` reach that section.
# >
# > - **Safe to run on its own:** everything above `# Hardware validation`. It runs on the host
# >   against a fake part, writes the module and its tests, and needs no board.
# > - **Hardware section:** run it one cell at a time, reading the **What to do** line above
# >   each cell first.
#
# **Where each cell runs.** Everything above `%board connect` (the first cells of
# `# Hardware validation`) runs in ordinary Python (CPython) on this computer, with no board: it
# writes the fake part (`vl53l4cd_fake.py`), the driver (`vl53l4cd.py`) and their tests, and
# checks the driver against the fake, in real and in simulated time. After `%board connect`,
# cells run in MicroPython on the board, except `%%host` cells, which run on the computer. That
# section uploads the freshly written package, checks the real VL53L4CD on bus 0 with a matching
# test for every unit test above, then disconnects.
#
# micropylab has `%%extract` built in, so there is no `%load_ext jupyter_lab_extractor` cell.

# %% [markdown]
# # Hardware and configuration
# What this notebook is validated with. The same table opens `# Hardware validation`.
#
# | | Host run (simulation) | Hardware run |
# |---|---|---|
# | **Runs on** | CPython 3.13 in the micropylab kernel, no board | Unexpected Maker FeatherS3 (ESP32-S3), MicroPython 1.28.0 (v1.28.0, 2026-04-06), USB `/dev/ttyACM0` |
# | **I2C bus** | `FakeBus` | `machine.I2C(0)`, **bus 0**, the STEMMA QT connector (scl=9 sda=8, on-board pull-ups), 100 kHz (`bus_open("feathers3", 0)`) |
# | **Mux** | none | none: the VL53L4CD is on bus 0 directly |
# | **Sensor** | `FakeVL53L4CD` at `0x29`: booted, a target at 123 mm, raw status 9 (valid), oscillator word `0x0BB8`, a sample 2 ms after each start or interrupt clear | VL53L4CD breakout at `0x29`; XSHUT and GPIO1 not wired |
# | **Other parts used** | none | none. Also on bus 0: SparkFun Qwiic Single Relay at `0x18` (not used here) |
# | **Datasheet** | ST DS13812 Rev 5 (June 2022), plus Adafruit's VL53L4CD library (a port of ST's ULD) for the register map | same |
# | **Result** | not run yet | not yet run on the hardware |
#
# The fake makes samples in 2 ms, not in the timing budget, so the unit tests stay fast; the
# simulated-time block uses longer samples, and the hardware section times the real ones.

# %% [markdown]
# # TODO:
# - Run the hardware section and fill in the results (this notebook has not been run on the
#   hardware yet). The driver itself was used on bus 0 on 2026-11-13, before this notebook.
# - The driver's `stop()` writes `0x00` to `0x0087`; ST's ULD and the Adafruit library write
#   `0x80`. The diagnostic "Does `stop()` stop ranging?" checks what the real part does with
#   each. Change the driver only after that has run.
#
# Decided (2026-11-16): this notebook replaces the hand-written
# `src/micropython_i2c/devices/sensors/distance/vl53l4cd.py`. The driver is ported unchanged
# (same API and behaviour). Its tests moved from `tests/test_drivers.py` to
# `tests/test_vl53l4cd.py`, one behaviour per test. The fake part replaces the old
# `micropython_i2c.fakes.fake_vl53l4cd()` register file: same registers, plus a distance that can
# be a signal, and sample timing (the old one always had a sample ready).
#
# Datasheet: ST DS13812 Rev 5, June 2022 (`docs/datasheets/VL53L4CD.PDF`). Like the VL53L1X's, it
# gives the I2C protocol, the reference registers (Table 7), the boot time (2.7) and the ranging
# limits, but no register map: it sends the reader to ST's ultra lite driver (ULD, UM2931). The
# register addresses, the init table and the timing-budget arithmetic come from Adafruit's
# `adafruit-circuitpython-vl53l4cd` 1.3.6, a port of the ULD.

# %% [markdown]
# # Python file
# location: `src/micropython_i2c/devices/sensors/distance/vl53l4cd.py` (driver) and
# `src/micropython_i2c/devices/sensors/distance/vl53l4cd_fake.py` (fake part, host only)
#
# tests: `tests/test_vl53l4cd.py` (both)

# %% [markdown]
# ## header

# %%
# %%extract ../../../src/micropython_i2c/devices/sensors/distance/vl53l4cd.py
"""ST VL53L4CD time-of-flight sensor in continuous ranging mode (datasheet DS13812 rev 5).

Usage on the board::

    from micropython_i2c import I2CConnection
    from micropython_i2c.devices.sensors.distance.vl53l4cd import VL53L4CD

    tof = VL53L4CD(I2CConnection(bus, 0x29, lock=bus_lock))
    tof.start()
    mm = tof.read()                 # always check tof.valid afterwards
    print(mm, "mm", tof.status_name)
    tof.stop()

The part has a 16-bit register map (``REG_ADDR_BITS = 16``). Only a reading with
``valid`` True can be trusted; open air past about 1.3 m mostly gives "sigma warning" or
"invalid phase" with a 0 or ~2000 mm value.

Ported from micropylab's examples/vl53l4cd.py, itself a port of
adafruit-circuitpython-vl53l4cd 1.3.6: the 0x2D..0x87 init table and the ranging logic come
from that library, whose licence follows.

    SPDX-FileCopyrightText: 2017 Scott Shawcroft, written for Adafruit Industries
    SPDX-FileCopyrightText: Copyright (c) 2022 Carter Nelson for Adafruit Industries
    SPDX-License-Identifier: MIT

    The MIT License (MIT)
    Copyright (c) 2022 Carter Nelson for Adafruit Industries
    Permission is hereby granted, free of charge, to any person obtaining a copy of this
    software and associated documentation files (the "Software"), to deal in the Software
    without restriction, including without limitation the rights to use, copy, modify,
    merge, publish, distribute, sublicense, and/or sell copies of the Software, and to
    permit persons to whom the Software is furnished to do so, subject to the following
    conditions: The above copyright notice and this permission notice shall be included in
    all copies or substantial portions of the Software. THE SOFTWARE IS PROVIDED "AS IS",
    WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
    WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
    IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR
    OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

Generated by ``device_notebooks/sensors/distance/vl53l4cd_devNB``: edit the notebook, not
this file.
"""

# %% [markdown]
# # Imports
# ## External
# None: the driver runs on MicroPython and uses only this package.
#
# ## Internal
# `RegisterI2CBase` for the 16-bit register map, and the timing helpers from `compat`.

# %%
# %%extract ../../../src/micropython_i2c/devices/sensors/distance/vl53l4cd.py -a
from micropython_i2c.compat import sleep_ms, ticks_diff, ticks_ms
from micropython_i2c.node import I2CDevice
from micropython_i2c.protocol import RegisterI2CBase

# %% [markdown]
# # Setup

# %% [markdown]
# Notebook-only: run the test cells in the notebook. `raise_on_error=True` makes a failing test
# fail the run.

# %%
import ipytest

ipytest.autoconfig(raise_on_error=True)

# %% [markdown]
# # Test file
# The test file header: imports only. It is written, not run.

# %%
# %%writefile ../../../tests/test_vl53l4cd.py
import pytest

from micropython_i2c import I2CConnection
from micropython_i2c.devices.sensors.distance.vl53l4cd import _INIT, STATUS_NAMES, VL53L4CD
from micropython_i2c.devices.sensors.distance.vl53l4cd_fake import FakeVL53L4CD
from micropython_i2c.sim.bus import FakeBus
from micropython_i2c.sim.clock import SimClock
from micropython_i2c.sim.signals import ramp

# %% [markdown]
# Notebook-only: the imports the validation and test cells use. The classes under test
# (`FakeVL53L4CD`, `VL53L4CD`) and the constants (`_INIT`, `STATUS_NAMES`) are the ones defined
# in this notebook, not the modules'.

# %%
import pytest

from micropython_i2c import I2CConnection
from micropython_i2c.sim.bus import FakeBus
from micropython_i2c.sim.clock import SimClock
from micropython_i2c.sim.signals import ramp

# %% [markdown]
# # Intent
# **Purpose:** measure short distances (0 to about 1.3 m) in millimetres with an ST VL53L4CD
# time-of-flight sensor, continuously, with a range status on every reading.
#
# **Why it exists:** the VL53L4CD has no documented register map. It is started by loading a
# 91-byte init table, running a one-time temperature (VHV) calibration and writing two timing
# words computed from the part's oscillator. A reading is only as good as its range status: the
# distance register always holds a number. The driver does the start-up once, and sets `valid`
# and `status_name` on every reading.
#
# **Used by:** the routing table and manager (`"driver": "VL53L4CD"`) and the bench (bus 0, the
# STEMMA QT connector, with the Qwiic Single Relay).
#
# **Not its job:** reaching the part (the `I2CConnection` does addressing and locking), the XSHUT
# and GPIO1 pins (not wired: the driver polls the interrupt line register), the address change,
# offset and crosstalk calibration, the inter-measurement period (the driver ranges
# back-to-back), and the signal rate and sigma results.
#
# ```python
# tof = VL53L4CD(I2CConnection(bus, 0x29, lock=bus_lock))  # boot, init table, VHV, 50 ms
# tof.timing_budget = 100      # 10..200 ms, only while stopped
# tof.start()                  # waits for the first sample
# mm = tof.read()              # next sample; tof.valid, tof.status_name
# mm = tof.read_valid()        # up to 10 tries for a valid one, else None
# tof.stop()
# ```
#
# ```mermaid
# stateDiagram-v2
#     [*] --> Boot: power (XSHUT pulled up on the breakout)
#     Boot --> Standby: firmware booted, 0x00E5 = 0x03 (tBOOT at most 1.2 ms)
#     Standby --> Ranging: 0x0087 = 0x21 (0x40 for the VHV calibration)
#     Ranging --> Ready: sample done, interrupt line active (0x0031 bit 0)
#     Ready --> Ranging: clear interrupt 0x0086 = 1
#     Ranging --> Standby: 0x0087 = 0x00 (the driver; the ULD writes 0x80)
# ```
#
# A sample takes the **timing budget**. If the interrupt is not cleared, ranging holds
# (datasheet 2.9): a new sample is only made after a clear.
#
# Limits from the datasheet:
#
# | Fact | Value | Source |
# |---|---|---|
# | address | `0x52` (8-bit write address), `0x29` as a 7-bit one | Table 1, section 3 |
# | I2C clock | up to 1 MHz (fast mode plus) | Table 1, section 3 |
# | reference registers | `0x010F` = `0xEB` (model ID), `0x0110` = `0xAA` (module type) | Table 7 |
# | boot time | 1.2 ms maximum | 2.7 |
# | ranging | 0 mm (linear from 1 mm) to about 1300 mm | 5.2, Table 14 |
#
# ## What the module defines
#
# | Name | What it is |
# |---|---|
# | `STATUS_NAMES` | raw range status to a readable name (only 9 is a valid range) |
# | `VL53L4CD` | the driver: identify, boot, init table and VHV calibration on construction, start/stop ranging, read the distance and its status, timing budget, the budget-word codec |
#
# The fake part, `FakeVL53L4CD`, is in its own module, `vl53l4cd_fake.py` (host only): see
# `# Fake part`.

# %% [markdown]
# # Fake part
# `FakeVL53L4CD` stands in for the real part on a `FakeBus`, so the driver can be written and
# tested on the host. It is written before the driver because the driver's tests need it. It
# goes to its own module next to the driver, `vl53l4cd_fake.py`, which the board never imports.

# %% [markdown]
# ## header

# %%
# %%extract ../../../src/micropython_i2c/devices/sensors/distance/vl53l4cd_fake.py
"""``FakeVL53L4CD``: an ST VL53L4CD register file and ranging sequencer, for host tests.

Usage on the host::

    from micropython_i2c.devices.sensors.distance.vl53l4cd_fake import FakeVL53L4CD
    from micropython_i2c.sim.bus import FakeBus
    from micropython_i2c.sim.signals import ramp

    bus = FakeBus()
    part = bus.attach(0x29, FakeVL53L4CD(distance_mm=ramp(1200, 30, 0, 60)))

The distance and the range status are plain values or signals
(``micropython_i2c.sim.signals``), read when the fake makes a sample. Timing goes through
``micropython_i2c.compat``, so ``SimClock`` drives it. The address is programmable
(``TAKES_ADDR``): the fake answers at whatever ``0x0001`` holds.

Host only.

Generated by ``device_notebooks/sensors/distance/vl53l4cd_devNB``: edit the notebook, not this
file.
"""

# %% [markdown]
# ## Imports
# The `compat` ticks for the sample timing, and `value` to read a quantity that may be a signal.

# %%
# %%extract ../../../src/micropython_i2c/devices/sensors/distance/vl53l4cd_fake.py -a
from micropython_i2c.compat import ticks_add, ticks_diff, ticks_ms
from micropython_i2c.sim.signals import value

# %% [markdown]
# ## Public names

# %%
# %%extract ../../../src/micropython_i2c/devices/sensors/distance/vl53l4cd_fake.py -a
__all__ = ["FakeVL53L4CD"]

# %% [markdown]
# ## `FakeVL53L4CD`
# A 64 KiB register file with a ranging sequencer around it. It does not model the optics: the
# distance and the status are attributes. What it models, and where each behaviour comes from:
#
# | Behaviour | Source |
# |---|---|
# | 16-bit register indexes, multi-byte values MSB first | datasheet section 3, Table 8 |
# | `0x010F`, `0x0110` read `EB AA`, and ignore writes | datasheet Table 7 |
# | `0x00E5` reads `0x03` once booted; with `booted=False` it reads 0 | Adafruit library (ULD `SensorInit`) waits for `0x03`; datasheet 2.7 (boot within 1.2 ms) |
# | `0x0006` holds the oscillator word, `0x0BB8` by default | Adafruit library (ULD `SetRangeTiming`) reads it; `0x0BB8` is the old `fake_vl53l4cd()`'s value, not measured |
# | `0x0087` = `0x21` (continuous) or `0x40` (autonomous, used for the VHV calibration) ranges; any other value stops | ULD `StartRanging` and `SensorInit`; the ULD stops with `0x80`, the driver with `0x00`, and the fake stops on both |
# | a sample is ready `sample_ms` after a start, or after an interrupt clear while ranging; until cleared, ranging holds | datasheet 2.9; `sample_ms` (default 2 ms) is the fake's, not the timing budget, so tests stay fast |
# | the interrupt line is `0x0031` bit 0; `0x0030` bit 4 set makes it active low (set at power-up here, and by the init table) | Adafruit library `data_ready` (ULD `CheckForDataReady`) |
# | a sample writes the raw status to `0x0089` and the distance to `0x0096` (16 bits) | Adafruit library (ULD `GetResult`) |
# | `0x0001` is the 7-bit address; the part answers there at once | ULD `SetI2CAddress` (the driver does not use it) |
#
# Not modelled: the optics, the timing budget and inter-measurement period (a sample takes
# `sample_ms`), the signal rate and sigma, offset and crosstalk, temperature, soft reset, GPIO1
# and XSHUT.
#
# The registers start as the old `fake_vl53l4cd()` had them, result included (status 9, 123 mm).
# Unlike it, the fake has no sample ready until ranging starts. `samples` counts the samples made
# and `writes` logs `(reg, data)` for every write.


# %%
# %%extract ../../../src/micropython_i2c/devices/sensors/distance/vl53l4cd_fake.py -a
class FakeVL53L4CD:
    """An ST VL53L4CD register file and ranging sequencer (datasheet DS13812 rev 5, ST's ULD).

    Args:
        distance_mm: Distance to the target, mm: a number or a signal ``f(t_s)``.
        status: Raw range status a sample reports (9 = valid): a number or a signal.
        osc: The oscillator word at ``0x0006``.
        sample_ms: Time from a start (or an interrupt clear) to the sample.
        addr: The address at power-up (``0x0001``).
        booted: False: the firmware never reports booted.

    Attributes:
        regs: The register file (``bytearray``, 64 KiB).
        mode: The last value written to ``0x0087``.
        ready: True while a sample waits for an interrupt clear.
        samples: How many samples the fake has made.
        writes: ``(reg, data)`` for every register write, in order.
    """

    TAKES_ADDR = True  # the address is programmable (0x0001)

    MODES_RANGING = (0x21, 0x40)
    READ_ONLY = {0x0006, 0x0007, 0x0031, 0x0089, 0x0096, 0x0097, 0x00E5, 0x010F, 0x0110}

    def __init__(self, distance_mm=123, status=9, osc=0x0BB8, sample_ms=2, addr=0x29,
                 booted=True):  # fmt: skip
        self.distance_mm = distance_mm
        self.status = status
        self.sample_ms = sample_ms
        self.samples = 0
        self.writes = []
        self.regs = bytearray(0x10000)
        self.regs[0x0001] = addr
        self.regs[0x0006], self.regs[0x0007] = osc >> 8, osc & 0xFF
        self.regs[0x0030] = 0x10  # interrupt active low: ready when the line reads 0
        self.regs[0x00E5] = 0x03 if booted else 0x00
        self.regs[0x010F], self.regs[0x0110] = 0xEB, 0xAA  # model ID, module type
        self._result()  # a result already there, as the old fake had
        self.mode = 0x00
        self.ready = False
        self.due = None  # ticks_ms when the next sample is ready (None: none coming)

    @property
    def bus_addr(self):
        """The address the part answers at (``0x0001``)."""
        return self.regs[0x0001] & 0x7F

    @property
    def ranging(self):
        """True while ``0x0087`` holds a ranging mode."""
        return self.mode in self.MODES_RANGING

    def _result(self):
        """Write the status and distance, reading the quantities (signals) now."""
        mm = max(0, min(0xFFFF, int(value(self.distance_mm))))
        self.regs[0x0089] = int(value(self.status)) & 0xFF
        self.regs[0x0096], self.regs[0x0097] = mm >> 8, mm & 0xFF

    def _tick(self):
        """Make the pending sample if its time has come."""
        if self.due is not None and not self.ready and ticks_diff(ticks_ms(), self.due) >= 0:
            self._result()
            self.ready = True
            self.samples += 1
            self.due = None

    def _schedule(self):
        self.due = ticks_add(ticks_ms(), self.sample_ms)

    def _line(self):
        active_low = bool(self.regs[0x0030] & 0x10)
        return int(self.ready) ^ int(active_low)

    def mem_read(self, reg, nbytes):
        self._tick()
        out = bytearray()
        for i in range(nbytes):
            r = (reg + i) & 0xFFFF
            out.append(self._line() if r == 0x0031 else self.regs[r])
        return bytes(out)

    def mem_write(self, reg, data):
        self._tick()
        self.writes.append((reg, bytes(data)))
        for i, val in enumerate(data):
            r = (reg + i) & 0xFFFF
            if r in self.READ_ONLY:
                continue
            self.regs[r] = val
            if r == 0x0086 and val & 1:  # clear the interrupt: ranging goes on
                self.ready = False
                if self.ranging:
                    self._schedule()
            elif r == 0x0087:
                self.mode = val
                if not self.ranging:
                    self.due = None
                elif not self.ready:  # a waiting sample holds ranging until cleared
                    self._schedule()


# %% [markdown]
# ### validation/test
# Notebook-only helpers for the fake's own checks: read and write the fake at `0x29` with
# 16-bit register indexes, the way the driver does.


# %%
def rd(bus, reg, n=1, addr=0x29):
    return bus.readfrom_mem(addr, reg, n, addrsize=16)


def wr(bus, reg, data, addr=0x29):
    bus.writeto_mem(addr, reg, bytes(data), addrsize=16)


# %% [markdown]
# #### validation: nominal usage
# The reference registers (Table 7), then one sample in simulated time with the target on a
# `ramp` from 1000 to 200 mm over 10 s: a sample made at 5 s reads about 600 mm. The interrupt
# is active low, so the line reads 0 when a sample is ready.

# %%
with SimClock() as clock:
    bus = FakeBus()
    part = bus.attach(0x29, FakeVL53L4CD(distance_mm=ramp(1000, 200, 0, 10)))
    print("0x010F..0x0110:", rd(bus, 0x010F, 2).hex(" "), "| want eb aa")
    print("line before ranging:", rd(bus, 0x0031)[0], "(1 = nothing ready)")
    clock.advance(5)
    wr(bus, 0x0087, [0x21])  # start continuous ranging
    clock.advance(0.002)
    print("line at 5.002 s:", rd(bus, 0x0031)[0], "(0 = sample ready)")
    print("status {}, distance {} mm (want about 600)".format(
        rd(bus, 0x0089)[0], int.from_bytes(rd(bus, 0x0096, 2), "big")))  # fmt: skip

# %% [markdown]
# #### validation: anti-nominal usage
# After the address register is set to `0x30`, the part no longer answers at `0x29`: the bus
# refuses the transfer, as a real NACK would.

# %%
bus = FakeBus()
part = bus.attach(0x29, FakeVL53L4CD())
wr(bus, 0x0001, [0x30])  # move the part to 0x30
try:
    rd(bus, 0x010F, 2)  # still talking to 0x29
except OSError as err:
    print("raised OSError, as expected:\n    {}".format(err))
else:
    raise AssertionError("a read at the old address should have raised OSError")
finally:
    print("answers at:", [hex(a) for a in bus.scan()])

# %% [markdown]
# #### unit

# %% [markdown]
# The reference registers read the Table 7 values and ignore writes; the firmware status reads
# `0x03` (booted), or 0 with `booted=False`.

# %%
# %%extract ../../../tests/test_vl53l4cd.py -a --strip-ipytest
# %%ipytest


def test_fake_reference_registers_follow_table_7():
    bus = FakeBus()
    bus.attach(0x29, FakeVL53L4CD())
    bus.attach(0x30, FakeVL53L4CD(addr=0x30, booted=False))
    bus.writeto_mem(0x29, 0x010F, b"\x00\x00", addrsize=16)
    assert bus.readfrom_mem(0x29, 0x010F, 2, addrsize=16) == b"\xeb\xaa"
    assert bus.readfrom_mem(0x29, 0x00E5, 1, addrsize=16) == b"\x03"
    assert bus.readfrom_mem(0x30, 0x00E5, 1, addrsize=16) == b"\x00"


# %% [markdown]
# A sample is made `sample_ms` after a start; the next one only after the interrupt is cleared
# (datasheet 2.9), however long the host waits. The line is active low.

# %%
# %%extract ../../../tests/test_vl53l4cd.py -a --strip-ipytest
# %%ipytest


def test_fake_new_sample_only_after_an_interrupt_clear():
    with SimClock() as clock:
        bus = FakeBus()
        part = bus.attach(0x29, FakeVL53L4CD(sample_ms=50))
        bus.writeto_mem(0x29, 0x0087, b"\x21", addrsize=16)
        clock.advance(0.049)
        assert bus.readfrom_mem(0x29, 0x0031, 1, addrsize=16) == b"\x01"  # not yet
        clock.advance(10)
        assert bus.readfrom_mem(0x29, 0x0031, 1, addrsize=16) == b"\x00" and part.samples == 1
        bus.writeto_mem(0x29, 0x0086, b"\x01", addrsize=16)
        clock.advance(0.050)
        assert bus.readfrom_mem(0x29, 0x0031, 1, addrsize=16) == b"\x00" and part.samples == 2


# %% [markdown]
# Stopping (`0x00`, as the driver writes, or `0x80`, as the ULD writes) ends the samples.

# %%
# %%extract ../../../tests/test_vl53l4cd.py -a --strip-ipytest
# %%ipytest


@pytest.mark.parametrize("stop", [0x00, 0x80])
def test_fake_stop_ends_the_samples(stop):
    with SimClock() as clock:
        bus = FakeBus()
        part = bus.attach(0x29, FakeVL53L4CD(sample_ms=50))
        bus.writeto_mem(0x29, 0x0087, b"\x21", addrsize=16)
        clock.advance(0.010)
        bus.writeto_mem(0x29, 0x0087, bytes([stop]), addrsize=16)
        clock.advance(1)
        assert bus.readfrom_mem(0x29, 0x0031, 1, addrsize=16) == b"\x01" and part.samples == 0


# %% [markdown]
# The distance and the status are read when the sample is made: a signal gives the value at that
# moment (the fake makes a sample at the first access after it is due).

# %%
# %%extract ../../../tests/test_vl53l4cd.py -a --strip-ipytest
# %%ipytest


def test_fake_quantities_are_read_when_sampled():
    with SimClock() as clock:
        bus = FakeBus()
        bus.attach(0x29, FakeVL53L4CD(distance_mm=ramp(1000, 0, 0, 10),
                                      status=lambda t: 9 if t < 5 else 4, sample_ms=1))  # fmt: skip
        clock.advance(2.5)
        bus.writeto_mem(0x29, 0x0087, b"\x21", addrsize=16)
        clock.advance(0.5)
        assert bus.readfrom_mem(0x29, 0x0096, 2, addrsize=16) == (700).to_bytes(2, "big")
        assert bus.readfrom_mem(0x29, 0x0089, 1, addrsize=16) == b"\x09"
        bus.writeto_mem(0x29, 0x0086, b"\x01", addrsize=16)
        clock.advance(3)
        assert bus.readfrom_mem(0x29, 0x0096, 2, addrsize=16) == (400).to_bytes(2, "big")
        assert bus.readfrom_mem(0x29, 0x0089, 1, addrsize=16) == b"\x04"


# %% [markdown]
# # Code
# ## TODO:
# Nothing

# %% [markdown]
# ## Public names

# %%
# %%extract ../../../src/micropython_i2c/devices/sensors/distance/vl53l4cd.py -a
__all__ = ["STATUS_NAMES", "VL53L4CD"]

# %% [markdown]
# ## The init table and the range statuses
# `_INIT` is written to registers `0x002D..0x0087` in one sequential write at construction: 91
# bytes, copied from the Adafruit library (ST's ULD `VL53L4CD_DEFAULT_CONFIGURATION`). It sets
# the interrupt active low (`0x0030` = `0x11`, bit 4) and ends with ranging stopped (`0x0087` =
# `0x00`).
#
# `STATUS_NAMES` names the raw range status (`0x0089` bits 4..0), from the Adafruit library's
# table. Only 9 is a valid range.


# %%
# %%extract ../../../src/micropython_i2c/devices/sensors/distance/vl53l4cd.py -a
# Init table for registers 0x2D..0x87, copied from the Adafruit source.
_INIT = (
    0x12, 0x00, 0x00, 0x11, 0x02, 0x00, 0x02, 0x08, 0x00, 0x08, 0x10, 0x01, 0x01, 0x00,
    0x00, 0x00, 0x00, 0xFF, 0x00, 0x0F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 0x0B, 0x00,
    0x00, 0x02, 0x14, 0x21, 0x00, 0x00, 0x05, 0x00, 0x00, 0x00, 0x00, 0xC8, 0x00, 0x00,
    0x38, 0xFF, 0x01, 0x00, 0x08, 0x00, 0x00, 0x01, 0xCC, 0x07, 0x01, 0xF1, 0x05, 0x00,
    0xA0, 0x00, 0x80, 0x08, 0x38, 0x00, 0x00, 0x00, 0x00, 0x0F, 0x89, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x01, 0x07, 0x05, 0x06, 0x06, 0x00, 0x00, 0x02, 0xC7, 0xFF,
    0x9B, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00,
)  # fmt: skip

# Raw range status values, from the Adafruit library's table. Only 9 is a valid range.
STATUS_NAMES = {
    3: "hardware fail",
    4: "sigma warning",
    5: "invalid phase",
    6: "sigma warning",
    7: "wrapped target",
    8: "below detection threshold",
    9: "valid",
    12: "crosstalk fail",
    18: "interrupt error",
    19: "no wrap-around check",
    22: "merged target",
    23: "signal too weak",
}

# %% [markdown]
# ### validation/test

# %% [markdown]
# #### validation: nominal usage

# %%
print("init table: {} bytes for 0x002D..0x0087".format(len(_INIT)))
print("0x0030 in the table: {:#04x} (bit 4 set: interrupt active low)".format(_INIT[0x30 - 0x2D]))
print("0x0087 in the table: {:#04x} (ranging stopped)".format(_INIT[0x87 - 0x2D]))
print("valid statuses:", [s for s, name in STATUS_NAMES.items() if name == "valid"])

# %% [markdown]
# #### validation: anti-nominal usage
# None: these are constants, with no failure path.

# %% [markdown]
# #### unit

# %% [markdown]
# The init table covers `0x002D..0x0087` exactly, sets the interrupt active low, and leaves
# ranging stopped and the interrupt alone.

# %%
# %%extract ../../../tests/test_vl53l4cd.py -a --strip-ipytest
# %%ipytest


def test_init_table_covers_its_range():
    from_table = dict(zip(range(0x2D, 0x88), _INIT))
    assert len(_INIT) == 0x87 - 0x2D + 1 == 91
    assert from_table[0x30] & 0x10 and from_table[0x87] == 0x00 and from_table[0x86] == 0x00


# %% [markdown]
# Only raw status 9 is named "valid".

# %%
# %%extract ../../../tests/test_vl53l4cd.py -a --strip-ipytest
# %%ipytest


def test_only_status_9_is_valid():
    assert [s for s, name in STATUS_NAMES.items() if name == "valid"] == [9]


# %% [markdown]
# ## `VL53L4CD`
# The driver, ported unchanged. Construction checks the model ID (anything else at the address is
# refused), waits for the firmware to boot, writes the init table, runs the one-time VHV
# calibration (one ranging start in mode `0x40`), writes the VHV and inter-measurement settings
# (`0x0008`, `0x000B`, `0x0024`, `0x006C` = 0, so ranging is back-to-back), and sets a 50 ms
# timing budget.
#
# | Rule | Why |
# |---|---|
# | the timing budget changes only while stopped (`RuntimeError`) | the sequencer reads it at the start |
# | the timing budget is 10..200 ms (`ValueError`) | the ULD's limits |
# | an oscillator word of 0 is refused (`OSError`) | the budget words divide by it |
# | `read()` sets `status` and `valid`; only status 9 is valid | the distance register always holds a number |
#
# The waits poll every millisecond: `_wait` up to 1000 times (boot, the first sample after a
# start), `read()` until `timeout_ms`. After clearing the interrupt, `read()` sleeps 2 ms so the
# line has dropped before the next `data_ready()`.
#
# `range_config_words(ms, osc)` turns a budget into the two range-config words, as the Adafruit
# port does: the budget less 2.5 ms, in macro periods of `2304 × 2^30 / osc` (scaled), coded as
# a mantissa byte and an exponent, for the A (×16) and B (×12) periods.


# %%
# %%extract ../../../src/micropython_i2c/devices/sensors/distance/vl53l4cd.py -a
class VL53L4CD(RegisterI2CBase, I2CDevice):
    """VL53L4CD driver: identify, boot, start/stop ranging, read distance in mm.

    Construction checks the model ID, waits for the firmware to boot, loads the init
    table, runs the one-time VHV calibration and sets a 50 ms timing budget.

    Attributes:
        ranging: True between ``start()`` and ``stop()``.
        status: Raw range status of the last ``read()`` (None before the first).
        valid: True if the last ``read()`` is trustworthy (status 9).

    Raises:
        RuntimeError: If the part at the address is not a VL53L4CD.
        OSError: If the firmware does not boot within about 1 s.
    """

    ALLOWED_ADDRS = tuple(range(0x08, 0x78))  # 0x29 by default, programmable
    ADDR_DEFAULT = 0x29
    BAUD_NOMINAL = 400_000
    BAUD_MAX = 1_000_000
    BAUD_MIN = 10_000

    REG_ADDR_BITS = 16
    REG_BYTES = 1
    BIG_ENDIAN = True

    MODEL_ID = 0xEBAA
    STATUS_VALID = 9

    REG_OSC_FREQ = 0x0006
    REG_GPIO_HV_MUX_CTRL = 0x0030  # bit 4: interrupt polarity
    REG_GPIO_TIO_HV_STATUS = 0x0031  # bit 0: interrupt line
    REG_SYSTEM_START = 0x0087
    REG_INTERRUPT_CLEAR = 0x0086
    REG_RANGE_STATUS = 0x0089
    REG_DISTANCE_MM = 0x0096
    REG_MODEL_ID = 0x010F
    REG_FIRMWARE_STATUS = 0x00E5
    REG_RANGE_CONFIG_A = 0x005E
    REG_RANGE_CONFIG_B = 0x0061

    START_RANGING = 0x21
    START_VHV = 0x40

    def __init__(self, conn, **kwargs):
        super().__init__(conn, **kwargs)
        self.ranging = False
        self.status = None
        self.valid = False
        self._budget = None
        if self.model_id() != self.MODEL_ID:
            raise RuntimeError("not a VL53L4CD at {:#04x}".format(self.addr))
        self._wait(lambda: self.reg_read(self.REG_FIRMWARE_STATUS) == 0x03, "boot timeout")
        self.write_mem(0x002D, bytes(_INIT))
        self.start(vhv=True)  # one-time VHV calibration
        self.stop()
        self.write_mem(0x0008, b"\x09")
        self.write_mem(0x000B, b"\x00")
        self.write_mem(0x0024, b"\x05\x00")
        self.write_mem(0x006C, b"\x00\x00\x00\x00")  # inter-measurement 0: continuous
        self.timing_budget = 50

    @staticmethod
    def _wait(ready, message, tries=1000):
        """Poll ``ready()`` every ms, up to ``tries`` times.

        Raises:
            OSError: With ``message`` if it never becomes true.
        """
        for _ in range(tries):
            if ready():
                return
            sleep_ms(1)
        raise OSError(message)

    def model_id(self):
        """0xEBAA for a VL53L4CD."""
        return self.reg_read(self.REG_MODEL_ID, nbytes=2)

    def data_ready(self):
        """True when a new sample is waiting (interrupt line active)."""
        polarity = 0 if self.reg_read(self.REG_GPIO_HV_MUX_CTRL) & 0x10 else 1
        return self.reg_read(self.REG_GPIO_TIO_HV_STATUS) & 1 == polarity

    def start(self, vhv=False):
        """Start ranging and wait for the first sample.

        Args:
            vhv: Internal: run the one-time VHV calibration instead of normal ranging.

        Raises:
            OSError: If no sample arrives within about 1 s.
        """
        self.reg_write(self.REG_SYSTEM_START, self.START_VHV if vhv else self.START_RANGING)
        self._wait(self.data_ready, "start timeout")
        self.reg_write(self.REG_INTERRUPT_CLEAR, 0x01)
        self.ranging = True

    def stop(self):
        """Stop ranging. Required before changing ``timing_budget``."""
        self.reg_write(self.REG_SYSTEM_START, 0x00)
        self.ranging = False

    def read(self, timeout_ms=1000):
        """Wait for the next sample and return the distance in mm.

        Sets ``status`` and ``valid``; check ``valid`` before trusting the value.

        Raises:
            OSError: If no sample arrives within ``timeout_ms`` (often: not started).
        """
        start = ticks_ms()
        while not self.data_ready():
            if ticks_diff(ticks_ms(), start) > timeout_ms:
                raise OSError("read timeout")
            sleep_ms(1)
        self.status = self.reg_read(self.REG_RANGE_STATUS) & 0x1F
        self.valid = self.status == self.STATUS_VALID
        mm = self.reg_read(self.REG_DISTANCE_MM, nbytes=2)
        self.reg_write(self.REG_INTERRUPT_CLEAR, 0x01)
        sleep_ms(2)  # let the interrupt flag drop, else data_ready() is stale
        return mm

    def read_valid(self, tries=10, timeout_ms=1000):
        """Read until a valid sample arrives; None if none of ``tries`` was valid."""
        for _ in range(tries):
            mm = self.read(timeout_ms)
            if self.valid:
                return mm
        return None

    @property
    def status_name(self):
        """Readable name of the last ``status``, e.g. "valid" or "sigma warning"."""
        if self.status is None:
            return "no reading yet"
        return STATUS_NAMES.get(self.status, "status {}".format(self.status))

    @staticmethod
    def range_config_words(ms, osc):
        """The two range-config register words for a ``ms`` budget at oscillator ``osc``.

        Converts the budget into macro periods as in the Adafruit port. Pure arithmetic,
        so it can be tested without a part.
        """
        us = (ms * 1000 - 2500) << 12
        macro = int(2304 * (1073741824.0 / osc)) >> 6
        words = []
        for mul in (16, 12):
            tmp = macro * mul
            ls = int(((us + ((tmp >> 6) >> 1)) / (tmp >> 6)) - 1)
            msb = 0
            while ls >> 8:
                ls >>= 1
                msb += 1
            words.append((msb << 8) + (ls & 0xFF))
        return words

    @property
    def timing_budget(self):
        """Time per measurement in ms (10..200). Longer is less noisy."""
        return self._budget

    @timing_budget.setter
    def timing_budget(self, ms):
        if self.ranging:
            raise RuntimeError("stop ranging before changing the timing budget")
        if not 10 <= ms <= 200:
            raise ValueError("timing budget must be 10..200 ms, not {}".format(ms))
        osc = self.reg_read(self.REG_OSC_FREQ, nbytes=2)
        if osc == 0:
            raise OSError("oscillator frequency reads 0")
        word_a, word_b = self.range_config_words(ms, osc)
        self.reg_write(self.REG_RANGE_CONFIG_A, word_a, nbytes=2)
        self.reg_write(self.REG_RANGE_CONFIG_B, word_b, nbytes=2)
        self._budget = ms

# %% [markdown]
# ### validation/test: codecs
# `range_config_words` is pure arithmetic. A word is an exponent byte and a mantissa byte; the
# number of macro periods it stands for is `(mantissa + 1) × 2^exponent`, so a longer budget must
# never give fewer. `BUDGET_CASES` are the budgets the checks use, at the old fake's oscillator
# word `0x0BB8`.

# %%
# %%extract ../../../tests/test_vl53l4cd.py -a
BUDGET_CASES = [10, 20, 33, 50, 100, 200]


def periods(word):
    """Macro periods a range-config word stands for: ``(mantissa + 1) << exponent``."""
    return ((word & 0xFF) + 1) << (word >> 8)


# %% [markdown]
# #### validation: nominal usage

# %%
for ms in BUDGET_CASES:
    a, b = VL53L4CD.range_config_words(ms, 0x0BB8)
    print("{:3} ms -> A {:#06x} ({} periods), B {:#06x} ({} periods)".format(
        ms, a, periods(a), b, periods(b)))  # fmt: skip

# %% [markdown]
# #### validation: anti-nominal usage
# The codec does not check its input (the `timing_budget` setter does); an oscillator word of 0
# fails in the division.

# %%
try:
    VL53L4CD.range_config_words(50, 0)  # the setter refuses this before calling
except ZeroDivisionError as err:
    print("raised ZeroDivisionError, as expected:\n    {}".format(err))
else:
    raise AssertionError("osc = 0 should have raised ZeroDivisionError")

# %% [markdown]
# #### unit

# %% [markdown]
# Every word fits its 16-bit register (ported from `tests/test_drivers.py`).

# %%
# %%extract ../../../tests/test_vl53l4cd.py -a --strip-ipytest
# %%ipytest


def test_budget_words_fit_registers():
    for ms in (10, 50, 200):
        for word in VL53L4CD.range_config_words(ms, 0x0BB8):
            assert 0 <= word <= 0xFFFF and word & 0xFF <= 0xFF


# %% [markdown]
# A longer budget never gives fewer macro periods, in either word, and the B word (×12 periods)
# always stands for more periods than the A word (×16).

# %%
# %%extract ../../../tests/test_vl53l4cd.py -a --strip-ipytest
# %%ipytest


def test_budget_words_grow_with_the_budget():
    words = [VL53L4CD.range_config_words(ms, 0x0BB8) for ms in BUDGET_CASES]
    for i in (0, 1):
        counts = [periods(w[i]) for w in words]
        assert counts == sorted(counts) and counts[0] < counts[-1]
    assert all(periods(b) > periods(a) for a, b in words)


# %% [markdown]
# ### validation/test: the driver on a fake part
# A helper that puts a `FakeVL53L4CD` at `0x29` on a fake bus and constructs the driver on it
# (keywords go to the fake). It returns the driver and the fake part, and is shared by the
# validation cells and the test file.


# %%
# %%extract ../../../tests/test_vl53l4cd.py -a
def on_fake(addr=0x29, **part_kw):
    """``(tof, part)``: a ``VL53L4CD`` on a fresh ``FakeVL53L4CD(**part_kw)`` at ``addr``."""
    bus = FakeBus()
    part = bus.attach(addr, FakeVL53L4CD(addr=addr, **part_kw))
    return VL53L4CD(I2CConnection(bus, addr)), part


# %% [markdown]
# #### validation: nominal usage
# Construct (identify, boot, init table, VHV calibration, 50 ms), then three continuous readings.

# %%
tof, part = on_fake(distance_mm=456)
print("budget {} ms | 0x002D = {:#04x} | samples so far {}".format(
    tof.timing_budget, part.regs[0x002D], part.samples))  # fmt: skip
tof.start()
for _ in range(3):
    mm = tof.read()
    print("{} mm, {} (status {}, valid {})".format(mm, tof.status_name, tof.status, tof.valid))
tof.stop()

# %% [markdown]
# #### validation: anti-nominal usage
# A part at `0x29` with another model ID is refused.

# %%
bus = FakeBus()
part = bus.attach(0x29, FakeVL53L4CD())
part.regs[0x010F] = 0x00  # not a VL53L4CD any more
try:
    VL53L4CD(I2CConnection(bus, 0x29))
except RuntimeError as err:
    print("raised RuntimeError, as expected:\n    {}".format(err))
else:
    raise AssertionError("model ID 0x00AA should have raised RuntimeError")

# %% [markdown]
# The timing budget cannot change while ranging.

# %%
tof, part = on_fake()
tof.start()
try:
    tof.timing_budget = 100  # still ranging
except RuntimeError as err:
    print("raised RuntimeError, as expected:\n    {}".format(err))
else:
    raise AssertionError("a budget change while ranging should have raised RuntimeError")
finally:
    tof.stop()
    print("budget still", tof.timing_budget, "ms")

# %% [markdown]
# #### unit

# %% [markdown]
# Construction reads the model ID, writes the init table at `0x002D`, runs the VHV calibration
# (one sample in mode `0x40`), leaves ranging stopped, and sets a 50 ms budget (ported from
# `tests/test_drivers.py`).

# %%
# %%extract ../../../tests/test_vl53l4cd.py -a --strip-ipytest
# %%ipytest


def test_init_loads_the_table_and_sets_50_ms():
    tof, part = on_fake()
    assert tof.timing_budget == 50 and not tof.ranging and part.mode == 0x00
    assert (0x002D, bytes(_INIT)) in part.writes and part.regs[0x002D] == 0x12
    assert (0x0087, b"\x40") in part.writes and part.samples >= 1
    assert any(e[0] == "rm" and e[2] == 0x010F for e in tof.conn.bus.log)
    words = VL53L4CD.range_config_words(50, 0x0BB8)
    assert [int.from_bytes(part.regs[r : r + 2], "big") for r in (0x005E, 0x0061)] == words


# %% [markdown]
# Once started, `read()` returns the fake's distance, with status 9: valid (ported).

# %%
# %%extract ../../../tests/test_vl53l4cd.py -a --strip-ipytest
# %%ipytest


def test_reads_distance_when_started():
    tof, _ = on_fake(distance_mm=456)
    assert tof.status_name == "no reading yet"
    tof.start()
    assert tof.read() == 456 and tof.valid and tof.status_name == "valid"
    tof.stop()


# %% [markdown]
# Each `read()` waits for a new sample: the fake makes one per interrupt clear. (It may make one
# more than were read: the sample after the last clear can be due before `stop()`.)

# %%
# %%extract ../../../tests/test_vl53l4cd.py -a --strip-ipytest
# %%ipytest


def test_each_read_is_a_new_sample():
    tof, part = on_fake()
    tof.start()
    before = part.samples
    for _ in range(3):
        tof.read()
    tof.stop()
    assert part.samples - before >= 3 and not tof.ranging


# %% [markdown]
# The timing budget is refused while ranging (ported), and outside 10..200 ms.

# %%
# %%extract ../../../tests/test_vl53l4cd.py -a --strip-ipytest
# %%ipytest


def test_timing_budget_rules():
    tof, _ = on_fake()
    tof.start()
    with pytest.raises(RuntimeError, match="stop"):
        tof.timing_budget = 100
    tof.stop()
    for bad in (9, 201):
        with pytest.raises(ValueError, match="10..200"):
            tof.timing_budget = bad
    tof.timing_budget = 100
    assert tof.timing_budget == 100


# %% [markdown]
# A part with another model ID is refused (ported).

# %%
# %%extract ../../../tests/test_vl53l4cd.py -a --strip-ipytest
# %%ipytest


def test_init_refuses_another_model():
    bus = FakeBus()
    part = bus.attach(0x29, FakeVL53L4CD())
    part.regs[0x010F] = 0x00
    with pytest.raises(RuntimeError, match="not a VL53L4CD"):
        VL53L4CD(I2CConnection(bus, 0x29))


# %% [markdown]
# A part whose firmware never boots makes the constructor raise `OSError` after about 1 s (in
# simulated time, so the test does not wait).

# %%
# %%extract ../../../tests/test_vl53l4cd.py -a --strip-ipytest
# %%ipytest


def test_init_times_out_without_boot():
    with SimClock() as clock:
        with pytest.raises(OSError, match="boot timeout"):
            on_fake(booted=False)
        assert 1.0 <= clock.now_s() < 1.1


# %% [markdown]
# An oscillator word of 0 makes the budget words impossible: the constructor raises `OSError`.

# %%
# %%extract ../../../tests/test_vl53l4cd.py -a --strip-ipytest
# %%ipytest


def test_oscillator_zero_is_refused():
    with pytest.raises(OSError, match="oscillator"):
        on_fake(osc=0)


# %% [markdown]
# An invalid status is flagged, not hidden: raw status 4 is "sigma warning", and `read_valid`
# gives up with None (ported).

# %%
# %%extract ../../../tests/test_vl53l4cd.py -a --strip-ipytest
# %%ipytest


def test_invalid_status_is_flagged():
    tof, _ = on_fake(status=4)
    tof.start()
    tof.read()
    assert not tof.valid and tof.status_name == "sigma warning"
    assert tof.read_valid(tries=3) is None
    tof.stop()


# %% [markdown]
# With ranging stopped no sample comes, and `read()` raises `OSError` after its timeout.

# %%
# %%extract ../../../tests/test_vl53l4cd.py -a --strip-ipytest
# %%ipytest


def test_read_times_out_when_not_started():
    tof, _ = on_fake()
    with pytest.raises(OSError, match="read timeout"):
        tof.read(timeout_ms=20)


# %% [markdown]
# ### validation/test: in simulated time
# The driver against the fake inside `with SimClock() as clock:`, where every sleep returns at
# once and moves the clock, so a minute of ranging runs in milliseconds. A hand approaches the
# sensor: the target is a `ramp` from 1200 mm to 30 mm over the first 60 s, then holds. The fake
# takes `sample_ms=50` per sample (the 50 ms budget).
#
# This block has no matching hardware test: the hardware section's ranging checks use a hand
# held still.

# %% [markdown]
# #### validation: nominal usage
# One reading every 10 simulated seconds over 80 s.

# %%
with SimClock() as clock:
    tof, part = on_fake(distance_mm=ramp(1200, 30, 0, 60), sample_ms=50)
    tof.start()
    for _ in range(9):
        mm = tof.read()
        print("t = {:5.1f} s: {:4} mm, {}".format(clock.now_s(), mm, tof.status_name))
        clock.advance(10)
    tof.stop()
    print("samples made:", part.samples, "| sleeps served:", clock.sleeps)

# %% [markdown]
# #### validation: anti-nominal usage
# The hand leaves the range: past 1300 mm the fake reports status 5 ("invalid phase"), and
# `read_valid` gives up with None.

# %%
with SimClock() as clock:
    hand = ramp(1000, 2000, 0, 10)
    tof, part = on_fake(distance_mm=hand, status=lambda t: 9 if hand(t) <= 1300 else 5,
                        sample_ms=50)  # fmt: skip
    tof.start()
    clock.advance(20)
    found = tof.read_valid(tries=5)
    tof.stop()
    print("read_valid at 20 s:", found, "| last status:", tof.status_name)
    if found is not None:
        raise AssertionError("no reading past 1300 mm should be valid")

# %% [markdown]
# #### unit

# %% [markdown]
# The driver follows a hand approaching along a ramp: each reading is within a few millimetres
# of the ramp at that moment, and the readings fall.

# %%
# %%extract ../../../tests/test_vl53l4cd.py -a --strip-ipytest
# %%ipytest


def test_follows_a_hand_approaching_in_simulated_time():
    hand = ramp(1200, 30, 0, 60)
    with SimClock() as clock:
        tof, _ = on_fake(distance_mm=hand, sample_ms=50)
        tof.start()
        readings = []
        for _ in range(7):
            mm = tof.read()
            assert tof.valid and abs(mm - hand(clock.now_s())) <= 5
            readings.append(mm)
            clock.advance(10)
        tof.stop()
    assert readings == sorted(readings, reverse=True) and readings[-1] <= 35


# %% [markdown]
# Back-to-back continuous reads come one sample apart: ten reads at a 50 ms sample take about
# half a second of simulated time.

# %%
# %%extract ../../../tests/test_vl53l4cd.py -a --strip-ipytest
# %%ipytest


def test_reads_come_one_sample_apart_in_simulated_time():
    with SimClock() as clock:
        tof, _ = on_fake(sample_ms=50)
        tof.start()
        t0 = clock.now_s()
        for _ in range(10):
            tof.read()
        took = clock.now_s() - t0
        tof.stop()
    assert 0.5 <= took <= 0.55


# %% [markdown]
# # Hardware validation
#
# > ## ⚠️ STOP — do not run this section unattended
# >
# > **Everything below talks to the real board.** It connects to the FeatherS3, uploads the
# > package, and ranges with the VL53L4CD on I2C bus 0 (a Class 1 laser: eye-safe, but do not
# > look into it through optics). Several steps ask you to hold your hand about 15 cm (6 in)
# > above it.
# >
# > - Do **not** use *Run All*, *Run All Below* or `jlx run` on this section and walk away.
# > - Run it **one cell at a time**. Read the **What to do** line above each cell first.
# > - Each cell stops ranging when it finishes, even if it fails.
# > - The section ends with `%board disconnect`. If you stop partway, run that cell yourself.
#
# **Hardware and configuration** (same as the table at the top):
#
# | | Host run (simulation) | Hardware run |
# |---|---|---|
# | **Runs on** | CPython 3.13 in the micropylab kernel, no board | Unexpected Maker FeatherS3 (ESP32-S3), MicroPython 1.28.0 (v1.28.0, 2026-04-06), USB `/dev/ttyACM0` |
# | **I2C bus** | `FakeBus` | `machine.I2C(0)`, **bus 0**, the STEMMA QT connector (scl=9 sda=8, on-board pull-ups), 100 kHz (`bus_open("feathers3", 0)`) |
# | **Mux** | none | none: the VL53L4CD is on bus 0 directly |
# | **Sensor** | `FakeVL53L4CD` at `0x29`: booted, a target at 123 mm, raw status 9 (valid), oscillator word `0x0BB8`, a sample 2 ms after each start or interrupt clear | VL53L4CD breakout at `0x29`; XSHUT and GPIO1 not wired |
# | **Other parts used** | none | none. Also on bus 0: SparkFun Qwiic Single Relay at `0x18` (not used here) |
# | **Datasheet** | ST DS13812 Rev 5 (June 2022), plus Adafruit's VL53L4CD library (a port of ST's ULD) for the register map | same |
# | **Result** | not run yet | not yet run on the hardware |
#
# Layout: connect and set up, find the part, then **matching tests** (one per unit test above,
# same name, run on the real part and on MicroPython; the fake's own tests are matched where the
# real part can show the same thing), hardware-only checks, **diagnostics** (they print what the
# part does and do not count towards the tally), then the tally and disconnect.

# %% [markdown]
# ## Connect and set up

# %% [markdown]
# Notebook-only: connect to the board. Only one notebook can own the board: if another notebook
# still has it, run `%board disconnect` there first.
#
# **What to do:** make sure the board is plugged in and no other notebook owns it.

# %%
# %board connect

# %% [markdown]
# Notebook-only: remove the package an earlier upload left on the board, and make sure `/lib`
# exists, so the copy below lands exactly at `/lib/micropython_i2c`.
#
# **What to do:** nothing.

# %%
import os


def rmtree(path):
    """Delete a board folder and everything in it."""
    for entry in os.ilistdir(path):
        child = path + "/" + entry[0]
        if entry[1] == 0x4000:  # folder
            rmtree(child)
        else:
            os.remove(child)
    os.rmdir(path)


try:
    rmtree("/lib/micropython_i2c")
    print("removed the old /lib/micropython_i2c")
except OSError:
    print("no /lib/micropython_i2c on the board yet")
try:
    os.mkdir("/lib")
except OSError:
    pass  # already there

# %% [markdown]
# Notebook-only: copy `src/micropython_i2c`, including the file this notebook just wrote, to
# `/lib/micropython_i2c`. The copy runs on the host; the path is relative to this notebook's folder.
#
# **What to do:** nothing. Takes a few seconds.

# %%
# %%host
from pathlib import Path

pkg = next(p for p in (Path("../../../src"), Path("src")) if p.is_dir()) / "micropython_i2c"
# %files copy -r {pkg} :/lib/micropython_i2c

# %% [markdown]
# Notebook-only: drop any modules an earlier run left on the board, so the new files load, then
# import what the checks use and zero the tally.
#
# **What to do:** nothing.

# %%
import sys
import time

for name in [m for m in sys.modules if m.startswith("micropython_i2c")]:
    del sys.modules[name]

from micropython_i2c import I2CConnection, default_lock
from micropython_i2c.boards import bus_open
from micropython_i2c.compat import sleep_ms
from micropython_i2c.debug import check, check_raises, check_reset, check_summary, info
from micropython_i2c.debug import check_counts  # noqa: F401  (read from the host with %eval)
from micropython_i2c.devices.sensors.distance.vl53l4cd import _INIT, STATUS_NAMES, VL53L4CD

check_reset()

# %% [markdown]
# Notebook-only: the bus and helpers. `rd(reg, n)` reads registers without a driver. `rest(dev)`
# stops ranging the way ST's ULD does (`0x80` to `0x0087`) and clears the interrupt, whatever the
# driver's `stop()` does. `on_board(b)` gives a check a fresh driver (its constructor loads the
# init table again). `ReplyingBus` does the real
# register read but hands back a changed reply for chosen registers, standing in for a part that
# is not a VL53L4CD, one that never boots, or a failed sample. The target is your hand, held by
# eye about 15 cm (6 in) above the sensor, so `at_hand(mm)` accepts any distance from
# `HAND_MIN_MM` to `HAND_MAX_MM`. If the bench is rewired, change the constants here.
#
# **What to do:** nothing.

# %%
TOF_BUS = 0  # STEMMA QT, scl=9 sda=8; the Qwiic Single Relay is also here, at 0x18
TOF_ADDR = 0x29
HAND_MIN_MM = 25  # your hand, held about 15 cm (6 in) above the sensor; no ruler needed
HAND_MAX_MM = 400

bus = bus_open("feathers3", TOF_BUS, freq=100_000)
bus_lock = default_lock()


class ReplyingBus:
    """The bus, but register reads from the VL53L4CD at a register in ``replies`` return
    ``replies[reg](data)`` (after doing the real read)."""

    def __init__(self, inner, replies):
        self.inner = inner
        self.replies = replies

    def __getattr__(self, name):
        return getattr(self.inner, name)

    def readfrom_mem(self, addr, reg, nbytes, addrsize=8):
        data = self.inner.readfrom_mem(addr, reg, nbytes, addrsize=addrsize)
        if addr == TOF_ADDR and reg in self.replies:
            return self.replies[reg](data)
        return data


def tof_conn(b=None):
    """A connection to the VL53L4CD on bus 0 (optionally on a wrapped bus)."""
    return I2CConnection(b or bus, TOF_ADDR, None, bus_lock)


def rd(reg, n=1):
    """``n`` bytes from register ``reg`` (16-bit index), without a driver."""
    return tof_conn().readfrom_mem(reg, n, addrsize=16)


def rest(dev=None):
    """Stop ranging (ST's ULD stop, 0x80) and clear the interrupt."""
    conn = tof_conn()
    conn.writeto_mem(0x0087, b"\x80", addrsize=16)
    conn.writeto_mem(0x0086, b"\x01", addrsize=16)
    if dev is not None:
        dev.ranging = False


def on_board(b=None):
    """A fresh ``VL53L4CD`` on the real part, with any ranging stopped first."""
    rest()
    return VL53L4CD(tof_conn(b))


def at_hand(mm):
    """True if ``mm`` is a distance to a hand held over the sensor."""
    return HAND_MIN_MM <= mm <= HAND_MAX_MM


# %% [markdown]
# ## Find the part
# Bus 0 should answer at `0x29` (the VL53L4CD) and `0x18` (the relay). If `0x29` is missing,
# check the Qwiic cable in the STEMMA QT port.
#
# **What to do:** nothing.

# %%
seen = bus.scan()
print("bus {} answers at:".format(TOF_BUS), [hex(a) for a in seen])
check("VL53L4CD ACKs at 0x29 on bus 0", TOF_ADDR in seen)
others = [a for a in seen if a != TOF_ADDR]
if others:
    info("also on bus 0: {}".format([hex(a) for a in others]))
rest()

# %% [markdown]
# ## Matching tests: the fake's datasheet facts
# The fake's unit tests that the real part can show too. `test_fake_stop_ends_the_samples` and
# `test_fake_quantities_are_read_when_sampled` have no match here: the first is the diagnostic
# "Does `stop()` stop ranging?" below, the second needs a moving target.

# %% [markdown]
# **Matches** `test_fake_reference_registers_follow_table_7`: the real part reads `EB AA`, and
# reports booted.
#
# **What to do:** nothing.

# %%
ref = rd(0x010F, 2)
fw = rd(0x00E5)[0]
print("0x010F..0x0110:", " ".join("{:02x}".format(b) for b in ref), "| want eb aa")
print("0x00E5: {:#04x} | want 0x03".format(fw))
check("test_fake_reference_registers_follow_table_7", tuple(ref) == (0xEB, 0xAA) and fw == 0x03)

# %% [markdown]
# **Matches** `test_fake_new_sample_only_after_an_interrupt_clear`: started and not cleared, the
# line stays active (ranging holds, datasheet 2.9); after a clear a new sample comes within the
# budget plus a margin.
#
# **What to do:** nothing.

# %%
tof = on_board()
try:
    tof.reg_write(0x0087, 0x21)
    t0 = time.ticks_ms()
    while not tof.data_ready() and time.ticks_diff(time.ticks_ms(), t0) < 1000:
        sleep_ms(1)
    first = tof.data_ready()
    sleep_ms(300)  # six budgets, no clear
    held = tof.data_ready()
    tof.reg_write(0x0086, 0x01)
    sleep_ms(2)
    t0 = time.ticks_ms()
    while not tof.data_ready() and time.ticks_diff(time.ticks_ms(), t0) < 1000:
        sleep_ms(1)
    took = time.ticks_diff(time.ticks_ms(), t0)
    print("first sample: {}, still ready after 300 ms: {}, next after a clear: {} ms".format(
        first, held, took))  # fmt: skip
    check("test_fake_new_sample_only_after_an_interrupt_clear",
          first and held and took < 200)  # fmt: skip
finally:
    rest(tof)

# %% [markdown]
# ## Matching tests: the init table, the statuses and the codecs

# %% [markdown]
# Notebook-only: the budget cases and the period count, copied to the board (it cannot import the
# test file, which needs pytest). The cell after it checks the copy against the notebook's.
#
# **What to do:** nothing.

# %%
BUDGET_CASES = [10, 20, 33, 50, 100, 200]


def periods(word):
    """Macro periods a range-config word stands for: ``(mantissa + 1) << exponent``."""
    return ((word & 0xFF) + 1) << (word >> 8)


# %% [markdown]
# Notebook-only: on the host, read the board's copy back with `%eval` and check it matches.
#
# **What to do:** nothing.

# %%
# %%host
if get_ipython().run_line_magic("eval", "BUDGET_CASES") != BUDGET_CASES:
    raise AssertionError("the board's copy of BUDGET_CASES differs from the notebook's")
print("board copy of BUDGET_CASES matches the notebook's")

# %% [markdown]
# **Matches** `test_init_table_covers_its_range`.
#
# **What to do:** nothing. No bus traffic.

# %%
table = _INIT
check("test_init_table_covers_its_range",
      len(table) == 91 and table[0x30 - 0x2D] & 0x10 and table[0x87 - 0x2D] == 0
      and table[0x86 - 0x2D] == 0)  # fmt: skip

# %% [markdown]
# **Matches** `test_only_status_9_is_valid`.
#
# **What to do:** nothing. No bus traffic.

# %%
check("test_only_status_9_is_valid",
      [s for s, name in STATUS_NAMES.items() if name == "valid"] == [9])  # fmt: skip

# %% [markdown]
# **Matches** `test_budget_words_fit_registers` and `test_budget_words_grow_with_the_budget`, on
# MicroPython, at the real part's oscillator word.
#
# **What to do:** nothing.

# %%
osc = int.from_bytes(rd(0x0006, 2), "big")
print("oscillator word: {:#06x}".format(osc))
words = [VL53L4CD.range_config_words(ms, osc) for ms in BUDGET_CASES]
check("test_budget_words_fit_registers",
      all(0 <= w <= 0xFFFF for pair in words for w in pair))  # fmt: skip
ok = all(periods(b) > periods(a) for a, b in words)
for i in (0, 1):
    counts = [periods(w[i]) for w in words]
    ok = ok and counts == sorted(counts) and counts[0] < counts[-1]
check("test_budget_words_grow_with_the_budget", ok)

# %% [markdown]
# ## Matching tests: `VL53L4CD`
# The driver on the real part. The fake reports exact distances; the real part measures your
# hand, so these accept any valid reading from 25 to 400 mm (see `at_hand` in the setup cell).

# %% [markdown]
# **Matches** `test_init_loads_the_table_and_sets_50_ms`: the part is a VL53L4CD, ranging is
# stopped, the first table byte reads back, and the budget words are the codec's for 50 ms. The
# cell also prints which registers of `0x002D..0x0085` read back differently from the table
# (expected: the live line `0x0031` and the budget words at `0x005E..0x0062`; anything else is
# information, not a failure).
#
# **What to do:** nothing.

# %%
t0 = time.ticks_ms()
tof = on_board()
took = time.ticks_diff(time.ticks_ms(), t0)
try:
    back = tof.read_mem(0x002D, 0x0086 - 0x002D)
    diff = [hex(0x2D + i) for i, (a, b) in enumerate(zip(_INIT, back)) if a != b]
    words = [tof.reg_read(r, 2) for r in (0x005E, 0x0061)]
    want = VL53L4CD.range_config_words(50, tof.reg_read(0x0006, 2))
    print("constructed in {} ms; registers that differ from the table: {}".format(took, diff))
    print("budget words {} | want {}".format([hex(w) for w in words], [hex(w) for w in want]))
    check("test_init_loads_the_table_and_sets_50_ms",
          tof.timing_budget == 50 and not tof.ranging and tof.model_id() == 0xEBAA
          and back[0] == 0x12 and words == want)  # fmt: skip
finally:
    rest(tof)

# %% [markdown]
# **Matches** `test_reads_distance_when_started`.
#
# **What to do:** hold your hand flat, palm down, about 15 cm (6 in) above the sensor until the
# cell finishes.

# %%
tof = on_board()
try:
    before = tof.status_name
    tof.start()
    mm = tof.read()
    tof.stop()
    print("{} mm, {} (status {})".format(mm, tof.status_name, tof.status))
    check("test_reads_distance_when_started",
          before == "no reading yet" and tof.valid and tof.status_name == "valid"
          and at_hand(mm), "{} mm".format(mm))  # fmt: skip
finally:
    rest(tof)

# %% [markdown]
# **Matches** `test_each_read_is_a_new_sample`: five reads, each waiting for its own sample, so
# five reads take about five budgets (50 ms each).
#
# **What to do:** hold your hand flat, palm down, about 15 cm (6 in) above the sensor until the
# cell finishes.

# %%
tof = on_board()
try:
    tof.start()
    t0 = time.ticks_ms()
    mms = [tof.read() for _ in range(5)]
    took = time.ticks_diff(time.ticks_ms(), t0)
    tof.stop()
    print("{} mm in {} ms".format(mms, took))
    check("test_each_read_is_a_new_sample", took >= 4 * 40 and not tof.ranging,
          "{} ms for 5 reads".format(took))  # fmt: skip
finally:
    rest(tof)

# %% [markdown]
# **Matches** `test_timing_budget_rules`.
#
# **What to do:** nothing.

# %%
tof = on_board()
try:
    tof.start()
    check_raises("test_timing_budget_rules: refused while ranging", RuntimeError,
                 setattr, tof, "timing_budget", 100)  # fmt: skip
    tof.stop()
    check_raises("test_timing_budget_rules: 9 ms refused", ValueError,
                 setattr, tof, "timing_budget", 9)  # fmt: skip
    check_raises("test_timing_budget_rules: 201 ms refused", ValueError,
                 setattr, tof, "timing_budget", 201)  # fmt: skip
    tof.timing_budget = 100
    check("test_timing_budget_rules: 100 ms set", tof.timing_budget == 100)
finally:
    rest(tof)

# %% [markdown]
# **Matches** `test_init_refuses_another_model`. `ReplyingBus` swaps the model ID for a
# VL53L1X's (`0xEACC`) on the way in; the real part must be fine afterwards.
#
# **What to do:** nothing.

# %%
fake_id = {0x010F: lambda data: b"\xea\xcc"[: len(data)]}
check_raises("test_init_refuses_another_model", RuntimeError,
             on_board, ReplyingBus(bus, fake_id))  # fmt: skip
tof = on_board()
check("the real part is still a VL53L4CD", tof.model_id() == 0xEBAA)
rest(tof)

# %% [markdown]
# **Matches** `test_init_times_out_without_boot`. `ReplyingBus` makes the firmware status read 0.
#
# **What to do:** nothing. Takes 1 to 2 s (1000 polls).

# %%
unbooted = {0x00E5: lambda data: bytes(len(data))}
check_raises("test_init_times_out_without_boot", OSError,
             on_board, ReplyingBus(bus, unbooted))  # fmt: skip

# %% [markdown]
# **Matches** `test_oscillator_zero_is_refused`. `ReplyingBus` makes the oscillator word read 0.
#
# **What to do:** nothing.

# %%
no_osc = {0x0006: lambda data: bytes(len(data))}
try:
    check_raises("test_oscillator_zero_is_refused", OSError,
                 on_board, ReplyingBus(bus, no_osc))  # fmt: skip
finally:
    rest()

# %% [markdown]
# **Matches** `test_invalid_status_is_flagged`. A real failed sample needs a target the part
# cannot see; here `ReplyingBus` sets the status bits of every real sample to 4.
#
# **What to do:** nothing.

# %%
failed = {0x0089: lambda data: bytes([(data[0] & 0xE0) | 4]) + data[1:]}
tof = on_board(ReplyingBus(bus, failed))
try:
    tof.start()
    tof.read()
    flagged = not tof.valid and tof.status_name == "sigma warning"
    check("test_invalid_status_is_flagged",
          flagged and tof.read_valid(tries=3) is None)  # fmt: skip
finally:
    rest(tof)

# %% [markdown]
# **Matches** `test_read_times_out_when_not_started`. The part is stopped with the ULD's stop and
# the interrupt cleared first, so no sample is waiting.
#
# **What to do:** nothing. Takes 0.3 s.

# %%
tof = on_board()
try:
    rest(tof)
    sleep_ms(100)
    check_raises("test_read_times_out_when_not_started", OSError, tof.read, 300)
finally:
    rest(tof)

# %% [markdown]
# `test_follows_a_hand_approaching_in_simulated_time` and
# `test_reads_come_one_sample_apart_in_simulated_time` have no match here: the first needs a
# moving target, the second is the hardware-only timing check below.

# %% [markdown]
# ## Hardware-only checks

# %% [markdown]
# The sample period at each budget: twenty back-to-back reads per budget. Each read should take
# about one budget (plus the 2 ms the driver sleeps after a clear, and the bus time).
#
# **What to do:** hold your hand flat, palm down, about 15 cm (6 in) above the sensor until the
# cell finishes. Takes about 7 s.

# %%
tof = on_board()
try:
    for ms in (20, 50, 100, 200):
        tof.timing_budget = ms
        tof.start()
        t0 = time.ticks_ms()
        mms = [tof.read() for _ in range(20)]
        per = time.ticks_diff(time.ticks_ms(), t0) / 20
        tof.stop()
        print("{:3} ms budget: {:6.1f} ms per read, {} .. {} mm".format(ms, per, min(mms),
                                                                        max(mms)))  # fmt: skip
        check("{} ms budget: one read per budget".format(ms), 0.8 * ms <= per <= 1.5 * ms + 10,
              "{:.1f} ms".format(per))  # fmt: skip
finally:
    rest(tof)

# %% [markdown]
# ## Diagnostics
# They print what the part does; they do not count towards the tally.

# %% [markdown]
# ### Does `stop()` stop ranging?
# The driver's `stop()` writes `0x00` to `0x0087`; ST's ULD and the Adafruit library write
# `0x80`. For each value: start, read one sample, write the value, clear the interrupt, wait
# 300 ms, and see whether a new sample arrived. "stopped" for both means the driver is fine as it
# is.
#
# **What to do:** nothing.

# %%
tof = on_board()
try:
    for value in (0x00, 0x80):
        tof.start()
        tof.read()
        tof.reg_write(0x0087, value)
        tof.reg_write(0x0086, 0x01)
        sleep_ms(300)
        print("0x0087 = {:#04x}: {}".format(
            value, "still ranging" if tof.data_ready() else "stopped"))  # fmt: skip
        rest(tof)
finally:
    rest(tof)

# %% [markdown]
# ## Tally and disconnect

# %% [markdown]
# Notebook-only: the PASS/FAIL tally of every check in this section.
#
# **What to do:** read it. Any `FAIL` line above says which check and why.

# %%
check_summary()

# %% [markdown]
# Notebook-only: copy the tally to the host before disconnecting, so the last cell can fail
# the run after the board is released.
#
# **What to do:** nothing.

# %%
# %%host
hw_passed, hw_failed = get_ipython().run_line_magic("eval", "check_counts()")

# %% [markdown]
# Notebook-only: release the board, so the next run (or a kernel restart) starts on the host.
#
# **What to do:** nothing.

# %%
# %board disconnect

# %% [markdown]
# Notebook-only: fail the run if any hardware check failed. It runs on the host, so it works
# whether or not the board is still attached.
#
# **What to do:** nothing. An error here means a `FAIL` line above.

# %%
# %%host
if hw_failed:
    raise AssertionError("{} of {} hardware checks failed".format(hw_failed, hw_passed + hw_failed))
print("all {} hardware checks passed".format(hw_passed))
