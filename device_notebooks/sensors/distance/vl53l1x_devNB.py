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
# > VL53L1X's laser (Class 1, eye-safe), changes its I2C address and resets it, and has steps
# > where you hold your hand over it. *Run All* and `jlx run` reach that section.
# >
# > - **Safe to run on its own:** everything above `# Hardware validation`. It runs on the host
# >   against a fake part, writes the module and its tests, and needs no board.
# > - **Hardware section:** run it one cell at a time, reading the **What to do** line above
# >   each cell first.
#
# **Where each cell runs.** Everything above `%board connect` (the first cells of
# `# Hardware validation`) runs in ordinary Python (CPython) on this computer, with no board: it
# writes the fake part (`vl53l1x_fake.py`), the driver (`vl53l1x.py`) and their tests, and checks
# the driver against the fake, in real and in simulated time. After `%board connect`, cells run
# in MicroPython on the board, except `%%host` cells, which run on the computer. That section
# uploads the freshly written package, checks the real VL53L1X behind mux channel 7 with a
# matching test for every unit test above, then disconnects.
#
# micropylab has `%%extract` built in, so there is no `%load_ext jupyter_lab_extractor` cell.

# %% [markdown]
# # Hardware and configuration
# What this notebook is validated with. The same table opens `# Hardware validation`.
#
# | | Host run (simulation) | Hardware run |
# |---|---|---|
# | **Runs on** | CPython 3.13.15 in the micropylab kernel (1.0.0a1), no board | Unexpected Maker FeatherS3 (ESP32-S3), MicroPython 1.28.0 (v1.28.0, 2026-04-06), USB `/dev/ttyACM0` |
# | **I2C bus** | `FakeBus` | `machine.I2C(1)`, **bus 1**, scl=15 sda=16, 100 kHz (`bus_open("feathers3", 1)`) |
# | **Mux** | none in the unit tests: the fake part is on the bus directly | SparkFun Qwiic Mux (TCA9548A) at `0x70` (address jumpers open), **channel 7** |
# | **Sensor** | `FakeVL53L1X` at `0x29`: booted, a target at 500 mm, raw status 9 (valid), oscillator word `0x0025` (as measured), a sample 2 ms after each start or interrupt clear | Adafruit VL53L1X breakout (STEMMA QT) at `0x29`, protective film removed; XSHUT and GPIO1 not wired |
# | **Other parts used** | none | none. On the same mux: SparkFun TMAG5273 (B1, `0x22`) on channel 0 and TMP102 (`0x48`) on channel 2, off while the VL53L1X is used |
# | **Datasheet** | ST DS12385 rev 8 (August 2024), plus ST's ULD driver source for the register map | same |
# | **Result** | not run yet | not run yet |
#
# The simulation leaves the mux out (it has its own notebook, `tca9548a_devNB`); the
# hardware run reaches the VL53L1X through it. The fake makes samples in 2 ms, not in the timing
# budget, so the unit tests stay fast; the hardware section checks the real timing.

# %% [markdown]
# # TODO:
# - Rerun the hardware section with the hand checks as they are now, and fill in the results.
#
# Measured on the bench (2026-11-14, two runs). The `## Diagnostics` cells print each of these
# again:
# - An interrupted soft reset (`0x0000` written `0x00`, never `0x01`) holds the part: it still
#   ACKs at `0x29`, but every register reads `00`, so the boot flag never sets. The driver now
#   releases a held reset before it waits for the boot. After a release it booted in 1.15 to
#   1.20 ms (the datasheet's maximum is 1.2 ms).
# - Asserting the reset while the part was at `0x30` moved it back to `0x29` at once (still held
#   there until released): a reset loses a programmed address, so the driver's `reset()` refuses
#   away from `0x29`.
# - Oscillator word `0x0025` (37). The real period was 98.6-98.7 ms for 100 ms and
#   196.4-197.3 ms for 200 ms: a factor of about 1.09 would be exact; ST's 1.075 (kept) is about
#   1.5% short.
# - After the constructor, only the two timing words differ from the default configuration
#   (the constructor sets the budget). Everything else reads back as written.
# - The interrupt line reads inactive at once after a clear (no settling wait needed), and the
#   first samples after a start were all valid.
# - Spread on a hand: 3.5-4.0 mm at 33 ms, 0.9-1.0 mm at 200 ms.
# - The ULD-only budgets (15 ms short, 20 ms long) gave valid samples; the driver still offers
#   only what the datasheet allows.
# - The Adafruit board came with a protective film on the sensor. With it on, a hand held
#   about 15 cm away read 60-75 mm and did not follow the hand. With it off, the reading
#   followed the hand (113 mm down to 33 mm). The ceiling reads about 1.8 m either way.
# - A hand held by eye "at 15 cm" read 90-115 mm and drifted closer while held, so the hardware
#   checks accept any valid reading from 25 to 400 mm, and check accuracy relatively: a +50 mm
#   offset read +49.8 mm, short and long mode agreed within 1-2 mm.
# - Close up, the central 4x4 ROI reads 25-30 mm shorter than the full 16x16 (33 against 60 mm,
#   60 against 92 mm): the centre sees the nearest part of the hand.
#
# Decided (2026-11-14): the datasheet (DS12385 rev 8, `docs/datasheets/vl53l1x.pdf`) gives the
# I2C protocol, the three reference registers (Table 8), the boot time, the timing-budget and
# distance-mode limits and the ranging performance, but **no register map**: it sends the reader
# to ST's driver. The register addresses, the 91-byte default configuration and the sequences
# here come from ST's ULD source (`vl53l1x_class.cpp`, BSD-3-Clause, in SparkFun's VL53L1X
# Arduino library 1.2.12). Where the two disagree the datasheet wins:
# - The ULD offers a 15 ms budget in short mode and 20 ms in long mode. The datasheet says
#   20 ms is the minimum and short mode only, and 33 ms is the minimum for every mode. The
#   driver offers only what the datasheet allows; a diagnostic tries the ULD-only values.
# - The ULD writes the inter-measurement period with a factor of 1.075 and reads it back with
#   1.065. The driver uses 1.075 both ways, so a setting reads back as written; a diagnostic
#   times the real period.
#
# Not in the ULD source used here: the soft reset sequence (write `0x00` then `0x01` to
# `0x0000`, 100 µs apart) is from ST's full VL53L1 API.
#
# Decided (2026-11-16): `FakeVL53L1X` moved out of `micropython_i2c.fakes` into this notebook
# (`vl53l1x_fake.py`, next to the driver). Its distance, signal rate and ambient rate are plain
# values or signals (`micropython_i2c.sim.signals`), read when the fake makes a sample, and its
# timing runs on the `compat` clock, so `SimClock` drives it.

# %% [markdown]
# # Python file
# location: `src/micropython_i2c/devices/sensors/distance/vl53l1x.py` (driver) and
# `src/micropython_i2c/devices/sensors/distance/vl53l1x_fake.py` (fake part, host only)
#
# tests: `tests/test_vl53l1x.py` (both)

# %% [markdown]
# ## header

# %%
# %%extract ../../../src/micropython_i2c/devices/sensors/distance/vl53l1x.py
"""ST VL53L1X time-of-flight distance sensor (datasheet DS12385 rev 8, August 2024).

Usage on the board::

    from micropython_i2c import I2CConnection
    from micropython_i2c.devices.sensors.distance.vl53l1x import VL53L1X

    tof = VL53L1X(I2CConnection(bus, 0x29, lock=bus_lock))   # long mode, 100 ms
    print(tof.measure())             # one-shot -> <VL53L1XResult 243 mm valid ...>
    tof.start()
    r = tof.read()                   # waits for the next sample
    if r.valid:
        print(r.distance_mm)
    tof.stop()

Three classes:

* ``VL53L1XRegs``: facts only (reference registers, register map, default configuration,
  timing-budget and distance-mode tables, range statuses), no methods.
* ``VL53L1XResult``: one ranging result, decoded from the 17-byte result block.
* ``VL53L1X``: the driver.

The datasheet publishes no register map. The register addresses, the default configuration
and the sequences come from ST's VL53L1X ULD driver (``vl53l1x_class.cpp``), whose licence
follows. Where the ULD and the datasheet disagree (the shortest timing budgets), the driver
follows the datasheet.

    COPYRIGHT(c) 2018 STMicroelectronics

    Redistribution and use in source and binary forms, with or without modification, are
    permitted provided that the following conditions are met:
      1. Redistributions of source code must retain the above copyright notice, this list
         of conditions and the following disclaimer.
      2. Redistributions in binary form must reproduce the above copyright notice, this
         list of conditions and the following disclaimer in the documentation and/or other
         materials provided with the distribution.
      3. Neither the name of STMicroelectronics nor the names of its contributors may be
         used to endorse or promote products derived from this software without specific
         prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
    EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
    MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
    THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
    SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
    PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
    INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
    LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
    OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

Generated by ``device_notebooks/sensors/distance/vl53l1x_devNB``: edit the notebook, not this file.
"""

# %% [markdown]
# # Imports
# ## External
# None: the driver runs on MicroPython and uses only this package.
#
# ## Internal
# `RegisterI2CBase` for the 16-bit register map, `I2CConnection` to rebind after an address
# change, and the timing helpers from `compat`.

# %%
# %%extract ../../../src/micropython_i2c/devices/sensors/distance/vl53l1x.py -a
from micropython_i2c.compat import sleep_ms, sleep_us, ticks_diff, ticks_ms
from micropython_i2c.connection import I2CConnection
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
# %%writefile ../../../tests/test_vl53l1x.py
import pytest

from micropython_i2c import I2CConnection
from micropython_i2c.devices.sensors.distance.vl53l1x import VL53L1X, VL53L1XRegs, VL53L1XResult
from micropython_i2c.devices.sensors.distance.vl53l1x_fake import FakeVL53L1X
from micropython_i2c.devices.sensors.distance.vl53l4cd_fake import FakeVL53L4CD
from micropython_i2c.sim.bus import FakeBus
from micropython_i2c.sim.clock import SimClock
from micropython_i2c.sim.signals import ramp

# %% [markdown]
# Notebook-only: the imports the validation and test cells use. The classes under test
# (`FakeVL53L1X`, `VL53L1XRegs`, `VL53L1XResult`, `VL53L1X`) are the ones defined in this
# notebook, not the modules'. `FakeVL53L4CD` (a part with another model ID) comes from
# `vl53l4cd_devNB`.

# %%
import pytest

from micropython_i2c import I2CConnection
from micropython_i2c.devices.sensors.distance.vl53l4cd_fake import FakeVL53L4CD
from micropython_i2c.sim.bus import FakeBus
from micropython_i2c.sim.clock import SimClock
from micropython_i2c.sim.signals import ramp

# %% [markdown]
# # Intent
# **Purpose:** measure distance in millimetres with an ST VL53L1X time-of-flight sensor, up to
# about 4 m, continuously or one shot at a time, with the distance mode, timing budget,
# inter-measurement period, region of interest and offset under control.
#
# **Why it exists:** the VL53L1X has no documented register map. It is driven by loading a
# 91-byte configuration blob and writing magic timing words from ST's tables, and a reading is
# only as good as its range status: the distance register always holds a number, valid or not.
# The driver keeps the tables in one place, refuses settings the datasheet does not allow, and
# returns the status with every distance.
#
# **Used by:** the routing table and manager (`"driver": "VL53L1X"`) and the bench (Adafruit
# VL53L1X breakout behind mux channel 7 on I2C bus 1).
#
# **Not its job:** reaching the part (the `I2CConnection` does addressing, mux routing and
# locking), the XSHUT and GPIO1 pins (not wired on the bench: the driver polls the interrupt
# status register), distance thresholds and window interrupts (they only matter with GPIO1
# wired), offset and crosstalk *calibration* (they need a target at a known distance; the
# driver sets and reads the values).
#
# ```python
# tof = VL53L1X(I2CConnection(bus, 0x29, path, lock))  # boots, loads, long mode, 100 ms
# tof.measure()                       # one-shot: <VL53L1XResult 243 mm valid ...>
# tof.distance_mode = "short"         # up to ~1.3 m, better in bright light
# tof.timing_budget_ms = 33           # 20..500 ms, from ST's table for the mode
# tof.inter_measurement_ms = 50       # continuous mode: one sample every 50 ms
# tof.start(); tof.read(); tof.stop()
# ```
#
# ```mermaid
# stateDiagram-v2
#     [*] --> Boot: power (XSHUT pulled up on the breakout)
#     Boot --> Standby: firmware booted, 0x00E5 bit 0 = 1, at most 1.2 ms
#     Standby --> Ranging: 0x0087 = 0x40 (continuous) or 0x10 (one shot)
#     Ranging --> Ready: sample done, interrupt line active (0x0031 bit 0)
#     Ready --> Ranging: clear interrupt 0x0086 = 1 (continuous)
#     Ready --> Standby: one shot done, or 0x0087 = 0x00
# ```
#
# Each sample takes the **timing budget**; in continuous mode a new one starts every
# **inter-measurement period**, which must be at least the timing budget. A new sample is only
# made after the interrupt is cleared (datasheet 3.4).
#
# Limits from the datasheet:
#
# | | Short mode | Long mode |
# |---|---|---|
# | Max distance, dark (Table 5) | 136 cm | 360 cm (400 cm at 140 ms) |
# | Max distance, strong ambient light | 135 cm | 73 cm |
# | Shortest timing budget (3.5.2) | 20 ms | 33 ms |
#
# Minimum distance 4 cm (7.3), ranging error ±20 mm in the dark (Table 17), ROI 4×4 to 16×16
# SPADs (3.4), I2C up to 400 kHz with 16-bit register addresses, MSB first (section 4).
#
# ## What the module defines
#
# | Name | What it is |
# |---|---|
# | `VL53L1XRegs` | facts: reference registers, register map (from the ULD), default configuration, distance-mode and timing-budget tables, range statuses |
# | `VL53L1XResult` | one ranging result: distance, range status (code, name, `valid`), signal and ambient rates, SPAD count |
# | `VL53L1X` | the driver: boot and identify on construction, continuous and one-shot ranging, distance mode, timing budget, inter-measurement period, ROI, offset, crosstalk, temperature update, soft reset, address change, codecs |
#
# The fake part, `FakeVL53L1X`, is in its own module, `vl53l1x_fake.py` (host only): see
# `# Fake part`.

# %% [markdown]
# # Fake part
# `FakeVL53L1X` stands in for the real part on a `FakeBus`, so the driver can be written and
# tested on the host. It is written before the driver because the driver's tests need it. It
# goes to its own module next to the driver, `vl53l1x_fake.py`, which the board never imports.

# %% [markdown]
# ## header

# %%
# %%extract ../../../src/micropython_i2c/devices/sensors/distance/vl53l1x_fake.py
"""``FakeVL53L1X``: an ST VL53L1X register file and ranging sequencer, for host tests.

Usage on the host::

    from micropython_i2c.devices.sensors.distance.vl53l1x_fake import FakeVL53L1X
    from micropython_i2c.sim.bus import FakeBus
    from micropython_i2c.sim.signals import ramp

    bus = FakeBus()
    part = bus.attach(0x29, FakeVL53L1X(distance_mm=ramp(1200, 100, 0, 60)))

The distance, signal rate and ambient rate are plain values or signals
(``micropython_i2c.sim.signals``), read when the fake makes a sample. Timing goes through
``micropython_i2c.compat``, so ``SimClock`` drives it. The address is programmable
(``TAKES_ADDR``): the fake answers at whatever ``0x0001`` holds.

Host only.

Generated by ``device_notebooks/sensors/distance/vl53l1x_devNB``: edit the notebook, not this
file.
"""

# %% [markdown]
# ## Imports
# `BitTools.sign_extend` for the offset register, the `compat` ticks for the sample timing, and
# `value` to read a quantity that may be a signal.

# %%
# %%extract ../../../src/micropython_i2c/devices/sensors/distance/vl53l1x_fake.py -a
from micropython_i2c.bits import BitTools
from micropython_i2c.compat import ticks_add, ticks_diff, ticks_ms
from micropython_i2c.sim.signals import value

# %% [markdown]
# ## Public names

# %%
# %%extract ../../../src/micropython_i2c/devices/sensors/distance/vl53l1x_fake.py -a
__all__ = ["FakeVL53L1X"]

# %% [markdown]
# ## `FakeVL53L1X`
# A 64 KiB register file with the ranging sequencer around it. It does not model the optics:
# the distance and the result fields are attributes. What it models, and where each behaviour
# comes from:
#
# | Behaviour | Source |
# |---|---|
# | 16-bit register indexes, multi-byte values MSB first | datasheet section 4, Table 9 |
# | `0x010F..0x0111` read `EA CC 10`, and ignore writes | datasheet Table 8 |
# | `0x00E5` bit 0 is "booted"; with `booted=False` it never sets | ULD `BootState`; datasheet 3.6 (boot within 1.2 ms) |
# | `0x00DE` holds the oscillator word, `0x0025` by default | ULD `SetInterMeasurementInMs`; the value measured on the bench, 2026-11-14, `vl53l1x_devNB` diagnostics |
# | `0x0087` = `0x40` ranges continuously, `0x10` once (then back to standby), `0x00` stops | ULD `StartRanging`, `StopRanging` |
# | a sample is ready `sample_ms` after a start, or after an interrupt clear while ranging; no new sample before a clear | datasheet 3.4; `sample_ms` (default 2 ms) is the fake's, not the timing budget, so tests stay fast |
# | the interrupt line is `0x0031` bit 0; `0x0030` bit 4 set makes it active low | ULD `GetInterruptPolarity`, `CheckForDataReady` |
# | a sample fills the 17-byte block at `0x0089`: raw status, SPAD count (`0x008C`), ambient (`0x0090`) and signal (`0x0098`) rates in units of 8 kcps, and the distance (`0x0096`) | ULD `GetResult` |
# | the distance includes the part-to-part offset in `0x001E` (signed 11.2 in 13 bits) | ULD `SetOffset` |
# | `0x0001` is the 7-bit address; the part answers there at once | ULD `SetI2CAddress` |
# | soft reset: `0x0000` written 0 holds the part at its power-on address, every register reads 0; written 1, the power-on registers come back and it boots | the reset sequence: ST's full VL53L1 API; the held state and the move back to `0x29` at once: measured on the bench, 2026-11-14, `vl53l1x_devNB` diagnostics |
#
# Not modelled: the optics (crosstalk, ROI, distance mode and budget do not change the result),
# the timing budget and inter-measurement period (a sample takes `sample_ms`), temperature,
# GPIO1 and XSHUT.
#
# `samples` counts the samples made and `writes` logs `(reg, data)` for every write, so a test
# can check what the driver did.


# %%
# %%extract ../../../src/micropython_i2c/devices/sensors/distance/vl53l1x_fake.py -a
class FakeVL53L1X:
    """An ST VL53L1X register file and ranging sequencer (datasheet DS12385 rev 8, ST's ULD).

    Args:
        distance_mm: Distance to the target, mm: a number or a signal ``f(t_s)``.
        status: Raw range status every sample reports (9 = valid).
        signal_kcps: Return signal rate, kcps: a number or a signal.
        ambient_kcps: Ambient rate, kcps: a number or a signal.
        spads: SPAD count every sample reports.
        osc: The oscillator calibration word at ``0x00DE``.
        sample_ms: Time from a start (or an interrupt clear) to the sample.
        addr: The power-on address (``0x0001`` after power-on and reset).
        booted: False: the firmware never boots.

    Attributes:
        regs: The register file (``bytearray``, 64 KiB).
        mode: The last mode-start value (``0x40``, ``0x10`` or ``0x00``).
        ready: True while a sample waits for an interrupt clear.
        in_reset: True while the soft reset is held.
        samples: How many samples the fake has made.
        writes: ``(reg, data)`` for every register write, in order.
    """

    TAKES_ADDR = True  # the address is programmable (0x0001)

    ID_REGS = {0x010F: 0xEA, 0x0110: 0xCC, 0x0111: 0x10}
    READ_ONLY = set(range(0x0089, 0x009A)) | {0x0031, 0x00DE, 0x00DF, 0x00E5} | set(ID_REGS)

    def __init__(self, distance_mm=500, status=9, signal_kcps=8000, ambient_kcps=400, spads=40,
                 osc=0x0025, sample_ms=2, addr=0x29, booted=True):  # fmt: skip
        self.distance_mm = distance_mm
        self.status = status
        self.signal_kcps = signal_kcps
        self.ambient_kcps = ambient_kcps
        self.spads = spads
        self.osc = osc
        self.sample_ms = sample_ms
        self.addr_por = addr
        self.can_boot = booted
        self.samples = 0
        self.writes = []
        self._power_on(booted)

    def _power_on(self, booted):
        self.regs = bytearray(0x10000)
        self.in_reset = False
        self.regs[0x0000] = 0x01  # soft reset released
        for reg, val in self.ID_REGS.items():
            self.regs[reg] = val
        self.regs[0x0001] = self.addr_por
        self.regs[0x00DE], self.regs[0x00DF] = self.osc >> 8, self.osc & 0xFF
        self.regs[0x00E5] = 1 if booted else 0
        self.mode = 0x00
        self.ready = False
        self.due = None  # ticks_ms when the next sample is ready (None: none coming)

    @property
    def bus_addr(self):
        """The address the part answers at (``0x0001``)."""
        return self.regs[0x0001] & 0x7F

    def _tick(self):
        """Make the pending sample if its time has come."""
        if self.due is not None and not self.ready and ticks_diff(ticks_ms(), self.due) >= 0:
            self._sample()
            self.due = None
            if self.mode == 0x10:  # one-shot: back to standby after its sample
                self.mode = 0x00

    @staticmethod
    def _rate(kcps):
        """A rate in kcps as the 16-bit register word (units of 8 kcps)."""
        return max(0, min(0xFFFF, int(kcps) // 8))

    def _sample(self):
        """Fill the result block, reading the quantities (signals) now."""
        raw = (self.regs[0x001E] << 8) | self.regs[0x001F]
        offset = BitTools.sign_extend(raw, 13) / 4
        mm = max(0, min(0xFFFF, int(value(self.distance_mm) + offset)))
        amb, sig = self._rate(value(self.ambient_kcps)), self._rate(value(self.signal_kcps))
        block = bytearray(17)
        block[0] = self.status & 0x1F
        block[3] = self.spads & 0xFF  # 0x008C: SPAD count, integer part
        block[7], block[8] = amb >> 8, amb & 0xFF  # 0x0090
        block[13], block[14] = mm >> 8, mm & 0xFF  # 0x0096
        block[15], block[16] = sig >> 8, sig & 0xFF  # 0x0098
        self.regs[0x0089:0x009A] = block
        self.ready = True
        self.samples += 1

    def _line(self):
        active_low = bool(self.regs[0x0030] & 0x10)
        return int(self.ready) ^ int(active_low)

    def _schedule(self):
        self.due = ticks_add(ticks_ms(), self.sample_ms)

    def mem_read(self, reg, nbytes):
        self._tick()
        out = bytearray()
        for i in range(nbytes):
            r = (reg + i) & 0xFFFF
            if self.in_reset:
                out.append(0)
            else:
                out.append(self._line() if r == 0x0031 else self.regs[r])
        return bytes(out)

    def mem_write(self, reg, data):
        self._tick()
        self.writes.append((reg, bytes(data)))
        for i, val in enumerate(data):
            r = (reg + i) & 0xFFFF
            if r == 0x0000:
                if val & 1 == 0:  # reset asserted: held, every register reads 0
                    self.mode, self.ready, self.due = 0x00, False, None
                    self.in_reset = True
                    self.regs[0x0001] = self.addr_por  # back at once (measured)
                elif self.in_reset:  # released: power-on registers, then boot
                    self._power_on(self.can_boot)
                continue
            if self.in_reset or r in self.READ_ONLY:
                continue
            self.regs[r] = val
            if r == 0x0086 and val & 1:  # clear the interrupt
                self.ready = False
                if self.mode == 0x40:
                    self._schedule()
            elif r == 0x0087:
                self.mode = val
                if val in (0x40, 0x10):
                    self._schedule()
                else:
                    self.due = None


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
# The reference registers (Table 8), then one continuous sample in simulated time with the
# target on a `ramp` from 1000 to 200 mm over 10 s: the sample made at 5 s reads about 600 mm.

# %%
with SimClock() as clock:
    bus = FakeBus()
    part = bus.attach(0x29, FakeVL53L1X(distance_mm=ramp(1000, 200, 0, 10)))
    print("0x010F..0x0111:", rd(bus, 0x010F, 3).hex(" "), "| want ea cc 10")
    clock.advance(5)
    wr(bus, 0x0087, [0x40])  # start continuous ranging
    clock.advance(0.002)
    print("interrupt line at 5.002 s:", rd(bus, 0x0031)[0], "(1 = sample ready, active high)")
    block = rd(bus, 0x0089, 17)
    print("raw status {}, distance {} mm (want about 600)".format(
        block[0], (block[13] << 8) | block[14]))  # fmt: skip

# %% [markdown]
# #### validation: anti-nominal usage
# After the address register is set to `0x30`, the part no longer answers at `0x29`: the bus
# refuses the transfer, as a real NACK would.

# %%
bus = FakeBus()
part = bus.attach(0x29, FakeVL53L1X())
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
# The reference registers read the Table 8 values and ignore writes.

# %%
# %%extract ../../../tests/test_vl53l1x.py -a --strip-ipytest
# %%ipytest


def test_fake_reference_registers_follow_table_8():
    bus = FakeBus()
    bus.attach(0x29, FakeVL53L1X())
    bus.writeto_mem(0x29, 0x010F, b"\x00\x00\x00", addrsize=16)
    assert bus.readfrom_mem(0x29, 0x010F, 3, addrsize=16) == b"\xea\xcc\x10"


# %% [markdown]
# A sample is made `sample_ms` after a start, and the next one only after the interrupt is
# cleared (datasheet 3.4), however long the host waits.

# %%
# %%extract ../../../tests/test_vl53l1x.py -a --strip-ipytest
# %%ipytest


def test_fake_new_sample_only_after_an_interrupt_clear():
    with SimClock() as clock:
        bus = FakeBus()
        part = bus.attach(0x29, FakeVL53L1X(sample_ms=50))
        bus.writeto_mem(0x29, 0x0087, b"\x40", addrsize=16)
        clock.advance(0.049)
        assert bus.readfrom_mem(0x29, 0x0031, 1, addrsize=16) == b"\x00"
        clock.advance(10)
        assert bus.readfrom_mem(0x29, 0x0031, 1, addrsize=16) == b"\x01" and part.samples == 1
        bus.writeto_mem(0x29, 0x0086, b"\x01", addrsize=16)
        clock.advance(0.050)
        assert bus.readfrom_mem(0x29, 0x0031, 1, addrsize=16) == b"\x01" and part.samples == 2


# %% [markdown]
# The distance, signal and ambient rates are read when the sample is made: a signal gives the
# value at that moment, and the rates land in units of 8 kcps.

# %%
# %%extract ../../../tests/test_vl53l1x.py -a --strip-ipytest
# %%ipytest


def test_fake_quantities_are_read_when_sampled():
    with SimClock() as clock:
        bus = FakeBus()
        part = FakeVL53L1X(distance_mm=ramp(1000, 0, 0, 10), signal_kcps=lambda t: 800 * t,
                           ambient_kcps=80, sample_ms=1)  # fmt: skip
        bus.attach(0x29, part)
        clock.advance(2.5)
        bus.writeto_mem(0x29, 0x0087, b"\x10", addrsize=16)  # one shot
        clock.advance(0.5)
        block = bus.readfrom_mem(0x29, 0x0089, 17, addrsize=16)
        assert (block[13] << 8) | block[14] == 700  # at 3.0 s: made at the first read after due
        assert ((block[15] << 8) | block[16], (block[7] << 8) | block[8]) == (300, 10)


# %% [markdown]
# Asserting the soft reset (`0x0000` = 0) moves the part back to its power-on address at once
# and makes every register read 0 (both measured); releasing it brings back the power-on
# registers and boots.

# %%
# %%extract ../../../tests/test_vl53l1x.py -a --strip-ipytest
# %%ipytest


def test_fake_soft_reset_holds_and_restores():
    bus = FakeBus()
    bus.attach(0x29, FakeVL53L1X())
    bus.writeto_mem(0x29, 0x0001, b"\x30", addrsize=16)
    bus.writeto_mem(0x30, 0x0000, b"\x00", addrsize=16)
    assert bus.scan() == [0x29]
    assert bus.readfrom_mem(0x29, 0x010F, 3, addrsize=16) == b"\x00\x00\x00"
    bus.writeto_mem(0x29, 0x0000, b"\x01", addrsize=16)
    assert bus.readfrom_mem(0x29, 0x010F, 3, addrsize=16) == b"\xea\xcc\x10"
    assert bus.readfrom_mem(0x29, 0x00E5, 1, addrsize=16) == b"\x01"


# %% [markdown]
# # Code
# ## TODO:
# Nothing

# %% [markdown]
# ## Public names

# %%
# %%extract ../../../src/micropython_i2c/devices/sensors/distance/vl53l1x.py -a
__all__ = ["VL53L1X", "VL53L1XRegs", "VL53L1XResult"]

# %% [markdown]
# ## The class hierarchy
# `VL53L1X` subclasses `VL53L1XRegs`, which is a `RegisterI2CBase` and an `I2CDevice`, so the
# facts are reachable from the class and from any instance. `VL53L1XResult` stands alone: the
# driver builds one from each result block.

# %% [markdown]
# ## `VL53L1XRegs`
# Facts only. The datasheet gives the address (`0x52` as an 8-bit write address, `0x29` as a
# 7-bit one), the 400 kHz limit, 16-bit register indexes (`REG_ADDR_BITS = 16`, one byte per
# register), the three reference registers and their reset values (Table 8), and the boot time.
# Everything else is from ST's ULD source, which the datasheet points to for "full register
# details".
#
# | Fact | Value | Source |
# |---|---|---|
# | model ID, module type, mask revision | `0x010F` = `0xEA`, `0x0110` = `0xCC`, `0x0111` = `0x10` | Table 8 |
# | boot time | 1.2 ms maximum | 3.6 |
# | default configuration | 91 bytes for `0x002D..0x0087` | ULD `VL51L1X_DEFAULT_CONFIGURATION` |
# | distance modes | the six register values each mode writes | ULD `SetDistanceMode` |
# | timing budgets | two 16-bit timeout words per budget and mode | ULD `SetTimingBudgetInMs`, limited by 3.5.2 |
# | range statuses | raw status (`0x0089` bits 4..0) to the ULD's code | ULD `GetRangeStatus` |
#
# The range-status names are ours, after ST's names for the codes. Only code 0 is a fully valid
# range; codes 3, 6 and 11 are ranges with a caveat, the rest are failures.


# %%
# %%extract ../../../src/micropython_i2c/devices/sensors/distance/vl53l1x.py -a
class VL53L1XRegs(RegisterI2CBase, I2CDevice):
    """Datasheet and ULD facts for the VL53L1X, with no driver logic."""

    ALLOWED_ADDRS = tuple(range(0x08, 0x78))  # 0x29 at power-up; programmable (0x0001)
    ADDR_DEFAULT = 0x29  # the datasheet's 0x52 is the 8-bit write address
    BAUD_NOMINAL = 400_000
    BAUD_MAX = 400_000  # Table 7
    BAUD_MIN = 10_000

    REG_ADDR_BITS = 16  # section 4: a 16-bit index, MSB first
    REG_BYTES = 1  # 8-bit registers; multi-byte values are MSB first (Table 9)
    BIG_ENDIAN = True

    # Reference registers and their values after a fresh reset (Table 8).
    REG_MODEL_ID = 0x010F
    REG_MODULE_TYPE = 0x0110
    REG_MASK_REVISION = 0x0111
    MODEL_ID = 0xEA
    MODULE_TYPE = 0xCC
    MASK_REVISION = 0x10
    ID_WORD = 0xEACC  # 16-bit read at REG_MODEL_ID (the ULD's GetSensorId)

    T_BOOT_US = 1200  # boot duration, maximum (3.6)
    T_SOFT_RESET_US = 100  # between reset asserted and released (ST's full VL53L1 API)

    # Register map (ULD vl53l1x_class.h).
    REG_SOFT_RESET = 0x0000
    REG_I2C_ADDRESS = 0x0001  # 7-bit address; lost at power-off
    REG_VHV_LOOP_BOUND = 0x0008
    REG_VHV_START = 0x000B
    REG_XTALK_PLANE_OFFSET = 0x0016  # kcps, 7.9
    REG_XTALK_X_GRADIENT = 0x0018
    REG_XTALK_Y_GRADIENT = 0x001A
    REG_OFFSET_MM = 0x001E  # part-to-part offset, mm, signed 11.2 in 13 bits
    REG_INNER_OFFSET_MM = 0x0020
    REG_OUTER_OFFSET_MM = 0x0022
    REG_GPIO_HV_MUX_CTRL = 0x0030  # bit 4: interrupt active low
    REG_GPIO_TIO_HV_STATUS = 0x0031  # bit 0: the interrupt line
    REG_INTERRUPT_CONFIG = 0x0046
    REG_PHASECAL_TIMEOUT = 0x004B  # also tells the distance mode
    REG_TIMEOUT_A = 0x005E  # 16 bits
    REG_VCSEL_PERIOD_A = 0x0060
    REG_TIMEOUT_B = 0x0061  # 16 bits
    REG_VCSEL_PERIOD_B = 0x0063
    REG_SIGMA_THRESH = 0x0064
    REG_MIN_COUNT_RATE = 0x0066
    REG_VALID_PHASE_HIGH = 0x0069
    REG_INTERMEASUREMENT = 0x006C  # 32 bits, oscillator counts
    REG_WOI_SD0 = 0x0078  # 16 bits
    REG_INITIAL_PHASE_SD0 = 0x007A  # 16 bits
    REG_ROI_CENTER = 0x007F
    REG_ROI_XY = 0x0080  # (height - 1) << 4 | (width - 1)
    REG_INTERRUPT_CLEAR = 0x0086
    REG_MODE_START = 0x0087
    REG_RESULT = 0x0089  # the result block, 0x0089..0x0099
    RESULT_BYTES = 17
    REG_OSC_CALIBRATE = 0x00DE  # 16 bits, low 10 bits used
    REG_FIRMWARE_STATUS = 0x00E5  # bit 0: booted

    MODE_STOP = 0x00
    MODE_CONTINUOUS = 0x40
    MODE_ONE_SHOT = 0x10

    F_INT_ACTIVE_LOW = (4, 1)  # in GPIO_HV_MUX_CTRL
    F_BOOTED = (0, 1)  # in FIRMWARE_STATUS
    F_RANGE_STATUS = (0, 5)  # in the first result byte

    # VHV (temperature) calibration settings around a ranging start (ULD SensorInit and
    # StartTemperatureUpdate): two bounds and start from the last temperature normally, a full
    # loop for a temperature update.
    VHV_NORMAL = (0x09, 0x00)  # (REG_VHV_LOOP_BOUND, REG_VHV_START)
    VHV_FULL = (0x81, 0x92)

    # Default configuration for 0x002D..0x0087 (ULD VL51L1X_DEFAULT_CONFIGURATION): 2V8 I/O,
    # interrupt active high on a new sample, long mode, ROI 16x16 centred on SPAD 199, ranging
    # stopped. The last two bytes are the interrupt clear and the mode start.
    DEFAULT_CONFIG_START = 0x002D
    DEFAULT_CONFIG = bytes((
        0x00, 0x01, 0x01, 0x01, 0x02, 0x00, 0x02, 0x08, 0x00, 0x08, 0x10, 0x01, 0x01,
        0x00, 0x00, 0x00, 0x00, 0xFF, 0x00, 0x0F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20,
        0x0B, 0x00, 0x00, 0x02, 0x0A, 0x21, 0x00, 0x00, 0x05, 0x00, 0x00, 0x00, 0x00,
        0xC8, 0x00, 0x00, 0x38, 0xFF, 0x01, 0x00, 0x08, 0x00, 0x00, 0x01, 0xDB, 0x0F,
        0x01, 0xF1, 0x0D, 0x01, 0x68, 0x00, 0x80, 0x08, 0xB8, 0x00, 0x00, 0x00, 0x00,
        0x0F, 0x89, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x0F, 0x0D, 0x0E,
        0x0E, 0x00, 0x00, 0x02, 0xC7, 0xFF, 0x9B, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00,
    ))  # fmt: skip

    # Distance modes (ULD SetDistanceMode): PHASECAL_TIMEOUT, VCSEL_PERIOD_A, VCSEL_PERIOD_B,
    # VALID_PHASE_HIGH (bytes), WOI_SD0, INITIAL_PHASE_SD0 (words).
    DISTANCE_MODES = {
        "short": (0x14, 0x07, 0x05, 0x38, 0x0705, 0x0606),
        "long": (0x0A, 0x0F, 0x0D, 0xB8, 0x0F0D, 0x0E0E),
    }

    # Timing budgets in ms -> (TIMEOUT_A, TIMEOUT_B) words (ULD SetTimingBudgetInMs). Only the
    # budgets the datasheet allows (3.5.2): 20 ms in short mode only, 33 ms and up in both.
    TIMING_BUDGETS = {
        "short": {
            20: (0x0051, 0x006E), 33: (0x00D6, 0x006E), 50: (0x01AE, 0x01E8),
            100: (0x02E1, 0x0388), 200: (0x03E1, 0x0496), 500: (0x0591, 0x05C1),
        },
        "long": {
            33: (0x0060, 0x006E), 50: (0x00AD, 0x00C6), 100: (0x01CC, 0x01EA),
            200: (0x02D9, 0x02F8), 500: (0x048F, 0x04A4),
        },
    }  # fmt: skip
    # In the ULD but outside the datasheet's limits; the driver refuses them.
    ULD_ONLY_BUDGETS = {"short": {15: (0x001D, 0x0027)}, "long": {20: (0x001E, 0x0022)}}
    TB_MIN_MS = 20  # datasheet 3.5.2: 20 ms (short mode) to 1000 ms
    TB_MAX_MS = 1000

    IM_FACTOR = 1.075  # inter-measurement counts = osc * ms * 1.075 (ULD SetInterMeasurementInMs)

    # Raw range status (result byte 0, bits 4..0) -> (ULD code, name) (ULD GetRangeStatus).
    RANGE_STATUS = {
        9: (0, "valid"),
        6: (1, "sigma failure"),
        4: (2, "signal failure"),
        8: (3, "valid, below minimum range"),
        5: (4, "out of bounds"),
        3: (5, "hardware failure"),
        19: (6, "valid, no wrap-around check"),
        7: (7, "wrapped target"),
        12: (9, "crosstalk signal failure"),
        18: (10, "synchronisation"),
        22: (11, "valid, merged pulse"),
        23: (12, "target present, lack of signal"),
        13: (13, "minimum range failure"),
    }
    STATUS_UNKNOWN = 255

    ROI_MIN = 4  # SPADs per side (3.4: 4x4 to 16x16)
    ROI_MAX = 16
    ROI_CENTER_DEFAULT = 199  # the default configuration's centre SPAD
    ROI_CENTER_FREE_MAX = 10  # a side above this forces the default centre (ULD SetROI)

    OFFSET_MIN_MM = -1024  # 13-bit signed, quarter millimetres
    OFFSET_MAX_MM = 1023
    XTALK_MAX_CPS = (0xFFFF * 1000) >> 9

    DISTANCE_MIN_MM = 40  # minimum ranging distance (7.3)


# %% [markdown]
# ### validation/test
# A helper that puts a `FakeVL53L1X` (booted, a target at 500 mm) at `0x29` on a fake bus and
# binds a class to it (`dev_kw` go to the class, the other keywords to the fake). It returns the object and the fake part, and is shared by the validation
# cells and the test file.


# %%
# %%extract ../../../tests/test_vl53l1x.py -a
def on_fake(cls, addr=0x29, dev_kw=None, **part_kw):
    """``(dev, part)``: ``cls(**dev_kw)`` bound to a fresh ``FakeVL53L1X(**part_kw)`` at
    ``addr`` on a fake bus."""
    bus = FakeBus()
    part = bus.attach(addr, FakeVL53L1X(addr=addr, **part_kw))
    return cls(I2CConnection(bus, addr), **(dev_kw or {})), part


# %% [markdown]
# #### validation: nominal usage
# Through the register layer alone: the three reference registers in one 16-bit-indexed read,
# the boot flag, and the default configuration written in one sequential write and read back.
# `0x0031` is the live interrupt line, not a stored value, so it does not read back as written.

# %%
regs, part = on_fake(VL53L1XRegs)
ref = regs.read_mem(regs.REG_MODEL_ID, 3)
print("0x010F..0x0111:", ref.hex(" "), "| want ea cc 10")
print("booted:", regs.field_extract(regs.reg_read(regs.REG_FIRMWARE_STATUS), regs.F_BOOTED))
regs.write_mem(regs.DEFAULT_CONFIG_START, regs.DEFAULT_CONFIG)
back = regs.read_mem(regs.DEFAULT_CONFIG_START, len(regs.DEFAULT_CONFIG))
diff = [hex(regs.DEFAULT_CONFIG_START + i) for i, (a, b) in enumerate(zip(regs.DEFAULT_CONFIG, back))
        if a != b]  # fmt: skip
print("default configuration read back; registers that differ:", diff, "(want ['0x31'])")

# %% [markdown]
# #### validation: anti-nominal usage
# The ID registers are read-only: writing them changes nothing.

# %%
regs, part = on_fake(VL53L1XRegs)
regs.reg_write(regs.REG_MODEL_ID, 0x00)  # a read-only register
try:
    regs.reg_set(regs.REG_MODEL_ID, 0x00)
except ValueError as err:
    print("raised ValueError, as expected:\n    {}".format(err))
else:
    raise AssertionError("writing the model ID should not have taken")

# %% [markdown]
# #### unit

# %% [markdown]
# The default configuration covers `0x002D..0x0087` exactly, leaves ranging stopped (the last
# byte is the mode start), sets the interrupt active high, and selects long mode.

# %%
# %%extract ../../../tests/test_vl53l1x.py -a --strip-ipytest
# %%ipytest


def test_default_config_covers_its_range():
    r = VL53L1XRegs
    assert len(r.DEFAULT_CONFIG) == 0x0087 - 0x002D + 1 == 91
    at = lambda reg: r.DEFAULT_CONFIG[reg - r.DEFAULT_CONFIG_START]  # noqa: E731
    assert at(r.REG_MODE_START) == 0x00 and at(r.REG_INTERRUPT_CLEAR) == 0x00
    assert at(r.REG_GPIO_HV_MUX_CTRL) & 0x10 == 0  # active high
    assert at(r.REG_PHASECAL_TIMEOUT) == r.DISTANCE_MODES["long"][0]


# %% [markdown]
# The timing-budget tables keep to the datasheet: 20 ms in short mode only, at least 33 ms in
# long mode, nothing above 1000 ms, and none of the ULD-only budgets.

# %%
# %%extract ../../../tests/test_vl53l1x.py -a --strip-ipytest
# %%ipytest


def test_timing_budgets_follow_the_datasheet():
    r = VL53L1XRegs
    assert min(r.TIMING_BUDGETS["short"]) == 20 and min(r.TIMING_BUDGETS["long"]) == 33
    for mode, table in r.TIMING_BUDGETS.items():
        assert all(r.TB_MIN_MS <= ms <= r.TB_MAX_MS for ms in table)
        assert not set(table) & set(r.ULD_ONLY_BUDGETS[mode])
        assert len({words[0] for words in table.values()}) == len(table)  # A words tell apart


# %% [markdown]
# The power-up address is allowed and `0x78` (a reserved address) is not; 400 kHz is the clock
# ceiling.

# %%
# %%extract ../../../tests/test_vl53l1x.py -a --strip-ipytest
# %%ipytest


def test_address_and_clock_limits():
    assert VL53L1XRegs.addr_ok(0x29) and not VL53L1XRegs.addr_ok(0x78)
    assert VL53L1XRegs.baud_ok(400_000) and not VL53L1XRegs.baud_ok(1_000_000)


# %% [markdown]
# The reference registers read the Table 8 values, and the 16-bit model ID read gives
# `0xEACC`.

# %%
# %%extract ../../../tests/test_vl53l1x.py -a --strip-ipytest
# %%ipytest


def test_reference_registers():
    regs, _ = on_fake(VL53L1XRegs)
    assert regs.reg_read(regs.REG_MODEL_ID) == regs.MODEL_ID
    assert regs.reg_read(regs.REG_MODULE_TYPE) == regs.MODULE_TYPE
    assert regs.reg_read(regs.REG_MASK_REVISION) == regs.MASK_REVISION
    assert regs.reg_read(regs.REG_MODEL_ID, 2) == regs.ID_WORD


# %% [markdown]
# ## `VL53L1XResult`
# One sample, decoded from the 17-byte block at `0x0089` read in one transfer (so every field
# is from the same sample). Byte offsets in the block:
#
# | Offset | Register | Field |
# |---|---|---|
# | 0 | `0x0089` | range status, bits 4..0 |
# | 3 | `0x008C` | effective SPAD count (integer part of an 8.8 word) |
# | 7, 8 | `0x0090` | ambient rate, ×8 = kcps |
# | 13, 14 | `0x0096` | distance, mm (offset and crosstalk corrected) |
# | 15, 16 | `0x0098` | signal rate, ×8 = kcps |
#
# `valid` is True only for ULD code 0: the distance register always holds a number, and only the
# status says whether to trust it.


# %%
# %%extract ../../../src/micropython_i2c/devices/sensors/distance/vl53l1x.py -a
class VL53L1XResult:
    """One ranging result.

    Attributes:
        status_raw: Range status as the part reports it (0..31).
        status: The ULD's status code (0 = valid, 255 = unknown).
        status_name: Readable name of the status.
        valid: True only for status 0.
        distance_mm: Distance in mm. Meaningful only when ``valid``.
        signal_kcps: Return signal rate, kcps.
        ambient_kcps: Ambient rate, kcps.
        spads: Number of SPADs used.
    """

    def __init__(self, status_raw, distance_mm, signal_kcps=0, ambient_kcps=0, spads=0):
        self.status_raw = status_raw & 0x1F
        self.status, self.status_name = self.status_decode(self.status_raw)
        self.valid = self.status == 0
        self.distance_mm = distance_mm
        self.signal_kcps = signal_kcps
        self.ambient_kcps = ambient_kcps
        self.spads = spads

    def __repr__(self):
        return "<VL53L1XResult {} mm {} (raw {}) signal {} kcps ambient {} kcps {} SPADs>".format(
            self.distance_mm, self.status_name, self.status_raw, self.signal_kcps,
            self.ambient_kcps, self.spads,
        )  # fmt: skip

    @staticmethod
    def status_decode(raw):
        """``(ULD code, name)`` for a raw range status."""
        raw &= 0x1F
        return VL53L1XRegs.RANGE_STATUS.get(
            raw, (VL53L1XRegs.STATUS_UNKNOWN, "unknown status {}".format(raw))
        )

    @classmethod
    def from_block(cls, block):
        """Decode the 17-byte result block read from ``0x0089``.

        Raises:
            ValueError: If ``block`` is not 17 bytes.
        """
        if len(block) != VL53L1XRegs.RESULT_BYTES:
            raise ValueError("result block is {} bytes, not 17".format(len(block)))
        return cls(
            block[0],
            (block[13] << 8) | block[14],
            ((block[15] << 8) | block[16]) * 8,
            ((block[7] << 8) | block[8]) * 8,
            block[3],
        )


# %% [markdown]
# ### validation/test
# The status cases `(raw, code, valid)` and a result block, shared by the unit tests below and
# copied to the board by the hardware section. The block is a valid 243 mm sample with
# 1000×8 kcps of signal, 50×8 of ambient and 42 SPADs.

# %%
# %%extract ../../../tests/test_vl53l1x.py -a
STATUS_CASES = [(9, 0, True), (6, 1, False), (4, 2, False), (5, 4, False), (7, 7, False),
                (8, 3, False), (0, 255, False)]  # fmt: skip
RESULT_BLOCK = bytes([0x09, 0, 0, 42, 0, 0, 0, 0, 50, 0, 0, 0, 0, 0, 243, 0x03, 0xE8])

# %% [markdown]
# #### validation: nominal usage

# %%
r = VL53L1XResult.from_block(RESULT_BLOCK)
print(r)
for raw, code, valid in STATUS_CASES:
    print("raw {:2}: {} (want code {}, valid {})".format(
        raw, VL53L1XResult.status_decode(raw), code, valid))  # fmt: skip

# %% [markdown]
# #### validation: anti-nominal usage
# A short block (a truncated read) is refused rather than decoded from the wrong offsets.

# %%
try:
    VL53L1XResult.from_block(RESULT_BLOCK[:16])  # one byte short
except ValueError as err:
    print("raised ValueError, as expected:\n    {}".format(err))
else:
    raise AssertionError("a 16-byte block should have raised ValueError")

# %% [markdown]
# #### unit

# %% [markdown]
# Each raw status maps to the ULD's code; only code 0 is valid, and an unlisted raw status is
# code 255.

# %%
# %%extract ../../../tests/test_vl53l1x.py -a --strip-ipytest
# %%ipytest


@pytest.mark.parametrize("raw, code, valid", STATUS_CASES)
def test_status_decode(raw, code, valid):
    r = VL53L1XResult(raw, 100)
    assert (r.status, r.valid) == (code, valid)
    assert r.status_name == VL53L1XResult.status_decode(raw)[1]


# %% [markdown]
# The block decodes at the right offsets and scales.

# %%
# %%extract ../../../tests/test_vl53l1x.py -a --strip-ipytest
# %%ipytest


def test_result_block_decodes():
    r = VL53L1XResult.from_block(RESULT_BLOCK)
    assert (r.valid, r.distance_mm, r.spads) == (True, 243, 42)
    assert (r.signal_kcps, r.ambient_kcps) == (1000 * 8, 50 * 8)
    with pytest.raises(ValueError, match="16 bytes"):
        VL53L1XResult.from_block(RESULT_BLOCK[:16])


# %% [markdown]
# ## `VL53L1X`
# The driver. Construction waits for the firmware to boot, checks the model ID (anything else
# at the address is refused), loads the default configuration, runs one ranging start to let
# the part calibrate (VHV), then sets the distance mode, timing budget and inter-measurement
# period. The ULD's own sequence, `SensorInit`, leaves the timing words at values in neither
# table, so the driver always sets a budget.
#
# Rules it enforces:
#
# | Rule | Why |
# |---|---|
# | settings only while not ranging (`RuntimeError`) | the sequencer reads them at the start |
# | a budget must be in the table for the current mode (`ValueError`, before any write) | the datasheet limits and the ULD's words |
# | a mode change keeps the budget, and is refused if the budget is not allowed in the new mode | the ULD rewrites the budget after a mode change |
# | inter-measurement period ≥ timing budget; setting a longer budget raises a shorter period to match | the next sample cannot start before the last one ends |
# | ROI 4..16 per side; a side above 10 forces the default centre, so a centre with it is refused | the datasheet's range; the ULD's rule |
# | `reset()` only at the default address | a reset moves the part back to `0x29` (measured) |
# | a soft reset left asserted is released before the boot wait | an interrupted reset holds the part with every register reading 0 (measured) |
#
# Ranging: `start()` (continuous) and `read()` (wait for the interrupt line, read the result
# block, clear), or `measure()` for one shot. `data_ready()` reads `0x0030..0x0031` in one
# transfer, so the polarity and the line come from the same moment.


# %%
# %%extract ../../../src/micropython_i2c/devices/sensors/distance/vl53l1x.py -a
class VL53L1X(VL53L1XRegs):
    """VL53L1X driver.

    Args:
        conn: Connection to the part.
        distance_mode: "short" or "long".
        timing_budget_ms: A budget from ``TIMING_BUDGETS[distance_mode]``.
        inter_measurement_ms: Period between continuous samples; defaults to the budget.

    Attributes:
        ident: The 16-bit model ID read at construction (``0xEACC``).
        ranging: True between ``start()`` and ``stop()``.
        last: The last ``VL53L1XResult`` that ``read()`` returned, or None.

    Raises:
        RuntimeError: From the constructor, if the part at the address is not a VL53L1X.
        OSError: From the constructor, if the firmware does not boot or the first ranging
            start never completes.
    """

    BOOT_TIMEOUT_MS = 100  # the datasheet's maximum is 1.2 ms
    INIT_TIMEOUT_MS = 300  # first sample after the default configuration (ULD: ~150 ms)
    READ_MARGIN_MS = 100

    def __init__(self, conn, distance_mode="long", timing_budget_ms=100,
                 inter_measurement_ms=None, **kwargs):  # fmt: skip
        super().__init__(conn, **kwargs)
        self.ranging = False
        self.last = None
        self._tb = None
        self._im = None
        self._config = (distance_mode, timing_budget_ms, inter_measurement_ms)
        self.boot_wait()
        ident = self.reg_read(self.REG_MODEL_ID, 2)
        if ident != self.ID_WORD:
            raise RuntimeError("not a VL53L1X at {:#04x} (ID {:#06x})".format(self.addr, ident))
        self.ident = ident
        self._setup()

    def _setup(self):
        """Load the default configuration, calibrate once, apply ``_config``."""
        self.sensor_init()
        mode, tb, im = self._config
        self.distance_mode = mode
        self.timing_budget_ms = tb
        self.inter_measurement_ms = tb if im is None else im

    def _wait(self, ready, timeout_ms, what):
        """Poll ``ready()`` about every ms.

        Raises:
            OSError: If it is not true within ``timeout_ms``.
        """
        start = ticks_ms()
        while not ready():
            if ticks_diff(ticks_ms(), start) > timeout_ms:
                raise OSError(
                    "VL53L1X at {:#04x}: {} timed out after {} ms".format(
                        self.addr, what, timeout_ms
                    )
                )
            sleep_ms(1)

    def _idle(self, what):
        if self.ranging:
            raise RuntimeError("stop ranging before changing {}".format(what))

    # Codecs: pure arithmetic, no bus traffic.

    @classmethod
    def timing_words(cls, mode, ms):
        """``(TIMEOUT_A, TIMEOUT_B)`` for a ``ms`` budget in ``mode``.

        Raises:
            ValueError: If the mode is unknown or the budget is not in its table.
        """
        try:
            table = cls.TIMING_BUDGETS[mode]
        except KeyError:
            raise ValueError(
                "distance mode {!r} not in {}".format(mode, sorted(cls.TIMING_BUDGETS))
            ) from None
        if ms not in table:
            raise ValueError(
                "timing budget {} ms not available in {} mode; choose from {}".format(
                    ms, mode, sorted(table)
                )
            )
        return table[ms]

    @classmethod
    def inter_measurement_word(cls, ms, osc):
        """The 32-bit period word for ``ms`` at oscillator calibration ``osc``.

        Raises:
            ValueError: If ``osc`` (its low 10 bits) is 0 or the word would not fit.
        """
        osc &= 0x3FF
        if osc == 0:
            raise ValueError("oscillator calibration reads 0")
        word = int(osc * ms * cls.IM_FACTOR)
        if not 0 <= word <= 0xFFFFFFFF:
            raise ValueError("inter-measurement period {} ms does not fit".format(ms))
        return word

    @classmethod
    def inter_measurement_from_word(cls, word, osc):
        """Milliseconds from a period word (the inverse of ``inter_measurement_word``)."""
        return round(word / ((osc & 0x3FF) * cls.IM_FACTOR))

    @classmethod
    def roi_encode(cls, width, height):
        """The ``ROI_XY`` byte for a ``width`` x ``height`` SPAD region.

        Raises:
            ValueError: If a side is outside 4..16.
        """
        for n in (width, height):
            if not cls.ROI_MIN <= n <= cls.ROI_MAX:
                raise ValueError(
                    "ROI {}x{}: each side must be {}..{} SPADs".format(
                        width, height, cls.ROI_MIN, cls.ROI_MAX
                    )
                )
        return ((height - 1) << 4) | (width - 1)

    @staticmethod
    def roi_decode(byte):
        """``(width, height)`` from an ``ROI_XY`` byte."""
        return (byte & 0x0F) + 1, (byte >> 4) + 1

    @classmethod
    def offset_encode(cls, mm):
        """The offset register word for ``mm`` (quarter-mm resolution).

        Raises:
            ValueError: If ``mm`` is outside -1024..1023.
        """
        if not cls.OFFSET_MIN_MM <= mm <= cls.OFFSET_MAX_MM:
            raise ValueError("offset {} mm outside -1024..1023".format(mm))
        return int(round(mm * 4)) & 0xFFFF

    @classmethod
    def offset_decode(cls, word):
        """Millimetres from the offset register word (13-bit signed, 11.2)."""
        return cls.sign_extend(word, 13) / 4

    @classmethod
    def xtalk_encode(cls, cps):
        """The crosstalk plane-offset word (kcps, 7.9) for ``cps`` counts per second.

        Raises:
            ValueError: If ``cps`` is negative or too large for the word.
        """
        if not 0 <= cps <= cls.XTALK_MAX_CPS:
            raise ValueError("crosstalk {} cps outside 0..{}".format(cps, cls.XTALK_MAX_CPS))
        return (int(cps) << 9) // 1000

    @staticmethod
    def xtalk_decode(word):
        """Counts per second from the crosstalk plane-offset word."""
        return (word * 1000) >> 9

    # Boot, identity, initialisation.

    def booted(self):
        """True once the firmware has booted."""
        return bool(self.field_extract(self.reg_read(self.REG_FIRMWARE_STATUS), self.F_BOOTED))

    def held_in_reset(self):
        """True if the soft reset is asserted (``0x0000`` bit 0 clear)."""
        return self.reg_read(self.REG_SOFT_RESET) & 1 == 0

    def boot_wait(self, timeout_ms=None):
        """Wait for the firmware to boot, first releasing a soft reset left asserted.

        A reset interrupted between its two writes holds the part: it still answers, but
        every register reads 0 and it never boots (measured).

        Raises:
            OSError: If it has not booted within ``timeout_ms`` (default ``BOOT_TIMEOUT_MS``).
        """
        if self.held_in_reset():
            self.reg_write(self.REG_SOFT_RESET, 0x01)
        self._wait(self.booted, timeout_ms or self.BOOT_TIMEOUT_MS, "boot")

    def _vhv(self, settings):
        self.reg_write(self.REG_VHV_LOOP_BOUND, settings[0])
        self.reg_write(self.REG_VHV_START, settings[1])

    def _one_cycle(self, timeout_ms):
        """Start, wait for the first sample, clear, stop (the ULD's calibration cycle)."""
        self.start()
        try:
            self.wait_ready(timeout_ms)
            self.clear_interrupt()
        finally:
            self.stop()

    def sensor_init(self):
        """Load the default configuration and run one ranging cycle (ULD ``SensorInit``).

        Leaves the part stopped, in long mode, with timing words in neither table: set a
        timing budget afterwards (the constructor does).
        """
        self._idle("the configuration")
        self.write_mem(self.DEFAULT_CONFIG_START, self.DEFAULT_CONFIG)
        self.cache_clear()
        self._tb = self._im = None
        self._one_cycle(self.INIT_TIMEOUT_MS)
        self._vhv(self.VHV_NORMAL)

    def temperature_update(self):
        """Recalibrate for temperature: one ranging cycle with the full VHV loop.

        ST's ULD offers this for when the temperature has changed by more than about 8 C
        since the last calibration.
        """
        self._idle("the calibration")
        self._vhv(self.VHV_FULL)
        try:
            self._one_cycle(self.INIT_TIMEOUT_MS + self._read_timeout())
        finally:
            self._vhv(self.VHV_NORMAL)

    def reset(self):
        """Soft reset, then boot and set up again with the constructor's settings.

        Raises:
            RuntimeError: If the part is not at the default address (a reset may move it
                back to ``0x29``, away from this connection).
        """
        if self.addr != self.ADDR_DEFAULT:
            raise RuntimeError("reset only at the default address {:#04x}".format(self.addr))
        self.reg_write(self.REG_SOFT_RESET, 0x00)
        sleep_us(self.T_SOFT_RESET_US)
        self.reg_write(self.REG_SOFT_RESET, 0x01)
        self.ranging = False
        self.cache_clear()
        self.boot_wait()
        self._setup()

    def address_set(self, new_addr):
        """Move the part to ``new_addr`` and rebind this driver to it.

        The new address lasts until power-off (or perhaps a soft reset).

        Raises:
            ValueError: If ``new_addr`` is not a 7-bit address the part allows.
            RuntimeError: If no VL53L1X answers at the new address.
        """
        if not self.addr_ok(new_addr):
            raise ValueError("address {:#04x} not allowed".format(new_addr))
        self.reg_write(self.REG_I2C_ADDRESS, new_addr)  # answers at new_addr from now on
        old = self.conn
        self.conn = I2CConnection(old.bus, new_addr, old.path, old.lock)
        ident = self.reg_read(self.REG_MODEL_ID, 2)
        if ident != self.ID_WORD:
            raise RuntimeError("no VL53L1X at {:#04x} after the move".format(new_addr))

    # Ranging.

    def data_ready(self):
        """True when a sample is waiting (the interrupt line is active)."""
        ctrl, line = self.read_mem(self.REG_GPIO_HV_MUX_CTRL, 2)
        return (line & 1) ^ self.field_extract(ctrl, self.F_INT_ACTIVE_LOW) == 1

    def clear_interrupt(self):
        """Clear the interrupt; in continuous mode this lets the next sample be made."""
        self.reg_write(self.REG_INTERRUPT_CLEAR, 0x01)

    def start(self):
        """Start continuous ranging (one sample per inter-measurement period)."""
        self.clear_interrupt()
        self.reg_write(self.REG_MODE_START, self.MODE_CONTINUOUS)
        self.ranging = True

    def stop(self):
        """Stop ranging."""
        self.reg_write(self.REG_MODE_START, self.MODE_STOP)
        self.ranging = False

    def _read_timeout(self):
        return (self._im or 1000) + (self._tb or 0) + self.READ_MARGIN_MS

    def wait_ready(self, timeout_ms=None):
        """Wait for a sample.

        Raises:
            OSError: If none is ready within ``timeout_ms`` (default: the period plus the
                budget plus a margin). Often: ranging was never started.
        """
        self._wait(self.data_ready, timeout_ms or self._read_timeout(), "waiting for a sample")

    def read_result(self):
        """The current result block as a ``VL53L1XResult``, without waiting or clearing."""
        return VL53L1XResult.from_block(self.read_mem(self.REG_RESULT, self.RESULT_BYTES))

    def read(self, timeout_ms=None):
        """Wait for the next sample, read it and clear the interrupt.

        Returns:
            A ``VL53L1XResult``; check ``valid`` before trusting ``distance_mm``.

        Raises:
            OSError: If no sample arrives in time.
        """
        self.wait_ready(timeout_ms)
        result = self.read_result()
        self.clear_interrupt()
        self.last = result
        return result

    def measure(self, timeout_ms=None):
        """One sample in one-shot mode; the part is back in standby afterwards.

        Raises:
            RuntimeError: If continuous ranging is running.
            OSError: If the sample does not arrive in time.
        """
        self._idle("to one-shot mode")
        self.clear_interrupt()
        self.reg_write(self.REG_MODE_START, self.MODE_ONE_SHOT)
        try:
            return self.read(timeout_ms or (self._tb or 1000) + self.READ_MARGIN_MS)
        finally:
            self.reg_write(self.REG_MODE_START, self.MODE_STOP)

    # Settings.

    @property
    def distance_mode(self):
        """"short", "long", or None if the registers match neither."""
        phasecal = self.reg_read(self.REG_PHASECAL_TIMEOUT)
        for mode, values in self.DISTANCE_MODES.items():
            if values[0] == phasecal:
                return mode
        return None

    @distance_mode.setter
    def distance_mode(self, mode):
        self._idle("the distance mode")
        if mode not in self.DISTANCE_MODES:
            raise ValueError(
                "distance mode {!r} not in {}".format(mode, sorted(self.DISTANCE_MODES))
            )
        tb = self.timing_budget_ms
        if tb is not None:
            self.timing_words(mode, tb)  # refuse before writing anything
        phasecal, vcsel_a, vcsel_b, phase_high, woi, initial = self.DISTANCE_MODES[mode]
        self.reg_set(self.REG_PHASECAL_TIMEOUT, phasecal)
        self.reg_set(self.REG_VCSEL_PERIOD_A, vcsel_a)
        self.reg_set(self.REG_VCSEL_PERIOD_B, vcsel_b)
        self.reg_set(self.REG_VALID_PHASE_HIGH, phase_high)
        self.reg_set(self.REG_WOI_SD0, woi, 2)
        self.reg_set(self.REG_INITIAL_PHASE_SD0, initial, 2)
        if tb is not None:
            self.timing_budget_ms = tb  # the words differ per mode

    @property
    def timing_budget_ms(self):
        """The timing budget in ms, or None if the timing words match no table entry."""
        mode = self.distance_mode
        word_a = self.reg_read(self.REG_TIMEOUT_A, 2)
        for ms, words in self.TIMING_BUDGETS.get(mode, {}).items():
            if words[0] == word_a:
                return ms
        return None

    @timing_budget_ms.setter
    def timing_budget_ms(self, ms):
        self._idle("the timing budget")
        word_a, word_b = self.timing_words(self.distance_mode, ms)
        self.reg_set(self.REG_TIMEOUT_A, word_a, 2)
        self.reg_set(self.REG_TIMEOUT_B, word_b, 2)
        self._tb = ms
        im = self.inter_measurement_ms
        if im is None or im < ms:
            self.inter_measurement_ms = ms

    @property
    def inter_measurement_ms(self):
        """Continuous-mode period in ms (None if the oscillator word reads 0)."""
        osc = self.reg_read(self.REG_OSC_CALIBRATE, 2)
        if osc & 0x3FF == 0:
            return None
        return self.inter_measurement_from_word(self.reg_read(self.REG_INTERMEASUREMENT, 4), osc)

    @inter_measurement_ms.setter
    def inter_measurement_ms(self, ms):
        self._idle("the inter-measurement period")
        tb = self.timing_budget_ms
        if tb is not None and ms < tb:
            raise ValueError(
                "inter-measurement period {} ms is shorter than the timing budget {} ms".format(
                    ms, tb
                )
            )
        osc = self.reg_read(self.REG_OSC_CALIBRATE, 2)
        self.reg_set(self.REG_INTERMEASUREMENT, self.inter_measurement_word(ms, osc), 4)
        self._im = ms

    @property
    def roi(self):
        """``(width, height)`` of the region of interest, in SPADs."""
        return self.roi_decode(self.reg_read(self.REG_ROI_XY))

    @property
    def roi_center(self):
        """The centre SPAD of the region of interest."""
        return self.reg_read(self.REG_ROI_CENTER)

    def roi_set(self, width, height, center=None):
        """Set the region of interest.

        Args:
            width, height: 4..16 SPADs each.
            center: Centre SPAD (0..255); default 199, the optical centre the default
                configuration uses. A side above 10 forces 199.

        Raises:
            ValueError: If a side is out of range, or a centre is given with a side above 10.
        """
        self._idle("the ROI")
        xy = self.roi_encode(width, height)
        big = width > self.ROI_CENTER_FREE_MAX or height > self.ROI_CENTER_FREE_MAX
        if center is not None and big:
            raise ValueError("a {}x{} ROI must use the default centre".format(width, height))
        if center is None:
            center = self.ROI_CENTER_DEFAULT
        if not 0 <= center <= 255:
            raise ValueError("ROI centre {} outside 0..255".format(center))
        self.reg_set(self.REG_ROI_CENTER, center)
        self.reg_set(self.REG_ROI_XY, xy)

    @property
    def offset_mm(self):
        """The part-to-part range offset in mm, added to every distance."""
        return self.offset_decode(self.reg_read(self.REG_OFFSET_MM, 2))

    @offset_mm.setter
    def offset_mm(self, mm):
        self._idle("the offset")
        word = self.offset_encode(mm)
        self.reg_set(self.REG_OFFSET_MM, word, 2, verify_mask=0x1FFF)
        self.reg_set(self.REG_INNER_OFFSET_MM, 0, 2)
        self.reg_set(self.REG_OUTER_OFFSET_MM, 0, 2)

    @property
    def xtalk_cps(self):
        """The crosstalk compensation, counts per second."""
        return self.xtalk_decode(self.reg_read(self.REG_XTALK_PLANE_OFFSET, 2))

    @xtalk_cps.setter
    def xtalk_cps(self, cps):
        self._idle("the crosstalk compensation")
        word = self.xtalk_encode(cps)
        self.reg_set(self.REG_XTALK_X_GRADIENT, 0, 2)
        self.reg_set(self.REG_XTALK_Y_GRADIENT, 0, 2)
        self.reg_set(self.REG_XTALK_PLANE_OFFSET, word, 2)


# %% [markdown]
# ### validation/test: codecs
# The codec cases, shared by the unit tests below and copied to the board by the hardware
# section. `OFFSET_CASES` are `(mm, word)`, `ROI_CASES` `(width, height, byte)`, `XTALK_CASES`
# `(cps, word)`, and `IM_CASES` `(ms, osc, word)`.

# %%
# %%extract ../../../tests/test_vl53l1x.py -a
OFFSET_CASES = [(0, 0x0000), (25, 0x0064), (-25, 0xFF9C), (1023, 0x0FFC), (-1024, 0xF000),
                (12.25, 0x0031)]  # fmt: skip
ROI_CASES = [(16, 16, 0xFF), (4, 4, 0x33), (8, 4, 0x37), (4, 16, 0xF3)]
XTALK_CASES = [(0, 0x0000), (1000, 0x0200), (64000, 0x8000)]
IM_CASES = [(100, 0x01F4, 53750), (33, 0x01F4, 17737), (100, 0xFDF4, 53750)]

# %% [markdown]
# #### validation: nominal usage

# %%
for mm, word in OFFSET_CASES:
    print("offset {:>7} mm -> {:#06x} (want {:#06x}) -> {} mm".format(
        mm, VL53L1X.offset_encode(mm), word, VL53L1X.offset_decode(VL53L1X.offset_encode(mm))))  # fmt: skip
for w, h, byte in ROI_CASES:
    print("ROI {}x{} -> {:#04x} (want {:#04x})".format(w, h, VL53L1X.roi_encode(w, h), byte))
print("timing words, long 100 ms:", [hex(w) for w in VL53L1X.timing_words("long", 100)])

# %% [markdown]
# #### validation: anti-nominal usage
# A 20 ms budget is short-mode only (datasheet 3.5.2).

# %%
try:
    VL53L1X.timing_words("long", 20)  # 20 ms needs short mode
except ValueError as err:
    print("raised ValueError, as expected:\n    {}".format(err))
else:
    raise AssertionError("20 ms in long mode should have raised ValueError")

# %% [markdown]
# #### unit

# %% [markdown]
# The offset is a 13-bit signed quarter-millimetre value: each case encodes to its word and
# decodes back, and values outside -1024..1023 mm are refused.

# %%
# %%extract ../../../tests/test_vl53l1x.py -a --strip-ipytest
# %%ipytest


@pytest.mark.parametrize("mm, word", OFFSET_CASES)
def test_offset_codec(mm, word):
    assert VL53L1X.offset_encode(mm) == word
    assert VL53L1X.offset_decode(word) == mm
    for bad in (-1025, 1024):
        with pytest.raises(ValueError):
            VL53L1X.offset_encode(bad)


# %% [markdown]
# The ROI byte is `(height - 1) << 4 | (width - 1)`, both ways; sides outside 4..16 are refused.

# %%
# %%extract ../../../tests/test_vl53l1x.py -a --strip-ipytest
# %%ipytest


@pytest.mark.parametrize("width, height, byte", ROI_CASES)
def test_roi_codec(width, height, byte):
    assert VL53L1X.roi_encode(width, height) == byte
    assert VL53L1X.roi_decode(byte) == (width, height)
    for bad in ((3, 4), (4, 17)):
        with pytest.raises(ValueError, match="4..16"):
            VL53L1X.roi_encode(*bad)


# %% [markdown]
# Crosstalk is kcps in 7.9 format: each case encodes and decodes, and negative or oversized
# values are refused.

# %%
# %%extract ../../../tests/test_vl53l1x.py -a --strip-ipytest
# %%ipytest


@pytest.mark.parametrize("cps, word", XTALK_CASES)
def test_xtalk_codec(cps, word):
    assert VL53L1X.xtalk_encode(cps) == word
    assert VL53L1X.xtalk_decode(word) == cps
    for bad in (-1, VL53L1X.XTALK_MAX_CPS + 1):
        with pytest.raises(ValueError):
            VL53L1X.xtalk_encode(bad)


# %% [markdown]
# The inter-measurement word is `osc × ms × 1.075` with only the oscillator word's low 10 bits
# used, it decodes back to the same milliseconds, and an oscillator word of 0 is refused.

# %%
# %%extract ../../../tests/test_vl53l1x.py -a --strip-ipytest
# %%ipytest


@pytest.mark.parametrize("ms, osc, word", IM_CASES)
def test_inter_measurement_codec(ms, osc, word):
    assert VL53L1X.inter_measurement_word(ms, osc) == word
    assert VL53L1X.inter_measurement_from_word(word, osc) == ms
    with pytest.raises(ValueError, match="oscillator"):
        VL53L1X.inter_measurement_word(ms, 0xFC00)  # low 10 bits are 0


# %% [markdown]
# ### validation/test: the driver on a fake part

# %% [markdown]
# #### validation: nominal usage
# Construct (boot, identify, configure), take a one-shot sample, then three continuous ones.

# %%
tof, part = on_fake(VL53L1X, distance_mm=243)
print("ID {:#06x} | {} mode, {} ms budget, {} ms period | ROI {} centre {}".format(
    tof.ident, tof.distance_mode, tof.timing_budget_ms, tof.inter_measurement_ms, tof.roi,
    tof.roi_center))  # fmt: skip
print("one-shot:", tof.measure())
tof.start()
print("continuous:", [tof.read().distance_mm for _ in range(3)])
tof.stop()
print("samples made by the fake:", part.samples)

# %% [markdown]
# #### validation: anti-nominal usage
# A VL53L4CD answers at the same address (`0x29`) and looks alike, but its model ID is
# `0xEBAA`: the constructor refuses it.

# %%
bus = FakeBus()
bus.attach(0x29, FakeVL53L4CD())
try:
    VL53L1X(I2CConnection(bus, 0x29))  # a VL53L4CD, not a VL53L1X
except RuntimeError as err:
    print("raised RuntimeError, as expected:\n    {}".format(err))
else:
    raise AssertionError("a VL53L4CD should have raised RuntimeError")

# %% [markdown]
# #### unit

# %% [markdown]
# The constructor identifies the part and leaves it stopped, in long mode, with a 100 ms budget
# and period, the full 16×16 ROI, and no offset.

# %%
# %%extract ../../../tests/test_vl53l1x.py -a --strip-ipytest
# %%ipytest


def test_init_identifies_and_configures():
    tof, part = on_fake(VL53L1X)
    assert tof.ident == 0xEACC and not tof.ranging and part.mode == 0x00
    assert (tof.distance_mode, tof.timing_budget_ms, tof.inter_measurement_ms) == ("long", 100, 100)
    assert tof.roi == (16, 16) and tof.roi_center == 199 and tof.offset_mm == 0


# %% [markdown]
# A part with another model ID (a VL53L4CD) is refused.

# %%
# %%extract ../../../tests/test_vl53l1x.py -a --strip-ipytest
# %%ipytest


def test_init_refuses_a_part_that_is_not_a_vl53l1x():
    bus = FakeBus()
    bus.attach(0x29, FakeVL53L4CD())
    with pytest.raises(RuntimeError, match="ID 0xebaa"):
        VL53L1X(I2CConnection(bus, 0x29))


# %% [markdown]
# A part whose firmware never boots makes the constructor raise `OSError` after the boot
# timeout.

# %%
# %%extract ../../../tests/test_vl53l1x.py -a --strip-ipytest
# %%ipytest


def test_init_times_out_without_boot():
    with pytest.raises(OSError, match="boot timed out"):
        on_fake(VL53L1X, booted=False)


# %% [markdown]
# A part left held in soft reset (the reset asserted, never released) reads 0 everywhere; the
# constructor releases it and it comes up.

# %%
# %%extract ../../../tests/test_vl53l1x.py -a --strip-ipytest
# %%ipytest


def test_init_releases_a_part_left_in_reset():
    bus = FakeBus()
    part = bus.attach(0x29, FakeVL53L1X())
    bus.writeto_mem(0x29, 0x0000, b"\x00", addrsize=16)  # asserted, never released
    assert bus.readfrom_mem(0x29, 0x010F, 2, addrsize=16) == b"\x00\x00"
    tof = VL53L1X(I2CConnection(bus, 0x29))
    assert tof.ident == 0xEACC and part.regs[0x0000] == 0x01 and tof.measure().valid


# %% [markdown]
# Continuous ranging returns valid samples at the fake's distance, one per interrupt clear, and
# `stop()` ends it.

# %%
# %%extract ../../../tests/test_vl53l1x.py -a --strip-ipytest
# %%ipytest


def test_continuous_ranging():
    tof, part = on_fake(VL53L1X, distance_mm=1234)
    before = part.samples
    tof.start()
    results = [tof.read() for _ in range(3)]
    tof.stop()
    assert all(r.valid and r.distance_mm == 1234 for r in results)
    assert part.samples - before >= 3 and part.mode == 0x00 and not tof.ranging  # +1 if slow
    assert tof.last is results[-1]


# %% [markdown]
# A one-shot measurement returns one valid sample and leaves the part stopped.

# %%
# %%extract ../../../tests/test_vl53l1x.py -a --strip-ipytest
# %%ipytest


def test_one_shot_measure():
    tof, part = on_fake(VL53L1X, distance_mm=87)
    r = tof.measure()
    assert r.valid and r.distance_mm == 87
    assert part.mode == 0x00 and not tof.data_ready()


# %% [markdown]
# With ranging stopped no sample comes, and `read()` raises `OSError` after its timeout.

# %%
# %%extract ../../../tests/test_vl53l1x.py -a --strip-ipytest
# %%ipytest


def test_read_times_out_when_not_ranging():
    tof, _ = on_fake(VL53L1X)
    with pytest.raises(OSError, match="timed out"):
        tof.read(timeout_ms=20)


# %% [markdown]
# A failed sample is reported with its status, not hidden: raw status 4 is ULD code 2,
# "signal failure", and not valid.

# %%
# %%extract ../../../tests/test_vl53l1x.py -a --strip-ipytest
# %%ipytest


def test_invalid_status_is_reported():
    tof, _ = on_fake(VL53L1X, status=4)
    r = tof.measure()
    assert not r.valid and (r.status, r.status_name) == (2, "signal failure")


# %% [markdown]
# Switching long to short writes short mode's registers and keeps the budget (with short mode's
# timing words).

# %%
# %%extract ../../../tests/test_vl53l1x.py -a --strip-ipytest
# %%ipytest


def test_distance_mode_switch_keeps_the_budget():
    tof, _ = on_fake(VL53L1X)
    tof.distance_mode = "short"
    assert tof.distance_mode == "short" and tof.timing_budget_ms == 100
    assert tof.reg_read(tof.REG_TIMEOUT_A, 2) == tof.TIMING_BUDGETS["short"][100][0]
    assert tof.reg_read(tof.REG_WOI_SD0, 2) == 0x0705


# %% [markdown]
# A budget outside the mode's table is refused before anything is written: 20 ms in long mode,
# and a switch to long mode while the budget is 20 ms.

# %%
# %%extract ../../../tests/test_vl53l1x.py -a --strip-ipytest
# %%ipytest


def test_budget_outside_the_mode_is_refused():
    tof, part = on_fake(VL53L1X)
    with pytest.raises(ValueError, match="not available in long mode"):
        tof.timing_budget_ms = 20
    tof.distance_mode = "short"
    tof.timing_budget_ms = 20
    writes = len(part.writes)
    with pytest.raises(ValueError, match="20 ms not available in long"):
        tof.distance_mode = "long"
    assert len(part.writes) == writes and tof.distance_mode == "short"


# %% [markdown]
# Settings are refused while ranging; after `stop()` they work.

# %%
# %%extract ../../../tests/test_vl53l1x.py -a --strip-ipytest
# %%ipytest


def test_settings_refused_while_ranging():
    tof, _ = on_fake(VL53L1X)
    tof.start()
    for name, value in (("timing_budget_ms", 50), ("distance_mode", "short"),
                        ("inter_measurement_ms", 200), ("offset_mm", 5)):  # fmt: skip
        with pytest.raises(RuntimeError, match="stop ranging"):
            setattr(tof, name, value)
    with pytest.raises(RuntimeError, match="stop ranging"):
        tof.measure()
    tof.stop()
    tof.timing_budget_ms = 50
    assert tof.timing_budget_ms == 50


# %% [markdown]
# The inter-measurement period reads back as set, a period shorter than the budget is refused,
# and a longer budget raises a shorter period to match.

# %%
# %%extract ../../../tests/test_vl53l1x.py -a --strip-ipytest
# %%ipytest


def test_inter_measurement_period():
    tof, _ = on_fake(VL53L1X)
    tof.inter_measurement_ms = 250
    assert tof.inter_measurement_ms == 250
    with pytest.raises(ValueError, match="shorter than the timing budget"):
        tof.inter_measurement_ms = 50
    tof.timing_budget_ms = 500
    assert tof.inter_measurement_ms == 500


# %% [markdown]
# The ROI reads back as set, with the default centre unless one is given; a centre with a side
# above 10 is refused.

# %%
# %%extract ../../../tests/test_vl53l1x.py -a --strip-ipytest
# %%ipytest


def test_roi_set():
    tof, _ = on_fake(VL53L1X)
    tof.roi_set(4, 4)
    assert tof.roi == (4, 4) and tof.roi_center == 199
    tof.roi_set(8, 8, center=167)
    assert tof.roi == (8, 8) and tof.roi_center == 167
    with pytest.raises(ValueError, match="default centre"):
        tof.roi_set(16, 16, center=167)


# %% [markdown]
# The offset reads back as set and is added to the distance the part reports.

# %%
# %%extract ../../../tests/test_vl53l1x.py -a --strip-ipytest
# %%ipytest


def test_offset_is_applied():
    tof, _ = on_fake(VL53L1X, distance_mm=300)
    tof.offset_mm = -12.5
    assert tof.offset_mm == -12.5
    assert tof.measure().distance_mm == 287


# %% [markdown]
# The crosstalk compensation reads back as set (to the 7.9 format's resolution).

# %%
# %%extract ../../../tests/test_vl53l1x.py -a --strip-ipytest
# %%ipytest


def test_xtalk_set():
    tof, _ = on_fake(VL53L1X)
    tof.xtalk_cps = 1000
    assert tof.xtalk_cps == 1000
    assert tof.reg_read(tof.REG_XTALK_X_GRADIENT, 2) == 0


# %% [markdown]
# A temperature update runs one ranging cycle with the full VHV loop, then restores the normal
# VHV settings and leaves the part stopped.

# %%
# %%extract ../../../tests/test_vl53l1x.py -a --strip-ipytest
# %%ipytest


def test_temperature_update():
    tof, part = on_fake(VL53L1X)
    before = part.samples
    tof.temperature_update()
    assert part.samples > before and part.mode == 0x00 and not tof.ranging
    assert (tof.reg_read(tof.REG_VHV_LOOP_BOUND), tof.reg_read(tof.REG_VHV_START)) == (0x09, 0)


# %% [markdown]
# A soft reset brings the part back up with the constructor's settings, even after they were
# changed.

# %%
# %%extract ../../../tests/test_vl53l1x.py -a --strip-ipytest
# %%ipytest


def test_reset_restores_the_constructor_settings():
    tof, part = on_fake(VL53L1X, dev_kw={"distance_mode": "short", "timing_budget_ms": 50})
    tof.timing_budget_ms = 200
    tof.reset()
    assert (0x0000, b"\x00") in part.writes and tof.booted()
    assert (tof.distance_mode, tof.timing_budget_ms, tof.inter_measurement_ms) == ("short", 50, 50)
    assert tof.measure().valid


# %% [markdown]
# The address moves: the part answers only at the new address, the driver follows it, and it
# can be moved back. A reset away from `0x29` is refused.

# %%
# %%extract ../../../tests/test_vl53l1x.py -a --strip-ipytest
# %%ipytest


def test_address_change():
    tof, _ = on_fake(VL53L1X)
    bus = tof.conn.bus
    tof.address_set(0x30)
    assert bus.scan() == [0x30] and tof.addr == 0x30 and tof.measure().valid
    with pytest.raises(RuntimeError, match="default address"):
        tof.reset()
    tof.address_set(0x29)
    assert bus.scan() == [0x29]
    with pytest.raises(ValueError):
        tof.address_set(0x78)


# %% [markdown]
# ### validation/test: in simulated time
# The driver against the fake inside `with SimClock() as clock:`, where every sleep returns at
# once and moves the clock, so a minute of ranging runs in milliseconds. A hand approaches the
# sensor: the target is a `ramp` from 1200 mm to 100 mm over the first 60 s, then holds. The
# fake takes `sample_ms=100` per sample (the 100 ms budget) and is read in continuous mode.
#
# This block has no matching hardware test: the hardware section's continuous-ranging checks
# use a hand held still.

# %% [markdown]
# #### validation: nominal usage
# One reading every 10 simulated seconds over 80 s.

# %%
with SimClock() as clock:
    tof, part = on_fake(VL53L1X, distance_mm=ramp(1200, 100, 0, 60), sample_ms=100)
    tof.start()
    for _ in range(9):
        r = tof.read()
        print("t = {:5.1f} s: {:4} mm, {}".format(clock.now_s(), r.distance_mm, r.status_name))
        clock.advance(10)
    tof.stop()
    print("samples made:", part.samples, "| sleeps served:", clock.sleeps)

# %% [markdown]
# #### validation: anti-nominal usage
# Ranging stopped, nothing follows the hand: `read()` waits its timeout in simulated time
# (instantly in real time) and raises `OSError`.

# %%
with SimClock() as clock:
    tof, part = on_fake(VL53L1X, distance_mm=ramp(1200, 100, 0, 60), sample_ms=100)
    t0 = clock.now_s()
    try:
        tof.read()  # never started
    except OSError as err:
        print("raised OSError, as expected:\n    {}".format(err))
    else:
        raise AssertionError("read() without start() should have raised OSError")
    finally:
        print("simulated wait: {:.3f} s".format(clock.now_s() - t0))

# %% [markdown]
# #### unit

# %% [markdown]
# The driver follows a hand approaching along a ramp: each continuous reading is within a few
# millimetres of the ramp at that moment, and the readings fall.

# %%
# %%extract ../../../tests/test_vl53l1x.py -a --strip-ipytest
# %%ipytest


def test_follows_a_hand_approaching_in_simulated_time():
    hand = ramp(1200, 100, 0, 60)
    with SimClock() as clock:
        tof, _ = on_fake(VL53L1X, distance_mm=hand, sample_ms=100)
        tof.start()
        readings = []
        for _ in range(7):
            r = tof.read()
            assert r.valid and abs(r.distance_mm - hand(clock.now_s())) <= 5
            readings.append(r.distance_mm)
            clock.advance(10)
        tof.stop()
    assert readings == sorted(readings, reverse=True) and readings[-1] <= 105


# %% [markdown]
# A one-shot measurement waits for the sample in simulated time: with a 100 ms sample it takes
# at least 100 ms of the clock, and the distance is the target's at that moment.

# %%
# %%extract ../../../tests/test_vl53l1x.py -a --strip-ipytest
# %%ipytest


def test_one_shot_waits_for_the_sample_in_simulated_time():
    with SimClock() as clock:
        tof, _ = on_fake(VL53L1X, distance_mm=lambda t: 1000 - 10 * t, sample_ms=100)
        t0 = clock.now_s()
        r = tof.measure()
        took = clock.now_s() - t0
    assert 0.1 <= took <= 0.11
    assert r.valid and abs(r.distance_mm - (1000 - 10 * (t0 + took))) <= 1


# %% [markdown]
# # Hardware validation
#
# > ## ⚠️ STOP — do not run this section unattended
# >
# > **Everything below talks to the real board.** It connects to the FeatherS3, uploads the
# > package, ranges with the VL53L1X (a Class 1 laser: eye-safe, but do not look into it
# > through optics), changes its I2C address and resets it. Most ranging steps ask you to hold
# > your hand about 15 cm (6 in) above it.
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
# | **Runs on** | CPython 3.13.15 in the micropylab kernel (1.0.0a1), no board | Unexpected Maker FeatherS3 (ESP32-S3), MicroPython 1.28.0 (v1.28.0, 2026-04-06), USB `/dev/ttyACM0` |
# | **I2C bus** | `FakeBus` | `machine.I2C(1)`, **bus 1**, scl=15 sda=16, 100 kHz (`bus_open("feathers3", 1)`) |
# | **Mux** | none in the unit tests: the fake part is on the bus directly | SparkFun Qwiic Mux (TCA9548A) at `0x70` (address jumpers open), **channel 7** |
# | **Sensor** | `FakeVL53L1X` at `0x29`: booted, a target at 500 mm, raw status 9 (valid), oscillator word `0x0025` (as measured), a sample 2 ms after each start or interrupt clear | Adafruit VL53L1X breakout (STEMMA QT) at `0x29`, protective film removed; XSHUT and GPIO1 not wired |
# | **Other parts used** | none | none. On the same mux: SparkFun TMAG5273 (B1, `0x22`) on channel 0 and TMP102 (`0x48`) on channel 2, off while the VL53L1X is used |
# | **Datasheet** | ST DS12385 rev 8 (August 2024), plus ST's ULD driver source for the register map | same |
# | **Result** | not run yet | not run yet |
#
# The simulation leaves the mux out (it has its own notebook, `tca9548a_devNB`); the
# hardware run reaches the VL53L1X through it. The fake makes samples in 2 ms, not in the timing
# budget, so the unit tests stay fast; the hardware section checks the real timing.
#
# Layout: connect and set up, the mux, then **matching tests** (one per unit test above, same
# name, run on the real part and on MicroPython), hardware-only checks, **diagnostics** (they
# print what the part does and do not count towards the tally), then the tally and disconnect.

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
from micropython_i2c.compat import sleep_ms, sleep_us
from micropython_i2c.debug import check, check_raises, check_reset, check_summary, info
from micropython_i2c.debug import check_counts  # noqa: F401  (read from the host with %eval)
from micropython_i2c.devices.muxes.tca9548a import TCA9548A
from micropython_i2c.devices.sensors.distance.vl53l1x import VL53L1X, VL53L1XRegs, VL53L1XResult

check_reset()

# %% [markdown]
# Notebook-only: the bus, the mux, and helpers. `on_board(cls)` stops any ranging left over and
# gives a check a fresh object (the driver's constructor loads the default configuration
# again). `rest(dev)` stops ranging and clears the interrupt. `ReplyingBus` does the real
# register read but hands back a changed reply for chosen registers, standing in for a part that
# is not a VL53L1X, one that never boots, or a failed sample. The target is your hand, held by
# eye about 15 cm (6 in) above the sensor. A hand is not a measured target (on the bench a hand
# "at 15 cm" read 9 to 12 cm, and sank while held), so `at_hand(mm)` accepts any distance from
# `HAND_MIN_MM` to `HAND_MAX_MM`: in front of the sensor and well short of the ceiling. Accuracy
# is checked relatively instead: the offset check and short against long mode compare samples
# taken back to back. If the bench is rewired, change the constants here.
#
# **What to do:** nothing.

# %%
MUX_BUS = 1  # second Qwiic chain, scl=15 sda=16
MUX_ADDR = 0x70
VL_ADDR = 0x29
VL_PATH = [[MUX_ADDR, 7]]
HAND_MIN_MM = 25  # your hand, held about 15 cm (6 in) above the sensor; no ruler needed
HAND_MAX_MM = 400  # the ceiling reads about 1.8 m

bus = bus_open("feathers3", MUX_BUS, freq=100_000)
bus_lock = default_lock()
mux = TCA9548A(bus, MUX_ADDR, lock=bus_lock)


class ReplyingBus:
    """The bus, but register reads from the VL53L1X at a register in ``replies`` return
    ``replies[reg](data)`` (after doing the real read)."""

    def __init__(self, inner, replies):
        self.inner = inner
        self.replies = replies

    def __getattr__(self, name):
        return getattr(self.inner, name)

    def readfrom_mem(self, addr, reg, nbytes, addrsize=8):
        data = self.inner.readfrom_mem(addr, reg, nbytes, addrsize=addrsize)
        if addr == VL_ADDR and reg in self.replies:
            return self.replies[reg](data)
        return data


def vl_conn(b=None, addr=VL_ADDR):
    """A connection to the VL53L1X behind mux channel 7 (optionally on a wrapped bus)."""
    return I2CConnection(b or bus, addr, VL_PATH, bus_lock)


def rest(dev):
    """Stop ranging and clear the interrupt."""
    dev.reg_write(dev.REG_MODE_START, dev.MODE_STOP)
    dev.reg_write(dev.REG_INTERRUPT_CLEAR, 0x01)
    if hasattr(dev, "ranging"):
        dev.ranging = False


def on_board(cls, b=None, **kw):
    """A fresh ``cls`` on the real VL53L1X, with any ranging stopped first."""
    rest(VL53L1XRegs(vl_conn()))
    return cls(vl_conn(b), **kw)


def at_hand(mm):
    """True if ``mm`` is a distance to a hand held over the sensor."""
    return HAND_MIN_MM <= mm <= HAND_MAX_MM


def mean(values):
    return sum(values) / len(values)


# %% [markdown]
# ## The mux
# With every channel off only the mux answers; with channel 7 on, the VL53L1X joins at `0x29`.
# If it is missing, check that the Qwiic cable is in mux port 7 (port 0 is the TMAG5273, port 2
# the TMP102).
#
# **What to do:** nothing yet. From here on, the cells that range ask you to hold your hand flat,
# palm down, about 15 cm (6 in) above the sensor while they run (their **What to do** line says
# so). At 15 cm the sensor sees a patch about 7 cm across, so a flat hand fills it.

# %%
mux.channel_deselect()
root = bus.scan()
print("bus {} with every channel off:".format(MUX_BUS), [hex(a) for a in root])
if check("mux ACKs at {:#04x}".format(MUX_ADDR), MUX_ADDR in root):
    check("VL53L1X hidden with every channel off", VL_ADDR not in root)
    mux.channel_select(7)
    seen = bus.scan()
    mux.channel_deselect()
    print("channel 7 on:", [hex(a) for a in seen])
    check("VL53L1X ACKs at 0x29 on channel 7", VL_ADDR in seen)
regs = VL53L1XRegs(vl_conn())
rest(regs)

# %% [markdown]
# ## Matching tests: `VL53L1XRegs`
# One cell per unit test above, with the same name. Tests that need no bus traffic run too: they
# check the tables on MicroPython itself.

# %% [markdown]
# **Matches** `test_default_config_covers_its_range`.
#
# **What to do:** nothing. No bus traffic.

# %%
r = VL53L1XRegs
at = lambda reg: r.DEFAULT_CONFIG[reg - r.DEFAULT_CONFIG_START]  # noqa: E731
check("test_default_config_covers_its_range",
      len(r.DEFAULT_CONFIG) == 91 and at(r.REG_MODE_START) == 0 and at(r.REG_INTERRUPT_CLEAR) == 0
      and at(r.REG_GPIO_HV_MUX_CTRL) & 0x10 == 0
      and at(r.REG_PHASECAL_TIMEOUT) == r.DISTANCE_MODES["long"][0])  # fmt: skip

# %% [markdown]
# **Matches** `test_timing_budgets_follow_the_datasheet`.
#
# **What to do:** nothing. No bus traffic.

# %%
ok = min(r.TIMING_BUDGETS["short"]) == 20 and min(r.TIMING_BUDGETS["long"]) == 33
for mode, table in r.TIMING_BUDGETS.items():
    ok = ok and all(r.TB_MIN_MS <= ms <= r.TB_MAX_MS for ms in table)
    ok = ok and not set(table) & set(r.ULD_ONLY_BUDGETS[mode])
    ok = ok and len({words[0] for words in table.values()}) == len(table)
check("test_timing_budgets_follow_the_datasheet", ok)

# %% [markdown]
# **Matches** `test_address_and_clock_limits`.
#
# **What to do:** nothing. No bus traffic.

# %%
check("test_address_and_clock_limits",
      r.addr_ok(0x29) and not r.addr_ok(0x78)
      and r.baud_ok(400_000) and not r.baud_ok(1_000_000))  # fmt: skip

# %% [markdown]
# **Matches** `test_reference_registers`: the real part reads the Table 8 values.
#
# **What to do:** nothing.

# %%
regs = VL53L1XRegs(vl_conn())
ref = regs.read_mem(regs.REG_MODEL_ID, 3)
print("0x010F..0x0111:", " ".join("{:02x}".format(b) for b in ref), "| want ea cc 10")
check("test_reference_registers",
      tuple(ref) == (regs.MODEL_ID, regs.MODULE_TYPE, regs.MASK_REVISION)
      and regs.reg_read(regs.REG_MODEL_ID, 2) == regs.ID_WORD)  # fmt: skip

# %% [markdown]
# ## Matching tests: `VL53L1XResult` and the codecs

# %% [markdown]
# Notebook-only: the status, block and codec cases, copied to the board (it cannot import the
# test file, which needs pytest). The cell after it checks the copy against the notebook's.
#
# **What to do:** nothing.

# %%
STATUS_CASES = [(9, 0, True), (6, 1, False), (4, 2, False), (5, 4, False), (7, 7, False),
                (8, 3, False), (0, 255, False)]  # fmt: skip
RESULT_BLOCK = bytes([0x09, 0, 0, 42, 0, 0, 0, 0, 50, 0, 0, 0, 0, 0, 243, 0x03, 0xE8])
OFFSET_CASES = [(0, 0x0000), (25, 0x0064), (-25, 0xFF9C), (1023, 0x0FFC), (-1024, 0xF000),
                (12.25, 0x0031)]  # fmt: skip
ROI_CASES = [(16, 16, 0xFF), (4, 4, 0x33), (8, 4, 0x37), (4, 16, 0xF3)]
XTALK_CASES = [(0, 0x0000), (1000, 0x0200), (64000, 0x8000)]
IM_CASES = [(100, 0x01F4, 53750), (33, 0x01F4, 17737), (100, 0xFDF4, 53750)]

# %% [markdown]
# Notebook-only: on the host, read the board's copies back with `%eval` and check they match.
#
# **What to do:** nothing.

# %%
# %%host
for name in ("STATUS_CASES", "OFFSET_CASES", "ROI_CASES", "XTALK_CASES", "IM_CASES"):
    board = get_ipython().run_line_magic("eval", name)
    if board != [list(c) for c in globals()[name]]:
        raise AssertionError("the board's copy of {} differs from the notebook's".format(name))
if get_ipython().run_line_magic("eval", "list(RESULT_BLOCK)") != list(RESULT_BLOCK):
    raise AssertionError("the board's copy of RESULT_BLOCK differs from the notebook's")
print("board copies of the cases match the notebook's")

# %% [markdown]
# **Matches** `test_status_decode`.
#
# **What to do:** nothing. No bus traffic.

# %%
for raw, code, valid in STATUS_CASES:
    res = VL53L1XResult(raw, 100)
    check("test_status_decode[{}]".format(raw), (res.status, res.valid) == (code, valid),
          res.status_name)  # fmt: skip

# %% [markdown]
# **Matches** `test_result_block_decodes`.
#
# **What to do:** nothing. No bus traffic.

# %%
res = VL53L1XResult.from_block(RESULT_BLOCK)
check("test_result_block_decodes",
      (res.valid, res.distance_mm, res.spads, res.signal_kcps, res.ambient_kcps)
      == (True, 243, 42, 8000, 400), repr(res))  # fmt: skip
check_raises("test_result_block_decodes: short block", ValueError,
             VL53L1XResult.from_block, RESULT_BLOCK[:16])  # fmt: skip

# %% [markdown]
# **Matches** `test_offset_codec`, `test_roi_codec`, `test_xtalk_codec` and
# `test_inter_measurement_codec`, on MicroPython.
#
# **What to do:** nothing. No bus traffic.

# %%
for mm, word in OFFSET_CASES:
    check("test_offset_codec[{}]".format(mm),
          VL53L1X.offset_encode(mm) == word and VL53L1X.offset_decode(word) == mm)  # fmt: skip
check_raises("test_offset_codec: 1024 mm refused", ValueError, VL53L1X.offset_encode, 1024)
for w, h, byte in ROI_CASES:
    check("test_roi_codec[{}x{}]".format(w, h),
          VL53L1X.roi_encode(w, h) == byte and VL53L1X.roi_decode(byte) == (w, h))  # fmt: skip
check_raises("test_roi_codec: 3x4 refused", ValueError, VL53L1X.roi_encode, 3, 4)
for cps, word in XTALK_CASES:
    check("test_xtalk_codec[{}]".format(cps),
          VL53L1X.xtalk_encode(cps) == word and VL53L1X.xtalk_decode(word) == cps)  # fmt: skip
for ms, osc, word in IM_CASES:
    check("test_inter_measurement_codec[{}-{:#06x}]".format(ms, osc),
          VL53L1X.inter_measurement_word(ms, osc) == word
          and VL53L1X.inter_measurement_from_word(word, osc) == ms)  # fmt: skip

# %% [markdown]
# ## Matching tests: `VL53L1X`
# The driver on the real part. The fake reports exact distances; the real part measures your
# hand, so these accept any valid reading from 25 to 400 mm (see `at_hand` in the setup cell).

# %% [markdown]
# **Matches** `test_init_identifies_and_configures`.
#
# **What to do:** nothing.

# %%
start = time.ticks_ms()
tof = on_board(VL53L1X)
took = time.ticks_diff(time.ticks_ms(), start)
try:
    print("constructed in {} ms: ID {:#06x}, {} mode, {} ms budget, {} ms period, ROI {} centre {},"
          " offset {} mm".format(took, tof.ident, tof.distance_mode, tof.timing_budget_ms,
                                 tof.inter_measurement_ms, tof.roi, tof.roi_center,
                                 tof.offset_mm))  # fmt: skip
    check("test_init_identifies_and_configures",
          tof.ident == 0xEACC and not tof.ranging and not tof.data_ready()
          and (tof.distance_mode, tof.timing_budget_ms, tof.inter_measurement_ms)
          == ("long", 100, 100) and tof.roi == (16, 16) and tof.roi_center == 199)  # fmt: skip
finally:
    rest(tof)

# %% [markdown]
# **Matches** `test_init_refuses_a_part_that_is_not_a_vl53l1x`. `ReplyingBus` swaps the model ID
# for a VL53L4CD's (`0xEBAA`) on the way in; the real part must be fine afterwards.
#
# **What to do:** nothing.

# %%
fake_id = {VL53L1XRegs.REG_MODEL_ID: lambda data: b"\xeb\xaa"[: len(data)]}
check_raises("test_init_refuses_a_part_that_is_not_a_vl53l1x", RuntimeError,
             on_board, VL53L1X, ReplyingBus(bus, fake_id))  # fmt: skip
check("the real part is still a VL53L1X", on_board(VL53L1X).ident == 0xEACC)

# %% [markdown]
# **Matches** `test_init_times_out_without_boot`. `ReplyingBus` makes the boot flag read 0.
#
# **What to do:** nothing. Takes 0.1 s.

# %%
unbooted = {VL53L1XRegs.REG_FIRMWARE_STATUS: lambda data: bytes(len(data))}
check_raises("test_init_times_out_without_boot", OSError,
             on_board, VL53L1X, ReplyingBus(bus, unbooted))  # fmt: skip

# %% [markdown]
# **Matches** `test_init_releases_a_part_left_in_reset`: assert the soft reset and leave it,
# check every register reads 0, then construct.
#
# **What to do:** nothing.

# %%
regs = VL53L1XRegs(vl_conn())
rest(regs)
regs.reg_write(regs.REG_SOFT_RESET, 0x00)  # asserted, never released
held = regs.read_mem(regs.REG_MODEL_ID, 3)
try:
    tof = VL53L1X(vl_conn())
    check("test_init_releases_a_part_left_in_reset",
          held == b"\x00\x00\x00" and tof.ident == 0xEACC and tof.measure().valid,
          "ID while held: {}".format(" ".join("{:02x}".format(b) for b in held)))  # fmt: skip
finally:
    regs.reg_write(regs.REG_SOFT_RESET, 0x01)  # released either way
    sleep_ms(2)
    rest(VL53L1XRegs(vl_conn()))

# %% [markdown]
# **Matches** `test_continuous_ranging`: five continuous samples, all valid and at your hand.
#
# **What to do:** hold your hand flat, palm down, about 15 cm (6 in) above the sensor until the
# cell finishes.

# %%
tof = on_board(VL53L1X)
try:
    tof.start()
    results = [tof.read() for _ in range(5)]
    tof.stop()
    for res in results:
        print(res)
    dists = [res.distance_mm for res in results]
    check("test_continuous_ranging: all valid", all(res.valid for res in results))
    check("test_continuous_ranging: at your hand", all(at_hand(d) for d in dists),
          "{} mm".format(dists))  # fmt: skip
    check("test_continuous_ranging: stopped", not tof.ranging and tof.last is results[-1])
finally:
    rest(tof)

# %% [markdown]
# **Matches** `test_one_shot_measure`: one valid sample at your hand, then no further samples
# (the part went back to standby).
#
# **What to do:** hold your hand flat, palm down, about 15 cm (6 in) above the sensor until the
# cell finishes.

# %%
tof = on_board(VL53L1X)
try:
    res = tof.measure()
    print(res)
    check("test_one_shot_measure: valid, at your hand", res.valid and at_hand(res.distance_mm),
          "{} mm".format(res.distance_mm))  # fmt: skip
    sleep_ms(300)
    check("test_one_shot_measure: no sample after it", not tof.data_ready())
finally:
    rest(tof)

# %% [markdown]
# **Matches** `test_read_times_out_when_not_ranging`.
#
# **What to do:** nothing. Takes 0.3 s.

# %%
tof = on_board(VL53L1X)
try:
    check_raises("test_read_times_out_when_not_ranging", OSError, tof.read, 300)
finally:
    rest(tof)

# %% [markdown]
# **Matches** `test_invalid_status_is_reported`. A real failed sample needs a target the part
# cannot see; here `ReplyingBus` sets the status bits of a real result block to 4.
#
# **What to do:** nothing.

# %%
failed = {VL53L1XRegs.REG_RESULT: lambda data: bytes([(data[0] & 0xE0) | 4]) + data[1:]}
tof = on_board(VL53L1X, ReplyingBus(bus, failed))
try:
    res = tof.measure()
    print(res)
    check("test_invalid_status_is_reported",
          not res.valid and (res.status, res.status_name) == (2, "signal failure"))  # fmt: skip
finally:
    rest(tof)

# %% [markdown]
# **Matches** `test_distance_mode_switch_keeps_the_budget`, plus a short-mode sample at the
# target (short mode reaches about 1.3 m).
#
# **What to do:** hold your hand flat, palm down, about 15 cm (6 in) above the sensor until the
# cell finishes.

# %%
tof = on_board(VL53L1X)
try:
    tof.distance_mode = "short"
    res = tof.measure()
    print("short mode:", res)
    check("test_distance_mode_switch_keeps_the_budget",
          tof.distance_mode == "short" and tof.timing_budget_ms == 100
          and tof.reg_read(tof.REG_TIMEOUT_A, 2) == tof.TIMING_BUDGETS["short"][100][0]
          and tof.reg_read(tof.REG_WOI_SD0, 2) == 0x0705)  # fmt: skip
    check("short mode ranges your hand", res.valid and at_hand(res.distance_mm),
          "{} mm".format(res.distance_mm))  # fmt: skip
finally:
    rest(tof)

# %% [markdown]
# **Matches** `test_budget_outside_the_mode_is_refused`. The real part has no write log, so the
# check is that the refused mode change left the registers as they were.
#
# **What to do:** nothing.

# %%
tof = on_board(VL53L1X)
try:
    check_raises("20 ms refused in long mode", ValueError, setattr, tof, "timing_budget_ms", 20)
    tof.distance_mode = "short"
    tof.timing_budget_ms = 20
    before = tof.read_mem(tof.REG_PHASECAL_TIMEOUT, 0x7C - 0x4B)
    check_raises("long mode refused at 20 ms", ValueError, setattr, tof, "distance_mode", "long")
    after = tof.read_mem(tof.REG_PHASECAL_TIMEOUT, 0x7C - 0x4B)
    check("test_budget_outside_the_mode_is_refused",
          before == after and tof.distance_mode == "short" and tof.timing_budget_ms == 20)  # fmt: skip
    res = tof.measure()
    print("short mode, 20 ms:", res)
finally:
    rest(tof)

# %% [markdown]
# **Matches** `test_settings_refused_while_ranging`.
#
# **What to do:** nothing.

# %%
tof = on_board(VL53L1X)
try:
    tof.start()
    for name, value in (("timing_budget_ms", 50), ("distance_mode", "short"),
                        ("inter_measurement_ms", 200), ("offset_mm", 5)):  # fmt: skip
        check_raises("refused while ranging: {}".format(name), RuntimeError,
                     setattr, tof, name, value)  # fmt: skip
    check_raises("refused while ranging: measure", RuntimeError, tof.measure)
    tof.stop()
    tof.timing_budget_ms = 50
    check("test_settings_refused_while_ranging", tof.timing_budget_ms == 50)
finally:
    rest(tof)

# %% [markdown]
# **Matches** `test_inter_measurement_period`.
#
# **What to do:** nothing.

# %%
tof = on_board(VL53L1X)
try:
    tof.inter_measurement_ms = 250
    ok = tof.inter_measurement_ms == 250
    check_raises("period shorter than the budget refused", ValueError,
                 setattr, tof, "inter_measurement_ms", 50)  # fmt: skip
    tof.timing_budget_ms = 500
    check("test_inter_measurement_period", ok and tof.inter_measurement_ms == 500,
          "osc word {:#06x}".format(tof.reg_read(tof.REG_OSC_CALIBRATE, 2)))  # fmt: skip
finally:
    rest(tof)

# %% [markdown]
# **Matches** `test_roi_set`, plus a 4×4 sample: the narrowest region still ranges your hand
# (the datasheet gives 4×4 a 15° field of view).
#
# **What to do:** hold your hand flat, palm down, about 15 cm (6 in) above the sensor until the
# cell finishes.

# %%
tof = on_board(VL53L1X)
try:
    tof.roi_set(4, 4)
    ok = tof.roi == (4, 4) and tof.roi_center == 199
    res = tof.measure()
    print("ROI 4x4:", res)
    tof.roi_set(8, 8, center=167)
    ok = ok and tof.roi == (8, 8) and tof.roi_center == 167
    check_raises("centre refused with a 16x16 ROI", ValueError, tof.roi_set, 16, 16, 167)
    check("test_roi_set", ok)
    check("a 4x4 ROI ranges your hand", res.valid and at_hand(res.distance_mm),
          "{} mm".format(res.distance_mm))  # fmt: skip
finally:
    rest(tof)

# %% [markdown]
# **Matches** `test_offset_is_applied`: five pairs of samples taken back to back, one with no
# offset and one with +50 mm, so a slowly moving hand cancels out. The mean difference should be
# 50 mm. (A negative offset larger than the distance reads 0: the register is unsigned.)
#
# **What to do:** hold your hand flat, palm down, about 15 cm (6 in) above the sensor until the
# cell finishes.

# %%
tof = on_board(VL53L1X)
try:
    diffs = []
    for _ in range(5):
        tof.offset_mm = 0
        plain = tof.measure().distance_mm
        tof.offset_mm = 50
        diffs.append(tof.measure().distance_mm - plain)
    print("+50 mm offset minus no offset, each pair:", diffs)
    check("test_offset_is_applied",
          tof.offset_mm == 50 and abs(mean(diffs) - 50) <= 15,
          "{:.1f} mm".format(mean(diffs)))  # fmt: skip
finally:
    rest(tof)

# %% [markdown]
# **Matches** `test_xtalk_set`.
#
# **What to do:** nothing.

# %%
tof = on_board(VL53L1X)
try:
    tof.xtalk_cps = 1000
    check("test_xtalk_set",
          tof.xtalk_cps == 1000 and tof.reg_read(tof.REG_XTALK_X_GRADIENT, 2) == 0)  # fmt: skip
    tof.xtalk_cps = 0
finally:
    rest(tof)

# %% [markdown]
# **Matches** `test_temperature_update`, plus a valid sample afterwards.
#
# **What to do:** hold your hand flat, palm down, about 15 cm (6 in) above the sensor until the
# cell finishes.

# %%
tof = on_board(VL53L1X)
try:
    tof.temperature_update()
    vhv = (tof.reg_read(tof.REG_VHV_LOOP_BOUND), tof.reg_read(tof.REG_VHV_START))
    res = tof.measure()
    print("after the update:", res)
    check("test_temperature_update", vhv == (0x09, 0) and not tof.ranging and res.valid)
finally:
    rest(tof)

# %% [markdown]
# **Matches** `test_reset_restores_the_constructor_settings`.
#
# **What to do:** hold your hand flat, palm down, about 15 cm (6 in) above the sensor until the
# cell finishes.

# %%
tof = on_board(VL53L1X, distance_mode="short", timing_budget_ms=50)
try:
    tof.timing_budget_ms = 200
    tof.reset()
    res = tof.measure()
    print("after the reset:", res)
    check("test_reset_restores_the_constructor_settings",
          tof.booted() and (tof.distance_mode, tof.timing_budget_ms, tof.inter_measurement_ms)
          == ("short", 50, 50) and res.valid)  # fmt: skip
finally:
    rest(tof)

# %% [markdown]
# **Matches** `test_address_change`: move the part to `0x30`, check it answers only there, then
# move it back. If the cell fails partway the `finally` moves it back; a power cycle (unplug the
# board) also restores `0x29`.
#
# **What to do:** nothing.

# %%
tof = on_board(VL53L1X)
try:
    tof.address_set(0x30)
    mux.channel_select(7)
    seen = bus.scan()
    mux.channel_deselect()
    print("channel 7 after the move:", [hex(a) for a in seen])
    res = tof.measure()
    ok = 0x30 in seen and VL_ADDR not in seen and tof.addr == 0x30 and res.valid
    check_raises("reset refused away from 0x29", RuntimeError, tof.reset)
    tof.address_set(0x29)
    mux.channel_select(7)
    seen = bus.scan()
    mux.channel_deselect()
    check("test_address_change", ok and VL_ADDR in seen and 0x30 not in seen,
          "back at {}".format([hex(a) for a in seen]))  # fmt: skip
finally:
    mux.channel_select(7)
    if 0x30 in bus.scan():
        VL53L1XRegs(vl_conn(addr=0x30)).reg_write(VL53L1XRegs.REG_I2C_ADDRESS, VL_ADDR)
        info("moved the part back to 0x29 in the cleanup")
    mux.channel_deselect()
    rest(VL53L1XRegs(vl_conn()))

# %% [markdown]
# ## Hardware-only checks

# %% [markdown]
# The continuous-mode period. With a 50 ms budget and a 100 ms period the samples should come
# every 100 ms. The cell times ten interrupt-line edges, polling every ms.
#
# **What to do:** nothing. Takes about 1.5 s.

# %%
tof = on_board(VL53L1X, timing_budget_ms=50, inter_measurement_ms=100)
try:
    tof.start()
    stamps = []
    for _ in range(11):
        tof.wait_ready()
        stamps.append(time.ticks_ms())
        tof.read_result()
        tof.clear_interrupt()
    tof.stop()
    gaps = [time.ticks_diff(b, a) for a, b in zip(stamps, stamps[1:])]
    print("gaps between samples, ms:", gaps)
    check("samples come every ~100 ms", 85 <= mean(gaps) <= 115, "{:.1f} ms".format(mean(gaps)))
finally:
    rest(tof)

# %% [markdown]
# Short and long mode agree on the same target: five pairs, one sample in each mode, back to
# back.
#
# **What to do:** hold your hand flat, palm down, about 15 cm (6 in) above the sensor until the
# cell finishes.

# %%
tof = on_board(VL53L1X)
try:
    diffs = []
    for _ in range(5):
        tof.distance_mode = "long"
        long_mm = tof.measure().distance_mm
        tof.distance_mode = "short"
        diffs.append(tof.measure().distance_mm - long_mm)
    print("short minus long, each pair:", diffs)
    check("short and long mode agree", abs(mean(diffs)) <= 30, "{:+.1f} mm".format(mean(diffs)))
finally:
    rest(tof)

# %% [markdown]
# The distance follows a real change: over 5 seconds the valid samples must span at least
# 40 mm. The cell prints a sample every half second.
#
# **What to do:** start with your hand about 15 cm (6 in) above the sensor. When the cell prints
# **move now**, bring your hand slowly down to about 5 cm (2 in) and back up to 15 cm.

# %%
tof = on_board(VL53L1X, timing_budget_ms=50, inter_measurement_ms=100)
try:
    tof.start()
    print("move now: slowly down to about 5 cm and back up to 15 cm")
    seen_mm = []
    for i in range(50):
        res = tof.read()
        if res.valid:
            seen_mm.append(res.distance_mm)
        if i % 5 == 0:
            print("  {:>4.1f} s: {:>5} mm {}".format(i / 10, res.distance_mm, res.status_name))
    tof.stop()
    lo, hi = (min(seen_mm), max(seen_mm)) if seen_mm else (None, None)
    print("valid samples: {}, closest {} mm, farthest {} mm".format(len(seen_mm), lo, hi))
    check("the distance followed your hand", lo is not None and hi - lo >= 40,
          "{}..{} mm".format(lo, hi))  # fmt: skip
finally:
    rest(tof)

# %% [markdown]
# ## Diagnostics
# These print what the part does in cases the datasheet and the ULD leave open; the TODO cell at
# the top records what they showed on the bench. They do not count towards the tally. Rerun them
# on a new part or board.

# %% [markdown]
# ### Which configuration registers does the part change after loading?
# The driver writes the 91-byte default configuration in one sequential write. This reads
# `0x002D..0x0087` straight back after the write, and again after the constructor finished
# (its calibration cycle and its own settings), and lists the registers that differ.
#
# **What to do:** nothing.

# %%
regs = VL53L1XRegs(vl_conn())
rest(regs)
regs.write_mem(regs.DEFAULT_CONFIG_START, regs.DEFAULT_CONFIG)
back = regs.read_mem(regs.DEFAULT_CONFIG_START, len(regs.DEFAULT_CONFIG))
diff = [(i + 0x2D, a, b) for i, (a, b) in enumerate(zip(regs.DEFAULT_CONFIG, back)) if a != b]
print("straight after the write, differs at:",
      ["{:#06x}: wrote {:#04x} read {:#04x}".format(*d) for d in diff] or "nothing")  # fmt: skip
tof = on_board(VL53L1X)
after = tof.read_mem(tof.DEFAULT_CONFIG_START, len(tof.DEFAULT_CONFIG))
diff = [(i + 0x2D, a, b) for i, (a, b) in enumerate(zip(tof.DEFAULT_CONFIG, after)) if a != b]
print("after the constructor, differs at:",
      ["{:#06x}: {:#04x} -> {:#04x}".format(*d) for d in diff] or "nothing")  # fmt: skip
rest(tof)

# %% [markdown]
# ### The oscillator word and the real period
# The ULD writes the period as `osc × ms × 1.075` and reads it back with 1.065. This times the
# real period for 100 ms and 200 ms settings, with a 33 ms budget, and prints the factor that
# would make the setting exact.
#
# **What to do:** nothing. Takes about 4 s.

# %%
tof = on_board(VL53L1X, timing_budget_ms=33)
try:
    osc = tof.reg_read(tof.REG_OSC_CALIBRATE, 2)
    print("oscillator calibration word {:#06x} (low 10 bits {})".format(osc, osc & 0x3FF))
    for ms in (100, 200):
        tof.inter_measurement_ms = ms
        tof.start()
        stamps = []
        for _ in range(11):
            tof.wait_ready()
            stamps.append(time.ticks_ms())
            tof.clear_interrupt()
        tof.stop()
        real = mean([time.ticks_diff(b, a) for a, b in zip(stamps, stamps[1:])])
        print("set {} ms: real {:.1f} ms -> exact factor {:.3f} (driver 1.075)".format(
            ms, real, 1.075 * ms / real))  # fmt: skip
finally:
    rest(tof)

# %% [markdown]
# ### Is the interrupt line still active right after a clear?
# The VL53L4CD driver waits 2 ms after clearing, because its interrupt line read stale. This
# reads the line at once and every 0.2 ms after a clear, five times over.
#
# **What to do:** nothing.

# %%
tof = on_board(VL53L1X, timing_budget_ms=50, inter_measurement_ms=100)
try:
    tof.start()
    for trial in range(5):
        tof.wait_ready()
        tof.read_result()
        tof.clear_interrupt()
        line = []
        for _ in range(5):
            line.append(int(tof.data_ready()))
            sleep_us(200)
        print("trial {}: data_ready right after the clear, every 0.2 ms: {}".format(trial, line))
    tof.stop()
finally:
    rest(tof)

# %% [markdown]
# ### The first samples after a start
# Some drivers drop the first sample after a start. This prints the first three samples of five
# starts.
#
# **What to do:** hold your hand flat, palm down, about 15 cm (6 in) above the sensor until the
# cell finishes.

# %%
tof = on_board(VL53L1X)
try:
    for trial in range(5):
        tof.start()
        first = [tof.read() for _ in range(3)]
        tof.stop()
        print("start {}: {}".format(trial, ", ".join(
            "{} mm {}".format(res.distance_mm, res.status_name) for res in first)))  # fmt: skip
finally:
    rest(tof)

# %% [markdown]
# ### Repeatability against the timing budget
# Twenty samples at 33 ms and at 200 ms (long mode): the spread should shrink with the longer
# budget (datasheet Figure 6). A hand is not a steady target, so expect more spread than the
# datasheet's few mm; the difference between the two budgets is what counts.
#
# **What to do:** hold your hand flat, palm down, about 15 cm (6 in) above the sensor until the
# cell finishes.

# %%
tof = on_board(VL53L1X)
try:
    for tb in (33, 200):
        tof.timing_budget_ms = tb
        d = [tof.measure().distance_mm for _ in range(20)]
        m = mean(d)
        sd = (sum((x - m) ** 2 for x in d) / len(d)) ** 0.5
        print("{:>3} ms: mean {:.1f} mm, standard deviation {:.1f} mm, range {}..{}".format(
            tb, m, sd, min(d), max(d)))  # fmt: skip
finally:
    rest(tof)

# %% [markdown]
# ### The ULD-only budgets
# The ULD offers 15 ms in short mode and 20 ms in long mode, which the datasheet does not. This
# writes those timing words directly and takes five samples each.
#
# **What to do:** hold your hand flat, palm down, about 15 cm (6 in) above the sensor until the
# cell finishes.

# %%
tof = on_board(VL53L1X)
try:
    for mode, table in tof.ULD_ONLY_BUDGETS.items():
        for ms, (word_a, word_b) in table.items():
            tof.distance_mode = mode
            tof.reg_write(tof.REG_TIMEOUT_A, word_a, 2)
            tof.reg_write(tof.REG_TIMEOUT_B, word_b, 2)
            samples = [tof.measure(timeout_ms=500) for _ in range(5)]
            print("{} mode, {} ms: {}".format(mode, ms, ", ".join(
                "{} mm {}".format(res.distance_mm, res.status_name) for res in samples)))  # fmt: skip
            tof.timing_budget_ms = 100 if mode == "long" else 50
finally:
    rest(tof)

# %% [markdown]
# ### Does a soft reset restore the address, and how long is the boot?
# Measured 2026-11-14: with the part at `0x30`, asserting the reset made it stop answering at
# `0x30` at once, and the release had to go to `0x29`. This asserts the reset at `0x30`, scans
# channel 7, releases it wherever it answers, then times three boots after a reset at `0x29`.
# The cleanup always releases the reset and moves the part back to `0x29`.
#
# **What to do:** nothing.

# %%
def release_everywhere():
    """Release a held reset and move the part back to 0x29, wherever it answers."""
    for addr in (0x30, VL_ADDR):
        try:
            r = VL53L1XRegs(vl_conn(addr=addr))
            r.reg_write(r.REG_SOFT_RESET, 0x01)
            sleep_ms(2)
            if addr != VL_ADDR:
                r.reg_write(r.REG_I2C_ADDRESS, VL_ADDR)
        except OSError:
            pass  # nothing at this address


tof = on_board(VL53L1X)
try:
    tof.address_set(0x30)
    tof.reg_write(tof.REG_SOFT_RESET, 0x00)  # asserted at 0x30
    mux.channel_select(7)
    seen = bus.scan()
    mux.channel_deselect()
    print("reset asserted at 0x30; channel 7 answers at:", [hex(a) for a in seen])
    for addr in (0x30, VL_ADDR):
        try:
            VL53L1XRegs(vl_conn(addr=addr)).reg_write(VL53L1XRegs.REG_SOFT_RESET, 0x01)
            print("released the reset at", hex(addr))
            break
        except OSError as e:
            print("no answer at {}: {}".format(hex(addr), e))
    sleep_ms(5)
    mux.channel_select(7)
    seen = bus.scan()
    mux.channel_deselect()
    print("after the release, channel 7 answers at:", [hex(a) for a in seen])
    regs = VL53L1XRegs(vl_conn())
    for trial in range(3):
        regs.reg_write(regs.REG_SOFT_RESET, 0x00)
        sleep_us(regs.T_SOFT_RESET_US)
        regs.reg_write(regs.REG_SOFT_RESET, 0x01)
        start, polls = time.ticks_us(), 0
        while not regs.reg_read(regs.REG_FIRMWARE_STATUS) & 1 and polls < 1000:
            polls += 1
        print("reset {}: booted after {:.2f} ms ({} polls)".format(
            trial, time.ticks_diff(time.ticks_us(), start) / 1000, polls))  # fmt: skip
finally:
    release_everywhere()
    rest(on_board(VL53L1X))

# %% [markdown]
# ### What does open space report?
# Twenty samples with nothing in range, counted by status. The datasheet says a missing or weak
# target gives a failure status, not a distance to trust.
#
# **What to do:** take your hand away and point the sensor at open space (more than 4 m clear,
# such as across a large room or out of a window). Skip the cell if there is no such space.

# %%
tof = on_board(VL53L1X)
try:
    counts = {}
    for _ in range(20):
        res = tof.measure()
        key = "{} (raw {})".format(res.status_name, res.status_raw)
        counts[key] = counts.get(key, 0) + 1
        last_mm = res.distance_mm
    for key, n in counts.items():
        print("{:>2} x {}".format(n, key))
    print("last distance register value:", last_mm, "mm")
finally:
    rest(tof)

# %% [markdown]
# ### What does the sensor see? A live readout
# Every 0.5 s for 15 s: one sample with the full 16×16 ROI (distance, status, signal, ambient,
# SPADs) and one with the central 4×4 ROI. Added 2026-11-14, when a protective film left on the
# sensor made a hand at 15 cm read about 7 cm: something fixed in front of the sensor shows as a
# distance that does not follow the hand.
#
# **What to do:** follow the `--- now:` lines: first 5 s nothing above the sensor, next 5 s your
# hand still about 15 cm (6 in) above it, last 5 s move your hand slowly down to about 5 cm.

# %%
tof = on_board(VL53L1X)
phases = ((0, "take your hand away (nothing above the sensor)"),
          (10, "hold your hand still about 15 cm (6 in) above the sensor"),
          (20, "move your hand slowly down to about 5 cm (2 in)"))  # fmt: skip
try:
    for i in range(30):
        for at, text in phases:
            if i == at:
                print("--- now:", text)
        tof.roi_set(16, 16)
        full = tof.measure()
        tof.roi_set(4, 4)
        centre = tof.measure()
        print("{:>4.1f} s  16x16: {:>5} mm {:<12} sig {:>6} amb {:>5} SPADs {:>3} | 4x4: {:>5} mm {}".format(
            i / 2, full.distance_mm, full.status_name[:12], full.signal_kcps, full.ambient_kcps,
            full.spads, centre.distance_mm, centre.status_name[:12]))  # fmt: skip
        sleep_ms(300)
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

# %% [raw]
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

# %%
