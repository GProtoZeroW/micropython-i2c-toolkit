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
# > The last section, **`# Hardware validation`**, connects to the real board, sends the SHTC3
# > commands on I2C bus 0, and has a step where you breathe on the sensor. *Run All* and `jlx run` reach that section.
# >
# > - **Safe to run on its own:** everything above `# Hardware validation`. It runs on the host
# >   against a fake part, writes the module and its tests, and needs no board.
# > - **Hardware section:** run it one cell at a time, reading the **What to do** line above
# >   each cell first.
#
# **Runs on the host first, the board last**, like the other dev notebooks. Everything down to
# `# Hardware validation` runs on CPython against `micropython_i2c.fakes.FakeSHTC3`. The last
# section connects to the FeatherS3, uploads the freshly written package, checks the real SHTC3
# with a matching test for every unit test above, then disconnects.

# %% [markdown]
# # Hardware and configuration
# What this notebook was validated with. The same table opens `# Hardware validation`.
#
# | | Host run (simulation) | Hardware run |
# |---|---|---|
# | **Runs on** | CPython 3.13.15 in the micropylab kernel (1.0.0a1), no board | Unexpected Maker FeatherS3 (ESP32-S3), MicroPython 1.28.0 (v1.28.0, 2026-04-06), USB `/dev/ttyACM0` |
# | **I2C bus** | `FakeBus` | `machine.I2C(0)`, **bus 0**, the STEMMA QT connector (scl=9 sda=8, on-board pull-ups), 100 kHz (`bus_open("feathers3", 0)`) |
# | **Mux** | none | **none**: the SHTC3 is always at `0x70`, the mux's own address |
# | **Sensor** | `FakeSHTC3` at `0x70`: idle (as after power-up), 25 C, 50 %RH, ID `0x0807`, measures in 11.0 ms (normal) / 0.5 ms (low power) | SparkFun Qwiic SHTC3 at `0x70` |
# | **Other parts used** | none | none: the SHTC3 was the only part connected to the board. The TMP102 cross-check (bus 1, mux channel 2) was skipped in this run; an earlier run with it connected read 23.43 C against 22.88 C |
# | **Datasheet** | Sensirion SHTC3, version 4, December 2022 | same |
# | **Result** | 15 unit tests pass | 35 of 35 hardware checks passed (2026-11-14) |
#
# The simulation and the bench differ on purpose here: a mux in front of the SHTC3 would have been
# the better test, but the bench's mux has the same address (`0x70`), so the SHTC3 sits alone on bus
# 0. The fake also refuses wake-up for 2 ms after a NACK (the real part: under 1 ms) so a test can
# see it at ms resolution.

# %% [markdown]
# # TODO:
# Nothing open.
#
# Measured on the bench (2026-11-14). The `## Diagnostics` cells print each of these again:
# - In sleep the part answers a scan and the wake-up command, and NACKs every other command.
# - Right after such a NACKed command, wake-up is NACKed too, for under 1 ms (one retry ~1 ms
#   later got through in every trial). The driver retries wake-up up to 3 times, 1 ms apart.
# - A wake-up command to a part that is already awake is acknowledged; a read with no result
#   waiting is NACKed.
# - With a quiet bus the part answered after 11.0 ms (normal) and 0.5 ms (low power); the driver
#   waits the datasheet maximums, 12.1 ms and 0.8 ms. On the ESP32 each NACKed transfer costs
#   about 0.8 ms, so polling cannot time a measurement finely.
# - Clock stretching works on the ESP32 (a stretched read took 12.9 ms); the driver still waits
#   instead, which works on every port.
# - Temperature 23.43 C against the TMP102's 22.88 C beside it.
#
# Decided (2026-11-14): the SHTC3 is on **I2C bus 0** (STEMMA QT, scl=9 sda=8), the only part on
# that bus, not behind the mux. It is always at `0x70`, the mux's own address, so it cannot
# share the mux's segment. Datasheet: Sensirion SHTC3, version 4, December 2022
# (`docs/datasheets/Datasheet_SHTC3.pdf`).

# %% [markdown]
# # Python file
# location: `src/micropython_i2c/devices/sensors/environmental/shtc3.py`
#
# tests: `tests/test_shtc3.py`

# %% [markdown]
# ## header

# %%
# %%extract ../../../src/micropython_i2c/devices/sensors/environmental/shtc3.py
"""Sensirion SHTC3 humidity and temperature sensor (datasheet version 4, December 2022).

Usage on the board::

    from micropython_i2c import I2CConnection
    from micropython_i2c.devices.sensors.environmental.shtc3 import SHTC3

    sht = SHTC3(I2CConnection(bus, 0x70, lock=bus_lock))
    t_c, rh = sht.measure()         # -> (23.73, 62.97)

Two classes:

* ``SHTC3Cmds``: datasheet facts only (commands, timings, the ID pattern), no methods.
  Bring-up uses it to drive ``CommandI2CBase`` directly against the part.
* ``SHTC3``: the driver.

Command-driven: every operation is a 16-bit command, and a read returns 16-bit words, each
followed by a CRC-8 (poly 0x31, init 0xFF, the ``CommandI2CBase`` default). The part sleeps
between operations: the driver wakes it, works, and always puts it back to sleep. In sleep it
answers only the wake-up command. While it measures it NACKs everything.

The address is fixed at ``0x70``, the TCA9548A's default address too: never put the two on
one segment.

Generated by ``device_notebooks/sensors/environmental/shtc3_devNB``: edit the notebook, not
this file.
"""

# %% [markdown]
# # Imports
# ## External
# None: the driver runs on MicroPython and uses only this package.
#
# ## Internal
# `CommandI2CBase` for commands and CRC-checked words; `sleep_us` from `compat` for the waits.

# %%
# %%extract ../../../src/micropython_i2c/devices/sensors/environmental/shtc3.py -a
from micropython_i2c.compat import sleep_us
from micropython_i2c.node import I2CDevice
from micropython_i2c.protocol import CommandI2CBase

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
# %%writefile ../../../tests/test_shtc3.py
import pytest

from micropython_i2c import I2CConnection
from micropython_i2c.devices.sensors.environmental.shtc3 import SHTC3, SHTC3Cmds
from micropython_i2c.fakes import FakeBus, FakeRegisterPart, FakeSHTC3

# %% [markdown]
# Notebook-only: the imports the validation and test cells use. The classes under test are the
# ones defined in this notebook, not the module's.

# %%
import pytest

from micropython_i2c import I2CConnection
from micropython_i2c.fakes import FakeBus, FakeRegisterPart, FakeSHTC3

# %% [markdown]
# # Intent
# **Purpose:** read relative humidity and temperature from a Sensirion SHTC3.
#
# **Why it exists:** the SHTC3 has no registers. Every operation is a command sequence (wake up,
# command, wait, read, sleep), a forgotten sleep costs 45 µA idle instead of 0.3 µA, and a read
# at the wrong moment is NACKed rather than answered late.
#
# **Used by:** the routing table and manager (`"driver": "SHTC3"`) and the bench (SparkFun Qwiic
# SHTC3 on I2C bus 0, no mux).
#
# **Not its job:** the general-call reset (it resets every part on the bus that supports it),
# clock stretching (the driver waits instead), the heater (the SHTC3 has none).
#
# ```python
# sht = SHTC3(I2CConnection(bus, 0x70, lock=lock))   # checks the ID, leaves it asleep
# sht.measure()                     # (23.73, 62.97)  C, %RH
# sht.measure(low_power=True)       # 0.8 ms instead of 12.1 ms, noisier
# ```
#
# ```mermaid
# sequenceDiagram
#     participant D as driver
#     participant S as SHTC3
#     D->>S: wake-up 0x3517
#     Note over S: up to 240 µs
#     D->>S: measure 0x7866 (T first, no clock stretching)
#     Note over S: up to 12.1 ms, NACKs everything
#     D->>S: read 6 bytes
#     S-->>D: T MSB, T LSB, CRC, RH MSB, RH LSB, CRC
#     D->>S: sleep 0xB098
# ```
#
# Conversion (section 5.11): $T = -45 + 175\,S_T / 2^{16}$ °C, $RH = 100\,S_{RH} / 2^{16}$ %RH.
#
# ## What the module defines
#
# | Name | What it is |
# |---|---|
# | `SHTC3Cmds` | datasheet facts: commands (all eight measurement commands), timings, ID pattern, CRC examples |
# | `SHTC3` | the driver: identify on construction, `measure`, `read_id`, `wake`, `soft_reset`, codecs |

# %% [markdown]
# # Code
# ## TODO:
# Nothing

# %% [markdown]
# ## Public names

# %%
# %%extract ../../../src/micropython_i2c/devices/sensors/environmental/shtc3.py -a
__all__ = ["SHTC3", "SHTC3Cmds"]

# %% [markdown]
# ## `SHTC3Cmds`
# Every number here is from the datasheet. `CMD_MEASURE` maps `(low_power, rh_first,
# clock_stretch)` to the opcode (Table 11), so the driver picks a command by meaning, not by hex.
# The wake-up time is the power-up time `t_PU` (Figure 7 points there for it).


# %%
# %%extract ../../../src/micropython_i2c/devices/sensors/environmental/shtc3.py -a
class SHTC3Cmds(CommandI2CBase, I2CDevice):
    """Datasheet facts for the SHTC3, with no driver logic."""

    ALLOWED_ADDRS = (0x70,)  # fixed (Table 8)
    ADDR_DEFAULT = 0x70
    BAUD_NOMINAL = 400_000
    BAUD_MAX = 1_000_000  # Fast-mode Plus (section 3.2)
    BAUD_MIN = 10_000

    # Commands (Tables 9, 10, 12, 14).
    CMD_WAKEUP = 0x3517
    CMD_SLEEP = 0xB098
    CMD_SOFT_RESET = 0x805D
    CMD_READ_ID = 0xEFC8

    # Measurement commands (Table 11): (low_power, rh_first, clock_stretch) -> opcode. Each
    # one measures both temperature and humidity; the flags set the read order.
    CMD_MEASURE = {
        (False, False, True): 0x7CA2,
        (False, True, True): 0x5C24,
        (False, False, False): 0x7866,
        (False, True, False): 0x58E0,
        (True, False, True): 0x6458,
        (True, True, True): 0x44DE,
        (True, False, False): 0x609C,
        (True, True, False): 0x401A,
    }

    # Timing (Table 5), microseconds, maximum values.
    T_WAKEUP_US = 240  # power-up time t_PU, also the wake-up time
    T_SOFT_RESET_US = 240
    T_MEAS_US = {False: 12_100, True: 800}  # normal, low power (one part: 11.0 and 0.5 ms)

    # Right after a NACKed command, wake-up is NACKed too for under 1 ms (measured): retry.
    WAKE_TRIES = 3
    WAKE_RETRY_US = 1000

    # ID register (Table 15): bits 11 and 5..0 identify the SHTC3; the rest vary per part.
    ID_MASK = 0x083F
    ID_SHTC3 = 0x0807

    # CRC (Table 16): CommandI2CBase's defaults are Sensirion's. The datasheet's examples:
    CRC_VECTORS = ((b"\x00", 0xAC), (b"\xbe\xef", 0x92))
    # Figure 7: a humidity-first read, 63 %RH and 23.7 C as sent on the wire.
    EXAMPLE_PACKET = b"\xa1\x33\x1c\x64\x8b\xc7"

    TEMP_MIN = -40.0  # specified range (Table 2)
    TEMP_MAX = 125.0


# %% [markdown]
# ### validation/test
# A helper that puts a `FakeSHTC3` (idle, as after power-up, 25 C and 50 %RH) at `0x70` on a fake
# bus and binds a class to it. It returns the object and the fake part, and is shared by the
# validation cells and the test file.


# %%
# %%extract ../../../tests/test_shtc3.py -a
def on_fake(cls, **part_kw):
    """``(dev, part)``: ``cls`` bound to a fresh ``FakeSHTC3`` at 0x70 on a fake bus."""
    bus = FakeBus()
    part = bus.attach(0x70, FakeSHTC3(**part_kw))
    return cls(I2CConnection(bus, 0x70)), part


# %% [markdown]
# #### validation: nominal usage
# A measurement by hand through `CommandI2CBase`: wake up, measure, wait the maximum time, read
# two CRC-checked words, sleep.

# %%
cmds, part = on_fake(SHTC3Cmds)
cmds.cmd_send(cmds.CMD_WAKEUP)
sleep_us(cmds.T_WAKEUP_US)
cmds.cmd_send(cmds.CMD_MEASURE[(False, False, False)])
sleep_us(cmds.T_MEAS_US[False])
t_word, rh_word = cmds.words_read(2)
cmds.cmd_send(cmds.CMD_SLEEP)
print(f"T word {t_word:#06x}, RH word {rh_word:#06x} | commands sent:",
      [hex(c) for c in part.commands], "| asleep:", part.asleep)  # fmt: skip

# %% [markdown]
# #### validation: anti-nominal usage
# A read straight after the measurement command, without the wait, is NACKed: the part does not
# answer while it measures.

# %%
cmds, part = on_fake(SHTC3Cmds)
cmds.cmd_send(cmds.CMD_MEASURE[(False, False, False)])
try:
    cmds.words_read(2)  # no wait: still measuring
except OSError as err:
    print(f"raised OSError, as expected:\n    {err}")
else:
    raise AssertionError("a read during the measurement should have raised OSError")

# %% [markdown]
# #### unit

# %% [markdown]
# The CRC parameters reproduce both examples in Table 16.

# %%
# %%extract ../../../tests/test_shtc3.py -a --strip-ipytest
# %%ipytest


def test_crc_vectors_match_datasheet():
    for data, want in SHTC3Cmds.CRC_VECTORS:
        assert SHTC3Cmds.crc(data) == want


# %% [markdown]
# The measurement command table is Table 11: eight distinct opcodes, and the driver's default
# (normal mode, temperature first, no clock stretching) is `0x7866`.

# %%
# %%extract ../../../tests/test_shtc3.py -a --strip-ipytest
# %%ipytest


def test_measurement_command_table():
    assert len(set(SHTC3Cmds.CMD_MEASURE.values())) == 8
    assert SHTC3Cmds.CMD_MEASURE[(False, False, False)] == 0x7866
    assert SHTC3Cmds.CMD_MEASURE[(True, True, True)] == 0x44DE


# %% [markdown]
# Figure 7's packet decodes: both CRCs check, and the humidity-first words give 63 %RH and
# 23.7 C.

# %%
# %%extract ../../../tests/test_shtc3.py -a --strip-ipytest
# %%ipytest


def test_datasheet_example_packet_decodes():
    p = SHTC3Cmds.EXAMPLE_PACKET
    assert SHTC3Cmds.crc(p[0:2]) == p[2] and SHTC3Cmds.crc(p[3:5]) == p[5]
    rh_word, t_word = (p[0] << 8) | p[1], (p[3] << 8) | p[4]
    # Section 5.11's formulas, written out (the driver's codecs are tested below).
    assert 100 * rh_word / 65536 == pytest.approx(63.0, abs=0.05)
    assert -45 + 175 * t_word / 65536 == pytest.approx(23.7, abs=0.05)


# %% [markdown]
# Only `0x70` is allowed, and Fast-mode Plus is the clock ceiling.

# %%
# %%extract ../../../tests/test_shtc3.py -a --strip-ipytest
# %%ipytest


def test_address_and_clock_limits():
    assert SHTC3Cmds.addr_ok(0x70) and not SHTC3Cmds.addr_ok(0x71)
    assert SHTC3Cmds.baud_ok(1_000_000) and not SHTC3Cmds.baud_ok(3_400_000)


# %% [markdown]
# A word with a flipped bit fails its CRC, naming the word.

# %%
# %%extract ../../../tests/test_shtc3.py -a --strip-ipytest
# %%ipytest


def test_words_read_rejects_a_bad_crc():
    cmds, part = on_fake(SHTC3Cmds)
    cmds.cmd_send(cmds.CMD_READ_ID)
    part.corrupt = 1
    with pytest.raises(ValueError, match="word 0"):
        cmds.words_read(1)


# %% [markdown]
# ## `SHTC3`
# The driver wraps every operation in wake-up ... sleep, and puts the part to sleep even when the
# read fails. Wake-up is retried (up to 3 times, 1 ms apart): right after a NACKed command the
# part refuses it for a moment. The constructor reads the ID, which also leaves the part asleep (the datasheet
# says to send it to sleep after power-up), and refuses a part whose ID is not an SHTC3's. A CRC
# failure on that read counts as "not an SHTC3" too.


# %%
# %%extract ../../../src/micropython_i2c/devices/sensors/environmental/shtc3.py -a
class SHTC3(SHTC3Cmds):
    """SHTC3 driver.

    Raises:
        RuntimeError: From the constructor, if the part at the address is not an SHTC3.
    """

    def __init__(self, conn, **kwargs):
        super().__init__(conn, **kwargs)
        try:
            ident = self.read_id()
        except ValueError as e:  # a CRC failure: something else answered
            raise RuntimeError("not an SHTC3 at {:#04x} ({})".format(self.addr, e)) from None
        if ident & self.ID_MASK != self.ID_SHTC3:
            raise RuntimeError("not an SHTC3 at {:#04x} (ID {:#06x})".format(self.addr, ident))
        self.ident = ident

    # Codecs (section 5.11).

    @staticmethod
    def celsius_from_raw(word):
        """Temperature word -> C."""
        return -45 + 175 * word / 65536

    @staticmethod
    def humidity_from_raw(word):
        """Humidity word -> %RH."""
        return 100 * word / 65536

    @staticmethod
    def celsius_to_raw(celsius):
        """C -> temperature word.

        Raises:
            ValueError: If the word would not fit 16 bits.
        """
        word = int((celsius + 45) * 65536 / 175 + 0.5)
        if not 0 <= word <= 0xFFFF:
            raise ValueError("{} C not representable".format(celsius))
        return word

    @staticmethod
    def humidity_to_raw(rh):
        """%RH -> humidity word.

        Raises:
            ValueError: If ``rh`` is outside 0..100.
        """
        if not 0.0 <= rh <= 100.0:
            raise ValueError("{} %RH outside 0-100".format(rh))
        return min(0xFFFF, int(rh * 65536 / 100 + 0.5))

    # Operations. Each one wakes the part and always puts it back to sleep.

    def wake(self):
        """Send wake-up and wait for the part to come up.

        Retried up to ``WAKE_TRIES`` times, ``WAKE_RETRY_US`` apart: right after a NACKed
        command the part refuses wake-up too, for under 1 ms (measured).

        Raises:
            OSError: If every try is NACKed (nothing at the address, or it is measuring).
        """
        for attempt in range(self.WAKE_TRIES):
            try:
                self.cmd_send(self.CMD_WAKEUP)
                break
            except OSError:
                if attempt == self.WAKE_TRIES - 1:
                    raise
                sleep_us(self.WAKE_RETRY_US)
        sleep_us(self.T_WAKEUP_US)

    def _awake(self, command, wait_us, n_words):
        """Wake, send ``command``, wait ``wait_us``, read ``n_words`` (CRC-checked), sleep."""
        self.wake()
        try:
            self.cmd_send(command)
            sleep_us(wait_us)
            return self.words_read(n_words)
        finally:
            self.cmd_send(self.CMD_SLEEP)

    def read_id(self):
        """The 16-bit ID register."""
        return self._awake(self.CMD_READ_ID, 0, 1)[0]

    def identify(self):
        """True if the ID register matches the SHTC3 pattern."""
        return self.read_id() & self.ID_MASK == self.ID_SHTC3

    def measure_raw(self, low_power=False):
        """``(temperature_word, humidity_word)`` from one measurement.

        Waits the maximum measurement time instead of using clock stretching.
        """
        cmd = self.CMD_MEASURE[(low_power, False, False)]  # temperature first, no stretching
        t_word, rh_word = self._awake(cmd, self.T_MEAS_US[low_power], 2)
        return t_word, rh_word

    def measure(self, low_power=False):
        """``(temperature_C, humidity_pct)`` from one measurement.

        Args:
            low_power: 0.8 ms instead of 12.1 ms, with poorer repeatability.

        Raises:
            ValueError: On a CRC mismatch in either word (the part is still put to sleep).
        """
        t_word, rh_word = self.measure_raw(low_power)
        return self.celsius_from_raw(t_word), self.humidity_from_raw(rh_word)

    def soft_reset(self):
        """Software reset (section 5.7), then back to sleep."""
        self.wake()
        self.cmd_send(self.CMD_SOFT_RESET)
        sleep_us(self.T_SOFT_RESET_US)
        self.cmd_send(self.CMD_SLEEP)


# %% [markdown]
# ### validation/test: codecs
# The codec cases `(word, C, %RH)`, shared by the unit tests below and copied to the board by the
# hardware section: both ends of the word, mid-scale, and Figure 7's temperature word.

# %%
# %%extract ../../../tests/test_shtc3.py -a
CODEC_CASES = [(0x0000, -45.0, 0.0), (0x8000, 42.5, 50.0), (0xFFFF, 129.99733, 99.99847),
               (0x648B, 23.73055, 39.27460)]  # fmt: skip

# %% [markdown]
# #### validation: nominal usage

# %%
for word, t, rh in CODEC_CASES:
    print(f"{word:#06x}: {SHTC3.celsius_from_raw(word):9.4f} C (want {t}) | "
          f"{SHTC3.humidity_from_raw(word):8.4f} %RH (want {rh})")  # fmt: skip
print("25 C ->", hex(SHTC3.celsius_to_raw(25.0)), "| 50 %RH ->", hex(SHTC3.humidity_to_raw(50.0)))

# %% [markdown]
# #### validation: anti-nominal usage
# 120 %RH does not exist.

# %%
try:
    SHTC3.humidity_to_raw(120.0)  # above 100 %RH
except ValueError as err:
    print(f"raised ValueError, as expected:\n    {err}")
else:
    raise AssertionError("120 %RH should have raised ValueError")

# %% [markdown]
# #### unit

# %% [markdown]
# The codecs decode the cases, and encode then decode returns the value to within one LSB.

# %%
# %%extract ../../../tests/test_shtc3.py -a --strip-ipytest
# %%ipytest


@pytest.mark.parametrize("word, t, rh", CODEC_CASES)
def test_codecs(word, t, rh):
    assert SHTC3.celsius_from_raw(word) == pytest.approx(t, abs=1e-4)
    assert SHTC3.humidity_from_raw(word) == pytest.approx(rh, abs=1e-4)
    assert SHTC3.celsius_from_raw(SHTC3.celsius_to_raw(t)) == pytest.approx(t, abs=175 / 65536)
    assert SHTC3.humidity_from_raw(SHTC3.humidity_to_raw(rh)) == pytest.approx(rh, abs=100 / 65536)


# %% [markdown]
# Values outside what a word can hold are refused.

# %%
# %%extract ../../../tests/test_shtc3.py -a --strip-ipytest
# %%ipytest


@pytest.mark.parametrize(
    "fn, value",
    [("celsius_to_raw", -50.0), ("celsius_to_raw", 131.0), ("humidity_to_raw", -1.0),
     ("humidity_to_raw", 101.0)],
)  # fmt: skip
def test_codecs_refuse_out_of_range(fn, value):
    with pytest.raises(ValueError):
        getattr(SHTC3, fn)(value)


# %% [markdown]
# ### validation/test: the driver on a fake part

# %% [markdown]
# #### validation: nominal usage

# %%
sht, part = on_fake(SHTC3, temp_c=23.7, rh=63.0)
print("ID", hex(sht.ident), "| identify:", sht.identify(), "| asleep after init:", part.asleep)
print("measure:", sht.measure(), "| low power:", sht.measure(low_power=True))
print("commands of the last measurement:", [hex(c) for c in part.commands[-3:]])

# %% [markdown]
# #### validation: anti-nominal usage
# Something at `0x70` that is not an SHTC3 (here a register part returning zeros) is refused.

# %%
bus = FakeBus()
bus.attach(0x70, FakeRegisterPart())
try:
    SHTC3(I2CConnection(bus, 0x70))
except RuntimeError as err:
    print(f"raised RuntimeError, as expected:\n    {err}")
else:
    raise AssertionError("a part that is not an SHTC3 should have raised RuntimeError")

# %% [markdown]
# #### unit

# %% [markdown]
# The constructor identifies the part and leaves it asleep.

# %%
# %%extract ../../../tests/test_shtc3.py -a --strip-ipytest
# %%ipytest


def test_identify_and_init_sleeps():
    sht, part = on_fake(SHTC3, id_word=0x5887)  # unspecified bits set
    assert sht.ident == 0x5887 and sht.identify()
    assert part.asleep and part.commands[-1] == SHTC3.CMD_SLEEP


# %% [markdown]
# A wrong ID, or a part whose answer fails the CRC, is refused.

# %%
# %%extract ../../../tests/test_shtc3.py -a --strip-ipytest
# %%ipytest


def test_init_refuses_a_part_that_is_not_an_shtc3():
    with pytest.raises(RuntimeError, match="ID 0x1234"):
        on_fake(SHTC3, id_word=0x1234)
    bus = FakeBus()
    bus.attach(0x70, FakeRegisterPart())
    with pytest.raises(RuntimeError, match="not an SHTC3"):
        SHTC3(I2CConnection(bus, 0x70))


# %% [markdown]
# A normal measurement sends wake-up, `0x7866`, sleep, and decodes the fake's values.

# %%
# %%extract ../../../tests/test_shtc3.py -a --strip-ipytest
# %%ipytest


def test_measure_normal():
    sht, part = on_fake(SHTC3, temp_c=23.7, rh=63.0)
    t, rh = sht.measure()
    assert t == pytest.approx(23.7, abs=0.01) and rh == pytest.approx(63.0, abs=0.01)
    assert part.commands[-3:] == [SHTC3.CMD_WAKEUP, 0x7866, SHTC3.CMD_SLEEP]
    assert part.asleep


# %% [markdown]
# A low-power measurement sends `0x609C` and decodes the same values.

# %%
# %%extract ../../../tests/test_shtc3.py -a --strip-ipytest
# %%ipytest


def test_measure_low_power():
    sht, part = on_fake(SHTC3, temp_c=-10.0, rh=5.0)
    t, rh = sht.measure(low_power=True)
    assert t == pytest.approx(-10.0, abs=0.01) and rh == pytest.approx(5.0, abs=0.01)
    assert part.commands[-2] == 0x609C


# %% [markdown]
# Ten measurements in a row each wait long enough: none is NACKed.

# %%
# %%extract ../../../tests/test_shtc3.py -a --strip-ipytest
# %%ipytest


def test_measure_waits_long_enough():
    sht, _ = on_fake(SHTC3)
    for _ in range(10):
        assert sht.measure() == pytest.approx((25.0, 50.0), abs=0.01)


# %% [markdown]
# A CRC failure raises, and the part is still put to sleep.

# %%
# %%extract ../../../tests/test_shtc3.py -a --strip-ipytest
# %%ipytest


def test_always_back_to_sleep_even_on_crc_error():
    sht, part = on_fake(SHTC3)
    part.corrupt = 1
    with pytest.raises(ValueError, match="checksum"):
        sht.measure()
    assert part.asleep and part.commands[-1] == SHTC3.CMD_SLEEP


# %% [markdown]
# A sleeping part refuses every command but wake-up, and right after that refuses wake-up for a
# moment too. The driver retries wake-up, so its operations still work.

# %%
# %%extract ../../../tests/test_shtc3.py -a --strip-ipytest
# %%ipytest


def test_commands_while_asleep_are_refused():
    sht, part = on_fake(SHTC3)
    assert part.asleep
    with pytest.raises(OSError):
        sht.cmd_send(SHTC3.CMD_READ_ID)
    with pytest.raises(OSError):  # straight after the refused command
        sht.cmd_send(SHTC3.CMD_WAKEUP)
    assert sht.identify()  # wake() retries


# %% [markdown]
# A soft reset leaves the part asleep and working.

# %%
# %%extract ../../../tests/test_shtc3.py -a --strip-ipytest
# %%ipytest


def test_soft_reset_leaves_it_asleep():
    sht, part = on_fake(SHTC3)
    sht.soft_reset()
    assert part.commands[-2:] == [SHTC3.CMD_SOFT_RESET, SHTC3.CMD_SLEEP]
    assert part.asleep and sht.identify()


# %% [markdown]
# # Hardware validation
#
# > ## ⚠️ STOP — do not run this section unattended
# >
# > **Everything below talks to the real board.** It connects to the FeatherS3, uploads the
# > package, and sends SHTC3 commands on I2C bus 0. One step asks you to breathe on the sensor.
# >
# > - Do **not** use *Run All*, *Run All Below* or `jlx run` on this section and walk away.
# > - Run it **one cell at a time**. Read the **What to do** line above each cell first.
# > - Each cell puts the SHTC3 back to sleep when it finishes, even if it fails.
# > - The section ends with `%board disconnect`. If you stop partway, run that cell yourself.
#
# **Hardware and configuration** (same as the table at the top):
#
# | | Host run (simulation) | Hardware run |
# |---|---|---|
# | **Runs on** | CPython 3.13.15 in the micropylab kernel (1.0.0a1), no board | Unexpected Maker FeatherS3 (ESP32-S3), MicroPython 1.28.0 (v1.28.0, 2026-04-06), USB `/dev/ttyACM0` |
# | **I2C bus** | `FakeBus` | `machine.I2C(0)`, **bus 0**, the STEMMA QT connector (scl=9 sda=8, on-board pull-ups), 100 kHz (`bus_open("feathers3", 0)`) |
# | **Mux** | none | **none**: the SHTC3 is always at `0x70`, the mux's own address |
# | **Sensor** | `FakeSHTC3` at `0x70`: idle (as after power-up), 25 C, 50 %RH, ID `0x0807`, measures in 11.0 ms (normal) / 0.5 ms (low power) | SparkFun Qwiic SHTC3 at `0x70` |
# | **Other parts used** | none | none: the SHTC3 was the only part connected to the board. The TMP102 cross-check (bus 1, mux channel 2) was skipped in this run; an earlier run with it connected read 23.43 C against 22.88 C |
# | **Datasheet** | Sensirion SHTC3, version 4, December 2022 | same |
# | **Result** | 15 unit tests pass | 35 of 35 hardware checks passed (2026-11-14) |
#
# The simulation and the bench differ on purpose here: a mux in front of the SHTC3 would have been
# the better test, but the bench's mux has the same address (`0x70`), so the SHTC3 sits alone on bus
# 0. The fake also refuses wake-up for 2 ms after a NACK (the real part: under 1 ms) so a test can
# see it at ms resolution.
#
# Layout: connect and set up, find the part, then **matching tests** (one per unit test above,
# same name, run on the real part and on MicroPython), hardware-only checks, **diagnostics**
# (they print what the part does and do not count towards the tally), then the tally and
# disconnect.

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
from micropython_i2c.devices.sensors.environmental.shtc3 import SHTC3, SHTC3Cmds
from micropython_i2c.devices.sensors.environmental.tmp102 import TMP102

check_reset()

# %% [markdown]
# Notebook-only: the bus and helpers. `rest(dev)` puts the part back to sleep from any state
# (mid-measurement too). `asleep(dev)` tells whether it is asleep by sending a command that is
# not wake-up: a sleeping part NACKs it. `CorruptingBus` flips one bit in the next read, for the
# CRC checks. `ReplyingBus` does the real read but hands back a fixed reply, standing in for a
# part that is not an SHTC3. If the bench is rewired, change the constants.
#
# **What to do:** nothing.

# %%
SHT_BUS = 0  # STEMMA QT, scl=9 sda=8: the SHTC3 alone
SHT_ADDR = 0x70
TMP_BUS, MUX_ADDR, TMP_ADDR = 1, 0x70, 0x48  # the TMP102, for the temperature check
TMP_PATH = [[MUX_ADDR, 2]]

bus = bus_open("feathers3", SHT_BUS, freq=100_000)
bus_lock = default_lock()


class CorruptingBus:
    """The bus, flipping one bit in the next ``corrupt`` reads from the SHTC3."""

    def __init__(self, inner):
        self.inner = inner
        self.corrupt = 0

    def __getattr__(self, name):
        return getattr(self.inner, name)

    def readfrom(self, addr, nbytes, *args):
        data = self.inner.readfrom(addr, nbytes, *args)
        if addr == SHT_ADDR and self.corrupt:
            self.corrupt -= 1
            data = bytes([data[0] ^ 0x01]) + data[1:]
        return data


class ReplyingBus:
    """The bus, but reads from the SHTC3 return ``reply`` (after doing the real read)."""

    def __init__(self, inner, reply):
        self.inner = inner
        self.reply = reply

    def __getattr__(self, name):
        return getattr(self.inner, name)

    def readfrom(self, addr, nbytes, *args):
        data = self.inner.readfrom(addr, nbytes, *args)
        return self.reply[:nbytes] if addr == SHT_ADDR else data


def sht_conn(b=None):
    """A connection to the SHTC3 on bus 0 (optionally on a wrapped bus)."""
    return I2CConnection(b or bus, SHT_ADDR, None, bus_lock)


def rest(dev):
    """Back to sleep from any state: wake it (waiting out a measurement), then sleep."""
    for _ in range(3):
        try:
            dev.cmd_send(dev.CMD_WAKEUP)
            break
        except OSError:
            sleep_ms(15)  # measuring: it NACKs until done
    sleep_us(dev.T_WAKEUP_US)
    dev.cmd_send(dev.CMD_SLEEP)


def asleep(dev):
    """True if the part NACKs a command other than wake-up (the sign of sleep)."""
    try:
        dev.cmd_send(dev.CMD_READ_ID)
    except OSError:
        return True
    rest(dev)  # it was awake (and now has an ID waiting): put it back to sleep
    return False


# %% [markdown]
# ## Find the part
# Bus 0 should answer at `0x70` only. If `0x70` is missing, check the Qwiic cable is in the
# STEMMA QT port.
#
# **What to do:** nothing.

# %%
seen = bus.scan()
print("bus {} answers at:".format(SHT_BUS), [hex(a) for a in seen])
check("SHTC3 ACKs at 0x70 on bus 0", SHT_ADDR in seen)
others = [a for a in seen if a != SHT_ADDR]
if others:
    info("also on bus 0: {}".format([hex(a) for a in others]))
cmds = SHTC3Cmds(sht_conn())
rest(cmds)

# %% [markdown]
# ## Matching tests: `SHTC3Cmds`
# One cell per unit test above, with the same name. Tests that need no bus traffic run too: they
# check the arithmetic on MicroPython itself.

# %% [markdown]
# **Matches** `test_crc_vectors_match_datasheet`.
#
# **What to do:** nothing. No bus traffic.

# %%
check("test_crc_vectors_match_datasheet",
      all(SHTC3Cmds.crc(d) == want for d, want in SHTC3Cmds.CRC_VECTORS))  # fmt: skip

# %% [markdown]
# **Matches** `test_measurement_command_table`.
#
# **What to do:** nothing. No bus traffic.

# %%
check("test_measurement_command_table",
      len(set(SHTC3Cmds.CMD_MEASURE.values())) == 8
      and SHTC3Cmds.CMD_MEASURE[(False, False, False)] == 0x7866
      and SHTC3Cmds.CMD_MEASURE[(True, True, True)] == 0x44DE)  # fmt: skip

# %% [markdown]
# **Matches** `test_datasheet_example_packet_decodes`, on MicroPython.
#
# **What to do:** nothing. No bus traffic.

# %%
p = SHTC3Cmds.EXAMPLE_PACKET
rh_ex = SHTC3.humidity_from_raw((p[0] << 8) | p[1])
t_ex = SHTC3.celsius_from_raw((p[3] << 8) | p[4])
check("test_datasheet_example_packet_decodes",
      SHTC3Cmds.crc(p[0:2]) == p[2] and SHTC3Cmds.crc(p[3:5]) == p[5]
      and abs(rh_ex - 63.0) < 0.05 and abs(t_ex - 23.7) < 0.05,
      "{:.2f} %RH, {:.2f} C".format(rh_ex, t_ex))  # fmt: skip

# %% [markdown]
# **Matches** `test_address_and_clock_limits`.
#
# **What to do:** nothing. No bus traffic.

# %%
check("test_address_and_clock_limits",
      SHTC3Cmds.addr_ok(0x70) and not SHTC3Cmds.addr_ok(0x71)
      and SHTC3Cmds.baud_ok(1_000_000) and not SHTC3Cmds.baud_ok(3_400_000))  # fmt: skip

# %% [markdown]
# **Matches** `test_words_read_rejects_a_bad_crc`: the real part's ID read verifies, and the same
# read with one bit flipped on the way in fails, naming word 0.
#
# **What to do:** nothing.

# %%
cbus = CorruptingBus(bus)
cmds = SHTC3Cmds(sht_conn(cbus))
try:
    cmds.cmd_send(cmds.CMD_WAKEUP)
    sleep_us(cmds.T_WAKEUP_US)
    cmds.cmd_send(cmds.CMD_READ_ID)
    good = cmds.words_read(1)[0]
    check("the real ID read verifies", good & cmds.ID_MASK == cmds.ID_SHTC3, "{:#06x}".format(good))
    cmds.cmd_send(cmds.CMD_READ_ID)
    cbus.corrupt = 1
    check_raises("test_words_read_rejects_a_bad_crc", ValueError, cmds.words_read, 1)
finally:
    rest(SHTC3Cmds(sht_conn()))

# %% [markdown]
# ## Matching tests: codecs

# %% [markdown]
# Notebook-only: the codec cases, copied to the board (it cannot import the test file, which
# needs pytest). The cell after it checks the copy against the notebook's.
#
# **What to do:** nothing.

# %%
CODEC_CASES = [(0x0000, -45.0, 0.0), (0x8000, 42.5, 50.0), (0xFFFF, 129.99733, 99.99847),
               (0x648B, 23.73055, 39.27460)]  # fmt: skip

# %% [markdown]
# Notebook-only: on the host, read the board's copy back with `%eval` and check it matches.
#
# **What to do:** nothing.

# %%
# %%host
board = get_ipython().run_line_magic("eval", "CODEC_CASES")
if board != [list(c) for c in CODEC_CASES]:
    raise AssertionError("the board's copy of CODEC_CASES differs from the notebook's")
print("board copy of CODEC_CASES matches the notebook's")

# %% [markdown]
# **Matches** `test_codecs`.
#
# **What to do:** nothing. No bus traffic.

# %%
for word, t, rh in CODEC_CASES:
    ok = (abs(SHTC3.celsius_from_raw(word) - t) < 1e-4
          and abs(SHTC3.humidity_from_raw(word) - rh) < 1e-4
          and abs(SHTC3.celsius_from_raw(SHTC3.celsius_to_raw(t)) - t) <= 175 / 65536
          and abs(SHTC3.humidity_from_raw(SHTC3.humidity_to_raw(rh)) - rh) <= 100 / 65536)  # fmt: skip
    check("test_codecs[{:#06x}]".format(word), ok)

# %% [markdown]
# **Matches** `test_codecs_refuse_out_of_range`.
#
# **What to do:** nothing. No bus traffic.

# %%
for fn, value in (("celsius_to_raw", -50.0), ("celsius_to_raw", 131.0),
                  ("humidity_to_raw", -1.0), ("humidity_to_raw", 101.0)):  # fmt: skip
    check_raises("test_codecs_refuse_out_of_range[{}-{}]".format(fn, value), ValueError,
                 getattr(SHTC3, fn), value)  # fmt: skip

# %% [markdown]
# ## Matching tests: `SHTC3`
# The driver on the real part. The fake holds exact values; the real part reads the room, so
# these check plausible ranges and agreement between readings.

# %% [markdown]
# **Matches** `test_identify_and_init_sleeps`: the ID matches the SHTC3 pattern, and the part is
# asleep after construction.
#
# **What to do:** nothing.

# %%
sht = SHTC3(sht_conn())
try:
    print("ID {:#06x}".format(sht.ident))
    check(
        "test_identify_and_init_sleeps", sht.identify() and asleep(sht), "{:#06x}".format(sht.ident)
    )
finally:
    rest(sht)

# %% [markdown]
# **Matches** `test_init_refuses_a_part_that_is_not_an_shtc3`. Nothing else is on the bus, so
# `ReplyingBus` stands in for a wrong part: the real transfers happen, but the reply is swapped
# for a wrong ID with a good CRC, then for zeros (a bad CRC). Both must be refused, and the real
# part must be fine afterwards.
#
# **What to do:** nothing.

# %%
wrong_id = bytes([0x12, 0x34, SHTC3Cmds.crc(b"\x12\x34")])
for name, reply in (("wrong ID", wrong_id), ("bad CRC", b"\x00\x00\x00")):
    check_raises("test_init_refuses_a_part_that_is_not_an_shtc3[{}]".format(name), RuntimeError,
                 SHTC3, sht_conn(ReplyingBus(bus, reply)))  # fmt: skip
rest(SHTC3Cmds(sht_conn()))
check("the real part is still an SHTC3", SHTC3(sht_conn()).identify())

# %% [markdown]
# **Matches** `test_measure_normal`: a plausible room reading, and the part asleep afterwards.
#
# **What to do:** keep your hands and breath away from the sensor.

# %%
sht = SHTC3(sht_conn())
try:
    t, rh = sht.measure()
    print("{:.2f} C, {:.2f} %RH".format(t, rh))
    check("test_measure_normal: temperature plausible", 10.0 <= t <= 40.0, "{:.2f} C".format(t))
    check("test_measure_normal: humidity plausible", 5.0 <= rh <= 95.0, "{:.2f} %RH".format(rh))
    check("test_measure_normal: asleep after", asleep(sht))
finally:
    rest(sht)

# %% [markdown]
# **Matches** `test_measure_low_power`: a low-power reading agrees with a normal one (low power
# has poorer repeatability, so the tolerance is loose).
#
# **What to do:** keep your hands and breath away from the sensor.

# %%
sht = SHTC3(sht_conn())
try:
    t_n, rh_n = sht.measure()
    t_lp, rh_lp = sht.measure(low_power=True)
    print(
        "normal {:.2f} C {:.2f} %RH | low power {:.2f} C {:.2f} %RH".format(t_n, rh_n, t_lp, rh_lp)
    )
    check("test_measure_low_power", abs(t_lp - t_n) < 1.0 and abs(rh_lp - rh_n) < 5.0)
finally:
    rest(sht)

# %% [markdown]
# **Matches** `test_measure_waits_long_enough`: ten measurements in a row, none NACKed, and they
# agree with each other.
#
# **What to do:** keep your hands and breath away from the sensor.

# %%
sht = SHTC3(sht_conn())
try:
    readings = [sht.measure() for _ in range(10)]
    ts, rhs = [r[0] for r in readings], [r[1] for r in readings]
    print("T {:.2f}..{:.2f} C, RH {:.2f}..{:.2f} %RH".format(min(ts), max(ts), min(rhs), max(rhs)))
    check("test_measure_waits_long_enough",
          max(ts) - min(ts) < 0.5 and max(rhs) - min(rhs) < 2.0)  # fmt: skip
finally:
    rest(sht)

# %% [markdown]
# **Matches** `test_always_back_to_sleep_even_on_crc_error`: one bit flipped on the way in fails
# the CRC, and the part is still asleep afterwards.
#
# **What to do:** nothing.

# %%
cbus = CorruptingBus(bus)
sht = SHTC3(sht_conn(cbus))
try:
    cbus.corrupt = 1
    check_raises("CRC failure raised", ValueError, sht.measure)
    check("test_always_back_to_sleep_even_on_crc_error", asleep(SHTC3Cmds(sht_conn())))
finally:
    rest(sht)

# %% [markdown]
# **Matches** `test_commands_while_asleep_are_refused`: the sleeping part NACKs the ID command
# when it is sent without wake-up. A bare wake-up straight after is usually NACKed too (for under
# 1 ms), and the driver's `identify()`, which retries wake-up, still works.
#
# **What to do:** nothing. An `OSError` inside a `PASS` line is the expected NACK.

# %%
sht = SHTC3(sht_conn())
try:
    check_raises("command while asleep NACKed", OSError, sht.cmd_send, sht.CMD_READ_ID)
    try:
        sht.cmd_send(sht.CMD_WAKEUP)
        info("a bare wake-up straight after was accepted this time")
    except OSError:
        print("a bare wake-up straight after: NACKed, as measured")
    check("test_commands_while_asleep_are_refused", sht.identify())
finally:
    rest(sht)

# %% [markdown]
# **Matches** `test_soft_reset_leaves_it_asleep`.
#
# **What to do:** nothing.

# %%
sht = SHTC3(sht_conn())
try:
    sht.soft_reset()
    check("test_soft_reset_leaves_it_asleep", asleep(sht) and sht.identify())
finally:
    rest(sht)

# %% [markdown]
# ## Hardware-only checks

# %% [markdown]
# The measurement times, with a quiet bus: each trial waits a fixed time with no bus traffic, then
# reads once (polling would cost about 0.8 ms per NACK on the ESP32). The cell finds the shortest
# wait that works, and checks that the driver's waits (the datasheet maximums, 12.1 ms and
# 0.8 ms) work and are not shorter than that. It also checks that a read during the measurement
# is NACKed.
#
# **What to do:** nothing. Takes a few seconds.

# %%
cmds = SHTC3Cmds(sht_conn())
rest(cmds)


def works_after(low_power, wait_us):
    """One measurement with a fixed wait and a single read: True if the read is answered."""
    cmds.cmd_send(cmds.CMD_WAKEUP)
    sleep_us(cmds.T_WAKEUP_US)
    cmds.cmd_send(cmds.CMD_MEASURE[(low_power, False, False)])
    sleep_us(wait_us)
    try:
        cmds.words_read(2)
        ok = True
    except OSError:
        ok = False
        sleep_ms(15)  # let it finish
    rest(cmds)
    return ok


try:
    for low_power, waits in ((False, range(9000, 12600, 500)), (True, range(200, 900, 100))):
        name = "low power" if low_power else "normal"
        driver_us = cmds.T_MEAS_US[low_power]
        shortest = None
        for w in waits:
            if works_after(low_power, w):
                shortest = w
                break
        print("{}: shortest working wait {} | the driver waits {:.1f} ms".format(
            name, "none" if shortest is None else "{:.1f} ms".format(shortest / 1000),
            driver_us / 1000))  # fmt: skip
        check("{}: the driver's wait works".format(name), works_after(low_power, driver_us))
        check("{}: done within the datasheet maximum".format(name),
              shortest is not None and shortest <= driver_us)  # fmt: skip
    check("a read 0.1 ms into the measurement is NACKed", not works_after(False, 100))
finally:
    rest(cmds)

# %% [markdown]
# The SHTC3's temperature against the TMP102 on bus 1 (behind the mux, channel 2), when that is
# connected. Both are accurate parts (±0.2 C and ±0.5 C typical). Skipped (reported, not failed)
# when the mux does not answer on bus 1.
#
# **What to do:** if the TMP102 is connected, keep your fingers off both sensors for a minute
# first.

# %%
bus1 = bus_open("feathers3", TMP_BUS, freq=100_000)
if MUX_ADDR not in bus1.scan():
    info("no mux on bus {}: the TMP102 is not connected, so no cross-check".format(TMP_BUS))
else:
    sht = SHTC3(sht_conn())
    try:
        t_sht, _ = sht.measure()
        t_tmp = TMP102(I2CConnection(bus1, TMP_ADDR, TMP_PATH)).temperature()
        print("SHTC3 {:.2f} C | TMP102 {:.2f} C".format(t_sht, t_tmp))
        check("SHTC3 agrees with the TMP102", abs(t_sht - t_tmp) < 1.5,
              "{:+.2f} C apart".format(t_sht - t_tmp))  # fmt: skip
    finally:
        rest(sht)

# %% [markdown]
# The humidity responds to a real change. The cell samples for 10 seconds and prints a reading
# every second.
#
# **What to do:** run the cell, and when it prints **breathe now**, breathe gently on the sensor
# from a few centimetres, two or three times. No time? Skip this cell.

# %%
sht = SHTC3(sht_conn())
try:
    rhs = [sht.measure()[1]]
    print("start {:.1f} %RH. Breathe now, for the next few seconds".format(rhs[0]))
    for i in range(10):
        sleep_ms(1000)
        rhs.append(sht.measure()[1])
        print("  {:>2} s: {:.1f} %RH".format(i + 1, rhs[-1]))
    print("RH from {:.1f} to {:.1f} %RH".format(min(rhs), max(rhs)))
    check("humidity rose with your breath", max(rhs) - rhs[0] > 5.0,
          "+{:.1f} %RH".format(max(rhs) - rhs[0]))  # fmt: skip
finally:
    rest(sht)

# %% [markdown]
# ## Diagnostics
# These print what the part does in cases the datasheet leaves open; the TODO cell at the top
# records what they showed on the bench. They do not count towards the tally. Rerun them on a
# new part or board.

# %% [markdown]
# ### Sleep: what does the part acknowledge?
# A scan (address only), a command other than wake-up, and the wake-up command, each sent to the
# sleeping part.
#
# **What to do:** nothing.

# %%
cmds = SHTC3Cmds(sht_conn())
rest(cmds)
try:
    print("scan while asleep sees 0x70:", SHT_ADDR in bus.scan())
    for name, cmd in (("read-ID command", cmds.CMD_READ_ID), ("wake-up command", cmds.CMD_WAKEUP)):
        try:
            cmds.cmd_send(cmd)
            print("{} while asleep: acknowledged".format(name))
        except OSError as e:
            print("{} while asleep: NACK ({})".format(name, e))
        rest(cmds)
finally:
    rest(cmds)

# %% [markdown]
# ### Wake-up when awake, and a read with nothing pending
# The fake assumes the wake-up command is acknowledged by a part that is already awake, and that
# a read with no result pending is NACKed.
#
# **What to do:** nothing.

# %%
cmds = SHTC3Cmds(sht_conn())
rest(cmds)
try:
    cmds.cmd_send(cmds.CMD_WAKEUP)
    sleep_us(cmds.T_WAKEUP_US)
    try:
        cmds.cmd_send(cmds.CMD_WAKEUP)
        print("second wake-up (part awake): acknowledged")
    except OSError as e:
        print("second wake-up (part awake): NACK ({})".format(e))
    try:
        raw = cmds.read_raw(3)
        print("read with nothing pending: answered", " ".join("{:02x}".format(b) for b in raw))
    except OSError as e:
        print("read with nothing pending: NACK ({})".format(e))
finally:
    rest(cmds)

# %% [markdown]
# ### Does a clock-stretching measurement work on this board?
# The driver waits instead of using clock stretching. This tries the stretching command: the
# read is sent at once, and the part holds SCL low for about 11 ms. The ESP32's I2C timeout may
# cut that short.
#
# **What to do:** nothing.

# %%
cmds = SHTC3Cmds(sht_conn())
try:
    cmds.cmd_send(cmds.CMD_WAKEUP)
    sleep_us(cmds.T_WAKEUP_US)
    cmds.cmd_send(cmds.CMD_MEASURE[(False, False, True)])
    start = time.ticks_us()
    try:
        t_word, rh_word = cmds.words_read(2)
        print("stretched read took {:.2f} ms: {:.2f} C, {:.2f} %RH".format(
            time.ticks_diff(time.ticks_us(), start) / 1000, SHTC3.celsius_from_raw(t_word),
            SHTC3.humidity_from_raw(rh_word)))  # fmt: skip
    except OSError as e:
        print("stretched read failed after {:.2f} ms: {}".format(
            time.ticks_diff(time.ticks_us(), start) / 1000, e))  # fmt: skip
finally:
    sleep_ms(15)
    rest(cmds)

# %% [markdown]
# ### How long after a refused command is wake-up refused?
# Measured 2026-11-14: under 1 ms (one NACK, then accepted ~1 ms later, in 4 of 5 trials). Five
# trials: put it to sleep, send the ID command (NACKed), then try wake-up every ~0.1 ms and time
# how long until it is accepted.
#
# **What to do:** nothing.

# %%
cmds = SHTC3Cmds(sht_conn())
rest(cmds)
try:
    for trial in range(5):
        rest(cmds)
        try:
            cmds.cmd_send(cmds.CMD_READ_ID)  # to the sleeping part: NACKed
            print("trial {}: the command was acknowledged (part awake?)".format(trial))
            continue
        except OSError:
            pass
        start, nacks = time.ticks_us(), 0
        while True:
            try:
                cmds.cmd_send(cmds.CMD_WAKEUP)
                break
            except OSError:
                nacks += 1
                if nacks > 500:
                    break
                sleep_us(100)
        took = time.ticks_diff(time.ticks_us(), start) / 1000
        print(
            "trial {}: wake-up accepted after {:.2f} ms ({} refused first)".format(
                trial, took, nacks
            )
        )
finally:
    rest(cmds)

# %% [markdown]
# ### The shortest wait that works, every step
# The full table behind the timing check: each trial waits a fixed time with no bus traffic, then
# reads once. `ok` means the read was answered. Measured 2026-11-14: from 11.0 ms (normal) and
# 0.5 ms (low power). The driver waits 12.1 ms and 0.8 ms.
#
# **What to do:** nothing. Takes a few seconds.

# %%
cmds = SHTC3Cmds(sht_conn())
rest(cmds)


def works_after(low_power, wait_us):
    """One measurement with a fixed wait and a single read: True if the read is answered."""
    cmds.cmd_send(cmds.CMD_WAKEUP)
    sleep_us(cmds.T_WAKEUP_US)
    cmds.cmd_send(cmds.CMD_MEASURE[(low_power, False, False)])
    sleep_us(wait_us)
    try:
        cmds.words_read(2)
        ok = True
    except OSError:
        ok = False
        sleep_ms(15)  # let it finish
    rest(cmds)
    return ok


try:
    for low_power, waits in ((False, range(8000, 12600, 500)), (True, range(200, 1300, 100))):
        row = [
            "{:.1f}:{}".format(w / 1000, "ok" if works_after(low_power, w) else "--") for w in waits
        ]
        print("{:<9} wait ms -> {}".format("low power" if low_power else "normal", "  ".join(row)))
finally:
    rest(cmds)

# %% [markdown]
# ### What does one read during a measurement do?
# An early polling check once saw a low-power read answered only after 24 ms with no NACK. Not
# reproduced: on 2026-11-14 one read 0.1 ms after each command was NACKed, and the NACK itself
# took about 0.8 ms. This sends that read and times it.
#
# **What to do:** nothing.

# %%
cmds = SHTC3Cmds(sht_conn())
rest(cmds)
try:
    for low_power in (False, True):
        cmds.cmd_send(cmds.CMD_WAKEUP)
        sleep_us(cmds.T_WAKEUP_US)
        cmds.cmd_send(cmds.CMD_MEASURE[(low_power, False, False)])
        sleep_us(100)  # well inside the measurement
        start = time.ticks_us()
        try:
            cmds.words_read(2)
            result = "answered"
        except OSError as e:
            result = "NACK ({})".format(e)
        took = time.ticks_diff(time.ticks_us(), start) / 1000
        print("{:<9} one read 0.1 ms after the command: {}, the read took {:.2f} ms".format(
            "low power" if low_power else "normal", result, took))  # fmt: skip
        sleep_ms(15)
        rest(cmds)
finally:
    rest(cmds)

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

# %%
