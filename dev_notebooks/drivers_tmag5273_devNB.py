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
# > The last section, **`# Hardware validation`**, connects to the real board, rewrites the
# > TMAG5273's configuration, puts it to sleep and wakes it, and has a step where you wave a
# > magnet near it. *Run All* and `jlx run` reach that section.
# >
# > - **Safe to run on its own:** everything above `# Hardware validation`. It runs on the host
# >   against a fake part, writes the module and its tests, and needs no board.
# > - **Hardware section:** run it one cell at a time, reading the **What to do** line above
# >   each cell first.
#
# **Runs on the host first, the board last**, like `drivers_tmp102_devNB`. Everything down to
# `# Hardware validation` runs on CPython against `micropython_i2c.fakes.FakeTMAG5273`. The last
# section connects to the FeatherS3, uploads the freshly written package, checks the real
# TMAG5273 behind the mux with a matching test for every unit test above, then disconnects.

# %% [markdown]
# # Hardware and configuration
# What this notebook was validated with. The same table opens `# Hardware validation`.
#
# | | Host run (simulation) | Hardware run |
# |---|---|---|
# | **Runs on** | CPython 3.13.15 in the micropylab kernel (1.0.0a1), no board | Unexpected Maker FeatherS3 (ESP32-S3), MicroPython 1.28.0 (v1.28.0, 2026-04-06), USB `/dev/ttyACM0` |
# | **I2C bus** | `FakeBus` | `machine.I2C(1)`, **bus 1**, scl=15 sda=16, 100 kHz (`bus_open("feathers3", 1)`) |
# | **Mux** | `FakeMux` at `0x70`, channel 0, only in tests that ask for it (`mux=True`); the rest talk to the part directly | SparkFun Qwiic Mux (TCA9548A) at `0x70` (address jumpers open), **channel 0** |
# | **Sensor** | `FakeTMAG5273`: a B1 at `0x22`, power-on state, field (5, -5, 0) mT, 25 C | SparkFun Qwiic TMAG5273, a **B1** (TMAG5273B1) at `0x22` |
# | **Other parts used** | none | TMP102 at `0x48` on mux channel 2: the reference for the temperature check |
# | **Datasheet** | TI SLYS045C (rev C, April 2026) | same |
# | **Result** | 36 unit tests pass | 79 of 79 hardware checks passed (2026-11-14) |
#
# Nothing else on bus 1 is touched. Bus 0 is not used by this notebook. To rerun the hardware
# section, wire the bench this way again (README "The bench").

# %% [markdown]
# # TODO:
# Nothing open.
#
# Measured on the bench (2026-11-13/14). The `## Diagnostics` cells print each of these again:
# - In a fast read mode a register read does not return the register (`DEVICE_ID` read
#   `00 90 00`, not `05 49 54`), so the driver refuses register reads there.
# - The CRC on a standard read covers just the four data bytes.
# - The fast-read CRC covers every I2C address byte the part heard since its own last
#   transfer, then its read command and the data. Behind the mux that is the mux's address
#   byte `0xe0` (twice after two channel selects). The driver accepts the path's mux bytes and
#   retries once, for when the part overheard another device (no simple rule matched that).
# - Fast reads send each axis once, in the order of the `MAG_CH_EN` name: `ZX` sends Z then X,
#   `XZX` sends X then Z. Rev C's "T, X, Y, and Z sequence" is not literal.
# - The die temperature reads about +5.3 C against the TMP102 in standby and climbs about
#   1.2 C over 30 s of continuous 32x conversions. The datasheet gives no accuracy figure.
#
# Decided (2026-11-13): this notebook replaces `notebooks/02_tmag5273_bringup`. It follows TI
# SLYS045**C** (rev C, April 2026, `docs/datasheets/tmag5273(1).pdf`). Rev A
# (`TMAG5273.PDF`) gave the temperature slope as 60.1 LSB/C; rev C gives **58**.

# %% [markdown]
# # Python file
# location: `src/micropython_i2c/drivers/tmag5273.py`
#
# tests: `tests/test_tmag5273.py`

# %% [markdown]
# ## header

# %%
# %%extract ../src/micropython_i2c/drivers/tmag5273.py
"""TI TMAG5273 three-axis Hall-effect magnetometer (SLYS045C, rev C).

Usage on the board::

    from micropython_i2c import I2CConnection
    from micropython_i2c.drivers.tmag5273 import TMAG5273

    mag = TMAG5273(I2CConnection(bus, 0x22, lock=bus_lock))   # SparkFun board: a B1, 0x22
    mag.configure(channels="XYZ", temperature=True)
    print(mag.magnetic())           # -> (0.317, -1.245, 12.038)

Two classes:

* ``TMAG5273Regs``: datasheet facts only, no methods. Bring-up uses it to exercise the
  mixins directly against the part, so a failure there cannot be driver logic.
* ``TMAG5273``: the driver.

Registers for configuration and a CRC on the read path: the first part here that needs both
protocol layers.

Read modes (``I2C_RD``): ``standard`` register reads, ``fast16`` and ``fast8``. In the fast
modes a read has no register pointer: the part streams the enabled channels (temperature
first, then the ``MAG_CH_EN`` sequence), ``CONV_STATUS``, then a CRC byte if enabled. The
length is implicit, so the driver computes it from the configuration it wrote. Writes are
register writes in every mode, but the driver does not read registers in a fast mode
(``read_mode = "standard"`` first).

CRC: CRC-8, poly 0x07, init 0xFF. On a fast read it covers every I2C address byte the part
heard since its own last transfer, then its read command ``(addr << 1) | 1``, then the data
(measured). Behind a mux that includes the mux's address byte, so the driver accepts the
path's mux bytes in front, and retries once when the part overheard another device. In
``fast16`` with CRC on, at most three channels can be enabled. On a standard read the CRC is
the fifth byte and covers the four data bytes, so with CRC on the driver reads at most four
bytes at a time.

Data formats:

* Magnetic: 16-bit two's complement, ``B = raw / 2^16 * 2|B_R|`` mT (Equation 10).
* Temperature: unsigned ADC code, ``T = 25 + (raw - 17508) / 58`` C (Equation 12, rev C).
  Not two's complement.
* Angle: 13 bits, D12:D4 whole degrees, D3:D0 sixteenths.
* Offset: 8-bit two's complement, ``raw / 2^12 * 2|B_R|`` mT (Equation 16), so it spans
  only 1/16 of full scale.

Generated by ``dev_notebooks/drivers_tmag5273_devNB``: edit the notebook, not this file.
"""

# %% [markdown]
# # Imports
# ## External
# None: the driver runs on MicroPython and uses only this package.
#
# ## Internal
# `RegisterI2CBase` for the registers, `CRCMixin` for the fast-read checksum, and `sleep_us`
# from `compat` for the wake-up delay.

# %%
# %%extract ../src/micropython_i2c/drivers/tmag5273.py -a
from micropython_i2c.compat import sleep_us
from micropython_i2c.crc import CRCMixin
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
# %%writefile ../tests/test_tmag5273.py
import math

import pytest

from micropython_i2c import I2CConnection
from micropython_i2c.drivers.tmag5273 import TMAG5273, TMAG5273Regs
from micropython_i2c.fakes import FakeBus, FakeMux, FakeRegisterPart, FakeTMAG5273

# %% [markdown]
# Notebook-only: the imports the validation and test cells use. The classes under test are the
# ones defined in this notebook, not the module's.

# %%
import math

import pytest

from micropython_i2c import I2CConnection
from micropython_i2c.fakes import FakeBus, FakeMux, FakeRegisterPart, FakeTMAG5273

# %% [markdown]
# # Intent
# **Purpose:** read a TI TMAG5273 3-axis Hall sensor in millitesla and degrees Celsius:
# configure it, read the field and temperature by register or by fast read (with the CRC
# checked), and use its angle engine and offset correction.
#
# **Why it exists:** the TMAG5273's numbers depend on state the part holds: the range scales
# every magnetic word (a stale range is off by exactly 2x), the fast-read packet length
# depends on four settings, and the CRC covers a byte the host sent, not one it received.
#
# **Used by:** the routing table and manager (`"driver": "TMAG5273"`) and the bench (SparkFun
# Qwiic TMAG5273, a **B1** at `0x22`, on mux channel 0).
#
# **Not its job:** reaching the part (the `I2CConnection` does addressing, mux routing and
# locking), thresholds and interrupts (the Qwiic cable has no INT line), gain correction.
#
# ```python
# mag = TMAG5273(I2CConnection(bus, 0x22, [[0x70, 0]], lock))
# mag.configure(channels="XYZ", temperature=False, averaging=32)
# mag.all_read()                    # (23.1, 0.031, -0.012, 0.044)  T in C, then X, Y, Z in mT
# mag.crc_enabled = True            # change settings in the standard read mode...
# mag.read_mode = "fast16"          # ...then switch to a fast one
# mag.fast_read()                   # ({"X": 0.031, "Y": -0.012, "Z": 0.044}, conv_status)
# ```
#
# ```mermaid
# classDiagram
#     RegisterI2CBase <|-- TMAG5273Regs
#     CRCMixin <|-- TMAG5273Regs
#     I2CDevice <|-- TMAG5273Regs
#     TMAG5273Regs <|-- TMAG5273
#     class TMAG5273Regs { datasheet facts only }
#     class TMAG5273 { codecs, configure(), magnetic(), fast_read(), angle, offsets, sleep/wake }
# ```
#
# **Variants** (rev C, Table 6-2 and section 8.1.14): the letter sets the factory address, the
# digit the ranges (it reads back as `DEVICE_ID.VER`).
#
# | Letter | Factory address |
# |---|---|
# | A | `0x35` |
# | **B** (the SparkFun board) | **`0x22`** |
# | C | `0x78` |
# | D | `0x44` |
#
# | Digit (`VER`) | Ranges |
# |---|---|
# | 1 | ±40 / ±80 mT |
# | 2 | ±133 / ±266 mT |
#
# **Fast-read packet** (rev C, 6.5.1.3.4-5, and measured): `[T] [axes] CONV_STATUS [CRC]`, two
# bytes per channel in `fast16`, the MSB only in `fast8`. Each axis comes once, in the order of
# the `MAG_CH_EN` name: `ZX` sends Z then X, `XYX` sends X then Y.
#
# ## What the module defines
#
# | Name | What it is |
# |---|---|
# | `TMAG5273Regs` | datasheet facts: registers, fields, code tables, CRC parameters, ranges, timing |
# | `TMAG5273` | the driver: codecs, configuration, standard and fast reads, angle, offsets, status, sleep/wake |

# %% [markdown]
# # Code
# ## TODO:
# Nothing

# %% [markdown]
# ## Public names

# %%
# %%extract ../src/micropython_i2c/drivers/tmag5273.py -a
__all__ = ["TMAG5273", "TMAG5273Regs"]

# %% [markdown]
# ## The axis sequences
# `MAG_CH_EN` codes and their names (rev C, Table 8-5). Codes 0x8-0xB are the
# pseudo-simultaneous modes: one axis sampled on both sides of another. A fast read sends each
# axis once, in the name's order (measured 2026-11-14): `ZX` sends Z then X, `XYX` sends X
# then Y. `CH_SEQUENCE` is that streamed order.

# %%
# %%extract ../src/micropython_i2c/drivers/tmag5273.py -a
_MAG_CH = {
    0x0: "", 0x1: "X", 0x2: "Y", 0x3: "XY", 0x4: "Z", 0x5: "ZX", 0x6: "YZ", 0x7: "XYZ",
    0x8: "XYX", 0x9: "YXY", 0xA: "YZY", 0xB: "XZX",
}  # fmt: skip


def _streamed(name):
    """The axes a fast read sends for a ``MAG_CH_EN`` name: each once, in name order."""
    out = ()
    for axis in name:
        if axis not in out:
            out += (axis,)
    return out


_CH_SEQUENCE = {code: _streamed(name) for code, name in _MAG_CH.items()}

# %% [markdown]
# ## `TMAG5273Regs`
# Every number here is from TI SLYS045C. The registers are 8 bits wide; the 16-bit results sit
# in adjacent MSB/LSB pairs and are read with a per-call `nbytes=2` and the part's
# auto-increment. The status bits are write-1-to-clear, so they can only be written raw.
#
# The SparkFun board is a **B1**: the factory address is `0x22`, not the A variant's `0x35`.
# `ADDR_DEFAULT` stays at `0x35`, the address the datasheet's register defaults assume.


# %%
# %%extract ../src/micropython_i2c/drivers/tmag5273.py -a
class TMAG5273Regs(RegisterI2CBase, CRCMixin, I2CDevice):
    """Datasheet facts for the TMAG5273, with no driver logic."""

    # Any 7-bit address can be programmed through I2C_ADDRESS (lost at power-off). 0x78 is
    # reserved in the I2C spec, but it is the C variant's factory address.
    ALLOWED_ADDRS = tuple(range(0x08, 0x79))
    ADDR_DEFAULT = 0x35  # A variant; the register reset values assume it
    FACTORY_ADDRS = {"A": 0x35, "B": 0x22, "C": 0x78, "D": 0x44}  # Table 6-2

    BAUD_NOMINAL = 400_000
    BAUD_MAX = 1_000_000  # Fast-mode Plus, needs VCC >= 2.3 V
    BAUD_MIN = 10_000

    BIG_ENDIAN = True
    REG_BYTES = 1

    # Register map (section 8.1).
    REG_DEVICE_CONFIG_1 = 0x00
    REG_DEVICE_CONFIG_2 = 0x01
    REG_SENSOR_CONFIG_1 = 0x02
    REG_SENSOR_CONFIG_2 = 0x03
    REG_X_THR_CONFIG = 0x04
    REG_Y_THR_CONFIG = 0x05
    REG_Z_THR_CONFIG = 0x06
    REG_T_CONFIG = 0x07
    REG_INT_CONFIG_1 = 0x08
    REG_MAG_GAIN_CONFIG = 0x09
    REG_MAG_OFFSET_CONFIG_1 = 0x0A
    REG_MAG_OFFSET_CONFIG_2 = 0x0B
    REG_I2C_ADDRESS = 0x0C
    REG_DEVICE_ID = 0x0D
    REG_MANUFACTURER_ID_LSB = 0x0E  # LSB at the lower address: LSB first on the wire
    REG_MANUFACTURER_ID_MSB = 0x0F
    REG_T_MSB_RESULT = 0x10
    REG_T_LSB_RESULT = 0x11
    REG_X_MSB_RESULT = 0x12
    REG_X_LSB_RESULT = 0x13
    REG_Y_MSB_RESULT = 0x14
    REG_Y_LSB_RESULT = 0x15
    REG_Z_MSB_RESULT = 0x16
    REG_Z_LSB_RESULT = 0x17
    REG_CONV_STATUS = 0x18
    REG_ANGLE_RESULT_MSB = 0x19
    REG_ANGLE_RESULT_LSB = 0x1A
    REG_MAGNITUDE_RESULT = 0x1B
    REG_DEVICE_STATUS = 0x1C

    # Bit 7 of the register address byte starts a conversion (6.5.1.3.1); addresses are 7 bits.
    TRIGGER_BIT = 0x80

    # Every config register that reset_config() restores (I2C_ADDRESS deliberately not).
    CONFIG_REGS = (
        REG_DEVICE_CONFIG_1,
        REG_DEVICE_CONFIG_2,
        REG_SENSOR_CONFIG_1,
        REG_SENSOR_CONFIG_2,
        REG_X_THR_CONFIG,
        REG_Y_THR_CONFIG,
        REG_Z_THR_CONFIG,
        REG_T_CONFIG,
        REG_INT_CONFIG_1,
        REG_MAG_GAIN_CONFIG,
        REG_MAG_OFFSET_CONFIG_1,
        REG_MAG_OFFSET_CONFIG_2,
    )
    CONFIG_POR = 0x00  # every config register resets to 0

    # DEVICE_CONFIG_1 (Table 8-3)
    F_CRC_EN = (7, 1)
    F_MAG_TEMPCO = (5, 2)
    F_CONV_AVG = (2, 3)
    F_I2C_RD = (0, 2)
    # DEVICE_CONFIG_2 (Table 8-4)
    F_THR_HYST = (5, 3)
    F_LP_LN = (4, 1)  # 0 low active current, 1 low noise
    F_GLITCH_FILTER = (3, 1)  # 0 filter on
    F_TRIGGER_MODE = (2, 1)  # 0 I2C command, 1 INT pin
    F_OPERATING_MODE = (0, 2)
    # SENSOR_CONFIG_1 (Table 8-5)
    F_MAG_CH_EN = (4, 4)
    F_SLEEPTIME = (0, 4)
    # SENSOR_CONFIG_2 (Table 8-6); bit 7 reserved
    F_THRX_COUNT = (6, 1)
    F_MAG_THR_DIR = (5, 1)
    F_MAG_GAIN_CH = (4, 1)
    F_ANGLE_EN = (2, 2)
    F_X_Y_RANGE = (1, 1)
    F_Z_RANGE = (0, 1)
    # T_CONFIG (Table 8-10)
    F_T_THR_CONFIG = (1, 7)
    F_T_CH_EN = (0, 1)
    # INT_CONFIG_1 (Table 8-11); bit 1 reserved
    F_RSLT_INT = (7, 1)
    F_THRSLD_INT = (6, 1)
    F_INT_STATE = (5, 1)  # 0 latched, 1 10 us pulse
    F_INT_MODE = (2, 3)
    F_MASK_INTB = (0, 1)
    # I2C_ADDRESS (Table 8-15)
    F_I2C_ADDRESS = (1, 7)
    F_I2C_ADDRESS_UPDATE_EN = (0, 1)
    # DEVICE_ID (Table 8-16)
    F_VER = (0, 2)  # 1: +-40/80 mT, 2: +-133/266 mT
    # CONV_STATUS (section 8.1.25)
    F_SET_COUNT = (5, 3)
    F_POR = (4, 1)  # W1C
    F_DIAG_STATUS = (1, 1)
    F_RESULT_STATUS = (0, 1)
    # DEVICE_STATUS (section 8.1.29)
    F_INTB_RB = (4, 1)
    F_OSC_ER = (3, 1)  # W1C
    F_INT_ER = (2, 1)  # W1C
    F_OTP_CRC_ER = (1, 1)  # W1C
    F_VCC_UV_ER = (0, 1)  # W1C

    # Code tables.
    CONV_AVG = {0: 1, 1: 2, 2: 4, 3: 8, 4: 16, 5: 32}  # samples averaged
    MAG_CH = _MAG_CH  # code -> name, e.g. 0xB -> "XZX"
    CH_SEQUENCE = _CH_SEQUENCE  # code -> axes a fast read sends, e.g. 0xB -> ("X", "Z")
    OP_MODE = {0: "standby", 1: "sleep", 2: "continuous", 3: "wake_sleep"}
    READ_MODE = {0: "standard", 1: "fast16", 2: "fast8"}
    TEMPCO = {0: 0.0, 1: 0.12, 3: 0.2}  # %/C; code 2 reserved
    ANGLE_PAIR = {0: "off", 1: "XY", 2: "YZ", 3: "XZ"}
    SLEEPTIME = {
        0x0: 1, 0x1: 5, 0x2: 10, 0x3: 15, 0x4: 20, 0x5: 30, 0x6: 50,
        0x7: 100, 0x8: 500, 0x9: 1000, 0xA: 2000, 0xB: 5000, 0xC: 20000,
    }  # ms  # fmt: skip

    # Identification.
    MANUFACTURER_ID = 0x5449  # "TI"

    # CRC (6.5.1.3.6): the SMBus polynomial, seeded 0xFF. Rev C's worked examples.
    CRC_WIDTH = 8
    CRC_POLY = 0x07
    CRC_INIT = 0xFF
    CRC_VECTORS = (
        (b"\x00", 0xF3),
        (b"\xff", 0x00),
        (b"\x80", 0x7A),
        (b"\x4c", 0x10),
        (b"\xe0", 0x5D),
        (b"\x00\x00\x00\x00", 0xD1),
        (b"\xff\xff\xff\xff", 0x0F),
    )
    STD_CRC_SPAN = 4  # a standard read sends its CRC after this many data bytes
    FAST16_CRC_MAX_CHANNELS = 3  # fast16 with CRC: at most three channels (T counts)

    # Magnetic full scale in mT per range bit, by DEVICE_ID.VER (sections 5.7, 5.8).
    RANGES_BY_VER = {1: {0: 40.0, 1: 80.0}, 2: {0: 133.0, 1: 266.0}}
    MAG_BITS = 16
    OFFSET_BITS = 8
    OFFSET_DIV = 4096  # 2^12, Equation 16

    # Temperature (section 5.6, rev C).
    T_ADC_T0 = 17508
    T_SENS_T0 = 25.0
    T_ADC_RES = 58.0  # LSB per C (rev A: 60.1)
    TEMP_MIN = -40.0
    TEMP_MAX = 170.0
    # One MSB step in fast8 mode: 256 ADC counts, about 4.4 C, truncated toward zero.
    TEMP_LSB_STEP_C = 256 / T_ADC_RES

    # T_THR_CONFIG: 8 C per code, -41 C = 0x1A (Table 8-10). Not used by the driver yet.
    T_THR_STEP = 8
    T_THR_OFFSET = 249

    # Angle (section 6.5.2.3).
    ANGLE_INT = (4, 9)
    ANGLE_FRAC = (0, 4)

    # Timing (section 5.11), microseconds. Conversion time is 25 + 25 * samples * channels.
    T_STARTUP_US = 270
    T_WAKE_US = 50  # sleep -> standby
    T_START_MEASURE_US = 70  # standby -> continuous
    T_CONV_BASE_US = 25
    T_CONV_SAMPLE_US = 25  # per sample per channel


# %% [markdown]
# ### validation/test
# A helper that puts a `FakeTMAG5273` (a B1 at `0x22`, power-on state, field (5, -5, 0) mT,
# 25 C) on a fake bus, behind channel 0 of a fake mux at `0x70` if asked (like the bench), and
# binds a class to it. It returns the object and the fake part, and is shared by the validation
# cells and the test file.


# %%
# %%extract ../tests/test_tmag5273.py -a
def on_fake(cls, addr=0x22, mux=False, **part_kw):
    """``(dev, part)``: ``cls`` bound to a fresh ``FakeTMAG5273`` at ``addr`` on a fake bus,
    behind channel 0 of a fake mux at 0x70 if ``mux``."""
    bus = FakeBus()
    path = []
    if mux:
        bus.attach(0x70, FakeMux())
        path = [[0x70, 0]]
    part = bus.attach(addr, FakeTMAG5273(addr=addr, **part_kw), path=path)
    return cls(I2CConnection(bus, addr, path)), part


# %% [markdown]
# #### validation: nominal usage
# Identify the part through the register layer alone: the manufacturer ID is split across two
# registers with the LSB at the lower address, and `DEVICE_ID.VER` gives the range family.

# %%
regs, part = on_fake(TMAG5273Regs)
lsb = regs.reg_read(regs.REG_MANUFACTURER_ID_LSB)
msb = regs.reg_read(regs.REG_MANUFACTURER_ID_MSB)
print(f"manufacturer ID {(msb << 8) | lsb:#06x} (expect {regs.MANUFACTURER_ID:#06x})")
print("VER:", regs.field_get(regs.REG_DEVICE_ID, regs.F_VER), "->", regs.RANGES_BY_VER[1])
print(f"I2C_ADDRESS register: {regs.reg_read(regs.REG_I2C_ADDRESS):#04x} (address 0x22 << 1)")
print("POR flag set at power-on:", regs.field_get(regs.REG_CONV_STATUS, regs.F_POR))

# %% [markdown]
# #### validation: anti-nominal usage
# `POR` is write-1-to-clear: writing 1 clears it, so a verified write of the bit can never read
# back as written. The raw write is the only way to clear it.

# %%
regs, part = on_fake(TMAG5273Regs)
por = regs.field_mask(regs.F_POR)
try:
    regs.reg_set(regs.REG_CONV_STATUS, por)  # W1C: reads back 0
except ValueError as err:
    print(f"raised ValueError, as expected:\n    {err}")
else:
    raise AssertionError("a verified write of a W1C bit should have raised ValueError")
finally:
    print("after the write, POR =", regs.field_get(regs.REG_CONV_STATUS, regs.F_POR))

# %% [markdown]
# #### unit

# %% [markdown]
# Within each config register the fields do not overlap.

# %%
# %%extract ../tests/test_tmag5273.py -a --strip-ipytest
# %%ipytest

FIELDS_BY_REG = {
    "DEVICE_CONFIG_1": ("F_CRC_EN", "F_MAG_TEMPCO", "F_CONV_AVG", "F_I2C_RD"),
    "DEVICE_CONFIG_2": ("F_THR_HYST", "F_LP_LN", "F_GLITCH_FILTER", "F_TRIGGER_MODE",
                        "F_OPERATING_MODE"),
    "SENSOR_CONFIG_1": ("F_MAG_CH_EN", "F_SLEEPTIME"),
    "SENSOR_CONFIG_2": ("F_THRX_COUNT", "F_MAG_THR_DIR", "F_MAG_GAIN_CH", "F_ANGLE_EN",
                        "F_X_Y_RANGE", "F_Z_RANGE"),
    "T_CONFIG": ("F_T_THR_CONFIG", "F_T_CH_EN"),
    "INT_CONFIG_1": ("F_RSLT_INT", "F_THRSLD_INT", "F_INT_STATE", "F_INT_MODE", "F_MASK_INTB"),
    "CONV_STATUS": ("F_SET_COUNT", "F_POR", "F_DIAG_STATUS", "F_RESULT_STATUS"),
    "DEVICE_STATUS": ("F_INTB_RB", "F_OSC_ER", "F_INT_ER", "F_OTP_CRC_ER", "F_VCC_UV_ER"),
}  # fmt: skip


@pytest.mark.parametrize("reg", sorted(FIELDS_BY_REG))
def test_fields_do_not_overlap(reg):
    covered = 0
    for name in FIELDS_BY_REG[reg]:
        mask = TMAG5273Regs.field_mask(getattr(TMAG5273Regs, name))
        assert covered & mask == 0 and mask <= 0xFF
        covered |= mask


# %% [markdown]
# The CRC parameters reproduce every worked example in rev C, section 6.5.1.3.6.

# %%
# %%extract ../tests/test_tmag5273.py -a --strip-ipytest
# %%ipytest


def test_crc_vectors_match_datasheet():
    assert len(TMAG5273Regs.CRC_VECTORS) == 7
    for data, want in TMAG5273Regs.CRC_VECTORS:
        assert TMAG5273Regs.crc(data) == want


# %% [markdown]
# Any 7-bit address is allowed (it is programmable), the four factory addresses among them
# (the C variant's `0x78` is outside the usual `0x08..0x77`), and 1 MHz is the clock ceiling.

# %%
# %%extract ../tests/test_tmag5273.py -a --strip-ipytest
# %%ipytest


def test_address_and_clock_limits():
    assert all(TMAG5273Regs.addr_ok(a) for a in TMAG5273Regs.FACTORY_ADDRS.values())
    assert not TMAG5273Regs.addr_ok(0x07) and not TMAG5273Regs.addr_ok(0x79)
    assert TMAG5273Regs.baud_ok(1_000_000) and not TMAG5273Regs.baud_ok(3_400_000)


# %% [markdown]
# One 2-byte read auto-increments across a register pair; the cache keeps the 1- and 2-byte
# reads of the same register apart.

# %%
# %%extract ../tests/test_tmag5273.py -a --strip-ipytest
# %%ipytest


def test_auto_increment_reads_register_pairs():
    regs, _ = on_fake(TMAG5273Regs)
    lsb = regs.reg_read(regs.REG_MANUFACTURER_ID_LSB)
    msb = regs.reg_read(regs.REG_MANUFACTURER_ID_MSB)
    assert regs.reg_read(regs.REG_MANUFACTURER_ID_LSB, nbytes=2) == (lsb << 8) | msb == 0x4954
    assert regs.reg_get(regs.REG_MANUFACTURER_ID_LSB) == lsb
    assert regs.reg_get(regs.REG_MANUFACTURER_ID_LSB, nbytes=2) == 0x4954


# %% [markdown]
# Status bits are write-1-to-clear: a verified write is refused, a raw write clears `POR`.

# %%
# %%extract ../tests/test_tmag5273.py -a --strip-ipytest
# %%ipytest


def test_status_bits_are_write_1_to_clear():
    regs, _ = on_fake(TMAG5273Regs)
    por = regs.field_mask(regs.F_POR)
    assert regs.field_get(regs.REG_CONV_STATUS, regs.F_POR) == 1
    with pytest.raises(ValueError):
        regs.reg_set(regs.REG_CONV_STATUS, por)
    regs.reg_write(regs.REG_CONV_STATUS, por)
    assert regs.field_get(regs.REG_CONV_STATUS, regs.F_POR) == 0


# %% [markdown]
# ## `TMAG5273`
# The codecs are classmethods with explicit scales, so they test with no part. The driver keeps
# the selected ranges in `xy_range` / `z_range` and resyncs them after every write that could
# change them; `ranges` (the table for this part's `VER`) is read at construction, and the
# constructor refuses a part whose manufacturer ID is not TI's.
#
# Three behaviours come from the datasheet's I2C frames:
#
# - **Fast read modes.** A read command alone starts the packet, so the driver assumes register
#   reads do not work in a fast mode (the diagnostics check this). In a fast mode `reg_read`
#   raises, getters answer from the cache (the driver wrote those registers), and setters ask
#   for `read_mode = "standard"` first. Entering a fast mode is a raw write, and the cache is
#   primed first so `fast_read` needs one transfer and no register reads.
# - **CRC on standard reads.** The fifth byte of a standard read is the CRC, so with CRC on the
#   multi-byte reads go in chunks of at most four bytes.
# - **Sleep.** About 20 us after entering sleep or wake-and-sleep the part stops acknowledging,
#   and the first address afterwards only wakes it. Those modes are written without a
#   read-back, and `wake()` sends the waking transfer and waits out the start-up time.


# %%
# %%extract ../src/micropython_i2c/drivers/tmag5273.py -a
def _round_half_away(x):
    return int(x + (0.5 if x >= 0 else -0.5))


def _code(table, value):
    """The code whose table entry is ``value``; ValueError listing the choices otherwise."""
    for code, v in table.items():
        if v == value:
            return code
    raise ValueError("{!r} not in {}".format(value, sorted(table.values())))


class TMAG5273(TMAG5273Regs):
    """TMAG5273 driver: field in mT, temperature in C, angle, fast reads, configuration.

    Args:
        conn: The ``I2CConnection`` to the part.
        reset: Write every config register back to its reset value first. Use it when the
            part may have been left in a fast read mode, where register reads do not work.

    Raises:
        RuntimeError: If the manufacturer ID or ``VER`` is not a TMAG5273's.
    """

    def __init__(self, conn, reset=False, **kwargs):
        super().__init__(conn, **kwargs)
        if reset:  # raw, so it works from a fast read mode
            self.reg_write(self.REG_DEVICE_CONFIG_1, self.CONFIG_POR)
        mid, ver = self.identify()
        if mid != self.MANUFACTURER_ID or ver not in self.RANGES_BY_VER:
            raise RuntimeError(
                "not a TMAG5273 at {:#04x} (manufacturer ID {:#06x}, VER {}); a part left in "
                "a fast read mode does not answer register reads: try reset=True".format(
                    self.addr, mid, ver
                )
            )
        self.ranges = self.RANGES_BY_VER[ver]
        self.xy_range = self.z_range = self.ranges[0]
        if reset:
            self.reset_config()
        else:
            self.range_refresh()

    # Codecs: classmethods with explicit scale, testable with no device.

    @classmethod
    def mag_from_raw(cls, word, full_scale):
        """16-bit two's-complement word -> mT (Equation 10)."""
        return cls.sign_extend(word, cls.MAG_BITS) / (1 << cls.MAG_BITS) * 2 * full_scale

    @classmethod
    def mag_to_raw(cls, mT, full_scale):
        """mT -> 16-bit word, rounding half away from zero.

        Raises:
            ValueError: If ``mT`` is outside +-``full_scale``.
        """
        counts = _round_half_away(mT * (1 << cls.MAG_BITS) / (2 * full_scale))
        if not -(1 << (cls.MAG_BITS - 1)) <= counts < (1 << (cls.MAG_BITS - 1)):
            raise ValueError("{} mT outside the +-{} mT range".format(mT, full_scale))
        return counts & ((1 << cls.MAG_BITS) - 1)

    @classmethod
    def celsius_from_raw(cls, word):
        """Unsigned 16-bit ADC code -> C (Equation 12). Not two's complement."""
        return cls.T_SENS_T0 + (word - cls.T_ADC_T0) / cls.T_ADC_RES

    @classmethod
    def celsius_to_raw(cls, celsius):
        """C -> 16-bit ADC code.

        Raises:
            ValueError: If the code would not fit 16 unsigned bits.
        """
        code = int(cls.T_ADC_T0 + (celsius - cls.T_SENS_T0) * cls.T_ADC_RES + 0.5)
        if not 0 <= code <= 0xFFFF:
            raise ValueError("{} C not representable".format(celsius))
        return code

    @classmethod
    def angle_from_raw(cls, word):
        """13-bit angle word -> degrees (0x1628 -> 354.5, 0x0114 -> 17.25)."""
        return cls.field_extract(word, cls.ANGLE_INT) + cls.field_extract(word, cls.ANGLE_FRAC) / 16

    @classmethod
    def angle_to_raw(cls, degrees):
        """Degrees -> 13-bit angle word, to the nearest sixteenth.

        Raises:
            ValueError: If ``degrees`` is outside 0..360.
        """
        if not 0.0 <= degrees < 360.0:
            raise ValueError("{} outside 0-360 degrees".format(degrees))
        whole = int(degrees)
        frac = int((degrees - whole) * 16 + 0.5)
        if frac == 16:
            whole, frac = whole + 1, 0
        return (whole << 4) | frac

    @classmethod
    def offset_from_raw(cls, byte, full_scale):
        """8-bit two's-complement offset code -> mT (Equation 16)."""
        return cls.sign_extend(byte, cls.OFFSET_BITS) / cls.OFFSET_DIV * 2 * full_scale

    @classmethod
    def offset_limit(cls, full_scale):
        """Largest representable offset magnitude at a range: 1/16 of full scale."""
        return 128 / cls.OFFSET_DIV * 2 * full_scale

    @classmethod
    def offset_to_raw(cls, mT, full_scale):
        """mT -> 8-bit offset code.

        Raises:
            ValueError: Naming the actual limit, if ``mT`` cannot be expressed.
        """
        counts = _round_half_away(mT * cls.OFFSET_DIV / (2 * full_scale))
        if not -128 <= counts <= 127:
            raise ValueError(
                "{} mT offset outside +-{:g} mT, the representable range at +-{:g} mT "
                "full scale".format(mT, cls.offset_limit(full_scale), full_scale)
            )
        return counts & 0xFF

    @classmethod
    def packet_length(cls, mode, channels, temp, crc):
        """Fast-read packet length in bytes for a hypothetical configuration.

        Layout: (temperature) + channels, 2 bytes each in fast16 or 1 in fast8, then
        CONV_STATUS, then CRC if enabled.

        Args:
            mode: ``I2C_RD`` code, 1 (fast16) or 2 (fast8).
            channels: Axis sequence, e.g. ``("X", "Y", "Z")``.
            temp: Temperature channel enabled.
            crc: CRC enabled.

        Raises:
            ValueError: If ``mode`` is not a fast-read mode.
        """
        if mode not in (1, 2):
            raise ValueError("fast read needs I2C_RD 1h or 2h, not {:#x}".format(mode))
        n = len(channels) + (1 if temp else 0)
        return n * (2 if mode == 1 else 1) + 1 + (1 if crc else 0)

    @classmethod
    def conversion_time_us(cls, channels, averaging):
        """Conversion time in us: ``25 + 25 * averaging * channels`` (section 5.11)."""
        return cls.T_CONV_BASE_US + cls.T_CONV_SAMPLE_US * averaging * max(1, channels)

    # Fast read modes: registers are not read while one is selected.

    @property
    def fast_mode(self):
        """True while the driver has a fast read mode selected (from the cache)."""
        cfg = self.cached(self.REG_DEVICE_CONFIG_1)
        return cfg is not None and self.field_extract(cfg, self.F_I2C_RD) != 0

    def _standard_check(self):
        if self.fast_mode:
            mode = self.field_extract(self.cached(self.REG_DEVICE_CONFIG_1), self.F_I2C_RD)
            raise RuntimeError(
                "registers cannot be read in {} read mode; set read_mode = 'standard' first".format(
                    self.READ_MODE.get(mode, mode)
                )
            )

    def reg_read(self, reg, nbytes=None):
        """Read a register. Raises ``RuntimeError`` in a fast read mode."""
        self._standard_check()
        return super().reg_read(reg, nbytes)

    def field_get(self, reg, field, nbytes=None, refresh=True):
        """Read a field; in a fast read mode, from the cache."""
        return super().field_get(reg, field, nbytes, refresh and not self.fast_mode)

    def _block_read(self, reg, nbytes):
        """Read ``nbytes`` from ``reg``; with CRC on, at most four bytes per transfer."""
        self._standard_check()
        if not self.field_extract(self.reg_get(self.REG_DEVICE_CONFIG_1), self.F_CRC_EN):
            return self.read_mem(reg, nbytes)
        out = b""
        while len(out) < nbytes:
            out += self.read_mem(reg + len(out), min(self.STD_CRC_SPAN, nbytes - len(out)))
        return out

    # Range tracking.

    def range_refresh(self):
        """Re-read both range bits and update the cached scales. Returns (xy, z)."""
        cfg = self.reg_read(self.REG_SENSOR_CONFIG_2)
        self.xy_range = self.ranges[self.field_extract(cfg, self.F_X_Y_RANGE)]
        self.z_range = self.ranges[self.field_extract(cfg, self.F_Z_RANGE)]
        return self.xy_range, self.z_range

    def axis_scale(self, axis):
        """Full-scale range in mT for an axis name."""
        return self.z_range if axis.upper() == "Z" else self.xy_range

    # Standard-mode measurement. Result registers are volatile: never cached.

    def _xyz_decode(self, raw):
        return tuple(
            self.mag_from_raw(self.bytes_to_int(raw[i * 2 : i * 2 + 2]), self.axis_scale(ax))
            for i, ax in enumerate("XYZ")
        )

    def magnetic(self):
        """(X, Y, Z) in mT from the six result registers."""
        return self._xyz_decode(self._block_read(self.REG_X_MSB_RESULT, 6))

    def magnetic_raw(self):
        """The three raw 16-bit axis words."""
        raw = self._block_read(self.REG_X_MSB_RESULT, 6)
        return tuple(self.bytes_to_int(raw[i * 2 : i * 2 + 2]) for i in range(3))

    def temperature(self):
        """Die temperature in C.

        Uncalibrated: rev C gives no accuracy. One part read 5.3 C above a TMP102 beside it,
        and continuous 32x conversions warmed it by about 1.2 C in 30 s.
        """
        return self.celsius_from_raw(self.temperature_raw())

    def temperature_raw(self):
        """Raw 16-bit temperature ADC code."""
        return self.reg_read(self.REG_T_MSB_RESULT, nbytes=2)

    def angle(self):
        """Hardware angle in degrees (0.0 with the angle engine off)."""
        return self.angle_from_raw(self.reg_read(self.REG_ANGLE_RESULT_MSB, nbytes=2))

    def magnitude(self):
        """Raw 8-bit vector magnitude across the angle pair (no mT scaling is given)."""
        return self.reg_read(self.REG_MAGNITUDE_RESULT)

    def all_read(self):
        """(T, X, Y, Z) decoded from the eight result registers 0x10..0x17."""
        raw = self._block_read(self.REG_T_MSB_RESULT, 8)
        return (self.celsius_from_raw(self.bytes_to_int(raw[0:2])),) + self._xyz_decode(raw[2:])

    def trigger(self):
        """Start one conversion (standby mode with ``TRIGGER_MODE`` 0).

        A one-byte register read with the trigger bit set in the address byte.
        """
        self.read_mem(self.REG_CONV_STATUS | self.TRIGGER_BIT, 1)

    # Fast read. The configuration comes from the cache, so a fast read is one transfer.

    def fast_read_channels(self):
        """The axes a fast read sends, in wire order, per ``MAG_CH_EN``."""
        cfg = self.reg_get(self.REG_SENSOR_CONFIG_1)
        return self.CH_SEQUENCE.get(self.field_extract(cfg, self.F_MAG_CH_EN), ())

    def fast_read_config(self):
        """(mode, channels, temp_on, crc_on) as the driver last wrote them."""
        cfg1 = self.reg_get(self.REG_DEVICE_CONFIG_1)
        return (
            self.field_extract(cfg1, self.F_I2C_RD),
            self.fast_read_channels(),
            bool(self.field_extract(self.reg_get(self.REG_T_CONFIG), self.F_T_CH_EN)),
            bool(self.field_extract(cfg1, self.F_CRC_EN)),
        )

    def fast_read_length(self):
        """Expected fast-read packet length for the current configuration."""
        return self.packet_length(*self.fast_read_config())

    def fast_crc_ok(self, packet):
        """True if a fast-read packet's last byte is its CRC.

        The part's CRC covers every I2C address byte it heard since its own last transfer,
        then its read command ``(addr << 1) | 1``, then the data. Behind a mux the
        connection's channel-select writes come first, so none, one or two rounds of the
        path's mux address bytes are accepted in front.
        """
        tail = bytes([(self.addr << 1) | 1]) + packet[:-1]
        hops = bytes([mux << 1 for mux, _ in self.conn.path])
        for prefix in (b"", hops, hops + hops):
            if self.crc(prefix + tail) == packet[-1]:
                return True
        return False

    def fast_read_raw(self, verify=True):
        """Fast read, undecoded. Returns ``([(name, word), ...], status, mode)``.

        With CRC on, a packet that fails ``fast_crc_ok`` is read once more: a transfer to
        another device that the part overheard spoils only its next CRC.

        Raises:
            ValueError: If the part is not in a fast mode, if fast16+CRC has more than
                three channels enabled, or if the checksum fails twice in a row.
        """
        mode, channels, temp_on, crc_on = self.fast_read_config()
        if mode not in (1, 2):
            raise ValueError(
                "device is in {} read mode; set read_mode to 'fast16' or 'fast8'".format(
                    self.READ_MODE.get(mode, mode)
                )
            )
        n = len(channels) + (1 if temp_on else 0)
        if mode == 1 and crc_on and n > self.FAST16_CRC_MAX_CHANNELS:
            raise ValueError(
                "16-bit fast read with CRC supports at most 3 channels; {} enabled".format(n)
            )

        length = self.packet_length(mode, channels, temp_on, crc_on)
        packet = self.read_raw(length)
        if crc_on:
            if verify and not self.fast_crc_ok(packet):
                packet = self.read_raw(length)  # the failed read started the part's CRC over
                if not self.fast_crc_ok(packet):
                    raise ValueError(
                        "fast read: checksum fail twice (received {:#04x})".format(packet[-1])
                    )
            packet = packet[:-1]

        status, body = packet[-1], packet[:-1]
        names = (("T",) if temp_on else ()) + channels
        step = 2 if mode == 1 else 1
        words = [
            (name, self.bytes_to_int(body[i * step : (i + 1) * step]))
            for i, name in enumerate(names)
        ]
        return words, status, mode

    def fast_read(self, verify=True):
        """Fast read, decoded. Returns ``({"T"|"X"|"Y"|"Z": value}, conv_status)``.

        In fast8 only each MSB is sent: magnetic values quantise to 1/256 of full scale and
        temperature to ``TEMP_LSB_STEP_C`` (about 4.4 C, reading low).
        """
        words, status, mode = self.fast_read_raw(verify=verify)
        readings = {}
        for name, word in words:
            if mode == 2:
                word <<= 8
            if name == "T":
                readings[name] = self.celsius_from_raw(word)
            else:
                readings[name] = self.mag_from_raw(word, self.axis_scale(name))
        return readings, status

    # Configuration.

    def configure(self, channels="XYZ", temperature=False, averaging=1, mode="continuous",
                  xy_range=None, z_range=None):  # fmt: skip
        """Set up the common case in dependency order; returns ``self``.

        Ranges first (so scales are right before any conversion), then channels,
        temperature and averaging, and the operating mode last because it starts
        conversions. Needs the standard read mode.

        Args:
            channels: A ``MAG_CH`` combination, e.g. "XYZ", "XY", "Z" or "XZX".
            temperature: Enable the temperature channel.
            averaging: Samples per conversion: 1, 2, 4, 8, 16 or 32.
            mode: "standby", "sleep", "continuous" or "wake_sleep".
            xy_range, z_range: Full scale in mT; None leaves it alone.
        """
        if xy_range is not None or z_range is not None:
            self.ranges_set(xy_range, z_range)
        self.channels = channels
        self.temperature_enabled = temperature
        self.averaging = averaging
        self.operating_mode = mode
        return self

    def ranges_set(self, xy_mT=None, z_mT=None):
        """Set the full-scale ranges in one write, then resync the cached scales."""
        cfg = self.reg_read(self.REG_SENSOR_CONFIG_2)
        for value, field in ((xy_mT, self.F_X_Y_RANGE), (z_mT, self.F_Z_RANGE)):
            if value is not None:
                cfg = self.field_insert(cfg, field, _code(self.ranges, value))
        self.reg_set(self.REG_SENSOR_CONFIG_2, cfg)
        return self.range_refresh()

    def reset_config(self):
        """Write every config register back to its reset value (there is no reset command).

        ``DEVICE_CONFIG_1`` goes first and raw, so this works from a fast read mode.
        ``I2C_ADDRESS`` is left alone so the connection keeps working.
        """
        self.reg_write(self.REG_DEVICE_CONFIG_1, self.CONFIG_POR)
        self.cache_clear()
        for reg in self.CONFIG_REGS:
            self.reg_set(reg, self.CONFIG_POR)
        return self.range_refresh()

    @property
    def operating_mode(self):
        """standby, sleep, continuous or wake_sleep."""
        return self.coded_get(self.REG_DEVICE_CONFIG_2, self.F_OPERATING_MODE, self.OP_MODE)

    @operating_mode.setter
    def operating_mode(self, name):
        code = _code(self.OP_MODE, name)
        if code in (1, 3):  # sleep, wake_sleep: the part stops answering ~20 us after the write
            cfg = self.reg_read(self.REG_DEVICE_CONFIG_2)
            self.reg_write(
                self.REG_DEVICE_CONFIG_2, self.field_insert(cfg, self.F_OPERATING_MODE, code)
            )
            self.cache_clear(self.REG_DEVICE_CONFIG_2)
        else:
            self.field_set(self.REG_DEVICE_CONFIG_2, self.F_OPERATING_MODE, code)

    def wake(self):
        """Wake the part from sleep or wake-and-sleep into standby.

        The first transfer to a sleeping part wakes it but is not acknowledged (section 5.11,
        note 1), so it is sent once with its ``OSError`` ignored, then the start-up time is
        waited out.
        """
        try:
            self.read_mem(self.REG_DEVICE_ID, 1)
        except OSError:
            pass
        sleep_us(self.T_WAKE_US)
        self.cache_clear(self.REG_DEVICE_CONFIG_2)

    @property
    def read_mode(self):
        """standard, fast16 or fast8."""
        return self.coded_get(self.REG_DEVICE_CONFIG_1, self.F_I2C_RD, self.READ_MODE)

    @read_mode.setter
    def read_mode(self, name):
        code = _code(self.READ_MODE, name)
        if code and not self.fast_mode:  # entering a fast mode: cache what fast_read needs
            self.reg_get(self.REG_SENSOR_CONFIG_1)
            self.reg_get(self.REG_T_CONFIG)
        cfg = self.field_insert(self.reg_get(self.REG_DEVICE_CONFIG_1), self.F_I2C_RD, code)
        self.reg_write(self.REG_DEVICE_CONFIG_1, cfg)
        self.reg_cache[self.REG_DEVICE_CONFIG_1] = (1, cfg)
        if code == 0:  # standard again: the write can be verified
            self.reg_set(self.REG_DEVICE_CONFIG_1, cfg)

    @property
    def crc_enabled(self):
        """Whether reads carry a CRC byte."""
        return self.flag_get(self.REG_DEVICE_CONFIG_1, self.F_CRC_EN)

    @crc_enabled.setter
    def crc_enabled(self, enable):
        self.flag_set(self.REG_DEVICE_CONFIG_1, self.F_CRC_EN, enable)

    @property
    def averaging(self):
        """Samples averaged per conversion: 1, 2, 4, 8, 16 or 32."""
        return self.coded_get(self.REG_DEVICE_CONFIG_1, self.F_CONV_AVG, self.CONV_AVG)

    @averaging.setter
    def averaging(self, samples):
        self.coded_set(self.REG_DEVICE_CONFIG_1, self.F_CONV_AVG, self.CONV_AVG, samples)

    @property
    def channels(self):
        """Enabled magnetic channel combination, e.g. "XYZ"."""
        return self.coded_get(self.REG_SENSOR_CONFIG_1, self.F_MAG_CH_EN, self.MAG_CH)

    @channels.setter
    def channels(self, combination):
        self.coded_set(self.REG_SENSOR_CONFIG_1, self.F_MAG_CH_EN, self.MAG_CH, combination.upper())

    @property
    def temperature_enabled(self):
        """Whether the temperature channel converts."""
        return self.flag_get(self.REG_T_CONFIG, self.F_T_CH_EN)

    @temperature_enabled.setter
    def temperature_enabled(self, enable):
        self.flag_set(self.REG_T_CONFIG, self.F_T_CH_EN, enable)

    @property
    def angle_pair(self):
        """Axis pair feeding the angle engine and the offset correction: off, XY, YZ or XZ."""
        return self.coded_get(self.REG_SENSOR_CONFIG_2, self.F_ANGLE_EN, self.ANGLE_PAIR)

    @angle_pair.setter
    def angle_pair(self, pair):
        self.coded_set(self.REG_SENSOR_CONFIG_2, self.F_ANGLE_EN, self.ANGLE_PAIR, pair)

    @property
    def low_noise(self):
        """False = low active current, True = low noise."""
        return self.flag_get(self.REG_DEVICE_CONFIG_2, self.F_LP_LN)

    @low_noise.setter
    def low_noise(self, enable):
        self.flag_set(self.REG_DEVICE_CONFIG_2, self.F_LP_LN, enable)

    @property
    def magnet_tempco(self):
        """Magnet temperature coefficient compensation, %/C."""
        return self.coded_get(self.REG_DEVICE_CONFIG_1, self.F_MAG_TEMPCO, self.TEMPCO)

    @magnet_tempco.setter
    def magnet_tempco(self, percent_per_c):
        self.coded_set(self.REG_DEVICE_CONFIG_1, self.F_MAG_TEMPCO, self.TEMPCO, percent_per_c)

    @property
    def sleep_time(self):
        """Sleep between conversions in wake_sleep mode, ms."""
        return self.coded_get(self.REG_SENSOR_CONFIG_1, self.F_SLEEPTIME, self.SLEEPTIME)

    @sleep_time.setter
    def sleep_time(self, ms):
        self.coded_set(self.REG_SENSOR_CONFIG_1, self.F_SLEEPTIME, self.SLEEPTIME, ms)

    # Offset correction. The two offsets apply to the axes of the selected angle pair.

    def _offset_scales(self):
        pair = self.angle_pair
        if pair == "off":
            return self.xy_range, self.xy_range
        return self.axis_scale(pair[0]), self.axis_scale(pair[1])

    def offsets_get(self):
        """The two offset corrections in mT, as (first, second)."""
        s1, s2 = self._offset_scales()
        return (
            self.offset_from_raw(self.reg_read(self.REG_MAG_OFFSET_CONFIG_1), s1),
            self.offset_from_raw(self.reg_read(self.REG_MAG_OFFSET_CONFIG_2), s2),
        )

    def offsets_set(self, first_mT=None, second_mT=None):
        """Set either or both offsets in mT. Returns ``offsets_get()``."""
        s1, s2 = self._offset_scales()
        if first_mT is not None:
            self.reg_set(self.REG_MAG_OFFSET_CONFIG_1, self.offset_to_raw(first_mT, s1))
        if second_mT is not None:
            self.reg_set(self.REG_MAG_OFFSET_CONFIG_2, self.offset_to_raw(second_mT, s2))
        return self.offsets_get()

    # Status and identification.

    def conv_status(self):
        """CONV_STATUS decoded: raw, set_count, por, diag_fail, ready. Always fresh."""
        raw = self.reg_read(self.REG_CONV_STATUS)
        x = self.field_extract
        return {
            "raw": raw,
            "set_count": x(raw, self.F_SET_COUNT),
            "por": bool(x(raw, self.F_POR)),
            "diag_fail": bool(x(raw, self.F_DIAG_STATUS)),
            "ready": bool(x(raw, self.F_RESULT_STATUS)),
        }

    def device_status(self):
        """DEVICE_STATUS decoded into a dict."""
        raw = self.reg_read(self.REG_DEVICE_STATUS)
        x = self.field_extract
        return {
            "raw": raw,
            "int_pin": bool(x(raw, self.F_INTB_RB)),
            "osc_error": bool(x(raw, self.F_OSC_ER)),
            "int_error": bool(x(raw, self.F_INT_ER)),
            "otp_crc_error": bool(x(raw, self.F_OTP_CRC_ER)),
            "vcc_uv_error": bool(x(raw, self.F_VCC_UV_ER)),
        }

    def status_clear(self):
        """Clear every write-1-to-clear status bit (raw writes; they cannot verify)."""
        m = self.field_mask
        self.reg_write(self.REG_CONV_STATUS, m(self.F_POR))
        self.reg_write(
            self.REG_DEVICE_STATUS,
            m(self.F_OSC_ER) | m(self.F_INT_ER) | m(self.F_OTP_CRC_ER) | m(self.F_VCC_UV_ER),
        )
        self.cache_clear(self.REG_CONV_STATUS)
        self.cache_clear(self.REG_DEVICE_STATUS)

    def manufacturer_id(self):
        """The 16-bit manufacturer ID (expected ``MANUFACTURER_ID``, 0x5449), LSB first."""
        word = self.reg_read(self.REG_MANUFACTURER_ID_LSB, nbytes=2)
        return ((word & 0xFF) << 8) | (word >> 8)

    def identify(self):
        """(manufacturer_id, version) read from the device."""
        return self.manufacturer_id(), self.field_get(self.REG_DEVICE_ID, self.F_VER)

    def variant(self):
        """Orderable variant, e.g. ``"B1"``: letter from ``I2C_ADDRESS``, digit from ``VER``.

        The letter is ``"?"`` if the address is not a factory one (it was reprogrammed).
        """
        addr = self.field_get(self.REG_I2C_ADDRESS, self.F_I2C_ADDRESS)
        letter = "?"
        for name, factory in self.FACTORY_ADDRS.items():
            if factory == addr:
                letter = name
        return letter + str(self.field_get(self.REG_DEVICE_ID, self.F_VER))


# %% [markdown]
# ### validation/test: codecs
# The datasheet's numbers as lists, shared by the unit tests below and copied to the board by
# the hardware section: magnetic decode cases `(word, full scale, mT)`, packet lengths
# `(mode, channels, temp, crc, bytes)` and conversion times `(channels, averaging, us)`.

# %%
# %%extract ../tests/test_tmag5273.py -a
MAG_CASES = [(0x0000, 40.0, 0.0), (0x1000, 40.0, 5.0), (0xF000, 40.0, -5.0), (0x0800, 40.0, 2.5),
             (0x7FFF, 40.0, 39.99878), (0x8000, 40.0, -40.0), (0x1000, 80.0, 10.0),
             (0x8000, 80.0, -80.0)]  # fmt: skip
PACKET_CASES = [(1, "XYZ", False, False, 7), (1, "XYZ", True, False, 9), (1, "X", False, True, 4),
                (1, "XY", True, True, 8), (2, "XYZ", True, False, 5), (2, "XYZ", True, True, 6),
                (2, "XY", False, True, 4)]  # fmt: skip
CONV_TIME_CASES = [(1, 1, 50), (1, 32, 825), (2, 1, 75), (2, 32, 1625), (3, 1, 100)]

# %% [markdown]
# #### validation: nominal usage
# A few decodes worked by hand: 0x1000 is 1/16 of the ±40 mT span either side, so 5 mT; room
# temperature from the ADC; the datasheet's two angle examples; the offset example from
# section 6.5.3.

# %%
print("0x1000 at ±40 mT:", TMAG5273.mag_from_raw(0x1000, 40.0), "mT")
print("ADC 17508:", TMAG5273.celsius_from_raw(17508), "C | ADC 17508 - 3*58:",
      TMAG5273.celsius_from_raw(17508 - 3 * 58), "C")  # fmt: skip
print(
    "angle 0x1628:", TMAG5273.angle_from_raw(0x1628), "| 0x0114:", TMAG5273.angle_from_raw(0x0114)
)
print("offset 0b10000000 at ±40 mT:", TMAG5273.offset_from_raw(0b10000000, 40.0), "mT")
print("fast16, XYZ + T + CRC would be", TMAG5273.packet_length(1, ("X", "Y", "Z"), True, True),
      "bytes (but fast16 + CRC allows only 3 channels)")  # fmt: skip

# %% [markdown]
# #### validation: anti-nominal usage
# A 5 mT offset cannot be expressed at ±40 mT: offsets span only 1/16 of full scale. The
# message names the real limit.

# %%
try:
    TMAG5273.offset_to_raw(5.0, 40.0)  # limit is 2.5 mT at ±40 mT
except ValueError as err:
    print(f"raised ValueError, as expected:\n    {err}")
else:
    raise AssertionError("a 5 mT offset at ±40 mT should have raised ValueError")

# %% [markdown]
# #### unit

# %% [markdown]
# Magnetic words decode per Equation 10, including both ends of the range.

# %%
# %%extract ../tests/test_tmag5273.py -a --strip-ipytest
# %%ipytest


@pytest.mark.parametrize("word, br, want", MAG_CASES)
def test_mag_decode(word, br, want):
    assert TMAG5273.mag_from_raw(word, br) == pytest.approx(want, abs=1e-4)


# %% [markdown]
# One LSB matches the datasheet's sensitivity: 820 LSB/mT at ±40 mT, 410 at ±80 mT.

# %%
# %%extract ../tests/test_tmag5273.py -a --strip-ipytest
# %%ipytest


@pytest.mark.parametrize("br, stated", [(40.0, 820), (80.0, 410)])
def test_sensitivity_matches_datasheet(br, stated):
    assert 1 / TMAG5273.mag_from_raw(1, br) == pytest.approx(stated, abs=1.0)


# %% [markdown]
# Encode then decode returns the field to within one LSB, at both ranges.

# %%
# %%extract ../tests/test_tmag5273.py -a --strip-ipytest
# %%ipytest


@pytest.mark.parametrize("br", [40.0, 80.0])
def test_mag_round_trip(br):
    for mT in (0.0, 5.0, -5.0, 0.001221, br * 0.99, -br * 0.99):
        w = TMAG5273.mag_to_raw(mT, br)
        assert TMAG5273.mag_from_raw(w, br) == pytest.approx(mT, abs=2 * br / 65536)


# %% [markdown]
# A field outside the range is refused.

# %%
# %%extract ../tests/test_tmag5273.py -a --strip-ipytest
# %%ipytest


@pytest.mark.parametrize("mT, br", [(50.0, 40.0), (-50.0, 40.0), (100.0, 80.0)])
def test_mag_out_of_range(mT, br):
    with pytest.raises(ValueError):
        TMAG5273.mag_to_raw(mT, br)


# %% [markdown]
# The temperature code is unsigned with rev C's 58 LSB/C: 17508 is 25 C, the round trip holds
# over the whole range, and sign-extending 0x8000 would be badly wrong.

# %%
# %%extract ../tests/test_tmag5273.py -a --strip-ipytest
# %%ipytest


def test_temperature_codec():
    assert TMAG5273.celsius_from_raw(17508) == 25.0
    assert TMAG5273.celsius_from_raw(17508 + 58) == 26.0
    for c in (-40.0, 0.0, 25.0, 100.0, 170.0):
        assert TMAG5273.celsius_from_raw(TMAG5273.celsius_to_raw(c)) == pytest.approx(c, abs=0.02)
    assert TMAG5273.celsius_to_raw(TMAG5273.TEMP_MAX) < 0x8000
    signed = 25.0 + (TMAG5273.sign_extend(0x8000, 16) - 17508) / 58
    assert abs(TMAG5273.celsius_from_raw(0x8000) - signed) > 1000


# %% [markdown]
# Angle words: the two datasheet examples, and a round trip to the nearest sixteenth.

# %%
# %%extract ../tests/test_tmag5273.py -a --strip-ipytest
# %%ipytest


def test_angle_codec():
    assert TMAG5273.angle_from_raw(0x1628) == 354.5
    assert TMAG5273.angle_from_raw(0x0114) == 17.25
    for deg in (0.0, 17.25, 90.0, 180.0, 354.5, 359.9375):
        assert TMAG5273.angle_from_raw(TMAG5273.angle_to_raw(deg)) == deg
    assert TMAG5273.angle_to_raw(359.9375) <= 0x1FFF


# %% [markdown]
# Offsets: the section 6.5.3 example, round trips at both ranges, and a refusal naming the limit.

# %%
# %%extract ../tests/test_tmag5273.py -a --strip-ipytest
# %%ipytest


def test_offset_codec():
    assert TMAG5273.offset_from_raw(0b10000000, 40.0) == -2.5
    assert TMAG5273.offset_from_raw(0b00010000, 40.0) == pytest.approx(0.3125)
    for br in (40.0, 80.0):
        limit = TMAG5273.offset_limit(br)
        for mT in (0.0, limit / 8, -limit / 8, -limit, limit * 127 / 128):
            assert TMAG5273.offset_from_raw(TMAG5273.offset_to_raw(mT, br), br) == pytest.approx(mT)
    with pytest.raises(ValueError, match="representable"):
        TMAG5273.offset_to_raw(5.0, 40.0)


# %% [markdown]
# Fast-read packet lengths for each mode, with and without temperature and CRC.

# %%
# %%extract ../tests/test_tmag5273.py -a --strip-ipytest
# %%ipytest


@pytest.mark.parametrize("mode, chans, temp, crc, want", PACKET_CASES)
def test_packet_length(mode, chans, temp, crc, want):
    assert TMAG5273.packet_length(mode, tuple(chans), temp, crc) == want


# %% [markdown]
# The standard read mode (and the reserved code) have no packet.

# %%
# %%extract ../tests/test_tmag5273.py -a --strip-ipytest
# %%ipytest


@pytest.mark.parametrize("mode", [0, 3])
def test_packet_length_rejects_standard(mode):
    with pytest.raises(ValueError):
        TMAG5273.packet_length(mode, ("X",), False, False)


# %% [markdown]
# Each code's streamed axes are its name's letters, once each, in name order (as measured).

# %%
# %%extract ../tests/test_tmag5273.py -a --strip-ipytest
# %%ipytest


def test_streamed_axes_follow_the_name_once_each():
    assert TMAG5273.MAG_CH[0x7] == "XYZ" and TMAG5273.MAG_CH[0xB] == "XZX"
    assert TMAG5273.CH_SEQUENCE[0x5] == ("Z", "X")
    assert TMAG5273.CH_SEQUENCE[0x8] == ("X", "Y")
    assert TMAG5273.CH_SEQUENCE[0xB] == ("X", "Z")
    for code, name in TMAG5273.MAG_CH.items():
        seq = TMAG5273.CH_SEQUENCE[code]
        assert len(set(seq)) == len(seq) and "".join(seq) == "".join(
            sorted(set(name), key=name.index)
        )


# %% [markdown]
# Conversion time follows section 5.11: 50 us for one channel unaveraged, 825 us at 32x, plus
# 25 us per sample for each extra channel.

# %%
# %%extract ../tests/test_tmag5273.py -a --strip-ipytest
# %%ipytest


@pytest.mark.parametrize("channels, averaging, want", CONV_TIME_CASES)
def test_conversion_time(channels, averaging, want):
    assert TMAG5273.conversion_time_us(channels, averaging) == want


# %% [markdown]
# ### validation/test: the driver on a fake part

# %% [markdown]
# #### validation: nominal usage
# Identify, configure, then read by register and by a CRC-checked fast read.

# %%
mag, part = on_fake(TMAG5273)
print("identify:", tuple(hex(v) for v in mag.identify()), "| variant:", mag.variant())
mag.configure(channels="XYZ", temperature=True, averaging=4)
print("ranges:", (mag.xy_range, mag.z_range), "| conversion",
      mag.conversion_time_us(3, mag.averaging), "us")  # fmt: skip
print("all_read (T, X, Y, Z):", mag.all_read())
mag.temperature_enabled = False
mag.crc_enabled = True
mag.read_mode = "fast16"
print("fast16 + CRC:", mag.fast_read(), "| packet", mag.fast_read_length(), "bytes")
mag.read_mode = "standard"

# %% [markdown]
# #### validation: anti-nominal usage
# In a fast read mode the driver will not read registers: a status read raises and says what to
# do. Getters still answer from the cache.

# %%
mag, part = on_fake(TMAG5273)
mag.configure(channels="XYZ")
mag.read_mode = "fast16"
try:
    mag.conv_status()  # a register read in fast16
except RuntimeError as err:
    print(f"raised RuntimeError, as expected:\n    {err}")
else:
    raise AssertionError("a register read in fast16 should have raised RuntimeError")
finally:
    print("channels from the cache:", mag.channels)
    mag.read_mode = "standard"

# %% [markdown]
# A flipped bit in a packet and in its retry fails the CRC (one spoiled packet alone is read
# again).

# %%
mag, part = on_fake(TMAG5273)
mag.configure(channels="XY")
mag.crc_enabled = True
mag.read_mode = "fast16"
part.corrupt = 2  # the read and its retry
try:
    mag.fast_read()
except ValueError as err:
    print(f"raised ValueError, as expected:\n    {err}")
else:
    raise AssertionError("a corrupted packet should have raised ValueError")

# %% [markdown]
# #### unit

# %% [markdown]
# The constructor identifies the part: TI's manufacturer ID, `VER` 1, and the B1's address.

# %%
# %%extract ../tests/test_tmag5273.py -a --strip-ipytest
# %%ipytest


def test_identify_and_variant():
    mag, _ = on_fake(TMAG5273)
    assert mag.identify() == (0x5449, 1)
    assert mag.variant() == "B1"
    assert mag.ranges == {0: 40.0, 1: 80.0}
    assert on_fake(TMAG5273, addr=0x35, version=2)[0].variant() == "A2"


# %% [markdown]
# A part that is not a TMAG5273 is refused, and nothing is written to it.

# %%
# %%extract ../tests/test_tmag5273.py -a --strip-ipytest
# %%ipytest


def test_init_refuses_a_part_that_is_not_a_tmag5273():
    bus = FakeBus()
    bus.attach(0x22, FakeRegisterPart())
    with pytest.raises(RuntimeError, match="not a TMAG5273"):
        TMAG5273(I2CConnection(bus, 0x22))
    assert not any(entry[0] in ("w", "wm") for entry in bus.log)


# %% [markdown]
# `configure` sets channels, temperature, averaging, ranges and mode, and the driver tracks the
# ranges. A bad channel name is refused.

# %%
# %%extract ../tests/test_tmag5273.py -a --strip-ipytest
# %%ipytest


def test_configure_sets_everything():
    mag, _ = on_fake(TMAG5273)
    mag.configure(channels="XYZ", temperature=True, averaging=4, xy_range=80.0)
    assert (mag.channels, mag.temperature_enabled, mag.averaging) == ("XYZ", True, 4)
    assert mag.operating_mode == "continuous"
    assert (mag.xy_range, mag.z_range) == (80.0, 40.0)
    with pytest.raises(ValueError):
        mag.channels = "QQ"


# %% [markdown]
# Register reads decode the field and temperature the fake holds.

# %%
# %%extract ../tests/test_tmag5273.py -a --strip-ipytest
# %%ipytest


def test_reads_results():
    mag, _ = on_fake(TMAG5273, field_mT=(5.0, -5.0, 0.0), temp_c=25.0)
    mag.configure(channels="XYZ", temperature=True)
    assert mag.all_read() == (25.0, 5.0, -5.0, 0.0)
    assert mag.magnetic() == (5.0, -5.0, 0.0)
    assert mag.temperature() == 25.0
    assert mag.magnetic_raw() == (0x1000, 0xF000, 0x0000)


# %% [markdown]
# The same field reads the same at both ranges: a driver that missed the range change would be
# off by exactly 2x.

# %%
# %%extract ../tests/test_tmag5273.py -a --strip-ipytest
# %%ipytest


def test_range_switch_reports_the_same_field():
    mag, _ = on_fake(TMAG5273, field_mT=(5.0, -5.0, 12.0))
    mag.configure(channels="XYZ")
    at40 = mag.magnetic()
    assert mag.ranges_set(80.0, 80.0) == (80.0, 80.0)
    at80 = mag.magnetic()
    assert at40 == pytest.approx(at80, abs=2 * 80.0 / 65536)


# %% [markdown]
# With CRC on, standard reads stay at four bytes or fewer (the fifth would be the CRC), and the
# values are unchanged.

# %%
# %%extract ../tests/test_tmag5273.py -a --strip-ipytest
# %%ipytest


def test_crc_on_standard_reads_stay_within_four_bytes():
    mag, _ = on_fake(TMAG5273, field_mT=(5.0, -5.0, 2.5))
    mag.configure(channels="XYZ", temperature=True)
    mag.crc_enabled = True
    bus = mag.conn.bus
    start = len(bus.log)
    assert mag.all_read() == (25.0, 5.0, -5.0, 2.5)
    assert mag.magnetic() == (5.0, -5.0, 2.5)
    reads = [e for e in bus.log[start:] if e[0] == "rm"]
    assert reads and all(e[3] <= 4 for e in reads)


# %% [markdown]
# A fast16 read with CRC decodes the same values as the register path, verifies its CRC, and is
# exactly one transfer.

# %%
# %%extract ../tests/test_tmag5273.py -a --strip-ipytest
# %%ipytest


def test_fast16_read_with_crc_is_one_transfer():
    mag, _ = on_fake(TMAG5273, field_mT=(5.0, -5.0, 2.5))
    mag.configure(channels="XYZ")
    mag.crc_enabled = True
    mag.read_mode = "fast16"
    bus = mag.conn.bus
    start = len(bus.log)
    readings, status = mag.fast_read()
    assert readings == {"X": 5.0, "Y": -5.0, "Z": 2.5}
    assert status & 0x01  # RESULT_STATUS
    assert [e[0] for e in bus.log[start:]] == ["r"]


# %% [markdown]
# fast8 sends only the MSBs: each axis is within one 1/256-of-span step, and the temperature
# within `TEMP_LSB_STEP_C`, reading low.

# %%
# %%extract ../tests/test_tmag5273.py -a --strip-ipytest
# %%ipytest


def test_fast8_read_quantises():
    mag, _ = on_fake(TMAG5273, field_mT=(5.1, -5.1, 0.2), temp_c=27.0)
    mag.configure(channels="XYZ", temperature=True)
    mag.read_mode = "fast8"
    readings, _ = mag.fast_read()
    for ax, want in zip("XYZ", (5.1, -5.1, 0.2)):
        assert abs(readings[ax] - want) <= 2 * 40.0 / 256
    assert 27.0 - TMAG5273.TEMP_LSB_STEP_C <= readings["T"] <= 27.0


# %% [markdown]
# A flipped bit in both the packet and its retry fails the CRC; `verify=False` returns the bad
# packet without checking.

# %%
# %%extract ../tests/test_tmag5273.py -a --strip-ipytest
# %%ipytest


def test_fast_read_detects_corruption():
    mag, part = on_fake(TMAG5273)
    mag.configure(channels="XY")
    mag.crc_enabled = True
    mag.read_mode = "fast16"
    part.corrupt = 2  # the read and its retry
    with pytest.raises(ValueError, match="checksum"):
        mag.fast_read()
    part.corrupt = 1
    mag.fast_read(verify=False)


# %% [markdown]
# One spoiled packet is read again and the retry passes: two reads, correct values.

# %%
# %%extract ../tests/test_tmag5273.py -a --strip-ipytest
# %%ipytest


def test_fast_read_retries_once():
    mag, part = on_fake(TMAG5273)
    mag.configure(channels="XY")
    mag.crc_enabled = True
    mag.read_mode = "fast16"
    part.corrupt = 1
    bus = mag.conn.bus
    start = len(bus.log)
    assert mag.fast_read()[0] == {"X": 5.0, "Y": -5.0}
    assert [e[0] for e in bus.log[start:]] == ["r", "r"]


# %% [markdown]
# Behind a mux the part's CRC also covers the mux's address byte from the channel select. The
# driver accepts it with no retry (one mux write, one read), and the plain rule would not.

# %%
# %%extract ../tests/test_tmag5273.py -a --strip-ipytest
# %%ipytest


def test_fast_read_crc_covers_the_mux_byte():
    mag, _ = on_fake(TMAG5273, mux=True)
    mag.configure(channels="XY")
    mag.crc_enabled = True
    mag.read_mode = "fast16"
    bus = mag.conn.bus
    start = len(bus.log)
    assert mag.fast_read()[0] == {"X": 5.0, "Y": -5.0}
    assert [e[0] for e in bus.log[start:]] == ["w", "r"]
    raw = mag.read_raw(mag.fast_read_length())
    tail = bytes([(0x22 << 1) | 1]) + raw[:-1]
    assert TMAG5273.crc(bytes([0x70 << 1]) + tail) == raw[-1] != TMAG5273.crc(tail)


# %% [markdown]
# fast16 with CRC allows three channels; four (XYZ plus T) are refused before any transfer.

# %%
# %%extract ../tests/test_tmag5273.py -a --strip-ipytest
# %%ipytest


def test_fast16_crc_refuses_four_channels():
    mag, _ = on_fake(TMAG5273)
    mag.configure(channels="XYZ", temperature=True)
    mag.crc_enabled = True
    mag.read_mode = "fast16"
    start = len(mag.conn.bus.log)
    with pytest.raises(ValueError, match="at most 3"):
        mag.fast_read()
    assert len(mag.conn.bus.log) == start


# %% [markdown]
# Pseudo-simultaneous and reordered codes send each axis once, in name order, with the right
# values.

# %%
# %%extract ../tests/test_tmag5273.py -a --strip-ipytest
# %%ipytest


def test_pseudo_simultaneous_axes_stream_once():
    mag, _ = on_fake(TMAG5273, field_mT=(5.0, -5.0, 2.5))
    for name, want in (("XZX", ["X", "Z"]), ("ZX", ["Z", "X"]), ("YXY", ["Y", "X"])):
        mag.configure(channels=name)
        mag.read_mode = "fast16"
        words, _, _ = mag.fast_read_raw()
        assert [n for n, _ in words] == want
        field = dict(zip("XYZ", (5.0, -5.0, 2.5)))
        assert mag.fast_read()[0] == {a: field[a] for a in want}
        mag.read_mode = "standard"


# %% [markdown]
# In a fast read mode register reads and setters raise, getters answer from the cache, and
# switching back to standard makes registers readable again.

# %%
# %%extract ../tests/test_tmag5273.py -a --strip-ipytest
# %%ipytest


def test_registers_unreadable_in_fast_mode():
    mag, _ = on_fake(TMAG5273)
    mag.configure(channels="XYZ")
    mag.read_mode = "fast16"
    with pytest.raises(RuntimeError, match="standard"):
        mag.conv_status()
    with pytest.raises(RuntimeError, match="standard"):
        mag.channels = "XY"
    assert (mag.channels, mag.read_mode) == ("XYZ", "fast16")
    mag.read_mode = "standard"
    assert mag.conv_status()["ready"]
    assert mag.read_mode == "standard"


# %% [markdown]
# With the `XY` angle pair the offsets shift X and Y by Equation 16, and the hardware angle is
# the `atan2` of the corrected field.

# %%
# %%extract ../tests/test_tmag5273.py -a --strip-ipytest
# %%ipytest


def test_offsets_and_angle():
    mag, _ = on_fake(TMAG5273, field_mT=(5.0, -5.0, 0.0))
    mag.configure(channels="XYZ")
    mag.angle_pair = "XY"
    assert mag.angle() == pytest.approx(315.0, abs=1 / 16)
    assert mag.offsets_set(-2.5, 1.25) == (-2.5, 1.25)
    x, y, z = mag.magnetic()
    assert (x, y, z) == pytest.approx((2.5, -3.75, 0.0), abs=1e-3)
    want = math.degrees(math.atan2(y, x)) % 360.0
    assert mag.angle() == pytest.approx(want, abs=1 / 16)


# %% [markdown]
# In standby the part converts only when triggered: `SET_COUNT` stays put, then advances once.

# %%
# %%extract ../tests/test_tmag5273.py -a --strip-ipytest
# %%ipytest


def test_trigger_converts_in_standby():
    mag, _ = on_fake(TMAG5273)
    mag.configure(channels="XYZ", mode="standby")
    first = mag.conv_status()["set_count"]
    assert mag.conv_status()["set_count"] == first
    mag.trigger()
    assert mag.conv_status()["set_count"] == (first + 1) % 8
    assert mag.magnetic() == (5.0, -5.0, 0.0)


# %% [markdown]
# After entering sleep the first transfer is not acknowledged; `wake()` absorbs it and leaves
# the part in standby.

# %%
# %%extract ../tests/test_tmag5273.py -a --strip-ipytest
# %%ipytest


def test_sleep_and_wake():
    mag, _ = on_fake(TMAG5273)
    mag.operating_mode = "sleep"
    with pytest.raises(OSError):
        mag.device_status()
    mag.operating_mode = "sleep"
    mag.wake()
    assert mag.operating_mode == "standby"
    assert not mag.device_status()["vcc_uv_error"]


# %% [markdown]
# `status_clear` clears the power-on flag (the fake starts with it set, like the part).

# %%
# %%extract ../tests/test_tmag5273.py -a --strip-ipytest
# %%ipytest


def test_status_clear_clears_por():
    mag, _ = on_fake(TMAG5273)
    assert mag.conv_status()["por"]
    mag.status_clear()
    assert not mag.conv_status()["por"]
    assert not any(v for k, v in mag.device_status().items() if k.endswith("error"))


# %% [markdown]
# `reset_config` writes every config register back to 0, even from a fast read mode, and leaves
# `I2C_ADDRESS` alone.

# %%
# %%extract ../tests/test_tmag5273.py -a --strip-ipytest
# %%ipytest


def test_reset_config_restores_por():
    mag, part = on_fake(TMAG5273)
    mag.configure(channels="XY", temperature=True, xy_range=80.0)
    mag.read_mode = "fast8"
    assert mag.reset_config() == (40.0, 40.0)
    assert all(part.regs[r] == 0 for r in TMAG5273.CONFIG_REGS)
    assert part.regs[0x0C] == 0x22 << 1
    assert mag.read_mode == "standard"


# %% [markdown]
# A part left in a fast read mode cannot be identified; `reset=True` puts it back to standard
# reads first.

# %%
# %%extract ../tests/test_tmag5273.py -a --strip-ipytest
# %%ipytest


def test_reset_flag_recovers_a_part_left_in_fast_mode():
    _, part = on_fake(TMAG5273)
    part.regs[0x00] = 0x01  # left in fast16 by an earlier session
    bus = FakeBus()
    bus.attach(0x22, part)
    with pytest.raises(RuntimeError, match="reset=True"):
        TMAG5273(I2CConnection(bus, 0x22))
    mag = TMAG5273(I2CConnection(bus, 0x22), reset=True)
    assert mag.read_mode == "standard" and mag.identify() == (0x5449, 1)


# %% [markdown]
# # Hardware validation
#
# > ## ⚠️ STOP — do not run this section unattended
# >
# > **Everything below talks to the real board.** It connects to the FeatherS3, uploads the
# > package, rewrites the TMAG5273's configuration, switches its read modes, puts it to sleep
# > and wakes it, and has a step where **you** wave a magnet near it.
# >
# > - Do **not** use *Run All*, *Run All Below* or `jlx run` on this section and walk away.
# > - Run it **one cell at a time**. Read the **What to do** line above each cell first.
# > - Each cell puts the sensor back to its power-on configuration when it finishes, even if
# >   it fails.
# > - The section ends with `%board disconnect`. If you stop partway, run that cell yourself.
#
# **Hardware and configuration** (same as the table at the top):
#
# | | Host run (simulation) | Hardware run |
# |---|---|---|
# | **Runs on** | CPython 3.13.15 in the micropylab kernel (1.0.0a1), no board | Unexpected Maker FeatherS3 (ESP32-S3), MicroPython 1.28.0 (v1.28.0, 2026-04-06), USB `/dev/ttyACM0` |
# | **I2C bus** | `FakeBus` | `machine.I2C(1)`, **bus 1**, scl=15 sda=16, 100 kHz (`bus_open("feathers3", 1)`) |
# | **Mux** | `FakeMux` at `0x70`, channel 0, only in tests that ask for it (`mux=True`); the rest talk to the part directly | SparkFun Qwiic Mux (TCA9548A) at `0x70` (address jumpers open), **channel 0** |
# | **Sensor** | `FakeTMAG5273`: a B1 at `0x22`, power-on state, field (5, -5, 0) mT, 25 C | SparkFun Qwiic TMAG5273, a **B1** (TMAG5273B1) at `0x22` |
# | **Other parts used** | none | TMP102 at `0x48` on mux channel 2: the reference for the temperature check |
# | **Datasheet** | TI SLYS045C (rev C, April 2026) | same |
# | **Result** | 36 unit tests pass | 79 of 79 hardware checks passed (2026-11-14) |
#
# Nothing else on bus 1 is touched. Bus 0 is not used by this notebook. To rerun the hardware
# section, wire the bench this way again (README "The bench").
#
# Layout: connect and set up, the mux, identification, then **matching tests** (one per unit test
# above, same name, run on the real part and on MicroPython), hardware-only checks,
# **diagnostics** (they print what the part does and do not count towards the tally), then the
# tally and disconnect.

# %% [markdown]
# ## Connect and set up

# %% [markdown]
# Notebook-only: connect to the board. Only one notebook can own the board: if another notebook
# (the TMP102 one, for example) still has it, run `%board disconnect` there first.
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
# `/lib/micropython_i2c`. The copy runs on the host; the path is relative to `dev_notebooks/`.
#
# **What to do:** nothing. Takes a few seconds.

# %%
# %%host
from pathlib import Path

pkg = next(p for p in (Path("../src"), Path("src")) if p.is_dir()) / "micropython_i2c"
# %files copy -r {pkg} :/lib/micropython_i2c

# %% [markdown]
# Notebook-only: drop any modules an earlier run left on the board, so the new files load, then
# import what the checks use and zero the tally.
#
# **What to do:** nothing.

# %%
import math
import sys

for name in [m for m in sys.modules if m.startswith("micropython_i2c")]:
    del sys.modules[name]

from micropython_i2c import I2CConnection, default_lock
from micropython_i2c.boards import bus_open
from micropython_i2c.compat import sleep_ms
from micropython_i2c.debug import check, check_raises, check_reset, check_summary, info, show
from micropython_i2c.debug import check_counts  # noqa: F401  (read from the host with %eval)
from micropython_i2c.drivers.tca9548a import TCA9548A
from micropython_i2c.drivers.tmag5273 import TMAG5273, TMAG5273Regs
from micropython_i2c.drivers.tmp102 import TMP102

check_reset()

# %% [markdown]
# Notebook-only: the bus, the mux, and helpers. `on_board(cls)` gives each check a fresh object
# on a part at power-on configuration; `restore` puts it back afterwards and also wakes a
# sleeping part. `CountingBus` wraps the bus and records the TMAG5273's reads, so a check can
# see how many transfers a call made and how long they were. If the bench is rewired, change
# the constants here.
#
# **What to do:** nothing.

# %%
MUX_BUS = 1  # second Qwiic chain, scl=15 sda=16
MUX_ADDR = 0x70
TMAG_ADDR = 0x22  # SparkFun board: a B1
TMAG_PATH = [[MUX_ADDR, 0]]
TMP_ADDR = 0x48
TMP_PATH = [[MUX_ADDR, 2]]

bus = bus_open("feathers3", MUX_BUS, freq=100_000)
bus_lock = default_lock()
mux = TCA9548A(bus, MUX_ADDR, lock=bus_lock)


class CountingBus:
    """The bus, recording the TMAG5273's reads as ``(kind, nbytes)``."""

    def __init__(self, inner):
        self.inner = inner
        self.reads = []

    def __getattr__(self, name):
        return getattr(self.inner, name)

    def readfrom(self, addr, nbytes, *args):
        if addr == TMAG_ADDR:
            self.reads.append(("raw", nbytes))
        return self.inner.readfrom(addr, nbytes, *args)

    def readfrom_mem(self, addr, reg, nbytes, addrsize=8):
        if addr == TMAG_ADDR:
            self.reads.append(("mem", nbytes))
        return self.inner.readfrom_mem(addr, reg, nbytes, addrsize=addrsize)


def tmag_conn(b=None):
    """A connection to the TMAG5273 behind mux channel 0 (optionally on a wrapped bus)."""
    return I2CConnection(b or bus, TMAG_ADDR, TMAG_PATH, bus_lock)


def restore(regs):
    """Back to power-on: awake, standard reads, every config register 0, POR cleared."""
    try:
        regs.read_mem(regs.REG_DEVICE_ID, 1)
    except OSError:
        sleep_ms(1)  # it was asleep: that transfer woke it
    regs.reg_write(regs.REG_DEVICE_CONFIG_1, regs.CONFIG_POR)  # raw: works from a fast mode
    # The raw write does not touch the cache: a TMAG5273 object would still think it is in a
    # fast mode and refuse the read-backs below. Forget everything it cached.
    regs.cache_clear()
    for reg in regs.CONFIG_REGS:
        regs.reg_set(reg, regs.CONFIG_POR)
    regs.reg_write(regs.REG_CONV_STATUS, regs.field_mask(regs.F_POR))
    regs.cache_clear()


def on_board(cls, b=None, **kw):
    """A fresh ``cls`` on the real TMAG5273, with the part at power-on configuration."""
    restore(TMAG5273Regs(tmag_conn()))
    return cls(tmag_conn(b), **kw)


# %% [markdown]
# ## The mux
# With every channel off only the mux answers; with channel 0 on, the TMAG5273 joins the bus.
# If it is missing, the Qwiic cable is probably in another mux port (port 2 is the TMP102).
#
# **What to do:** nothing.

# %%
root = bus.scan()
print("bus {} answers at:".format(MUX_BUS), [hex(a) for a in root])
if check("mux ACKs at {:#04x}".format(MUX_ADDR), MUX_ADDR in root):
    try:
        mux.channel_deselect()
        check("TMAG5273 hidden with every channel off", TMAG_ADDR not in bus.scan())
        mux.channel_select(0)
        seen = bus.scan()
        print("channel 0 on:    ", [hex(a) for a in seen])
        check("TMAG5273 ACKs at {:#04x} on channel 0".format(TMAG_ADDR), TMAG_ADDR in seen)
        if TMAG_ADDR not in seen and 0x35 in seen:
            info("0x35 answers: an A variant. Set TMAG_ADDR = 0x35 above")
    finally:
        mux.channel_deselect()
else:
    info("no mux: run drivers_tca9548a_devNB's 'Find the mux', or check the cable")

# %% [markdown]
# ## Identification
# Through `TMAG5273Regs` alone, so a failure here is the mixins, the connection or the wiring,
# never driver logic: the manufacturer ID, the version, the address register, and the error
# flags.
#
# **What to do:** nothing.

# %%
regs = TMAG5273Regs(tmag_conn())
restore(regs)
lsb = regs.reg_read(regs.REG_MANUFACTURER_ID_LSB)
msb = regs.reg_read(regs.REG_MANUFACTURER_ID_MSB)
check(
    "manufacturer ID", (msb << 8) | lsb == regs.MANUFACTURER_ID, "{:#06x}".format((msb << 8) | lsb)
)
ver = regs.field_get(regs.REG_DEVICE_ID, regs.F_VER)
check("VER 1 (+-40/80 mT)", ver == 1, "VER = {}".format(ver))
addr_reg = regs.reg_read(regs.REG_I2C_ADDRESS)
show(addr_reg, "I2C_ADDRESS", 1)
check(
    "I2C_ADDRESS holds the bus address", addr_reg >> 1 == TMAG_ADDR, "{:#04x}".format(addr_reg >> 1)
)
status = regs.reg_read(regs.REG_DEVICE_STATUS)
show(status, "DEVICE_STATUS", 1)
for name, field in (("VCC undervoltage", regs.F_VCC_UV_ER), ("OTP CRC error", regs.F_OTP_CRC_ER),
                    ("INT pin error", regs.F_INT_ER), ("oscillator error", regs.F_OSC_ER)):  # fmt: skip
    check(name + " clear", regs.field_extract(status, field) == 0)

# %% [markdown]
# ## Matching tests: `TMAG5273Regs`
# One cell per unit test above, with the same name. Tests that need no bus traffic run too: they
# check the arithmetic on MicroPython itself, which the host tests cannot.

# %% [markdown]
# Notebook-only: the lists the parametrized tests use, copied to the board (it cannot import the
# test file, which needs pytest). The cell after it checks the copy against the notebook's.
#
# **What to do:** nothing.

# %%
FIELDS_BY_REG = {
    "DEVICE_CONFIG_1": ("F_CRC_EN", "F_MAG_TEMPCO", "F_CONV_AVG", "F_I2C_RD"),
    "DEVICE_CONFIG_2": ("F_THR_HYST", "F_LP_LN", "F_GLITCH_FILTER", "F_TRIGGER_MODE",
                        "F_OPERATING_MODE"),
    "SENSOR_CONFIG_1": ("F_MAG_CH_EN", "F_SLEEPTIME"),
    "SENSOR_CONFIG_2": ("F_THRX_COUNT", "F_MAG_THR_DIR", "F_MAG_GAIN_CH", "F_ANGLE_EN",
                        "F_X_Y_RANGE", "F_Z_RANGE"),
    "T_CONFIG": ("F_T_THR_CONFIG", "F_T_CH_EN"),
    "INT_CONFIG_1": ("F_RSLT_INT", "F_THRSLD_INT", "F_INT_STATE", "F_INT_MODE", "F_MASK_INTB"),
    "CONV_STATUS": ("F_SET_COUNT", "F_POR", "F_DIAG_STATUS", "F_RESULT_STATUS"),
    "DEVICE_STATUS": ("F_INTB_RB", "F_OSC_ER", "F_INT_ER", "F_OTP_CRC_ER", "F_VCC_UV_ER"),
}  # fmt: skip
MAG_CASES = [(0x0000, 40.0, 0.0), (0x1000, 40.0, 5.0), (0xF000, 40.0, -5.0), (0x0800, 40.0, 2.5),
             (0x7FFF, 40.0, 39.99878), (0x8000, 40.0, -40.0), (0x1000, 80.0, 10.0),
             (0x8000, 80.0, -80.0)]  # fmt: skip
PACKET_CASES = [(1, "XYZ", False, False, 7), (1, "XYZ", True, False, 9), (1, "X", False, True, 4),
                (1, "XY", True, True, 8), (2, "XYZ", True, False, 5), (2, "XYZ", True, True, 6),
                (2, "XY", False, True, 4)]  # fmt: skip
CONV_TIME_CASES = [(1, 1, 50), (1, 32, 825), (2, 1, 75), (2, 32, 1625), (3, 1, 100)]

# %% [markdown]
# Notebook-only: on the host, read the board's copies back with `%eval` and check they match the
# notebook's own, so the two cannot drift apart.
#
# **What to do:** nothing.

# %%
# %%host
names = "[sorted(FIELDS_BY_REG.items()), MAG_CASES, PACKET_CASES, CONV_TIME_CASES]"
board = get_ipython().run_line_magic("eval", names)
host = [sorted(FIELDS_BY_REG.items()), MAG_CASES, PACKET_CASES, CONV_TIME_CASES]
as_lists = [[list(item) if isinstance(item, tuple) else item for item in part] for part in host]
as_lists[0] = [[k, list(v)] for k, v in host[0]]
if board != as_lists:
    raise AssertionError("the board's copy of the test lists differs from the notebook's")
print("board copies of the test lists match the notebook's")

# %% [markdown]
# **Matches** `test_fields_do_not_overlap`, for every config and status register.
#
# **What to do:** nothing. No bus traffic.

# %%
for reg, names in sorted(FIELDS_BY_REG.items()):
    covered, ok = 0, True
    for name in names:
        mask = TMAG5273Regs.field_mask(getattr(TMAG5273Regs, name))
        ok = ok and not covered & mask and mask <= 0xFF
        covered |= mask
    check("test_fields_do_not_overlap[{}]".format(reg), ok, "{:#04x}".format(covered))

# %% [markdown]
# **Matches** `test_crc_vectors_match_datasheet`: rev C's seven CRC examples on MicroPython.
#
# **What to do:** nothing. No bus traffic.

# %%
bad = [d for d, want in TMAG5273Regs.CRC_VECTORS if TMAG5273Regs.crc(d) != want]
check("test_crc_vectors_match_datasheet", len(TMAG5273Regs.CRC_VECTORS) == 7 and not bad,
      "bad: {}".format(bad) if bad else "7 vectors")  # fmt: skip

# %% [markdown]
# **Matches** `test_address_and_clock_limits`.
#
# **What to do:** nothing. No bus traffic.

# %%
check("test_address_and_clock_limits",
      all(TMAG5273Regs.addr_ok(a) for a in TMAG5273Regs.FACTORY_ADDRS.values())
      and not TMAG5273Regs.addr_ok(0x07) and not TMAG5273Regs.addr_ok(0x79)
      and TMAG5273Regs.baud_ok(1_000_000) and not TMAG5273Regs.baud_ok(3_400_000))  # fmt: skip

# %% [markdown]
# **Matches** `test_auto_increment_reads_register_pairs`, on the real part's ID pair (constant,
# unlike the result registers).
#
# **What to do:** nothing.

# %%
regs = TMAG5273Regs(tmag_conn())
try:
    lsb = regs.reg_read(regs.REG_MANUFACTURER_ID_LSB)
    msb = regs.reg_read(regs.REG_MANUFACTURER_ID_MSB)
    pair = regs.reg_read(regs.REG_MANUFACTURER_ID_LSB, nbytes=2)
    regs.cache_clear()
    narrow = regs.reg_get(regs.REG_MANUFACTURER_ID_LSB)
    wide = regs.reg_get(regs.REG_MANUFACTURER_ID_LSB, nbytes=2)
    check("test_auto_increment_reads_register_pairs",
          pair == (lsb << 8) | msb == 0x4954 and narrow == lsb and wide == 0x4954,
          "{:#06x}".format(pair))  # fmt: skip
finally:
    restore(regs)

# %% [markdown]
# **Matches** `test_status_bits_are_write_1_to_clear`: a verified write of `POR` is refused, a
# raw write clears it. `POR` is only set after a power cycle; if an earlier run cleared it, the
# cell says so.
#
# **What to do:** nothing.

# %%
regs = TMAG5273Regs(tmag_conn())
try:
    por = regs.field_mask(regs.F_POR)
    if not regs.field_get(regs.REG_CONV_STATUS, regs.F_POR):
        info("POR already clear (no power cycle since the last run)")
    check_raises(
        "verified write of POR refused", ValueError, regs.reg_set, regs.REG_CONV_STATUS, por
    )
    regs.reg_write(regs.REG_CONV_STATUS, por)
    check("test_status_bits_are_write_1_to_clear",
          regs.field_get(regs.REG_CONV_STATUS, regs.F_POR) == 0)  # fmt: skip
finally:
    restore(regs)

# %% [markdown]
# ## Matching tests: codecs
# The datasheet numbers on MicroPython (its float formatting and integer division differ from
# CPython's in places).

# %% [markdown]
# **Matches** `test_mag_decode`.
#
# **What to do:** nothing. No bus traffic.

# %%
bad = [(hex(w), br) for w, br, want in MAG_CASES if abs(TMAG5273.mag_from_raw(w, br) - want) > 1e-4]
check(
    "test_mag_decode", not bad, "bad: {}".format(bad) if bad else "{} cases".format(len(MAG_CASES))
)

# %% [markdown]
# **Matches** `test_sensitivity_matches_datasheet`.
#
# **What to do:** nothing. No bus traffic.

# %%
for br, stated in ((40.0, 820), (80.0, 410)):
    lsb_per_mT = 1 / TMAG5273.mag_from_raw(1, br)
    check("test_sensitivity_matches_datasheet[{}]".format(br), abs(lsb_per_mT - stated) <= 1.0,
          "{:.1f} LSB/mT".format(lsb_per_mT))  # fmt: skip

# %% [markdown]
# **Matches** `test_mag_round_trip`.
#
# **What to do:** nothing. No bus traffic.

# %%
for br in (40.0, 80.0):
    worst = max(abs(TMAG5273.mag_from_raw(TMAG5273.mag_to_raw(mT, br), br) - mT)
                for mT in (0.0, 5.0, -5.0, 0.001221, br * 0.99, -br * 0.99))  # fmt: skip
    check(
        "test_mag_round_trip[{}]".format(br),
        worst <= 2 * br / 65536,
        "worst {:.6f} mT".format(worst),
    )

# %% [markdown]
# **Matches** `test_mag_out_of_range`.
#
# **What to do:** nothing. No bus traffic.

# %%
for mT, br in ((50.0, 40.0), (-50.0, 40.0), (100.0, 80.0)):
    check_raises(
        "test_mag_out_of_range[{}-{}]".format(mT, br), ValueError, TMAG5273.mag_to_raw, mT, br
    )

# %% [markdown]
# **Matches** `test_temperature_codec`.
#
# **What to do:** nothing. No bus traffic.

# %%
worst = max(abs(TMAG5273.celsius_from_raw(TMAG5273.celsius_to_raw(c)) - c)
            for c in (-40.0, 0.0, 25.0, 100.0, 170.0))  # fmt: skip
signed = 25.0 + (TMAG5273.sign_extend(0x8000, 16) - 17508) / 58
check("test_temperature_codec",
      TMAG5273.celsius_from_raw(17508) == 25.0 and TMAG5273.celsius_from_raw(17508 + 58) == 26.0
      and worst <= 0.02 and TMAG5273.celsius_to_raw(TMAG5273.TEMP_MAX) < 0x8000
      and abs(TMAG5273.celsius_from_raw(0x8000) - signed) > 1000, "worst {:.4f} C".format(worst))  # fmt: skip

# %% [markdown]
# **Matches** `test_angle_codec`.
#
# **What to do:** nothing. No bus traffic.

# %%
trips = all(TMAG5273.angle_from_raw(TMAG5273.angle_to_raw(d)) == d
            for d in (0.0, 17.25, 90.0, 180.0, 354.5, 359.9375))  # fmt: skip
check("test_angle_codec",
      TMAG5273.angle_from_raw(0x1628) == 354.5 and TMAG5273.angle_from_raw(0x0114) == 17.25
      and trips and TMAG5273.angle_to_raw(359.9375) <= 0x1FFF)  # fmt: skip

# %% [markdown]
# **Matches** `test_offset_codec`.
#
# **What to do:** nothing. No bus traffic.

# %%
worst = 0.0
for br in (40.0, 80.0):
    limit = TMAG5273.offset_limit(br)
    for mT in (0.0, limit / 8, -limit / 8, -limit, limit * 127 / 128):
        worst = max(worst, abs(TMAG5273.offset_from_raw(TMAG5273.offset_to_raw(mT, br), br) - mT))
check("test_offset_codec",
      TMAG5273.offset_from_raw(0b10000000, 40.0) == -2.5
      and abs(TMAG5273.offset_from_raw(0b00010000, 40.0) - 0.3125) < 1e-9 and worst < 1e-6,
      "worst {:.2e} mT".format(worst))  # fmt: skip
check_raises("offset outside the range refused", ValueError, TMAG5273.offset_to_raw, 5.0, 40.0)

# %% [markdown]
# **Matches** `test_packet_length`.
#
# **What to do:** nothing. No bus traffic.

# %%
bad = [c for c in PACKET_CASES if TMAG5273.packet_length(c[0], tuple(c[1]), c[2], c[3]) != c[4]]
check(
    "test_packet_length",
    not bad,
    "bad: {}".format(bad) if bad else "{} cases".format(len(PACKET_CASES)),
)

# %% [markdown]
# **Matches** `test_packet_length_rejects_standard`.
#
# **What to do:** nothing. No bus traffic.

# %%
for mode in (0, 3):
    check_raises("test_packet_length_rejects_standard[{}]".format(mode), ValueError,
                 TMAG5273.packet_length, mode, ("X",), False, False)  # fmt: skip

# %% [markdown]
# **Matches** `test_streamed_axes_follow_the_name_once_each`.
#
# **What to do:** nothing. No bus traffic.

# %%
ok = TMAG5273.CH_SEQUENCE[0x5] == ("Z", "X") and TMAG5273.CH_SEQUENCE[0x8] == ("X", "Y")
for code, name in TMAG5273.MAG_CH.items():
    seq = TMAG5273.CH_SEQUENCE[code]
    firsts = "".join(a for i, a in enumerate(name) if a not in name[:i])
    ok = ok and len(set(seq)) == len(seq) and "".join(seq) == firsts
check("test_streamed_axes_follow_the_name_once_each", ok)

# %% [markdown]
# **Matches** `test_conversion_time`.
#
# **What to do:** nothing. No bus traffic.

# %%
bad = [c for c in CONV_TIME_CASES if TMAG5273.conversion_time_us(c[0], c[1]) != c[2]]
check(
    "test_conversion_time",
    not bad,
    "bad: {}".format(bad) if bad else "{} cases".format(len(CONV_TIME_CASES)),
)

# %% [markdown]
# ## Matching tests: `TMAG5273`
# The driver on the real part. The fake holds an exact field; the real part sees the Earth's
# field (about 0.05 mT) plus whatever is nearby, and noise. So these compare readings with each
# other, at 32x averaging, with tolerances of a few tenths of a millitesla.

# %% [markdown]
# **Matches** `test_identify_and_variant`: TI's ID, `VER` 1, and the SparkFun board's variant,
# B1.
#
# **What to do:** nothing.

# %%
mag = on_board(TMAG5273)
try:
    ident, variant = mag.identify(), mag.variant()
    print("identify:", [hex(v) for v in ident], "| variant:", variant, "| ranges:", mag.ranges)
    check("test_identify_and_variant", ident == (0x5449, 1) and variant == "B1"
          and mag.ranges == {0: 40.0, 1: 80.0}, variant)  # fmt: skip
finally:
    restore(mag)

# %% [markdown]
# **Matches** `test_init_refuses_a_part_that_is_not_a_tmag5273`, on a real wrong part: the
# TMP102 on channel 2. The constructor only reads, so nothing is written to the TMP102.
#
# **What to do:** nothing.

# %%
check_raises("test_init_refuses_a_part_that_is_not_a_tmag5273", RuntimeError, TMAG5273,
             I2CConnection(bus, TMP_ADDR, TMP_PATH, bus_lock))  # fmt: skip

# %% [markdown]
# **Matches** `test_configure_sets_everything`.
#
# **What to do:** nothing.

# %%
mag = on_board(TMAG5273)
try:
    mag.configure(channels="XYZ", temperature=True, averaging=4, xy_range=80.0)
    got = (mag.channels, mag.temperature_enabled, mag.averaging, mag.operating_mode,
           mag.xy_range, mag.z_range)  # fmt: skip
    print("read back:", got)
    check("test_configure_sets_everything", got == ("XYZ", True, 4, "continuous", 80.0, 40.0))
    check_raises("bad channel name refused", ValueError, setattr, mag, "channels", "QQ")
finally:
    restore(mag)

# %% [markdown]
# **Matches** `test_reads_results`: a plausible temperature and field, and two register reads of
# the static field that agree.
#
# **What to do:** keep magnets away from the sensor.

# %%
mag = on_board(TMAG5273)
try:
    mag.configure(channels="XYZ", temperature=True, averaging=32)
    sleep_ms(10)
    t, x, y, z = mag.all_read()
    x2, y2, z2 = mag.magnetic()
    print("T {:+.2f} C | X {:+.4f}  Y {:+.4f}  Z {:+.4f} mT".format(t, x, y, z))
    print("raw words:", [hex(w) for w in mag.magnetic_raw()], "| temperature()",
          "{:.2f} C".format(mag.temperature()))  # fmt: skip
    check("test_reads_results: temperature plausible", 10.0 <= t <= 45.0, "{:.2f} C".format(t))
    check("test_reads_results: field inside the range", max(abs(x), abs(y), abs(z)) < 40.0)
    check("test_reads_results: repeat agrees",
          max(abs(x - x2), abs(y - y2), abs(z - z2)) < 0.3)  # fmt: skip
finally:
    restore(mag)

# %% [markdown]
# **Matches** `test_range_switch_reports_the_same_field`: the field does not change when the range
# does, so a driver that missed the change would be off by 2x.
#
# **What to do:** keep magnets away and the sensor still.

# %%
mag = on_board(TMAG5273)
try:
    mag.configure(channels="XYZ", averaging=32)
    sleep_ms(10)
    at40 = mag.magnetic()
    ranges = mag.ranges_set(80.0, 80.0)
    sleep_ms(10)
    at80 = mag.magnetic()
    print("+-40 mT:", ["{:+.4f}".format(v) for v in at40])
    print("+-80 mT:", ["{:+.4f}".format(v) for v in at80])
    check("driver tracked the range", ranges == (80.0, 80.0))
    check("test_range_switch_reports_the_same_field",
          all(abs(a - b) < max(0.3, abs(a) * 0.1) for a, b in zip(at40, at80)))  # fmt: skip
finally:
    restore(mag)

# %% [markdown]
# **Matches** `test_crc_on_standard_reads_stay_within_four_bytes`: with CRC on, the driver's
# reads are never longer than four bytes, and the readings agree with CRC off.
#
# **What to do:** keep magnets away and the sensor still.

# %%
cbus = CountingBus(bus)
mag = on_board(TMAG5273, b=cbus)
try:
    mag.configure(channels="XYZ", temperature=True, averaging=32)
    sleep_ms(10)
    off = mag.all_read()
    mag.crc_enabled = True
    sleep_ms(10)
    cbus.reads = []
    on = mag.all_read()
    sizes = [n for kind, n in cbus.reads]
    print("CRC off:", ["{:+.3f}".format(v) for v in off])
    print("CRC on: ", ["{:+.3f}".format(v) for v in on], "| reads:", cbus.reads)
    check("test_crc_on_standard_reads_stay_within_four_bytes",
          sizes and max(sizes) <= 4 and abs(on[0] - off[0]) < 1.0
          and all(abs(a - b) < 0.3 for a, b in zip(on[1:], off[1:])))  # fmt: skip
finally:
    restore(mag)

# %% [markdown]
# **Matches** `test_fast16_read_with_crc_is_one_transfer`: fast16 with CRC agrees with the register
# path, verifies, and is one read of the part.
#
# **What to do:** keep magnets away and the sensor still.

# %%
cbus = CountingBus(bus)
mag = on_board(TMAG5273, b=cbus)
try:
    mag.configure(channels="XYZ", averaging=32)
    sleep_ms(10)
    ref = dict(zip("XYZ", mag.magnetic()))
    mag.crc_enabled = True
    mag.read_mode = "fast16"
    sleep_ms(10)
    cbus.reads = []
    readings, status = mag.fast_read()
    print("standard:", ref, "\nfast16:  ", readings, "| status {:#04x}".format(status),
          "| reads:", cbus.reads)  # fmt: skip
    check("test_fast16_read_with_crc_is_one_transfer",
          cbus.reads == [("raw", mag.fast_read_length())] and status & 0x01
          and all(abs(readings[a] - ref[a]) < 0.3 for a in "XYZ"))  # fmt: skip
finally:
    restore(mag)

# %% [markdown]
# **Matches** `test_fast8_read_quantises`: each axis within one 1/256-of-span step of the register
# reading (plus noise), and the temperature within one MSB step, reading low.
#
# **What to do:** keep magnets away and the sensor still.

# %%
mag = on_board(TMAG5273)
try:
    mag.configure(channels="XYZ", temperature=True, averaging=32)
    sleep_ms(10)
    t, x, y, z = mag.all_read()
    mag.crc_enabled = True
    mag.read_mode = "fast8"
    sleep_ms(10)
    r8, _ = mag.fast_read()
    print("standard T {:.2f}, X/Y/Z {}".format(t, ["{:+.3f}".format(v) for v in (x, y, z)]))
    print("fast8:", r8)
    step = 2 * 40.0 / 256
    check("test_fast8_read_quantises: axes",
          all(abs(r8[a] - v) <= step + 0.3 for a, v in zip("XYZ", (x, y, z))))  # fmt: skip
    check("test_fast8_read_quantises: temperature",
          t - TMAG5273.TEMP_LSB_STEP_C - 1.0 <= r8["T"] <= t + 1.0)  # fmt: skip
finally:
    restore(mag)

# %% [markdown]
# **Matches** `test_fast_read_detects_corruption`: the part's own packet verifies under the
# measured rule (mux byte, read command, data), the same packet with one bit flipped does not,
# and leaving out the read command does not either.
#
# **What to do:** nothing.

# %%
mag = on_board(TMAG5273)
try:
    mag.configure(channels="XY", averaging=4)
    mag.crc_enabled = True
    mag.read_mode = "fast16"
    sleep_ms(10)
    raw = mag.read_raw(mag.fast_read_length())
    cmd = bytes([(TMAG_ADDR << 1) | 1])
    print("packet:", " ".join("{:02x}".format(b) for b in raw))
    flipped = bytearray(raw)
    flipped[0] ^= 0x01
    mux_byte = bytes([MUX_ADDR << 1])
    check(
        "CRC covers the mux byte, read command and data",
        mag.crc_ok(mux_byte + cmd + raw[:-1], raw[-1]),
    )
    check("fast_crc_ok accepts the part's packet", mag.fast_crc_ok(raw))
    check("leaving out the read command fails", not mag.crc_ok(mux_byte + raw[:-1], raw[-1]))
    check("test_fast_read_detects_corruption", not mag.fast_crc_ok(bytes(flipped)))
finally:
    restore(mag)

# %% [markdown]
# **Matches** `test_fast_read_retries_once`: with channels 0 and 2 both on, the TMAG5273 overhears
# a TMP102 read, which spoils its next CRC. `fast_read` reads again and succeeds.
#
# **What to do:** nothing.

# %%
cbus = CountingBus(bus)
mag = on_board(TMAG5273, b=cbus)
try:
    mag.configure(channels="XY", averaging=4)
    mag.crc_enabled = True
    mag.read_mode = "fast16"
    sleep_ms(10)
    mux.channel_mask_set(0b101)  # channels 0 and 2: the TMAG5273 hears the TMP102 read
    bus.readfrom_mem(TMP_ADDR, 0, 2)
    cbus.reads = []
    readings, _ = mag.fast_read()
    print("readings:", readings, "| reads:", cbus.reads)
    check("test_fast_read_retries_once", len(cbus.reads) == 2 and set(readings) == {"X", "Y"},
          "{} reads".format(len(cbus.reads)))  # fmt: skip
finally:
    mux.channel_deselect()
    restore(mag)

# %% [markdown]
# **Matches** `test_fast_read_crc_covers_the_mux_byte`: twenty fast reads through the mux, each
# verified with no retry (one read per packet).
#
# **What to do:** nothing.

# %%
cbus = CountingBus(bus)
mag = on_board(TMAG5273, b=cbus)
try:
    mag.configure(channels="XY", averaging=4)
    mag.crc_enabled = True
    mag.read_mode = "fast16"
    sleep_ms(10)
    cbus.reads = []
    for _ in range(20):
        mag.fast_read()
        sleep_ms(5)
    check("test_fast_read_crc_covers_the_mux_byte", len(cbus.reads) == 20,
          "{} reads for 20 packets".format(len(cbus.reads)))  # fmt: skip
finally:
    restore(mag)

# %% [markdown]
# **Matches** `test_fast16_crc_refuses_four_channels`: refused by the driver before any transfer.
#
# **What to do:** nothing.

# %%
mag = on_board(TMAG5273)
try:
    mag.configure(channels="XYZ", temperature=True)
    mag.crc_enabled = True
    mag.read_mode = "fast16"
    check_raises("test_fast16_crc_refuses_four_channels", ValueError, mag.fast_read)
finally:
    restore(mag)

# %% [markdown]
# **Matches** `test_pseudo_simultaneous_axes_stream_once`: `XZX`, `ZX` and `YXY` each send their
# axes once, in name order, and each fast16 word agrees with that axis's result register.
#
# **What to do:** keep magnets away and the sensor still.

# %%
mag = on_board(TMAG5273)
try:
    for name, want in (("XZX", ["X", "Z"]), ("ZX", ["Z", "X"]), ("YXY", ["Y", "X"])):
        mag.configure(channels=name, averaging=32)
        sleep_ms(20)
        ref = dict(zip("XYZ", mag.magnetic()))
        mag.read_mode = "fast16"
        sleep_ms(5)
        words, _, _ = mag.fast_read_raw()
        got = [(n, mag.mag_from_raw(w, mag.axis_scale(n))) for n, w in words]
        mag.read_mode = "standard"
        print(name, "sends", ["{} {:+.3f}".format(n, v) for n, v in got],
              "| registers", ["{} {:+.3f}".format(a, ref[a]) for a in want])  # fmt: skip
        check("test_pseudo_simultaneous_axes_stream_once[{}]".format(name),
              [n for n, _ in got] == want and all(abs(v - ref[n]) < 0.3 for n, v in got))  # fmt: skip
finally:
    restore(mag)

# %% [markdown]
# **Matches** `test_registers_unreadable_in_fast_mode`: the driver's guard on the real part, and
# registers readable again after switching back.
#
# **What to do:** nothing.

# %%
mag = on_board(TMAG5273)
try:
    mag.configure(channels="XYZ")
    mag.read_mode = "fast16"
    check_raises("register read refused in fast16", RuntimeError, mag.conv_status)
    check_raises("setter refused in fast16", RuntimeError, setattr, mag, "channels", "XY")
    cached = (mag.channels, mag.read_mode)
    mag.read_mode = "standard"
    check("test_registers_unreadable_in_fast_mode",
          cached == ("XYZ", "fast16") and mag.read_mode == "standard"
          and mag.conv_status()["ready"])  # fmt: skip
finally:
    restore(mag)

# %% [markdown]
# **Matches** `test_offsets_and_angle`: with the `XY` angle pair, offsets of -2.5 mT on X and
# +1.25 mT on Y shift the readings by Equation 16, and that offset field is strong enough for
# the hardware angle to be compared with `atan2` of the corrected readings.
#
# **What to do:** keep magnets away and the sensor still.

# %%
mag = on_board(TMAG5273)
try:
    mag.configure(channels="XYZ", averaging=32)
    mag.angle_pair = "XY"
    sleep_ms(10)
    bx, by, _ = mag.magnetic()
    offsets = mag.offsets_set(-2.5, 1.25)
    sleep_ms(10)
    ox, oy, _ = mag.magnetic()
    hw = mag.angle()
    sw = math.degrees(math.atan2(oy, ox)) % 360.0
    diff = min(abs(hw - sw), 360.0 - abs(hw - sw))
    print("offsets {} | shift X {:+.4f}, Y {:+.4f} mT | angle hw {:.2f}, atan2 {:.2f}".format(
        offsets, ox - bx, oy - by, hw, sw))  # fmt: skip
    check("offset shifts X by Equation 16", abs((ox - bx) + 2.5) < 0.3)
    check("offset shifts Y by Equation 16", abs((oy - by) - 1.25) < 0.3)
    check("test_offsets_and_angle", offsets == (-2.5, 1.25) and diff < 5.0,
          "hw {:.2f}, sw {:.2f}".format(hw, sw))  # fmt: skip
finally:
    restore(mag)

# %% [markdown]
# **Matches** `test_trigger_converts_in_standby`: in standby `SET_COUNT` stays put, then a trigger
# makes exactly one conversion.
#
# **What to do:** nothing.

# %%
mag = on_board(TMAG5273)
try:
    mag.configure(channels="XYZ", mode="standby")
    first = mag.conv_status()["set_count"]
    sleep_ms(20)
    still = mag.conv_status()["set_count"]
    mag.trigger()
    sleep_ms(2)
    after = mag.conv_status()
    print("SET_COUNT", first, "->", still, "-> (trigger) ->", after["set_count"])
    check("test_trigger_converts_in_standby", still == first
          and after["set_count"] == (first + 1) % 8 and after["ready"])  # fmt: skip
finally:
    restore(mag)

# %% [markdown]
# **Matches** `test_sleep_and_wake`: after entering sleep the first transfer is not acknowledged;
# after `wake()` the part answers and is in standby.
#
# **What to do:** nothing. An `OSError` printed inside a `PASS` line is the expected NAK.

# %%
mag = on_board(TMAG5273)
try:
    mag.operating_mode = "sleep"
    sleep_ms(1)
    check_raises("first transfer after sleep not acknowledged", OSError, mag.device_status)
    mag.operating_mode = "sleep"
    sleep_ms(1)
    mag.wake()
    mode = mag.operating_mode
    print("after wake():", mode, mag.device_status())
    check("test_sleep_and_wake", mode == "standby")
finally:
    restore(mag)

# %% [markdown]
# **Matches** `test_status_clear_clears_por`.
#
# **What to do:** nothing.

# %%
mag = on_board(TMAG5273)
try:
    mag.status_clear()
    st, dev = mag.conv_status(), mag.device_status()
    print("CONV_STATUS", st, "\nDEVICE_STATUS", dev)
    check("test_status_clear_clears_por",
          not st["por"] and not any(v for k, v in dev.items() if k.endswith("error")))  # fmt: skip
finally:
    restore(mag)

# %% [markdown]
# **Matches** `test_reset_config_restores_por`: from a fast read mode, every config register goes
# back to 0 and the address register is untouched.
#
# **What to do:** nothing.

# %%
mag = on_board(TMAG5273)
try:
    mag.configure(channels="XY", temperature=True, xy_range=80.0)
    mag.read_mode = "fast8"
    ranges = mag.reset_config()
    values = [mag.reg_read(r) for r in mag.CONFIG_REGS]
    print("config registers:", [hex(v) for v in values])
    check("test_reset_config_restores_por",
          ranges == (40.0, 40.0) and not any(values) and mag.read_mode == "standard"
          and mag.reg_read(mag.REG_I2C_ADDRESS) >> 1 == TMAG_ADDR)  # fmt: skip
finally:
    restore(mag)

# %% [markdown]
# **Matches** `test_reset_flag_recovers_a_part_left_in_fast_mode`: the part is left in fast16 by a
# raw write, then constructed. `reset=True` must recover it. Whether the plain constructor
# refuses depends on what the part answers to register reads in fast16, which no run has seen
# yet: the cell reports either outcome (the diagnostics below look closer).
#
# **What to do:** nothing.

# %%
regs = TMAG5273Regs(tmag_conn())
restore(regs)
try:
    regs.reg_write(regs.REG_DEVICE_CONFIG_1, 0x01)  # fast16, as an earlier session might leave it
    try:
        TMAG5273(tmag_conn())
        info("plain constructor worked in fast16: register reads still answer in that mode")
    except RuntimeError as e:
        print("plain constructor refused, as on the fake:", e)
    mag = TMAG5273(tmag_conn(), reset=True)
    check("test_reset_flag_recovers_a_part_left_in_fast_mode",
          mag.read_mode == "standard" and mag.identify() == (0x5449, 1))  # fmt: skip
finally:
    restore(regs)

# %% [markdown]
# ## Hardware-only checks
# What only the real part can show.

# %% [markdown]
# In continuous mode the part converts on its own: `SET_COUNT` advances between reads and
# `RESULT_STATUS` is set.
#
# **What to do:** nothing.

# %%
mag = on_board(TMAG5273)
try:
    mag.configure(channels="XYZ", temperature=True, averaging=4)
    sleep_ms(5)
    counts = []
    for _ in range(5):
        counts.append(mag.conv_status()["set_count"])
        sleep_ms(3)
    print("SET_COUNT over 15 ms:", counts)
    check("SET_COUNT advances in continuous mode", len(set(counts)) > 1)
    check("RESULT_STATUS set", mag.conv_status()["ready"])
finally:
    restore(mag)

# %% [markdown]
# The TMAG5273's die temperature against the TMP102 on channel 2 (same room, a few centimetres
# apart), from one triggered conversion in standby so self-heating is small. Rev C gives no
# accuracy, and this part read about +5.3 C high (2026-11-14), so the check only catches a
# decoding error (more than 10 C off). It also shows what rev A's slope (60.1 LSB/C) would read.
#
# **What to do:** keep your fingers off both sensors for a minute first.

# %%
mag = on_board(TMAG5273)
try:
    mag.configure(channels="X", temperature=True, averaging=32, mode="standby")
    mag.trigger()
    sleep_ms(5)
    code = mag.temperature_raw()
    t_mag = mag.celsius_from_raw(code)
    t_rev_a = 25.0 + (code - 17508) / 60.1
    t_tmp = TMP102(I2CConnection(bus, TMP_ADDR, TMP_PATH, bus_lock)).temperature()
    print(
        "TMAG5273 {:.2f} C (rev A slope: {:.2f} C) | TMP102 {:.2f} C".format(t_mag, t_rev_a, t_tmp)
    )
    info("offset {:+.2f} C against the TMP102 (this part: about +5.3 C)".format(t_mag - t_tmp))
    check("TMAG5273 temperature within 10 C of the TMP102", abs(t_mag - t_tmp) < 10.0,
          "{:+.2f} C apart".format(t_mag - t_tmp))  # fmt: skip
finally:
    restore(mag)

# %% [markdown]
# The signal path moves with a real field. The cell samples for 3 seconds.
#
# **What to do:** run the cell, then **wave a magnet** near the sensor for the 3 seconds. No
# magnet to hand? Skip this cell.

# %%
mag = on_board(TMAG5273)
try:
    mag.configure(channels="XYZ", averaging=4)
    sleep_ms(5)
    print("sampling for 3 s: wave the magnet now")
    seen = []
    for _ in range(60):
        x, y, z = mag.magnetic()
        seen.append((x * x + y * y + z * z) ** 0.5)
        sleep_ms(50)
    print("|B| from {:.3f} to {:.3f} mT".format(min(seen), max(seen)))
    check("field moved with the magnet", max(seen) - min(seen) > 1.0,
          "spread {:.3f} mT".format(max(seen) - min(seen)))  # fmt: skip
finally:
    restore(mag)

# %% [markdown]
# ## Diagnostics
# These print what the part does in cases the datasheet leaves open; the TODO cell at the top
# records what they showed on the bench. They do not count towards the tally. Rerun them on a
# new part or board.

# %% [markdown]
# ### What does a register read return in fast16?
# Measured: not the register (`00 90 00`), so the driver refuses register reads there. Here: the
# `DEVICE_ID`, manufacturer-ID bytes read in standard mode, then the same register read in
# fast16, and a plain fast16 packet for comparison.
#
# **What to do:** nothing. Read the verdict line.

# %%
regs = TMAG5273Regs(tmag_conn())
restore(regs)
try:
    regs.reg_set(regs.REG_SENSOR_CONFIG_1, 0x70)  # X, Y, Z
    regs.reg_set(regs.REG_DEVICE_CONFIG_2, 0x02)  # continuous
    sleep_ms(5)
    std = regs.read_mem(regs.REG_DEVICE_ID, 3)
    regs.reg_write(regs.REG_DEVICE_CONFIG_1, 0x01)  # fast16, raw
    via_reg = regs.read_mem(regs.REG_DEVICE_ID, 3)
    packet = regs.read_raw(7)
    hexes = lambda b: " ".join("{:02x}".format(v) for v in b)  # noqa: E731
    print("standard, reg 0x0D:", hexes(std))
    print("fast16,   reg 0x0D:", hexes(via_reg))
    print("fast16,   packet:  ", hexes(packet))
    if via_reg == std:
        print("verdict: register reads still work in fast16; the driver's guard can be relaxed")
    else:
        print("verdict: register reads return other data in fast16; the driver's guard is needed")
finally:
    restore(regs)

# %% [markdown]
# ### What does the standard-read CRC cover?
# With CRC on, a standard read sends a CRC as its fifth byte. Read the four constant bytes
# `0x0C..0x0F` plus that CRC, and see which candidate byte string the part's CRC matches.
#
# **What to do:** nothing.

# %%
regs = TMAG5273Regs(tmag_conn())
restore(regs)
try:
    regs.reg_set(regs.REG_DEVICE_CONFIG_1, 0x80)  # CRC on, standard reads
    raw = regs.read_mem(regs.REG_I2C_ADDRESS, 5)
    data, crc = raw[:4], raw[4]
    w = TMAG_ADDR << 1
    candidates = (
        ("the 4 data bytes", data),
        ("read command + data", bytes([w | 1]) + data),
        ("write command, register, read command, data", bytes([w, 0x0C, w | 1]) + data),
        ("register + data", bytes([0x0C]) + data),
    )
    print("data", " ".join("{:02x}".format(b) for b in data), "| CRC {:02x}".format(crc))
    for name, b in candidates:
        mark = "  <- matches" if regs.crc(b) == crc else ""
        print("  {:<46} {:02x}{}".format(name, regs.crc(b), mark))
finally:
    restore(regs)

# %% [markdown]
# ### Which bytes does the fast-read CRC cover?
# The part feeds **every address byte it hears** into its CRC and starts over after each of its
# own transfers (2026-11-14: every history below matched one rule 10/10, except an overheard
# TMP102 read, which matched none: the part hears more of that than the address bytes). This
# cell reads 10 packets for each bus history and counts which candidate byte string matched:
#
# - **channel held:** nothing else on the bus between reads
# - **via the mux:** one mux write first (what the connection does)
# - **two mux writes:** two mux writes first
# - **TMP102 read, heard:** channels 0 and 2 both on during a TMP102 read
# - **TMP102 read, not heard:** only channel 2 on during the TMP102 read
#
# Expected: channel held "read cmd only", via the mux "mux", two mux writes "mux, mux", heard
# none, not heard "mux".
#
# **What to do:** nothing.

# %%
mag = on_board(TMAG5273)
try:
    mag.configure(channels="XY", averaging=1)
    mag.crc_enabled = True
    mag.read_mode = "fast16"
    n = mag.fast_read_length()
    R = (TMAG_ADDR << 1) | 1  # 0x45: the part's read command
    M = MUX_ADDR << 1  # 0xe0: the mux's write address byte
    TW, TR = TMP_ADDR << 1, (TMP_ADDR << 1) | 1  # 0x90, 0x91: a TMP102 register read
    rules = (
        ("read cmd only", []),
        ("mux", [M]),
        ("mux, mux", [M, M]),
        ("mux, TMP102 w, TMP102 r, mux", [M, TW, TR, M]),
        ("mux, TMP102 w, TMP102 r", [M, TW, TR]),
    )

    def held():
        return bus.readfrom(TMAG_ADDR, n)

    def routed():
        return mag.read_raw(n)

    def two_mux_writes():
        mux.channel_select(0)
        return mag.read_raw(n)

    def tmp102_heard():
        mux.channel_mask_set(0b101)  # channels 0 and 2 on: the TMAG5273 hears the TMP102 read
        bus.readfrom_mem(TMP_ADDR, 0, 2)
        return mag.read_raw(n)

    def tmp102_unheard():
        mux.channel_select(2)  # only channel 2: the TMAG5273 is cut off during the TMP102 read
        bus.readfrom_mem(TMP_ADDR, 0, 2)
        return mag.read_raw(n)

    cases = (("channel held", held), ("via the mux", routed), ("two mux writes", two_mux_writes),
             ("TMP102 read, heard", tmp102_heard), ("TMP102 read, not heard", tmp102_unheard))  # fmt: skip
    mux.channel_select(0)
    held()  # so the first "channel held" read follows one of the part's own reads
    for name, read in cases:
        hits = {}
        for _ in range(10):
            raw = read()
            for rule, prefix in rules:
                if mag.crc(bytes(prefix + [R]) + raw[:-1]) == raw[-1]:
                    hits[rule] = hits.get(rule, 0) + 1
            sleep_ms(5)
        print("{:<24} {}".format(name, hits or "no rule matched"))
finally:
    mux.channel_deselect()
    restore(mag)

# %% [markdown]
# ### What does fast16 send for the two-axis and pseudo-simultaneous codes?
# Measured 2026-11-14: each axis once, in name order. For each `MAG_CH_EN` code that is not plain X-then-Y-then-Z
# (`ZX`, `YZ`, and the four pseudo-simultaneous ones), this prints the X, Y, Z result registers
# (standard read) next to an 8-byte fast16 packet (no CRC), read right after each other. With a
# static field, matching words show the order. `XYZ` comes first as a reference.
#
# **What to do:** keep magnets away and the sensor still.

# %%
regs = TMAG5273Regs(tmag_conn())
restore(regs)
hexes = lambda b: " ".join("{:02x}".format(v) for v in b)  # noqa: E731
try:
    regs.reg_set(regs.REG_DEVICE_CONFIG_1, 0x14)  # 32x averaging, standard reads
    regs.reg_set(regs.REG_DEVICE_CONFIG_2, 0x02)  # continuous
    for code in (0x7, 0x5, 0x6, 0x8, 0x9, 0xA, 0xB):
        regs.reg_set(regs.REG_SENSOR_CONFIG_1, code << 4)
        sleep_ms(20)
        res = regs.read_mem(regs.REG_X_MSB_RESULT, 6)
        regs.reg_write(regs.REG_DEVICE_CONFIG_1, 0x15)  # fast16 (raw: no read-back in fast mode)
        pkt = regs.read_raw(8)
        regs.reg_write(regs.REG_DEVICE_CONFIG_1, 0x14)  # standard again
        regs.cache_clear()
        print("{:#x} {:<4} registers X {} Y {} Z {} | fast16: {}".format(
            code, TMAG5273Regs.MAG_CH[code], hexes(res[0:2]), hexes(res[2:4]), hexes(res[4:6]),
            hexes(pkt)))  # fmt: skip
finally:
    restore(regs)

# %% [markdown]
# ### Is the temperature offset self-heating?
# Self-heating would shrink after 30 s in standby (no conversions) and grow back in continuous
# mode; a fixed offset stays put. Measured 2026-11-14: +5.3 C flat in standby, then +1.2 C over
# 30 s of continuous 32x conversions. Printed every 10 s.
#
# **What to do:** keep your fingers off both sensors. Takes about a minute.

# %%
mag = on_board(TMAG5273)
tmp = TMP102(I2CConnection(bus, TMP_ADDR, TMP_PATH, bus_lock))


def both(label):
    print("{:<22} TMAG5273 {:6.2f} C | TMP102 {:6.2f} C".format(label, mag.temperature(),
                                                              tmp.temperature()))  # fmt: skip


try:
    mag.configure(channels="X", temperature=True, averaging=32, mode="standby")
    for s in (0, 10, 20, 30):  # standby: no conversions, the die cools
        if s:
            sleep_ms(10_000)
        mag.trigger()
        sleep_ms(5)
        both("standby +{:>2} s".format(s))
    mag.operating_mode = "continuous"
    for s in (0, 10, 20, 30):  # converting all the time
        if s:
            sleep_ms(10_000)
        both("continuous +{:>2} s".format(s))
finally:
    restore(mag)

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
