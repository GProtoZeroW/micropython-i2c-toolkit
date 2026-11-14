# ST VL53L4CD time-of-flight distance sensor, on the micropython_i2c connection layer.
#
# Ported from micropylab's examples/vl53l4cd.py, itself a PORT of
# adafruit-circuitpython-vl53l4cd 1.3.6: the 0x2D..0x87 init table and the ranging
# logic come from that library.
# SPDX-FileCopyrightText: 2017 Scott Shawcroft, written for Adafruit Industries
# SPDX-FileCopyrightText: Copyright (c) 2022 Carter Nelson for Adafruit Industries
# SPDX-License-Identifier: MIT
#
# The MIT License (MIT)
# Copyright (c) 2022 Carter Nelson for Adafruit Industries
# Permission is hereby granted, free of charge, to any person obtaining a copy of this
# software and associated documentation files (the "Software"), to deal in the Software
# without restriction, including without limitation the rights to use, copy, modify,
# merge, publish, distribute, sublicense, and/or sell copies of the Software, and to
# permit persons to whom the Software is furnished to do so, subject to the following
# conditions: The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software. THE SOFTWARE IS PROVIDED "AS IS",
# WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
# WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
# IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR
# OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
"""ST VL53L4CD time-of-flight sensor in continuous ranging mode.

Usage on the board::

    from micropython_i2c import I2CConnection
    from micropython_i2c.drivers.vl53l4cd import VL53L4CD

    tof = VL53L4CD(I2CConnection(bus, 0x29, lock=bus_lock))
    tof.start()
    mm = tof.read()                 # always check tof.valid afterwards
    print(mm, "mm", tof.status_name)
    tof.stop()

The part has a 16-bit register map (``REG_ADDR_BITS = 16``). Only a reading with
``valid`` True can be trusted; open air past about 1.3 m mostly gives "sigma warning" or
"invalid phase" with a 0 or ~2000 mm value.
"""

from micropython_i2c.compat import sleep_ms, ticks_diff, ticks_ms
from micropython_i2c.node import I2CDevice
from micropython_i2c.protocol import RegisterI2CBase

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
