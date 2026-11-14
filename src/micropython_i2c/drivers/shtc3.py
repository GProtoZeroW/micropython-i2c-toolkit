"""Sensirion SHTC3 humidity and temperature sensor (command-driven, CRC per word).

Usage on the board::

    from micropython_i2c import I2CConnection
    from micropython_i2c.drivers.shtc3 import SHTC3

    sht = SHTC3(I2CConnection(bus, 0x70, lock=bus_lock))
    t_c, rh = sht.measure()

The part sleeps between measurements: every operation wakes it, runs, and puts it back
to sleep. Every word read is CRC-checked (``CommandI2CBase`` defaults are the Sensirion
parameters, so nothing needs overriding). Note 0x70 is also the TCA9548A's default
address; strap the mux elsewhere when both share a segment.
"""

from micropython_i2c.compat import sleep_ms, sleep_us
from micropython_i2c.node import I2CDevice
from micropython_i2c.protocol import CommandI2CBase


class SHTC3(CommandI2CBase, I2CDevice):
    """SHTC3 driver: ``read_id()``, ``measure()`` and ``identify()``."""

    ALLOWED_ADDRS = (0x70,)
    BAUD_NOMINAL = 400_000
    BAUD_MAX = 1_000_000  # Fast-mode Plus
    BAUD_MIN = 10_000

    CMD_WAKEUP = 0x3517
    CMD_SLEEP = 0xB098
    CMD_SOFT_RESET = 0x805D
    CMD_READ_ID = 0xEFC8
    CMD_MEASURE_T_FIRST = 0x7866  # normal mode, no clock stretching
    CMD_MEASURE_T_FIRST_LP = 0x609C  # low-power mode, no clock stretching

    T_WAKEUP_US = 240  # max wake-up time
    T_MEASURE_MS = 13  # normal mode max is 12.1 ms
    T_MEASURE_LP_MS = 1  # low-power mode max is 0.8 ms

    # ID register: bits 11 and 5..0 identify the SHTC3 (datasheet section 5.9).
    ID_MASK = 0x083F
    ID_SHTC3 = 0x0807

    def _awake(self, command, delay_us, n_words):
        """Wake, send ``command``, wait, read ``n_words`` (CRC-checked), sleep."""
        self.cmd_send(self.CMD_WAKEUP)
        sleep_us(self.T_WAKEUP_US)
        try:
            self.cmd_send(command)
            sleep_us(delay_us)
            return self.words_read(n_words)
        finally:
            self.cmd_send(self.CMD_SLEEP)

    def read_id(self):
        """The 16-bit ID register."""
        return self._awake(self.CMD_READ_ID, 1000, 1)[0]

    def identify(self):
        """True if the ID register matches the SHTC3 pattern."""
        return self.read_id() & self.ID_MASK == self.ID_SHTC3

    @staticmethod
    def celsius_from_raw(word):
        """Temperature word -> C."""
        return -45 + 175 * word / 65536

    @staticmethod
    def humidity_from_raw(word):
        """Humidity word -> %RH."""
        return 100 * word / 65536

    def measure(self, low_power=False):
        """Return ``(temperature_C, humidity_pct)`` from one measurement.

        Raises:
            ValueError: On a CRC mismatch in either word.
        """
        if low_power:
            cmd, wait_ms = self.CMD_MEASURE_T_FIRST_LP, self.T_MEASURE_LP_MS
        else:
            cmd, wait_ms = self.CMD_MEASURE_T_FIRST, self.T_MEASURE_MS
        t_raw, rh_raw = self._awake(cmd, wait_ms * 1000, 2)
        return self.celsius_from_raw(t_raw), self.humidity_from_raw(rh_raw)

    def soft_reset(self):
        """Software reset (the part must be awake)."""
        self.cmd_send(self.CMD_WAKEUP)
        sleep_us(self.T_WAKEUP_US)
        self.cmd_send(self.CMD_SOFT_RESET)
        sleep_ms(1)
