"""SparkFun Qwiic Single Relay (ATtiny firmware), command-per-byte protocol.

Usage on the board::

    from micropython_i2c import I2CConnection
    from micropython_i2c.devices.drivers.qwiic_relay import QwiicRelay

    relay = QwiicRelay(I2CConnection(bus, 0x18, lock=bus_lock))
    relay.on()
    print(relay.state)              # True
    relay.off()

The firmware takes one-byte commands. State and version are read the way SparkFun's
libraries do it: write the command byte, then read one byte back (an SMBus "read byte"
with the command as the register number). Command values follow SparkFun's
Qwiic_Relay library for the single-relay board.
"""

from micropython_i2c.node import I2CDevice
from micropython_i2c.protocol import RegisterI2CBase


class QwiicRelay(RegisterI2CBase, I2CDevice):
    """SparkFun Qwiic Single Relay: ``on()``, ``off()``, ``toggle()``, ``state``."""

    # 0x18 by default, 0x19 with the ADR jumper closed. The firmware can also be given
    # another address by command; widen this tuple if you do that.
    ALLOWED_ADDRS = (0x18, 0x19)
    BAUD_NOMINAL = 100_000
    BAUD_MAX = 400_000
    BAUD_MIN = 10_000

    REG_BYTES = 1

    CMD_OFF = 0x00
    CMD_ON = 0x01
    CMD_FIRMWARE_VERSION = 0x04
    CMD_STATUS = 0x05

    def on(self):
        """Close the relay."""
        self.write_raw(bytes([self.CMD_ON]))

    def off(self):
        """Open the relay."""
        self.write_raw(bytes([self.CMD_OFF]))

    @property
    def state(self):
        """True if the relay is closed. Always read fresh from the board."""
        return bool(self.reg_read(self.CMD_STATUS))

    @state.setter
    def state(self, closed):
        if closed:
            self.on()
        else:
            self.off()

    def toggle(self):
        """Flip the relay. Returns the new state as read back."""
        self.state = not self.state
        return self.state

    def firmware_version(self):
        """The firmware version byte."""
        return self.reg_read(self.CMD_FIRMWARE_VERSION)
