"""In-memory stand-ins for `machine.I2C` and parts, for host-side tests and dev notebooks.

``FakeBus`` models mux routing: a node attached behind ``[(mux_addr, ch), ...]`` only
answers while every mux on that path has the channel enabled, the way real silicon does.
"""

import math

from micropython_i2c.bits import BitTools
from micropython_i2c.compat import ticks_add, ticks_diff, ticks_ms


class FakeRegisterPart:
    """A register-mapped part with an auto-incrementing 8-bit register file.

    ``read_only`` maps register -> mask of bits that keep their value on write.
    """

    def __init__(self, regs=None, read_only=None, size=256):
        self.regs = bytearray(size)
        for reg, value in (regs or {}).items():
            self.regs[reg] = value
        self.read_only = read_only or {}

    def mem_read(self, reg, nbytes):
        return bytes(self.regs[(reg + i) % len(self.regs)] for i in range(nbytes))

    def mem_write(self, reg, data):
        for i, b in enumerate(data):
            r = (reg + i) % len(self.regs)
            keep = self.read_only.get(r, 0)
            self.regs[r] = (self.regs[r] & keep) | (b & ~keep & 0xFF)

    def raw_read(self, nbytes):
        return self.mem_read(0, nbytes)

    def raw_write(self, data):
        pass


class FakeTMP102:
    """16-bit pointer-register part behaving like a TMP102, quirks included.

    Two behaviours measured on a real part (``drivers_tmp102_devNB`` diagnostics, 2026-11-13):

    * After EM changes, D0 of the temperature word follows EM at once, but the data keeps the
      old 12/13-bit format until the next conversion (about 60 ms later at 4 Hz). In shutdown
      it keeps the old format until a one-shot.
    * OS reads 1 in shutdown (the last conversion is done) and 0 in continuous mode.

    The temperature is held as a count of 1/16 C, so it is the same in both formats.
    """

    LAG_MS = 60  # measured delay before the data takes the new format

    def __init__(self, temp_word=0x1900):  # 25 C, as a 12-bit wire word
        self.counts = BitTools.sign_extend(temp_word >> 4, 12)
        self.regs = {1: 0x60A0, 2: 0x4B00, 3: 0x5000}
        self.fmt_em = False  # the format the stored temperature word is in
        self.catch_up = None  # ticks when the format catches up with EM (None: not pending)

    def _em(self):
        return bool(self.regs[1] & 0x0010)

    def _temp_word(self):
        if self.catch_up is not None and ticks_diff(ticks_ms(), self.catch_up) >= 0:
            self.fmt_em, self.catch_up = self._em(), None
        shift, bits = (3, 13) if self.fmt_em else (4, 12)
        word = (self.counts & ((1 << bits) - 1)) << shift
        return word | (1 if self._em() else 0)  # D0 is the EM indicator, set at once

    def mem_read(self, reg, nbytes):
        word = self._temp_word() if reg & 3 == 0 else self.regs[reg & 3]
        return word.to_bytes(2, "big")[:nbytes]

    def mem_write(self, reg, data):
        reg &= 3
        value = int.from_bytes(data, "big")
        if reg == 0:
            return  # read-only
        if reg != 1:
            self.regs[reg] = value
            return
        old = self.regs[1]
        one_shot = bool(value & 0x8000) and bool(value & 0x0100)
        # R1/R0 always read 11, AL is read-only, the low nibble is reserved, OS reads SD.
        value = (value & ~0xE020 & 0xFFF0) | 0x6000 | (old & 0x0020)
        value |= 0x8000 if value & 0x0100 else 0
        self.regs[1] = value
        if one_shot:  # a conversion now, in the current format
            self.fmt_em, self.catch_up = self._em(), None
        elif (value ^ old) & 0x0010 and not value & 0x0100:  # EM changed while converting
            self.catch_up = ticks_add(ticks_ms(), self.LAG_MS)


class FakeMux:
    """TCA9548A-style mux: one control byte, one bit per channel."""

    def __init__(self):
        self.state = 0

    def raw_write(self, data):
        self.state = data[-1]

    def raw_read(self, nbytes):
        return bytes([self.state] * nbytes)


class FakeSensirion:
    """Command part returning canned words, each followed by a Sensirion CRC."""

    def __init__(self, words, crc_fn):
        self.words = words
        self.crc_fn = crc_fn
        self.commands = []

    def raw_write(self, data):
        self.commands.append(int.from_bytes(data, "big"))

    def raw_read(self, nbytes):
        out = b""
        for w in self.words:
            pair = w.to_bytes(2, "big")
            out += pair + bytes([self.crc_fn(pair)])
        return out[:nbytes]


class FakeBus:
    """Stands in for ``machine.I2C``. Attach parts with ``attach(addr, part, path)``."""

    def __init__(self):
        self.nodes = []  # (path tuple, addr, part)
        self.log = []  # ("w"|"r", addr, payload)

    def attach(self, addr, part, path=()):
        self.nodes.append((tuple(tuple(h) for h in path), addr, part))
        return part

    def _mux_at(self, path, addr):
        for p, a, part in self.nodes:
            if p == path and a == addr:
                return part
        raise LookupError("no mux at {:#04x} on {}".format(addr, path))

    def _visible(self, path):
        for i, (maddr, ch) in enumerate(path):
            if not self._visible(path[:i]):
                return False
            if not self._mux_at(path[:i], maddr).state & (1 << ch):
                return False
        return True

    def _targets(self, addr):
        hits = [part for p, a, part in self.nodes if a == addr and self._visible(p)]
        if not hits:
            raise OSError(19, "ENODEV")
        return hits

    def _target(self, addr):
        """The one part a read talks to. Two parts at one address answering a read is a
        collision (their data would be mixed); a write simply reaches both."""
        hits = self._targets(addr)
        if len(hits) > 1:
            raise RuntimeError("bus collision at {:#04x}".format(addr))
        return hits[0]

    def scan(self):
        return sorted({a for p, a, _ in self.nodes if self._visible(p)})

    def _hear(self, addr, read):
        """Give the address byte to every part that can hear the bus and has a ``hear`` hook
        (visibility before the transfer: a mux switches only after its own address byte)."""
        byte = (addr << 1) | (1 if read else 0)
        for path, _, part in self.nodes:
            if hasattr(part, "hear") and self._visible(path):
                part.hear(byte)

    def writeto(self, addr, data):
        self.log.append(("w", addr, bytes(data)))
        self._hear(addr, False)
        for part in self._targets(addr):  # every part at the address receives a write
            part.raw_write(bytes(data))
        return len(data)

    def readfrom(self, addr, nbytes):
        self.log.append(("r", addr, nbytes))
        self._hear(addr, True)
        return self._target(addr).raw_read(nbytes)

    def readfrom_into(self, addr, buf):
        buf[:] = self.readfrom(addr, len(buf))

    def writeto_mem(self, addr, reg, data, addrsize=8):
        self.log.append(("wm", addr, reg, bytes(data)))
        self._hear(addr, False)
        self._target(addr).mem_write(reg, bytes(data))

    def readfrom_mem(self, addr, reg, nbytes, addrsize=8):
        self.log.append(("rm", addr, reg, nbytes))
        self._hear(addr, False)
        self._hear(addr, True)  # the repeated start
        return self._target(addr).mem_read(reg, nbytes)

    def readfrom_mem_into(self, addr, reg, buf, addrsize=8):
        buf[:] = self.readfrom_mem(addr, reg, len(buf), addrsize)


def _crc8_07(data, crc=0xFF):
    """CRC-8, polynomial 0x07, seeded 0xFF (the TMAG5273's)."""
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = ((crc << 1) ^ 0x07) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc


class FakeTMAG5273:
    """A TMAG5273 register file and fast-read stream, close enough to test a driver on the host.

    Modelled from TI SLYS045C (rev C, April 2026):

    * ID and result registers are read-only, the reserved bits read 0, and the status bits
      (``POR``, the four ``DEVICE_STATUS`` errors) are write-1-to-clear. ``POR`` starts set.
    * Bit 7 of the register address is the conversion trigger.
    * Continuous mode converts before every transfer; standby converts on a trigger. A
      conversion fills the enabled result registers, rolls ``SET_COUNT`` and sets
      ``RESULT_STATUS``. The angle and the offset correction apply to the ``ANGLE_EN`` pair.
    * Fast reads (``I2C_RD`` 1 or 2) stream T (if enabled), then each ``MAG_CH_EN`` axis
      once, in the order of the setting's name (``ZX``: Z, X; ``XZX``: X, Z), then
      ``CONV_STATUS``, then the CRC.
    * The fast-read CRC covers every address byte the part heard on the bus since its own last
      transfer (``FakeBus`` passes them to ``hear``), then the data. Behind a mux that
      includes the mux's address byte. The real part also hears more of other devices'
      transfers than their address bytes; the fake does not model that.
    * With ``CRC_EN`` set, a standard read longer than four bytes gets a CRC over the four data
      bytes as its fifth byte.
    * In a fast read mode a register read does not return the register (the real part sent
      ``00 90 00`` for ``DEVICE_ID``); the fake returns the packet.
    * In sleep or wake-and-sleep mode the first transfer wakes the part but is not
      acknowledged (``OSError``); the part is then in standby.

    The CRC, packet-order and fast-mode behaviour was measured on a SparkFun B1
    (``drivers_tmag5273_devNB`` diagnostics, 2026-11-13/14).

    The field is ``field_mT`` (X, Y, Z) and the die temperature ``temp_c``; the result words
    follow the selected range, as on the real part. Set ``corrupt`` to the number of upcoming
    fast-read packets to flip one bit in.
    """

    T_ADC_T0 = 17508
    T_ADC_RES = 58.0  # LSB per C, rev C (rev A said 60.1)
    RANGES = {1: (40.0, 80.0), 2: (133.0, 266.0)}  # per DEVICE_ID.VER
    SEQ = {0x1: "X", 0x2: "Y", 0x3: "XY", 0x4: "Z", 0x5: "ZX", 0x6: "YZ", 0x7: "XYZ",
           0x8: "XYX", 0x9: "YXY", 0xA: "YZY", 0xB: "XZX"}  # fmt: skip
    PAIRS = {1: (0, 1), 2: (1, 2), 3: (0, 2)}  # ANGLE_EN -> axis indices
    WRITABLE = {0x03: 0x7F, 0x08: 0xFD}  # registers with reserved bits; others 0x00-0x0C all
    W1C = {0x18: 0x10, 0x1C: 0x0F}

    def __init__(self, addr=0x35, field_mT=(5.0, -5.0, 0.0), temp_c=25.0, version=1):
        self.addr = addr
        self.field_mT = list(field_mT)
        self.temp_c = temp_c
        self.version = version
        self.regs = bytearray(0x1D)
        self.regs[0x0C] = addr << 1
        self.regs[0x0D] = version
        self.regs[0x0E], self.regs[0x0F] = 0x49, 0x54  # "TI", LSB first
        self.regs[0x18] = 0x10  # POR
        self.regs[0x1C] = 0x10  # INTB_RB: the INT line reads high
        self.pointer = 0
        self.asleep = False
        self.corrupt = 0
        self.heard = bytearray()  # address bytes heard since this part's last transfer
        self.conversions = 0

    # Helpers.

    def _field(self, reg, shift, width):
        return (self.regs[reg] >> shift) & ((1 << width) - 1)

    def _range(self, axis):
        bit = self._field(0x03, 0, 1) if axis == 2 else self._field(0x03, 1, 1)
        return self.RANGES[self.version][bit]

    def _wake_check(self):
        if self.asleep:
            self.asleep = False
            self.regs[0x01] &= ~0x03  # woken into standby
            raise OSError(19, "ENODEV")  # the waking address is not acknowledged

    def _begin(self):
        self._wake_check()
        if self._field(0x01, 0, 2) == 2:  # continuous: a fresh conversion every access
            self._convert()

    def _convert(self):
        self.conversions += 1
        field = list(self.field_mT)
        pair = self.PAIRS.get(self._field(0x03, 2, 2))
        if pair:
            for reg, ax in zip((0x0A, 0x0B), pair):
                off = BitTools.sign_extend(self.regs[reg], 8)
                field[ax] += off / 4096 * 2 * self._range(ax)
        axes = set(self.SEQ.get(self._field(0x02, 4, 4), ""))
        for i, name in enumerate("XYZ"):
            if name in axes:
                fs = self._range(i)
                counts = int(round(field[i] * 65536 / (2 * fs)))
                counts = max(-32768, min(32767, counts)) & 0xFFFF
                self.regs[0x12 + 2 * i], self.regs[0x13 + 2 * i] = counts >> 8, counts & 0xFF
        if self.regs[0x07] & 1:
            code = int(self.T_ADC_T0 + (self.temp_c - 25.0) * self.T_ADC_RES + 0.5)
            self.regs[0x10], self.regs[0x11] = code >> 8, code & 0xFF
        if pair:
            deg = math.degrees(math.atan2(field[pair[1]], field[pair[0]])) % 360.0
            word = (int(deg) << 4) | int((deg - int(deg)) * 16)
            self.regs[0x19], self.regs[0x1A] = word >> 8, word & 0xFF
        count = (self.regs[0x18] >> 5) + 1
        self.regs[0x18] = ((count & 7) << 5) | (self.regs[0x18] & 0x10) | 0x01

    def _streamed(self):
        out = "T" if self.regs[0x07] & 1 else ""
        for axis in self.SEQ.get(self._field(0x02, 4, 4), ""):
            if axis not in out:
                out += axis
        return out

    def _packet(self):
        mode = self._field(0x00, 0, 2)
        names = self._streamed()
        body = bytearray()
        for name in names:
            reg = 0x10 if name == "T" else 0x12 + 2 * "XYZ".index(name)
            body += self.regs[reg : reg + (2 if mode == 1 else 1)]
        body.append(self.regs[0x18])
        if self.regs[0x00] & 0x80:
            body.append(_crc8_07(bytes(self.heard) + body))  # heard ends with the read command
        if self.corrupt:
            body[0] ^= 0x01
            self.corrupt -= 1
        return bytes(body)

    def _read(self, reg, nbytes):
        if self._field(0x00, 0, 2):  # fast mode: any read returns the packet (assumed)
            return self._packet()[:nbytes]
        data = bytes(self.regs[(reg + i) % len(self.regs)] for i in range(nbytes))
        if self.regs[0x00] & 0x80 and nbytes > 4:
            data = data[:4] + bytes([_crc8_07(data[:4])]) + b"\xff" * (nbytes - 5)
        return data

    def _write(self, reg, data):
        for i, b in enumerate(data):
            r = reg + i
            if r in self.W1C:
                self.regs[r] &= ~(b & self.W1C[r]) & 0xFF
            elif r <= 0x0C:
                self.regs[r] = b & self.WRITABLE.get(r, 0xFF)
        if self._field(0x01, 0, 2) in (1, 3):  # sleep, wake-and-sleep
            self.asleep = True

    def _trigger(self, reg):
        if reg & 0x80 and self._field(0x01, 0, 2) == 0:
            self._convert()
        return reg & 0x7F

    # The bus interface. Each of the part's own transfers starts its CRC over afterwards.

    def hear(self, byte):
        self.heard.append(byte)

    def mem_read(self, reg, nbytes):
        self._begin()
        try:
            return self._read(self._trigger(reg), nbytes)
        finally:
            self.heard = bytearray()

    def mem_write(self, reg, data):
        self._begin()
        self._write(self._trigger(reg), data)
        self.heard = bytearray()

    def raw_read(self, nbytes):
        self._begin()
        try:
            return self._read(self.pointer, nbytes)
        finally:
            self.heard = bytearray()

    def raw_write(self, data):
        self._begin()
        self.pointer = self._trigger(data[0])
        if len(data) > 1:
            self._write(self.pointer, data[1:])
        self.heard = bytearray()


class FakeRelay:
    """SparkFun Qwiic Single Relay: one-byte commands, status read as a command byte."""

    def __init__(self, version=0x01):
        self.closed = False
        self.version = version

    def raw_write(self, data):
        if data[0] in (0x00, 0x01):
            self.closed = bool(data[0])

    def mem_read(self, reg, nbytes):
        return bytes([{0x05: int(self.closed), 0x04: self.version}[reg]]) * nbytes


def fake_vl53l4cd(distance_mm=123, status=9, osc=0x0BB8):
    """A VL53L4CD register file that has booted, always has a sample ready."""
    regs = {
        0x010F: 0xEB, 0x0110: 0xAA,  # model ID
        0x00E5: 0x03,  # firmware booted
        0x0030: 0x10,  # interrupt polarity bit set -> ready when line reads 0
        0x0031: 0x00,
        0x0006: osc >> 8, 0x0007: osc & 0xFF,
        0x0089: status,
        0x0096: distance_mm >> 8, 0x0097: distance_mm & 0xFF,
    }  # fmt: skip
    return FakeRegisterPart(regs, size=0x10000)
