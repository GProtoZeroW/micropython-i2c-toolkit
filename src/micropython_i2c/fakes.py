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

    Two behaviours measured on a real part (``tmp102_devNB`` diagnostics, 2026-11-13):

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


def _crc8_31(data, crc=0xFF):
    """CRC-8, polynomial 0x31, seeded 0xFF (Sensirion's)."""
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = ((crc << 1) ^ 0x31) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc


class FakeSHTC3:
    """A Sensirion SHTC3, close enough to test a driver on the host (datasheet version 4).

    * It starts **idle** (awake), as after power-up. In sleep it acknowledges only the wake-up
      command; anything else raises ``OSError`` (a NACK), and for ``REFUSE_MS`` after that it
      NACKs wake-up too.
    * A measurement command starts a measurement of ``MEAS_MS`` (11 ms normal, 0.5 ms low
      power). Until it ends every transfer is NACKed, except a read after a clock-stretching
      command, which waits for the result.
    * The read returns two words, each followed by its CRC: temperature first or humidity
      first, as the command says. The result is gone after one read; a read with nothing
      waiting is NACKed.
    * ``CMD_READ_ID`` makes the next read return ``id_word``. Soft reset returns to idle. A
      wake-up to a part that is already awake is acknowledged.

    All of this was measured on a SparkFun SHTC3 (``shtc3_devNB`` diagnostics,
    2026-11-14); the real refusal after a NACK lasts under 1 ms, the fake's ``REFUSE_MS`` is
    longer so a test can see it at ms resolution. Set ``corrupt`` to the number of upcoming
    reads to flip one bit in.
    """

    WAKEUP, SLEEP, SOFT_RESET, READ_ID = 0x3517, 0xB098, 0x805D, 0xEFC8
    # opcode -> (low_power, rh_first, clock_stretch), Table 11
    MEASURE = {
        0x7CA2: (False, False, True), 0x5C24: (False, True, True),
        0x7866: (False, False, False), 0x58E0: (False, True, False),
        0x6458: (True, False, True), 0x44DE: (True, True, True),
        0x609C: (True, False, False), 0x401A: (True, True, False),
    }  # fmt: skip
    MEAS_MS = {False: 11.0, True: 0.5}  # measured; the datasheet maximums are 12.1 and 0.8
    REFUSE_MS = 2

    def __init__(self, temp_c=25.0, rh=50.0, id_word=0x0807):
        self.temp_c = temp_c
        self.rh = rh
        self.id_word = id_word
        self.asleep = False
        self.pending = None  # words the next read returns
        self.stretch = False
        self.busy_until = None  # ticks_ms when the measurement ends
        self.refuse_until = None  # ticks_ms until which wake-up is NACKed
        self.corrupt = 0
        self.commands = []

    def _busy(self):
        if self.busy_until is not None and ticks_diff(self.busy_until, ticks_ms()) > 0:
            return True
        self.busy_until = None
        return False

    def _nack(self):
        raise OSError(19, "ENODEV")

    def raw_write(self, data):
        cmd = int.from_bytes(bytes(data), "big")
        if self.asleep and cmd != self.WAKEUP:
            self.refuse_until = ticks_add(ticks_ms(), self.REFUSE_MS)
            self._nack()
        if self.refuse_until is not None:
            if ticks_diff(self.refuse_until, ticks_ms()) > 0:
                self._nack()
            self.refuse_until = None
        if self._busy():
            self._nack()
        self.commands.append(cmd)
        if cmd == self.WAKEUP:
            self.asleep = False
        elif cmd == self.SLEEP:
            self.asleep = True
        elif cmd == self.SOFT_RESET:
            self.pending = None
        elif cmd == self.READ_ID:
            self.pending, self.stretch = [self.id_word], False
        elif cmd in self.MEASURE:
            low_power, rh_first, self.stretch = self.MEASURE[cmd]
            t = max(0, min(0xFFFF, int(round((self.temp_c + 45) * 65536 / 175))))
            h = max(0, min(0xFFFF, int(round(self.rh * 65536 / 100))))
            self.pending = [h, t] if rh_first else [t, h]
            self.busy_until = ticks_add(ticks_ms(), int(self.MEAS_MS[low_power]))  # ms ticks
        else:
            self._nack()  # not a command

    def raw_read(self, nbytes):
        if self.asleep:
            self._nack()
        if self._busy():
            if not self.stretch:
                self._nack()
            while self._busy():  # clock stretching: SCL held low until the result is ready
                pass
        if self.pending is None:
            self._nack()  # nothing to read (assumed)
        out = bytearray()
        for w in self.pending:
            pair = bytes([w >> 8, w & 0xFF])
            out += pair + bytes([_crc8_31(pair)])
        self.pending = None
        if self.corrupt:
            out[0] ^= 0x01
            self.corrupt -= 1
        return bytes(out[:nbytes])


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

    @staticmethod
    def _addr_of(addr, part):
        """The address a part answers at: its ``bus_addr`` if it has one (a part with a
        programmable address), else the one it was attached at."""
        return getattr(part, "bus_addr", addr)

    def _targets(self, addr):
        hits = [
            part for p, a, part in self.nodes if self._addr_of(a, part) == addr and self._visible(p)
        ]
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
        return sorted({self._addr_of(a, part) for p, a, part in self.nodes if self._visible(p)})

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
    (``tmag5273_devNB`` diagnostics, 2026-11-13/14).

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


class FakeVL53L1X:
    """An ST VL53L1X register file and ranging sequencer, close enough to test a driver on the
    host. It does not model the optics: the distance and the result fields are attributes.

    Modelled from the datasheet (DS12385 rev 8) and ST's ULD driver source:

    * A 16-bit register map. ``0x010F..0x0111`` read ``EA CC 10`` (datasheet Table 8),
      ``0x00E5`` bit 0 is "booted", ``0x00DE`` holds the oscillator calibration word. The ID,
      status, oscillator and result registers ignore writes.
    * ``0x0087`` (mode start): ``0x40`` ranges continuously, ``0x10`` once, ``0x00`` stops.
      A sample is ready ``sample_ms`` after the start or after the interrupt is cleared
      (``0x0086`` bit 0); a new sample is only made after a clear, as the datasheet says.
    * The interrupt line is ``0x0031`` bit 0; ``0x0030`` bit 4 set means active low.
    * A sample fills the 17-byte result block at ``0x0089``: raw status, SPAD count
      (``0x008C``, 8.8), ambient and signal rates (``/8``, kcps) and the distance
      (``0x0096``), which includes the part-to-part offset in ``0x001E`` (11.2, signed).
    * ``0x0001`` is the 7-bit address; the part answers there at once (``bus_addr``).
    * A soft reset: ``0x0000`` written 0 holds the part in reset, where every register reads 0
      (measured: an interrupted reset left a real part answering ``00`` everywhere); written 1,
      it restores the power-on registers, address included, and boots. With ``booted=False``
      the firmware never boots.

    ``samples`` counts the samples made, ``writes`` logs ``(reg, data)`` for every write.
    """

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
        for reg, value in self.ID_REGS.items():
            self.regs[reg] = value
        self.regs[0x0001] = self.addr_por
        self.regs[0x00DE], self.regs[0x00DF] = self.osc >> 8, self.osc & 0xFF
        self.regs[0x00E5] = 1 if booted else 0
        self.mode = 0x00
        self.ready = False
        self.due = None  # ticks when the next sample is ready (None: none coming)

    @property
    def bus_addr(self):
        return self.regs[0x0001] & 0x7F

    def _now(self):
        return ticks_ms()

    def _tick(self):
        """Make the pending sample if its time has come."""
        if self.due is not None and not self.ready and ticks_diff(self._now(), self.due) >= 0:
            self._sample()
            self.due = None
            if self.mode == 0x10:  # one-shot: back to standby after its sample
                self.mode = 0x00

    def _sample(self):
        raw = (self.regs[0x001E] << 8) | self.regs[0x001F]
        offset = BitTools.sign_extend(raw, 13) / 4
        mm = max(0, min(0xFFFF, int(self.distance_mm + offset)))
        block = bytearray(17)
        block[0] = self.status & 0x1F
        block[3] = self.spads & 0xFF  # 0x008C: SPAD count, integer part
        amb, sig = self.ambient_kcps // 8, self.signal_kcps // 8
        block[7], block[8] = amb >> 8 & 0xFF, amb & 0xFF  # 0x0090
        block[13], block[14] = mm >> 8, mm & 0xFF  # 0x0096
        block[15], block[16] = sig >> 8 & 0xFF, sig & 0xFF  # 0x0098
        self.regs[0x0089:0x009A] = block
        self.ready = True
        self.samples += 1

    def _line(self):
        active_low = bool(self.regs[0x0030] & 0x10)
        return int(self.ready) ^ int(active_low)

    def _schedule(self):
        self.due = ticks_add(self._now(), self.sample_ms)

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
        for i, value in enumerate(data):
            r = (reg + i) & 0xFFFF
            if r == 0x0000:
                if value & 1 == 0:  # reset asserted: held, every register reads 0
                    self.mode, self.ready, self.due = 0x00, False, None
                    self.in_reset = True
                elif self.in_reset:  # released: power-on registers, then boot
                    self._power_on(self.can_boot)
                continue
            if self.in_reset:
                continue
            if r in self.READ_ONLY:
                continue
            self.regs[r] = value
            if r == 0x0086 and value & 1:  # clear the interrupt
                self.ready = False
                if self.mode == 0x40:
                    self._schedule()
            elif r == 0x0087:
                self.mode = value
                if value in (0x40, 0x10):
                    self._schedule()
                else:
                    self.due = None
