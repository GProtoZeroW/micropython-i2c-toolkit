import pytest

from micropython_i2c.compat import NullLock
from micropython_i2c.connection import I2CConnection
from micropython_i2c.node import I2CDevice, I2CMux, I2CNode
from micropython_i2c.sim.bus import FakeBus, FakeMux, FakeRegisterPart
# Source: node_devNB.ipynb | Cell In[10] | 2026-11-16 13:33:39
class RecordingConn:
    """Stands in for an ``I2CConnection``: records each call, reads return zeros."""

    def __init__(self, addr=0x48):
        self.addr = addr
        self.calls = []

    def __repr__(self):
        return "<RecordingConn {:#04x}>".format(self.addr)

    def readfrom_mem(self, reg, nbytes, addrsize=8):
        self.calls.append(("readfrom_mem", reg, nbytes, addrsize))
        return bytes(nbytes)

    def writeto_mem(self, reg, data, addrsize=8):
        self.calls.append(("writeto_mem", reg, bytes(data), addrsize))

    def readfrom(self, nbytes):
        self.calls.append(("readfrom", nbytes))
        return bytes(nbytes)

    def writeto(self, data):
        self.calls.append(("writeto", bytes(data)))
        return len(data)

# Source: node_devNB.ipynb | Cell In[11] | 2026-11-16 13:33:39
class DemoPart(I2CNode):
    """A part strappable to 0x48..0x4B, 1 kHz..400 kHz."""

    ALLOWED_ADDRS = (0x48, 0x49, 0x4A, 0x4B)
    BAUD_MIN = 1_000


class WidePart(DemoPart):
    """The same part with a 16-bit register map."""

    REG_ADDR_BITS = 16

# Source: node_devNB.ipynb | Cell In[14] | 2026-11-16 13:33:39
def test_datasheet_defaults():
    assert I2CNode.ALLOWED_ADDRS == ()
    assert (I2CNode.BAUD_MIN, I2CNode.BAUD_NOMINAL, I2CNode.BAUD_MAX) == (10_000, 100_000, 400_000)
    assert I2CNode.REG_ADDR_BITS == 8
    assert not I2CNode.addr_ok(0x48)

# Source: node_devNB.ipynb | Cell In[15] | 2026-11-16 13:33:39
def test_addr_ok_and_baud_ok_need_no_instance():
    assert DemoPart.addr_ok(0x4B) and not DemoPart.addr_ok(0x4C)
    assert DemoPart.baud_ok(1_000) and DemoPart.baud_ok(400_000)
    assert not DemoPart.baud_ok(999) and not DemoPart.baud_ok(400_001)

# Source: node_devNB.ipynb | Cell In[16] | 2026-11-16 13:33:39
def test_addr_repr_and_kwargs():
    dev = DemoPart(RecordingConn(0x4A), label="left", offset=0.5)
    assert dev.addr == 0x4A
    assert repr(dev) == "<DemoPart <RecordingConn 0x4a>>"
    assert dev.kwargs == {"label": "left", "offset": 0.5}

# Source: node_devNB.ipynb | Cell In[17] | 2026-11-16 13:33:40
def test_transport_methods_pass_reg_addr_bits():
    conn = RecordingConn()
    DemoPart(conn).read_mem(0x01, 2)
    DemoPart(conn).write_mem(0x02, b"\x12")
    WidePart(conn).read_mem(0x010F, 1)
    WidePart(conn).write_mem(0x0110, b"\x34")
    assert conn.calls == [
        ("readfrom_mem", 0x01, 2, 8),
        ("writeto_mem", 0x02, b"\x12", 8),
        ("readfrom_mem", 0x010F, 1, 16),
        ("writeto_mem", 0x0110, b"\x34", 16),
    ]

# Source: node_devNB.ipynb | Cell In[18] | 2026-11-16 13:33:40
def test_raw_transport_has_no_register_phase():
    conn = RecordingConn()
    dev = DemoPart(conn)
    assert dev.read_raw(3) == b"\x00\x00\x00"
    assert dev.write_raw(b"\x2c\x06") is None  # the ACK count is not passed on
    assert conn.calls == [("readfrom", 3), ("writeto", b"\x2c\x06")]

# Source: node_devNB.ipynb | Cell In[19] | 2026-11-16 13:33:40
def test_device_reads_through_a_routed_connection():
    bus = FakeBus()
    bus.attach(0x71, FakeMux())
    bus.attach(0x48, FakeRegisterPart({0x01: 0x60, 0x02: 0xA0}), path=[(0x71, 2)])
    dev = DemoPart(I2CConnection(bus, 0x48, [[0x71, 2]], NullLock()))
    assert dev.read_mem(0x01, 2) == b"\x60\xa0"

# Source: node_devNB.ipynb | Cell In[22] | 2026-11-16 13:33:41
def test_device_is_a_plain_node():
    dev = I2CDevice(RecordingConn(0x4A), label="x")
    assert isinstance(dev, I2CNode)
    assert dev.addr == 0x4A and dev.kwargs == {"label": "x"}
    assert repr(dev) == "<I2CDevice <RecordingConn 0x4a>>"
    assert I2CDevice.ALLOWED_ADDRS == () and not hasattr(dev, "channel_select")

# Source: node_devNB.ipynb | Cell In[24] | 2026-11-16 13:33:41
class DemoMux(I2CMux):
    """Eight channels, one bit each in a single control byte."""

    ALLOWED_ADDRS = tuple(range(0x70, 0x78))
    CHANNELS = 8

    def channel_select(self, channel):
        self.channel_check(channel)
        self.write_raw(bytes([1 << channel]))

    def channel_deselect(self):
        self.write_raw(b"\x00")

    def control_read(self):
        return self.read_raw(1)[0]


class RecordingPin:
    """Stands in for ``machine.Pin``: ``levels`` lists every value driven."""

    def __init__(self):
        self.levels = []

    def value(self, v):
        self.levels.append(v)

# Source: node_devNB.ipynb | Cell In[28] | 2026-11-16 13:33:41
def test_mux_builds_its_own_upstream_connection():
    bus, shared = FakeBus(), NullLock()
    mux = DemoMux(bus, 0x72, shared, path=[[0x71, 5]], label="inner")
    assert isinstance(mux.conn, I2CConnection)
    assert (mux.conn.bus, mux.conn.addr, mux.conn.path) == (bus, 0x72, [[0x71, 5]])
    assert mux.conn.lock is shared
    assert mux.addr == 0x72 and mux.kwargs == {"label": "inner"}
    assert DemoMux(bus, 0x70).conn.path == []

# Source: node_devNB.ipynb | Cell In[29] | 2026-11-16 13:33:41
def test_mux_selects_through_its_transport():
    bus = FakeBus()
    bus.attach(0x71, FakeMux())
    bus.attach(0x48, FakeRegisterPart(), path=[(0x71, 2)])
    mux = DemoMux(bus, 0x71, NullLock())
    mux.channel_select(2)
    assert mux.control_read() == 0x04 and 0x48 in bus.scan()
    mux.channel_deselect()
    assert mux.control_read() == 0 and bus.scan() == [0x71]

# Source: node_devNB.ipynb | Cell In[30] | 2026-11-16 13:33:42
def test_channel_check_bounds():
    mux = DemoMux(FakeBus(), 0x70)
    mux.channel_check(0)
    mux.channel_check(7)
    for bad in (-1, 8):
        with pytest.raises(ValueError, match="channel {} out of range 0..7".format(bad)):
            mux.channel_check(bad)
    with pytest.raises(ValueError, match="out of range 0..-1"):
        I2CMux(FakeBus(), 0x70).channel_check(0)  # the base has no channels

# Source: node_devNB.ipynb | Cell In[31] | 2026-11-16 13:33:42
def test_base_mux_api_is_abstract():
    mux = I2CMux(FakeBus(), 0x70)
    for call in (lambda: mux.channel_select(0), mux.channel_deselect, mux.control_read):
        with pytest.raises(NotImplementedError):
            call()

# Source: node_devNB.ipynb | Cell In[32] | 2026-11-16 13:33:42
def test_reset_without_pin_returns_false():
    mux = DemoMux(FakeBus(), 0x70)
    assert mux.reset_pin is None
    assert mux.reset() is False

# Source: node_devNB.ipynb | Cell In[33] | 2026-11-16 13:33:42
def test_reset_pulses_the_pin_low_then_high():
    mux = DemoMux(FakeBus(), 0x70)
    mux.reset_pin = RecordingPin()  # what __init__ builds from reset_pin= on the board
    assert mux.reset() is True
    assert mux.reset_pin.levels == [0, 1]

# Source: node_devNB.ipynb | Cell In[34] | 2026-11-16 13:33:43
def test_reset_pin_needs_machine():
    with pytest.raises(ImportError):
        DemoMux(FakeBus(), 0x70, reset_pin=5)  # no machine module on the host

