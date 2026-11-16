import pytest

from micropython_i2c.compat import NullLock, default_lock
from micropython_i2c.connection import I2CConnection, path_release, path_select
from micropython_i2c.sim.bus import FakeBus, FakeMux, FakeRegisterPart
# Source: connection_devNB.ipynb | Cell In[10] | 2026-11-16 13:30:33
CHAIN = [[0x70, 1], [0x71, 3]]  # the path to the part at 0x35


def chain_bus():
    """Mux 0x70 -> ch 1 -> mux 0x71 -> ch 3 -> register part at 0x35 (regs 0x0E, 0x0F)."""
    bus = FakeBus()
    bus.attach(0x70, FakeMux())
    bus.attach(0x71, FakeMux(), path=[(0x70, 1)])
    bus.attach(0x35, FakeRegisterPart({0x0E: 0x49, 0x0F: 0x54}), path=[(0x70, 1), (0x71, 3)])
    return bus

# Source: connection_devNB.ipynb | Cell In[14] | 2026-11-16 13:30:33
def test_path_select_writes_one_hot_masks_outermost_first():
    bus = chain_bus()
    path_select(bus, CHAIN)
    assert bus.log == [("w", 0x70, b"\x02"), ("w", 0x71, b"\x08")]
    assert 0x35 in bus.scan()

# Source: connection_devNB.ipynb | Cell In[15] | 2026-11-16 13:30:34
def test_path_release_deselects_innermost_first():
    bus = chain_bus()
    path_select(bus, CHAIN)
    bus.log.clear()
    path_release(bus, CHAIN)
    assert bus.log == [("w", 0x71, b"\x00"), ("w", 0x70, b"\x00")]
    assert bus.scan() == [0x70]

# Source: connection_devNB.ipynb | Cell In[16] | 2026-11-16 13:30:34
def test_empty_path_writes_nothing():
    bus = chain_bus()
    path_select(bus, [])
    path_release(bus, [])
    assert bus.log == []

# Source: connection_devNB.ipynb | Cell In[17] | 2026-11-16 13:30:34
def test_path_select_through_absent_mux_raises_oserror():
    bus = chain_bus()
    with pytest.raises(OSError):
        path_select(bus, [[0x74, 0]])

# Source: connection_devNB.ipynb | Cell In[19] | 2026-11-16 13:30:35
def one_mux_bus():
    """Mux 0x71 -> ch 3 -> register part at 0x35 (regs 0x0D..0x0F)."""
    bus = FakeBus()
    bus.attach(0x71, FakeMux())
    bus.attach(0x35, FakeRegisterPart({0x0E: 0x49, 0x0F: 0x54, 0x0D: 0x01}), path=[(0x71, 3)])
    return bus

# Source: connection_devNB.ipynb | Cell In[23] | 2026-11-16 13:30:35
def test_connection_selects_path_each_transfer():
    bus = one_mux_bus()
    conn = I2CConnection(bus, 0x35, [[0x71, 3]], NullLock())
    with pytest.raises(OSError):
        bus.readfrom_mem(0x35, 0x0E, 1)  # unreachable until the mux is switched
    assert conn.readfrom_mem(0x0E, 2) == b"\x49\x54"
    assert bus.log[-2] == ("w", 0x71, b"\x08")

# Source: connection_devNB.ipynb | Cell In[24] | 2026-11-16 13:30:35
def test_path_reselected_after_another_connection_switched_the_mux():
    bus = one_mux_bus()
    conn = I2CConnection(bus, 0x35, [[0x71, 3]], NullLock())
    conn.readfrom_mem(0x0E, 1)
    bus.writeto(0x71, b"\x01")  # someone else routes the mux to channel 0
    assert conn.readfrom_mem(0x0F, 1) == b"\x54"

# Source: connection_devNB.ipynb | Cell In[25] | 2026-11-16 13:30:35
def test_every_transfer_method_routes_and_drops_the_address():
    bus = one_mux_bus()
    conn = I2CConnection(bus, 0x35, [[0x71, 3]], NullLock())
    conn.writeto_mem(0x20, b"\xaa\xbb")
    assert conn.readfrom_mem(0x20, 2) == b"\xaa\xbb"
    buf = bytearray(2)
    conn.readfrom_mem_into(0x0E, buf)
    assert bytes(buf) == b"\x49\x54"
    assert conn.writeto(b"\x01\x02") == 2
    assert conn.readfrom(1) == b"\x00"  # FakeRegisterPart raw reads start at register 0
    raw = bytearray(1)
    conn.readfrom_into(raw)
    assert bytes(raw) == b"\x00"
    transfers = [e for e in bus.log if e[1] == 0x35]
    selects = [e for e in bus.log if e == ("w", 0x71, b"\x08")]
    assert len(selects) == len(transfers) == 6

# Source: connection_devNB.ipynb | Cell In[26] | 2026-11-16 13:30:35
def test_two_level_path_routes_through_both_muxes():
    bus = chain_bus()
    conn = I2CConnection(bus, 0x35, CHAIN, NullLock())
    assert conn.readfrom_mem(0x0E, 2) == b"\x49\x54"
    assert bus.log[:2] == [("w", 0x70, b"\x02"), ("w", 0x71, b"\x08")]

# Source: connection_devNB.ipynb | Cell In[27] | 2026-11-16 13:30:36
def test_absent_part_raises_oserror():
    bus = one_mux_bus()
    with pytest.raises(OSError):
        I2CConnection(bus, 0x36, [[0x71, 3]]).readfrom_mem(0x0E, 1)

# Source: connection_devNB.ipynb | Cell In[28] | 2026-11-16 13:30:36
def test_defaults_and_repr():
    bus = FakeBus()
    bare = I2CConnection(bus, 0x48)
    assert bare.path == [] and bare.addr == 0x48 and bare.bus is bus
    assert repr(bare) == "<I2CConnection 0x48>"
    routed = I2CConnection(bus, 0x35, [[0x70, 1], [0x71, 3]])
    assert repr(routed) == "<I2CConnection 0x35 via 0x70:1 via 0x71:3>"

# Source: connection_devNB.ipynb | Cell In[29] | 2026-11-16 13:30:36
class LockWatchBus:
    """Wraps a bus; ``held`` records, per transfer, whether ``lock`` was held at that moment."""

    def __init__(self, inner, lock):
        self.inner = inner
        self.lock = lock
        self.held = []

    def _note(self):
        free = self.lock.acquire(False)
        if free:
            self.lock.release()
        self.held.append(not free)

    def writeto(self, addr, data, stop=True):
        self._note()
        return self.inner.writeto(addr, data, stop)

    def readfrom(self, addr, nbytes, stop=True):
        self._note()
        return self.inner.readfrom(addr, nbytes, stop)

    def readfrom_into(self, addr, buf, stop=True):
        self._note()
        self.inner.readfrom_into(addr, buf, stop)

    def writeto_mem(self, addr, reg, data, addrsize=8):
        self._note()
        self.inner.writeto_mem(addr, reg, data, addrsize)

    def readfrom_mem(self, addr, reg, nbytes, addrsize=8):
        self._note()
        return self.inner.readfrom_mem(addr, reg, nbytes, addrsize)

    def readfrom_mem_into(self, addr, reg, buf, addrsize=8):
        self._note()
        self.inner.readfrom_mem_into(addr, reg, buf, addrsize)

# Source: connection_devNB.ipynb | Cell In[32] | 2026-11-16 13:30:36
def test_select_and_transfer_happen_under_the_lock():
    lock = default_lock()
    watch = LockWatchBus(one_mux_bus(), lock)
    conn = I2CConnection(watch, 0x35, [[0x71, 3]], lock)
    conn.readfrom_mem(0x0E, 2)
    conn.writeto(b"\x00")
    assert watch.held == [True, True, True, True]
    assert lock.acquire(False)
    lock.release()

# Source: connection_devNB.ipynb | Cell In[33] | 2026-11-16 13:30:36
def test_lock_released_when_transfer_fails():
    lock = default_lock()
    conn = I2CConnection(LockWatchBus(one_mux_bus(), lock), 0x36, [[0x71, 3]], lock)
    with pytest.raises(OSError):
        conn.readfrom_mem(0x0E, 1)
    assert lock.acquire(False)
    lock.release()

# Source: connection_devNB.ipynb | Cell In[34] | 2026-11-16 13:30:37
def test_shared_lock_is_used_and_default_lock_is_private():
    bus, shared = FakeBus(), NullLock()
    assert I2CConnection(bus, 0x48, lock=shared).lock is shared
    a, b = I2CConnection(bus, 0x48), I2CConnection(bus, 0x49)
    assert a.lock is not b.lock

