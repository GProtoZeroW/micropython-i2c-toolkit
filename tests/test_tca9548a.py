import pytest

from micropython_i2c import NullLock
from micropython_i2c.devices.muxes.tca9548a import TCA9548A
from micropython_i2c.devices.muxes.tca9548a_fake import FakeTCA9548A
from micropython_i2c.sim.bus import FakeBus, FakeMux, FakeRegisterPart
# Source: tca9548a_devNB.ipynb | Cell In[13] | 2026-11-16 13:35:23
def test_fake_powers_on_with_every_channel_off():
    bus = FakeBus()
    mux = bus.attach(0x70, FakeTCA9548A())
    assert isinstance(mux, FakeMux)
    assert mux.state == FakeTCA9548A.CONTROL_POR == 0x00
    assert bus.readfrom(0x70, 1) == b"\x00"

# Source: tca9548a_devNB.ipynb | Cell In[14] | 2026-11-16 13:35:23
def test_fake_reads_back_the_last_byte_written():
    bus = FakeBus()
    bus.attach(0x70, FakeTCA9548A())
    bus.writeto(0x70, b"\x01\x80\x24")
    assert bus.readfrom(0x70, 3) == b"\x24\x24\x24"

# Source: tca9548a_devNB.ipynb | Cell In[15] | 2026-11-16 13:35:23
def test_fake_address_only_write_changes_nothing():
    bus = FakeBus()
    bus.attach(0x70, FakeTCA9548A())
    bus.writeto(0x70, b"\x04")
    bus.writeto(0x70, b"")
    assert bus.readfrom(0x70, 1) == b"\x04"

# Source: tca9548a_devNB.ipynb | Cell In[18] | 2026-11-16 13:35:24
def fake_bench(mux_addr=0x71):
    """``(bus, mux)``: a ``TCA9548A`` at ``mux_addr`` with parts behind channels 0 and 2."""
    bus = FakeBus()
    bus.attach(mux_addr, FakeTCA9548A())
    bus.attach(0x70, FakeRegisterPart(), path=[(mux_addr, 0)])
    bus.attach(0x48, FakeRegisterPart(), path=[(mux_addr, 2)])
    return bus, TCA9548A(bus, mux_addr, lock=NullLock())

# Source: tca9548a_devNB.ipynb | Cell In[23] | 2026-11-16 13:35:24
@pytest.mark.parametrize("ch", range(8))
def test_channel_select_enables_only_that_channel(ch):
    _, mux = fake_bench()
    mux.channel_select(ch)
    assert mux.control_read() == 1 << ch
    assert mux.channels_get() == [ch]

# Source: tca9548a_devNB.ipynb | Cell In[24] | 2026-11-16 13:35:24
@pytest.mark.parametrize("ch", [-1, 8, 99])
def test_out_of_range_channel_is_refused(ch):
    bus, mux = fake_bench()
    before = len(bus.log)
    with pytest.raises(ValueError, match="out of range 0..7"):
        mux.channel_select(ch)
    assert len(bus.log) == before

# Source: tca9548a_devNB.ipynb | Cell In[25] | 2026-11-16 13:35:24
def test_mask_set_and_deselect():
    _, mux = fake_bench()
    mux.channel_mask_set(0b1010_0101)
    assert mux.channels_get() == [0, 2, 5, 7]
    mux.channel_deselect()
    assert mux.control_read() == mux.CONTROL_POR
    assert mux.channels_get() == []

# Source: tca9548a_devNB.ipynb | Cell In[26] | 2026-11-16 13:35:24
def test_parts_appear_only_on_their_channel():
    bus, mux = fake_bench()
    mux.channel_deselect()
    assert bus.scan() == [0x71]
    mux.channel_select(2)
    assert bus.scan() == [0x48, 0x71]
    mux.channel_select(0)
    assert bus.scan() == [0x70, 0x71]

# Source: tca9548a_devNB.ipynb | Cell In[27] | 2026-11-16 13:35:25
def test_channel_scan_finds_the_bench():
    _, mux = fake_bench()
    mux.channel_select(5)  # left on before the scan
    found = mux.channel_scan()
    assert found == {0: [0x70], 1: [], 2: [0x48], 3: [], 4: [], 5: [], 6: [], 7: []}
    assert mux.channels_get() == []

# Source: tca9548a_devNB.ipynb | Cell In[28] | 2026-11-16 13:35:25
def test_channel_scan_cannot_see_a_part_at_the_mux_address():
    _, mux = fake_bench(mux_addr=0x70)
    assert mux.channel_scan([0, 2]) == {0: [], 2: [0x48]}

# Source: tca9548a_devNB.ipynb | Cell In[29] | 2026-11-16 13:35:25
def test_channel_scan_refuses_bad_channels_first():
    bus, mux = fake_bench()
    before = len(bus.log)
    with pytest.raises(ValueError, match="out of range"):
        mux.channel_scan([0, 9])
    assert len(bus.log) == before

# Source: tca9548a_devNB.ipynb | Cell In[30] | 2026-11-16 13:35:26
def test_datasheet_facts():
    assert TCA9548A.CHANNELS == 8
    assert TCA9548A.ALLOWED_ADDRS == tuple(range(0x70, 0x78))
    assert TCA9548A.addr_ok(0x70) and TCA9548A.addr_ok(0x77) and not TCA9548A.addr_ok(0x78)
    assert TCA9548A.baud_ok(400_000) and not TCA9548A.baud_ok(1_000_000)

