import pytest

from micropython_i2c import NullLock
from micropython_i2c.drivers.tca9548a import TCA9548A
from micropython_i2c.fakes import FakeBus, FakeMux, FakeRegisterPart
# Source: drivers_tca9548a_devNB.ipynb | Cell In[9] | 2026-11-13 20:41:54
def fake_bench(mux_addr=0x71):
    """``(bus, mux)``: a ``TCA9548A`` at ``mux_addr`` with parts behind channels 0 and 2."""
    bus = FakeBus()
    bus.attach(mux_addr, FakeMux())
    bus.attach(0x70, FakeRegisterPart(), path=[(mux_addr, 0)])
    bus.attach(0x48, FakeRegisterPart(), path=[(mux_addr, 2)])
    return bus, TCA9548A(bus, mux_addr, lock=NullLock())

# Source: drivers_tca9548a_devNB.ipynb | Cell In[14] | 2026-11-13 20:41:56
@pytest.mark.parametrize("ch", range(8))
def test_channel_select_enables_only_that_channel(ch):
    _, mux = fake_bench()
    mux.channel_select(ch)
    assert mux.control_read() == 1 << ch
    assert mux.channels_get() == [ch]

# Source: drivers_tca9548a_devNB.ipynb | Cell In[15] | 2026-11-13 20:41:59
@pytest.mark.parametrize("ch", [-1, 8, 99])
def test_out_of_range_channel_is_refused(ch):
    bus, mux = fake_bench()
    before = len(bus.log)
    with pytest.raises(ValueError, match="out of range 0..7"):
        mux.channel_select(ch)
    assert len(bus.log) == before

# Source: drivers_tca9548a_devNB.ipynb | Cell In[16] | 2026-11-13 20:42:00
def test_mask_set_and_deselect():
    _, mux = fake_bench()
    mux.channel_mask_set(0b1010_0101)
    assert mux.channels_get() == [0, 2, 5, 7]
    mux.channel_deselect()
    assert mux.control_read() == mux.CONTROL_POR
    assert mux.channels_get() == []

# Source: drivers_tca9548a_devNB.ipynb | Cell In[17] | 2026-11-13 20:42:01
def test_parts_appear_only_on_their_channel():
    bus, mux = fake_bench()
    mux.channel_deselect()
    assert bus.scan() == [0x71]
    mux.channel_select(2)
    assert bus.scan() == [0x48, 0x71]
    mux.channel_select(0)
    assert bus.scan() == [0x70, 0x71]

# Source: drivers_tca9548a_devNB.ipynb | Cell In[18] | 2026-11-13 20:42:02
def test_channel_scan_finds_the_bench():
    _, mux = fake_bench()
    mux.channel_select(5)  # left on before the scan
    found = mux.channel_scan()
    assert found == {0: [0x70], 1: [], 2: [0x48], 3: [], 4: [], 5: [], 6: [], 7: []}
    assert mux.channels_get() == []

# Source: drivers_tca9548a_devNB.ipynb | Cell In[19] | 2026-11-13 20:42:03
def test_channel_scan_cannot_see_a_part_at_the_mux_address():
    _, mux = fake_bench(mux_addr=0x70)
    assert mux.channel_scan([0, 2]) == {0: [], 2: [0x48]}

# Source: drivers_tca9548a_devNB.ipynb | Cell In[20] | 2026-11-13 20:42:04
def test_channel_scan_refuses_bad_channels_first():
    bus, mux = fake_bench()
    before = len(bus.log)
    with pytest.raises(ValueError, match="out of range"):
        mux.channel_scan([0, 9])
    assert len(bus.log) == before

# Source: drivers_tca9548a_devNB.ipynb | Cell In[21] | 2026-11-13 20:42:05
def test_datasheet_facts():
    assert TCA9548A.CHANNELS == 8
    assert TCA9548A.ALLOWED_ADDRS == tuple(range(0x70, 0x78))
    assert TCA9548A.addr_ok(0x70) and TCA9548A.addr_ok(0x77) and not TCA9548A.addr_ok(0x78)
    assert TCA9548A.baud_ok(400_000) and not TCA9548A.baud_ok(1_000_000)

