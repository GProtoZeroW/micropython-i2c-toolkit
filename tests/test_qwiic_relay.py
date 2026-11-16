import pytest

from micropython_i2c import I2CConnection
from micropython_i2c.devices.drivers.qwiic_relay import QwiicRelay
from micropython_i2c.devices.drivers.qwiic_relay_fake import FakeQwiicRelay
from micropython_i2c.sim.bus import FakeBus
from micropython_i2c.sim.clock import SimClock
from micropython_i2c.sim.signals import lag, value
# Source: qwiic_relay_devNB.ipynb | Cell In[12] | 2026-11-16 13:42:55
def test_fake_switches_on_commands():
    bus = FakeBus()
    part = bus.attach(0x18, FakeQwiicRelay())
    for cmd, closed in ((0x01, True), (0x01, True), (0x04, True), (0x00, False), (0x05, False)):
        bus.writeto(0x18, bytes([cmd]))
        assert part.closed is closed
    assert part.switches == 2

# Source: qwiic_relay_devNB.ipynb | Cell In[13] | 2026-11-16 13:42:55
def test_fake_reads_status_and_version_both_ways():
    bus = FakeBus()
    bus.attach(0x18, FakeQwiicRelay(version=0x07))
    bus.writeto(0x18, b"\x01")
    assert bus.readfrom_mem(0x18, 0x05, 1) == b"\x01"
    assert bus.readfrom_mem(0x18, 0x04, 1) == b"\x07"
    bus.writeto(0x18, b"\x05")
    assert bus.readfrom(0x18, 1) == b"\x01"

# Source: qwiic_relay_devNB.ipynb | Cell In[14] | 2026-11-16 13:42:55
def test_fake_refuses_a_read_without_a_reply():
    bus = FakeBus()
    bus.attach(0x18, FakeQwiicRelay())
    with pytest.raises(OSError):
        bus.readfrom_mem(0x18, 0x07, 1)
    with pytest.raises(OSError):
        bus.readfrom(0x18, 1)  # no command written yet

# Source: qwiic_relay_devNB.ipynb | Cell In[17] | 2026-11-16 13:42:55
def on_fake(addr=0x18, **part_kw):
    """``(relay, part)``: a ``QwiicRelay`` on a fresh ``FakeQwiicRelay(**part_kw)`` at ``addr``."""
    bus = FakeBus()
    part = bus.attach(addr, FakeQwiicRelay(**part_kw))
    return QwiicRelay(I2CConnection(bus, addr)), part

# Source: qwiic_relay_devNB.ipynb | Cell In[20] | 2026-11-16 13:42:55
def test_on_off_and_state():
    relay, part = on_fake()
    assert relay.state is False
    relay.on()
    assert part.closed and relay.state is True
    relay.off()
    assert not part.closed and relay.state is False

# Source: qwiic_relay_devNB.ipynb | Cell In[21] | 2026-11-16 13:42:56
def test_toggle_returns_the_new_state():
    relay, part = on_fake()
    relay.on()
    assert relay.toggle() is False and not part.closed
    assert relay.toggle() is True and part.closed

# Source: qwiic_relay_devNB.ipynb | Cell In[22] | 2026-11-16 13:42:56
def test_state_setter():
    relay, part = on_fake()
    relay.state = True
    assert part.closed
    relay.state = False
    assert not part.closed

# Source: qwiic_relay_devNB.ipynb | Cell In[23] | 2026-11-16 13:42:56
def test_state_is_read_fresh():
    relay, part = on_fake()
    part.closed = True  # someone else switched it
    assert relay.state is True

# Source: qwiic_relay_devNB.ipynb | Cell In[24] | 2026-11-16 13:42:56
def test_firmware_version():
    relay, _ = on_fake(version=0x02)
    assert relay.firmware_version() == 0x02

# Source: qwiic_relay_devNB.ipynb | Cell In[25] | 2026-11-16 13:42:56
def test_address_and_clock_limits():
    assert QwiicRelay.addr_ok(0x18) and QwiicRelay.addr_ok(0x19)
    assert not QwiicRelay.addr_ok(0x20)
    assert QwiicRelay.baud_ok(400_000) and not QwiicRelay.baud_ok(1_000_000)

# Source: qwiic_relay_devNB.ipynb | Cell In[29] | 2026-11-16 13:42:57
def test_thermostat_holds_the_band_in_simulated_time():
    with SimClock() as clock:
        relay, part = on_fake()
        temp = lag(lambda t: 30.0 if part.closed else 18.0, tau_s=120, initial=18.0)
        late = []
        for _ in range(720):  # one hour, every 5 s
            t_c = value(temp)
            if t_c < 21.5:
                relay.on()
            elif t_c > 22.5:
                relay.off()
            if clock.now_s() >= 600:
                late.append(t_c)
            clock.advance(5)
    assert 21.0 <= min(late) and max(late) <= 23.0
    assert part.switches >= 10 and part.closed == relay.state

