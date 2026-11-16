import pytest

from micropython_i2c import I2CConnection
from micropython_i2c.devices.sensors.distance.vl53l4cd import _INIT, STATUS_NAMES, VL53L4CD
from micropython_i2c.devices.sensors.distance.vl53l4cd_fake import FakeVL53L4CD
from micropython_i2c.sim.bus import FakeBus
from micropython_i2c.sim.clock import SimClock
from micropython_i2c.sim.signals import ramp
# Source: vl53l4cd_devNB.ipynb | Cell In[14] | 2026-11-16 13:38:48
def test_fake_reference_registers_follow_table_7():
    bus = FakeBus()
    bus.attach(0x29, FakeVL53L4CD())
    bus.attach(0x30, FakeVL53L4CD(addr=0x30, booted=False))
    bus.writeto_mem(0x29, 0x010F, b"\x00\x00", addrsize=16)
    assert bus.readfrom_mem(0x29, 0x010F, 2, addrsize=16) == b"\xeb\xaa"
    assert bus.readfrom_mem(0x29, 0x00E5, 1, addrsize=16) == b"\x03"
    assert bus.readfrom_mem(0x30, 0x00E5, 1, addrsize=16) == b"\x00"

# Source: vl53l4cd_devNB.ipynb | Cell In[15] | 2026-11-16 13:38:48
def test_fake_new_sample_only_after_an_interrupt_clear():
    with SimClock() as clock:
        bus = FakeBus()
        part = bus.attach(0x29, FakeVL53L4CD(sample_ms=50))
        bus.writeto_mem(0x29, 0x0087, b"\x21", addrsize=16)
        clock.advance(0.049)
        assert bus.readfrom_mem(0x29, 0x0031, 1, addrsize=16) == b"\x01"  # not yet
        clock.advance(10)
        assert bus.readfrom_mem(0x29, 0x0031, 1, addrsize=16) == b"\x00" and part.samples == 1
        bus.writeto_mem(0x29, 0x0086, b"\x01", addrsize=16)
        clock.advance(0.050)
        assert bus.readfrom_mem(0x29, 0x0031, 1, addrsize=16) == b"\x00" and part.samples == 2

# Source: vl53l4cd_devNB.ipynb | Cell In[16] | 2026-11-16 13:38:48
@pytest.mark.parametrize("stop", [0x00, 0x80])
def test_fake_stop_ends_the_samples(stop):
    with SimClock() as clock:
        bus = FakeBus()
        part = bus.attach(0x29, FakeVL53L4CD(sample_ms=50))
        bus.writeto_mem(0x29, 0x0087, b"\x21", addrsize=16)
        clock.advance(0.010)
        bus.writeto_mem(0x29, 0x0087, bytes([stop]), addrsize=16)
        clock.advance(1)
        assert bus.readfrom_mem(0x29, 0x0031, 1, addrsize=16) == b"\x01" and part.samples == 0

# Source: vl53l4cd_devNB.ipynb | Cell In[17] | 2026-11-16 13:38:48
def test_fake_quantities_are_read_when_sampled():
    with SimClock() as clock:
        bus = FakeBus()
        bus.attach(0x29, FakeVL53L4CD(distance_mm=ramp(1000, 0, 0, 10),
                                      status=lambda t: 9 if t < 5 else 4, sample_ms=1))  # fmt: skip
        clock.advance(2.5)
        bus.writeto_mem(0x29, 0x0087, b"\x21", addrsize=16)
        clock.advance(0.5)
        assert bus.readfrom_mem(0x29, 0x0096, 2, addrsize=16) == (700).to_bytes(2, "big")
        assert bus.readfrom_mem(0x29, 0x0089, 1, addrsize=16) == b"\x09"
        bus.writeto_mem(0x29, 0x0086, b"\x01", addrsize=16)
        clock.advance(3)
        assert bus.readfrom_mem(0x29, 0x0096, 2, addrsize=16) == (400).to_bytes(2, "big")
        assert bus.readfrom_mem(0x29, 0x0089, 1, addrsize=16) == b"\x04"

# Source: vl53l4cd_devNB.ipynb | Cell In[21] | 2026-11-16 13:38:49
def test_init_table_covers_its_range():
    from_table = dict(zip(range(0x2D, 0x88), _INIT))
    assert len(_INIT) == 0x87 - 0x2D + 1 == 91
    assert from_table[0x30] & 0x10 and from_table[0x87] == 0x00 and from_table[0x86] == 0x00

# Source: vl53l4cd_devNB.ipynb | Cell In[22] | 2026-11-16 13:38:49
def test_only_status_9_is_valid():
    assert [s for s, name in STATUS_NAMES.items() if name == "valid"] == [9]

# Source: vl53l4cd_devNB.ipynb | Cell In[24] | 2026-11-16 13:38:49
BUDGET_CASES = [10, 20, 33, 50, 100, 200]


def periods(word):
    """Macro periods a range-config word stands for: ``(mantissa + 1) << exponent``."""
    return ((word & 0xFF) + 1) << (word >> 8)

# Source: vl53l4cd_devNB.ipynb | Cell In[27] | 2026-11-16 13:38:49
def test_budget_words_fit_registers():
    for ms in (10, 50, 200):
        for word in VL53L4CD.range_config_words(ms, 0x0BB8):
            assert 0 <= word <= 0xFFFF and word & 0xFF <= 0xFF

# Source: vl53l4cd_devNB.ipynb | Cell In[28] | 2026-11-16 13:38:49
def test_budget_words_grow_with_the_budget():
    words = [VL53L4CD.range_config_words(ms, 0x0BB8) for ms in BUDGET_CASES]
    for i in (0, 1):
        counts = [periods(w[i]) for w in words]
        assert counts == sorted(counts) and counts[0] < counts[-1]
    assert all(periods(b) > periods(a) for a, b in words)

# Source: vl53l4cd_devNB.ipynb | Cell In[29] | 2026-11-16 13:38:50
def on_fake(addr=0x29, **part_kw):
    """``(tof, part)``: a ``VL53L4CD`` on a fresh ``FakeVL53L4CD(**part_kw)`` at ``addr``."""
    bus = FakeBus()
    part = bus.attach(addr, FakeVL53L4CD(addr=addr, **part_kw))
    return VL53L4CD(I2CConnection(bus, addr)), part

# Source: vl53l4cd_devNB.ipynb | Cell In[33] | 2026-11-16 13:38:50
def test_init_loads_the_table_and_sets_50_ms():
    tof, part = on_fake()
    assert tof.timing_budget == 50 and not tof.ranging and part.mode == 0x00
    assert (0x002D, bytes(_INIT)) in part.writes and part.regs[0x002D] == 0x12
    assert (0x0087, b"\x40") in part.writes and part.samples >= 1
    assert any(e[0] == "rm" and e[2] == 0x010F for e in tof.conn.bus.log)
    words = VL53L4CD.range_config_words(50, 0x0BB8)
    assert [int.from_bytes(part.regs[r : r + 2], "big") for r in (0x005E, 0x0061)] == words

# Source: vl53l4cd_devNB.ipynb | Cell In[34] | 2026-11-16 13:38:50
def test_reads_distance_when_started():
    tof, _ = on_fake(distance_mm=456)
    assert tof.status_name == "no reading yet"
    tof.start()
    assert tof.read() == 456 and tof.valid and tof.status_name == "valid"
    tof.stop()

# Source: vl53l4cd_devNB.ipynb | Cell In[35] | 2026-11-16 13:38:50
def test_each_read_is_a_new_sample():
    tof, part = on_fake()
    tof.start()
    before = part.samples
    for _ in range(3):
        tof.read()
    tof.stop()
    assert part.samples - before >= 3 and not tof.ranging

# Source: vl53l4cd_devNB.ipynb | Cell In[36] | 2026-11-16 13:38:50
def test_timing_budget_rules():
    tof, _ = on_fake()
    tof.start()
    with pytest.raises(RuntimeError, match="stop"):
        tof.timing_budget = 100
    tof.stop()
    for bad in (9, 201):
        with pytest.raises(ValueError, match="10..200"):
            tof.timing_budget = bad
    tof.timing_budget = 100
    assert tof.timing_budget == 100

# Source: vl53l4cd_devNB.ipynb | Cell In[37] | 2026-11-16 13:38:51
def test_init_refuses_another_model():
    bus = FakeBus()
    part = bus.attach(0x29, FakeVL53L4CD())
    part.regs[0x010F] = 0x00
    with pytest.raises(RuntimeError, match="not a VL53L4CD"):
        VL53L4CD(I2CConnection(bus, 0x29))

# Source: vl53l4cd_devNB.ipynb | Cell In[38] | 2026-11-16 13:38:51
def test_init_times_out_without_boot():
    with SimClock() as clock:
        with pytest.raises(OSError, match="boot timeout"):
            on_fake(booted=False)
        assert 1.0 <= clock.now_s() < 1.1

# Source: vl53l4cd_devNB.ipynb | Cell In[39] | 2026-11-16 13:38:51
def test_oscillator_zero_is_refused():
    with pytest.raises(OSError, match="oscillator"):
        on_fake(osc=0)

# Source: vl53l4cd_devNB.ipynb | Cell In[40] | 2026-11-16 13:38:51
def test_invalid_status_is_flagged():
    tof, _ = on_fake(status=4)
    tof.start()
    tof.read()
    assert not tof.valid and tof.status_name == "sigma warning"
    assert tof.read_valid(tries=3) is None
    tof.stop()

# Source: vl53l4cd_devNB.ipynb | Cell In[41] | 2026-11-16 13:38:51
def test_read_times_out_when_not_started():
    tof, _ = on_fake()
    with pytest.raises(OSError, match="read timeout"):
        tof.read(timeout_ms=20)

# Source: vl53l4cd_devNB.ipynb | Cell In[44] | 2026-11-16 13:38:52
def test_follows_a_hand_approaching_in_simulated_time():
    hand = ramp(1200, 30, 0, 60)
    with SimClock() as clock:
        tof, _ = on_fake(distance_mm=hand, sample_ms=50)
        tof.start()
        readings = []
        for _ in range(7):
            mm = tof.read()
            assert tof.valid and abs(mm - hand(clock.now_s())) <= 5
            readings.append(mm)
            clock.advance(10)
        tof.stop()
    assert readings == sorted(readings, reverse=True) and readings[-1] <= 35

# Source: vl53l4cd_devNB.ipynb | Cell In[45] | 2026-11-16 13:38:52
def test_reads_come_one_sample_apart_in_simulated_time():
    with SimClock() as clock:
        tof, _ = on_fake(sample_ms=50)
        tof.start()
        t0 = clock.now_s()
        for _ in range(10):
            tof.read()
        took = clock.now_s() - t0
        tof.stop()
    assert 0.5 <= took <= 0.55

