import pytest

from micropython_i2c import I2CConnection
from micropython_i2c.devices.sensors.distance.vl53l1x import VL53L1X, VL53L1XRegs, VL53L1XResult
from micropython_i2c.devices.sensors.distance.vl53l1x_fake import FakeVL53L1X
from micropython_i2c.devices.sensors.distance.vl53l4cd_fake import FakeVL53L4CD
from micropython_i2c.sim.bus import FakeBus
from micropython_i2c.sim.clock import SimClock
from micropython_i2c.sim.signals import ramp
# Source: vl53l1x_devNB.ipynb | Cell In[14] | 2026-11-16 13:39:18
def test_fake_reference_registers_follow_table_8():
    bus = FakeBus()
    bus.attach(0x29, FakeVL53L1X())
    bus.writeto_mem(0x29, 0x010F, b"\x00\x00\x00", addrsize=16)
    assert bus.readfrom_mem(0x29, 0x010F, 3, addrsize=16) == b"\xea\xcc\x10"

# Source: vl53l1x_devNB.ipynb | Cell In[15] | 2026-11-16 13:39:18
def test_fake_new_sample_only_after_an_interrupt_clear():
    with SimClock() as clock:
        bus = FakeBus()
        part = bus.attach(0x29, FakeVL53L1X(sample_ms=50))
        bus.writeto_mem(0x29, 0x0087, b"\x40", addrsize=16)
        clock.advance(0.049)
        assert bus.readfrom_mem(0x29, 0x0031, 1, addrsize=16) == b"\x00"
        clock.advance(10)
        assert bus.readfrom_mem(0x29, 0x0031, 1, addrsize=16) == b"\x01" and part.samples == 1
        bus.writeto_mem(0x29, 0x0086, b"\x01", addrsize=16)
        clock.advance(0.050)
        assert bus.readfrom_mem(0x29, 0x0031, 1, addrsize=16) == b"\x01" and part.samples == 2

# Source: vl53l1x_devNB.ipynb | Cell In[16] | 2026-11-16 13:39:18
def test_fake_quantities_are_read_when_sampled():
    with SimClock() as clock:
        bus = FakeBus()
        part = FakeVL53L1X(distance_mm=ramp(1000, 0, 0, 10), signal_kcps=lambda t: 800 * t,
                           ambient_kcps=80, sample_ms=1)  # fmt: skip
        bus.attach(0x29, part)
        clock.advance(2.5)
        bus.writeto_mem(0x29, 0x0087, b"\x10", addrsize=16)  # one shot
        clock.advance(0.5)
        block = bus.readfrom_mem(0x29, 0x0089, 17, addrsize=16)
        assert (block[13] << 8) | block[14] == 700  # at 3.0 s: made at the first read after due
        assert ((block[15] << 8) | block[16], (block[7] << 8) | block[8]) == (300, 10)

# Source: vl53l1x_devNB.ipynb | Cell In[17] | 2026-11-16 13:39:19
def test_fake_soft_reset_holds_and_restores():
    bus = FakeBus()
    bus.attach(0x29, FakeVL53L1X())
    bus.writeto_mem(0x29, 0x0001, b"\x30", addrsize=16)
    bus.writeto_mem(0x30, 0x0000, b"\x00", addrsize=16)
    assert bus.scan() == [0x29]
    assert bus.readfrom_mem(0x29, 0x010F, 3, addrsize=16) == b"\x00\x00\x00"
    bus.writeto_mem(0x29, 0x0000, b"\x01", addrsize=16)
    assert bus.readfrom_mem(0x29, 0x010F, 3, addrsize=16) == b"\xea\xcc\x10"
    assert bus.readfrom_mem(0x29, 0x00E5, 1, addrsize=16) == b"\x01"

# Source: vl53l1x_devNB.ipynb | Cell In[20] | 2026-11-16 13:39:19
def on_fake(cls, addr=0x29, dev_kw=None, **part_kw):
    """``(dev, part)``: ``cls(**dev_kw)`` bound to a fresh ``FakeVL53L1X(**part_kw)`` at
    ``addr`` on a fake bus."""
    bus = FakeBus()
    part = bus.attach(addr, FakeVL53L1X(addr=addr, **part_kw))
    return cls(I2CConnection(bus, addr), **(dev_kw or {})), part

# Source: vl53l1x_devNB.ipynb | Cell In[23] | 2026-11-16 13:39:19
def test_default_config_covers_its_range():
    r = VL53L1XRegs
    assert len(r.DEFAULT_CONFIG) == 0x0087 - 0x002D + 1 == 91
    at = lambda reg: r.DEFAULT_CONFIG[reg - r.DEFAULT_CONFIG_START]  # noqa: E731
    assert at(r.REG_MODE_START) == 0x00 and at(r.REG_INTERRUPT_CLEAR) == 0x00
    assert at(r.REG_GPIO_HV_MUX_CTRL) & 0x10 == 0  # active high
    assert at(r.REG_PHASECAL_TIMEOUT) == r.DISTANCE_MODES["long"][0]

# Source: vl53l1x_devNB.ipynb | Cell In[24] | 2026-11-16 13:39:19
def test_timing_budgets_follow_the_datasheet():
    r = VL53L1XRegs
    assert min(r.TIMING_BUDGETS["short"]) == 20 and min(r.TIMING_BUDGETS["long"]) == 33
    for mode, table in r.TIMING_BUDGETS.items():
        assert all(r.TB_MIN_MS <= ms <= r.TB_MAX_MS for ms in table)
        assert not set(table) & set(r.ULD_ONLY_BUDGETS[mode])
        assert len({words[0] for words in table.values()}) == len(table)  # A words tell apart

# Source: vl53l1x_devNB.ipynb | Cell In[25] | 2026-11-16 13:39:19
def test_address_and_clock_limits():
    assert VL53L1XRegs.addr_ok(0x29) and not VL53L1XRegs.addr_ok(0x78)
    assert VL53L1XRegs.baud_ok(400_000) and not VL53L1XRegs.baud_ok(1_000_000)

# Source: vl53l1x_devNB.ipynb | Cell In[26] | 2026-11-16 13:39:19
def test_reference_registers():
    regs, _ = on_fake(VL53L1XRegs)
    assert regs.reg_read(regs.REG_MODEL_ID) == regs.MODEL_ID
    assert regs.reg_read(regs.REG_MODULE_TYPE) == regs.MODULE_TYPE
    assert regs.reg_read(regs.REG_MASK_REVISION) == regs.MASK_REVISION
    assert regs.reg_read(regs.REG_MODEL_ID, 2) == regs.ID_WORD

# Source: vl53l1x_devNB.ipynb | Cell In[28] | 2026-11-16 13:39:20
STATUS_CASES = [(9, 0, True), (6, 1, False), (4, 2, False), (5, 4, False), (7, 7, False),
                (8, 3, False), (0, 255, False)]  # fmt: skip
RESULT_BLOCK = bytes([0x09, 0, 0, 42, 0, 0, 0, 0, 50, 0, 0, 0, 0, 0, 243, 0x03, 0xE8])

# Source: vl53l1x_devNB.ipynb | Cell In[31] | 2026-11-16 13:39:20
@pytest.mark.parametrize("raw, code, valid", STATUS_CASES)
def test_status_decode(raw, code, valid):
    r = VL53L1XResult(raw, 100)
    assert (r.status, r.valid) == (code, valid)
    assert r.status_name == VL53L1XResult.status_decode(raw)[1]

# Source: vl53l1x_devNB.ipynb | Cell In[32] | 2026-11-16 13:39:20
def test_result_block_decodes():
    r = VL53L1XResult.from_block(RESULT_BLOCK)
    assert (r.valid, r.distance_mm, r.spads) == (True, 243, 42)
    assert (r.signal_kcps, r.ambient_kcps) == (1000 * 8, 50 * 8)
    with pytest.raises(ValueError, match="16 bytes"):
        VL53L1XResult.from_block(RESULT_BLOCK[:16])

# Source: vl53l1x_devNB.ipynb | Cell In[34] | 2026-11-16 13:39:20
OFFSET_CASES = [(0, 0x0000), (25, 0x0064), (-25, 0xFF9C), (1023, 0x0FFC), (-1024, 0xF000),
                (12.25, 0x0031)]  # fmt: skip
ROI_CASES = [(16, 16, 0xFF), (4, 4, 0x33), (8, 4, 0x37), (4, 16, 0xF3)]
XTALK_CASES = [(0, 0x0000), (1000, 0x0200), (64000, 0x8000)]
IM_CASES = [(100, 0x01F4, 53750), (33, 0x01F4, 17737), (100, 0xFDF4, 53750)]

# Source: vl53l1x_devNB.ipynb | Cell In[37] | 2026-11-16 13:39:20
@pytest.mark.parametrize("mm, word", OFFSET_CASES)
def test_offset_codec(mm, word):
    assert VL53L1X.offset_encode(mm) == word
    assert VL53L1X.offset_decode(word) == mm
    for bad in (-1025, 1024):
        with pytest.raises(ValueError):
            VL53L1X.offset_encode(bad)

# Source: vl53l1x_devNB.ipynb | Cell In[38] | 2026-11-16 13:39:21
@pytest.mark.parametrize("width, height, byte", ROI_CASES)
def test_roi_codec(width, height, byte):
    assert VL53L1X.roi_encode(width, height) == byte
    assert VL53L1X.roi_decode(byte) == (width, height)
    for bad in ((3, 4), (4, 17)):
        with pytest.raises(ValueError, match="4..16"):
            VL53L1X.roi_encode(*bad)

# Source: vl53l1x_devNB.ipynb | Cell In[39] | 2026-11-16 13:39:21
@pytest.mark.parametrize("cps, word", XTALK_CASES)
def test_xtalk_codec(cps, word):
    assert VL53L1X.xtalk_encode(cps) == word
    assert VL53L1X.xtalk_decode(word) == cps
    for bad in (-1, VL53L1X.XTALK_MAX_CPS + 1):
        with pytest.raises(ValueError):
            VL53L1X.xtalk_encode(bad)

# Source: vl53l1x_devNB.ipynb | Cell In[40] | 2026-11-16 13:39:21
@pytest.mark.parametrize("ms, osc, word", IM_CASES)
def test_inter_measurement_codec(ms, osc, word):
    assert VL53L1X.inter_measurement_word(ms, osc) == word
    assert VL53L1X.inter_measurement_from_word(word, osc) == ms
    with pytest.raises(ValueError, match="oscillator"):
        VL53L1X.inter_measurement_word(ms, 0xFC00)  # low 10 bits are 0

# Source: vl53l1x_devNB.ipynb | Cell In[43] | 2026-11-16 13:39:21
def test_init_identifies_and_configures():
    tof, part = on_fake(VL53L1X)
    assert tof.ident == 0xEACC and not tof.ranging and part.mode == 0x00
    assert (tof.distance_mode, tof.timing_budget_ms, tof.inter_measurement_ms) == ("long", 100, 100)
    assert tof.roi == (16, 16) and tof.roi_center == 199 and tof.offset_mm == 0

# Source: vl53l1x_devNB.ipynb | Cell In[44] | 2026-11-16 13:39:21
def test_init_refuses_a_part_that_is_not_a_vl53l1x():
    bus = FakeBus()
    bus.attach(0x29, FakeVL53L4CD())
    with pytest.raises(RuntimeError, match="ID 0xebaa"):
        VL53L1X(I2CConnection(bus, 0x29))

# Source: vl53l1x_devNB.ipynb | Cell In[45] | 2026-11-16 13:39:22
def test_init_times_out_without_boot():
    with pytest.raises(OSError, match="boot timed out"):
        on_fake(VL53L1X, booted=False)

# Source: vl53l1x_devNB.ipynb | Cell In[46] | 2026-11-16 13:39:22
def test_init_releases_a_part_left_in_reset():
    bus = FakeBus()
    part = bus.attach(0x29, FakeVL53L1X())
    bus.writeto_mem(0x29, 0x0000, b"\x00", addrsize=16)  # asserted, never released
    assert bus.readfrom_mem(0x29, 0x010F, 2, addrsize=16) == b"\x00\x00"
    tof = VL53L1X(I2CConnection(bus, 0x29))
    assert tof.ident == 0xEACC and part.regs[0x0000] == 0x01 and tof.measure().valid

# Source: vl53l1x_devNB.ipynb | Cell In[47] | 2026-11-16 13:39:22
def test_continuous_ranging():
    tof, part = on_fake(VL53L1X, distance_mm=1234)
    before = part.samples
    tof.start()
    results = [tof.read() for _ in range(3)]
    tof.stop()
    assert all(r.valid and r.distance_mm == 1234 for r in results)
    assert part.samples - before >= 3 and part.mode == 0x00 and not tof.ranging  # +1 if slow
    assert tof.last is results[-1]

# Source: vl53l1x_devNB.ipynb | Cell In[48] | 2026-11-16 13:39:22
def test_one_shot_measure():
    tof, part = on_fake(VL53L1X, distance_mm=87)
    r = tof.measure()
    assert r.valid and r.distance_mm == 87
    assert part.mode == 0x00 and not tof.data_ready()

# Source: vl53l1x_devNB.ipynb | Cell In[49] | 2026-11-16 13:39:23
def test_read_times_out_when_not_ranging():
    tof, _ = on_fake(VL53L1X)
    with pytest.raises(OSError, match="timed out"):
        tof.read(timeout_ms=20)

# Source: vl53l1x_devNB.ipynb | Cell In[50] | 2026-11-16 13:39:23
def test_invalid_status_is_reported():
    tof, _ = on_fake(VL53L1X, status=4)
    r = tof.measure()
    assert not r.valid and (r.status, r.status_name) == (2, "signal failure")

# Source: vl53l1x_devNB.ipynb | Cell In[51] | 2026-11-16 13:39:23
def test_distance_mode_switch_keeps_the_budget():
    tof, _ = on_fake(VL53L1X)
    tof.distance_mode = "short"
    assert tof.distance_mode == "short" and tof.timing_budget_ms == 100
    assert tof.reg_read(tof.REG_TIMEOUT_A, 2) == tof.TIMING_BUDGETS["short"][100][0]
    assert tof.reg_read(tof.REG_WOI_SD0, 2) == 0x0705

# Source: vl53l1x_devNB.ipynb | Cell In[52] | 2026-11-16 13:39:23
def test_budget_outside_the_mode_is_refused():
    tof, part = on_fake(VL53L1X)
    with pytest.raises(ValueError, match="not available in long mode"):
        tof.timing_budget_ms = 20
    tof.distance_mode = "short"
    tof.timing_budget_ms = 20
    writes = len(part.writes)
    with pytest.raises(ValueError, match="20 ms not available in long"):
        tof.distance_mode = "long"
    assert len(part.writes) == writes and tof.distance_mode == "short"

# Source: vl53l1x_devNB.ipynb | Cell In[53] | 2026-11-16 13:39:24
def test_settings_refused_while_ranging():
    tof, _ = on_fake(VL53L1X)
    tof.start()
    for name, value in (("timing_budget_ms", 50), ("distance_mode", "short"),
                        ("inter_measurement_ms", 200), ("offset_mm", 5)):  # fmt: skip
        with pytest.raises(RuntimeError, match="stop ranging"):
            setattr(tof, name, value)
    with pytest.raises(RuntimeError, match="stop ranging"):
        tof.measure()
    tof.stop()
    tof.timing_budget_ms = 50
    assert tof.timing_budget_ms == 50

# Source: vl53l1x_devNB.ipynb | Cell In[54] | 2026-11-16 13:39:24
def test_inter_measurement_period():
    tof, _ = on_fake(VL53L1X)
    tof.inter_measurement_ms = 250
    assert tof.inter_measurement_ms == 250
    with pytest.raises(ValueError, match="shorter than the timing budget"):
        tof.inter_measurement_ms = 50
    tof.timing_budget_ms = 500
    assert tof.inter_measurement_ms == 500

# Source: vl53l1x_devNB.ipynb | Cell In[55] | 2026-11-16 13:39:24
def test_roi_set():
    tof, _ = on_fake(VL53L1X)
    tof.roi_set(4, 4)
    assert tof.roi == (4, 4) and tof.roi_center == 199
    tof.roi_set(8, 8, center=167)
    assert tof.roi == (8, 8) and tof.roi_center == 167
    with pytest.raises(ValueError, match="default centre"):
        tof.roi_set(16, 16, center=167)

# Source: vl53l1x_devNB.ipynb | Cell In[56] | 2026-11-16 13:39:24
def test_offset_is_applied():
    tof, _ = on_fake(VL53L1X, distance_mm=300)
    tof.offset_mm = -12.5
    assert tof.offset_mm == -12.5
    assert tof.measure().distance_mm == 287

# Source: vl53l1x_devNB.ipynb | Cell In[57] | 2026-11-16 13:39:24
def test_xtalk_set():
    tof, _ = on_fake(VL53L1X)
    tof.xtalk_cps = 1000
    assert tof.xtalk_cps == 1000
    assert tof.reg_read(tof.REG_XTALK_X_GRADIENT, 2) == 0

# Source: vl53l1x_devNB.ipynb | Cell In[58] | 2026-11-16 13:39:25
def test_temperature_update():
    tof, part = on_fake(VL53L1X)
    before = part.samples
    tof.temperature_update()
    assert part.samples > before and part.mode == 0x00 and not tof.ranging
    assert (tof.reg_read(tof.REG_VHV_LOOP_BOUND), tof.reg_read(tof.REG_VHV_START)) == (0x09, 0)

# Source: vl53l1x_devNB.ipynb | Cell In[59] | 2026-11-16 13:39:25
def test_reset_restores_the_constructor_settings():
    tof, part = on_fake(VL53L1X, dev_kw={"distance_mode": "short", "timing_budget_ms": 50})
    tof.timing_budget_ms = 200
    tof.reset()
    assert (0x0000, b"\x00") in part.writes and tof.booted()
    assert (tof.distance_mode, tof.timing_budget_ms, tof.inter_measurement_ms) == ("short", 50, 50)
    assert tof.measure().valid

# Source: vl53l1x_devNB.ipynb | Cell In[60] | 2026-11-16 13:39:25
def test_address_change():
    tof, _ = on_fake(VL53L1X)
    bus = tof.conn.bus
    tof.address_set(0x30)
    assert bus.scan() == [0x30] and tof.addr == 0x30 and tof.measure().valid
    with pytest.raises(RuntimeError, match="default address"):
        tof.reset()
    tof.address_set(0x29)
    assert bus.scan() == [0x29]
    with pytest.raises(ValueError):
        tof.address_set(0x78)

# Source: vl53l1x_devNB.ipynb | Cell In[63] | 2026-11-16 13:39:25
def test_follows_a_hand_approaching_in_simulated_time():
    hand = ramp(1200, 100, 0, 60)
    with SimClock() as clock:
        tof, _ = on_fake(VL53L1X, distance_mm=hand, sample_ms=100)
        tof.start()
        readings = []
        for _ in range(7):
            r = tof.read()
            assert r.valid and abs(r.distance_mm - hand(clock.now_s())) <= 5
            readings.append(r.distance_mm)
            clock.advance(10)
        tof.stop()
    assert readings == sorted(readings, reverse=True) and readings[-1] <= 105

# Source: vl53l1x_devNB.ipynb | Cell In[64] | 2026-11-16 13:39:26
def test_one_shot_waits_for_the_sample_in_simulated_time():
    with SimClock() as clock:
        tof, _ = on_fake(VL53L1X, distance_mm=lambda t: 1000 - 10 * t, sample_ms=100)
        t0 = clock.now_s()
        r = tof.measure()
        took = clock.now_s() - t0
    assert 0.1 <= took <= 0.11
    assert r.valid and abs(r.distance_mm - (1000 - 10 * (t0 + took))) <= 1

