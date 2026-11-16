import pytest

from micropython_i2c import I2CConnection
from micropython_i2c.devices.sensors.environmental.tmp102 import TMP102, TMP102Regs
from micropython_i2c.devices.sensors.environmental.tmp102_fake import FakeTMP102
from micropython_i2c.sim.bus import FakeBus
from micropython_i2c.sim.clock import SimClock
from micropython_i2c.sim.signals import ramp
# Source: tmp102_devNB.ipynb | Cell In[11] | 2026-11-16 13:34:47
def temp_word(part):
    """The 16-bit temperature word ``part`` returns now."""
    return int.from_bytes(part.mem_read(0, 2), "big")

# Source: tmp102_devNB.ipynb | Cell In[14] | 2026-11-16 13:34:47
def test_fake_power_on_registers():
    part = FakeTMP102()
    assert [part.mem_read(r, 2) for r in (1, 2, 3)] == [b"\x60\xa0", b"\x4b\x00", b"\x50\x00"]
    assert temp_word(part) == 0x1900  # 25 C

# Source: tmp102_devNB.ipynb | Cell In[15] | 2026-11-16 13:34:47
@pytest.mark.parametrize("celsius, word_12, word_13", [
    (127.9375, 0x7FF0, 0x3FF8), (25.0, 0x1900, 0x0C80), (0.25, 0x0040, 0x0020),
    (-25.0, 0xE700, 0xF380), (-55.0, 0xC900, 0xE480)])  # fmt: skip
def test_fake_temperature_words_match_datasheet(celsius, word_12, word_13):
    part = FakeTMP102(temp_c=celsius)
    assert temp_word(part) == word_12
    part.mem_write(1, b"\x61\xb0")  # shutdown and EM on
    part.mem_write(1, b"\xe1\xb0")  # one-shot: a conversion in the 13-bit format
    assert temp_word(part) == word_13 | 1

# Source: tmp102_devNB.ipynb | Cell In[16] | 2026-11-16 13:34:47
def test_fake_saturates_outside_the_format():
    part = FakeTMP102(temp_c=140.0)
    assert temp_word(part) == 0x7FF0
    part.mem_write(1, b"\x61\xb0")
    part.mem_write(1, b"\xe1\xb0")
    assert temp_word(part) == 0x4600 | 1  # 140 C in 13 bits

# Source: tmp102_devNB.ipynb | Cell In[17] | 2026-11-16 13:34:47
def test_fake_read_only_bits_keep_their_value():
    part = FakeTMP102()
    part.mem_write(0, b"\x00\x00")
    assert temp_word(part) == 0x1900
    part.mem_write(1, b"\x00\x0f")
    assert part.regs[1] == 0x6020

# Source: tmp102_devNB.ipynb | Cell In[18] | 2026-11-16 13:34:48
def test_fake_os_follows_shutdown():
    part = FakeTMP102()
    assert not part.regs[1] & 0x8000
    part.mem_write(1, b"\x61\xa0")  # SD on
    assert part.regs[1] & 0x8000
    part.mem_write(1, b"\x60\xa0")  # SD off
    assert not part.regs[1] & 0x8000

# Source: tmp102_devNB.ipynb | Cell In[19] | 2026-11-16 13:34:48
def test_fake_em_format_lags_one_conversion():
    part = FakeTMP102()
    with SimClock() as clock:
        part.mem_write(1, b"\x60\xb0")  # EM on
        assert temp_word(part) == 0x1900 | 1
        clock.advance((part.LAG_MS - 1) / 1000)
        assert temp_word(part) == 0x1900 | 1
        clock.advance(0.001)
        assert temp_word(part) == 0x0C80 | 1

# Source: tmp102_devNB.ipynb | Cell In[20] | 2026-11-16 13:34:48
def test_fake_em_change_in_shutdown_waits_for_a_one_shot():
    part = FakeTMP102()
    with SimClock() as clock:
        part.mem_write(1, b"\x61\xa0")  # shutdown
        part.mem_write(1, b"\x61\xb0")  # EM on
        clock.advance(10.0)
        assert temp_word(part) == 0x1900 | 1
        part.mem_write(1, b"\xe1\xb0")  # one-shot
        assert temp_word(part) == 0x0C80 | 1

# Source: tmp102_devNB.ipynb | Cell In[21] | 2026-11-16 13:34:48
def test_fake_temperature_follows_a_signal_and_holds_in_shutdown():
    with SimClock() as clock:
        part = FakeTMP102(temp_c=ramp(20.0, 30.0, 0, 100))
        assert temp_word(part) == 0x1400  # 20 C
        clock.advance(50)
        assert temp_word(part) == 0x1900  # 25 C
        part.mem_write(1, b"\x61\xa0")  # shutdown: the 25 C conversion is kept
        clock.advance(50)
        assert temp_word(part) == 0x1900
        part.mem_write(1, b"\xe1\xa0")  # one-shot
        assert temp_word(part) == 0x1E00  # 30 C

# Source: tmp102_devNB.ipynb | Cell In[24] | 2026-11-16 13:34:49
def on_fake(cls, temp_c=25.0):
    """Bind ``cls`` to a ``FakeTMP102`` at 0x48 on a fresh fake bus (25 C by default).

    ``temp_c`` is a number or a signal.
    """
    bus = FakeBus()
    bus.attach(0x48, FakeTMP102(temp_c=temp_c))
    return cls(I2CConnection(bus, 0x48))

# Source: tmp102_devNB.ipynb | Cell In[27] | 2026-11-16 13:34:49
def test_fields_tile_the_data_bits():
    covered = 0
    for name in ("F_OS", "F_RES", "F_FQ", "F_POL", "F_TM", "F_SD", "F_CR", "F_AL", "F_EM"):
        mask = TMP102Regs.field_mask(getattr(TMP102Regs, name))
        assert covered & mask == 0
        covered |= mask
    assert covered == 0xFFF0  # the low nibble is reserved

# Source: tmp102_devNB.ipynb | Cell In[28] | 2026-11-16 13:34:49
def test_config_verify_mask_excludes_os_res_al():
    m = TMP102Regs.field_mask
    excluded = m(TMP102Regs.F_OS) | m(TMP102Regs.F_RES) | m(TMP102Regs.F_AL)
    assert 0xFFFF & ~TMP102Regs.CONFIG_VERIFY_MASK == excluded
    assert TMP102Regs.VERIFY_MASKS == {TMP102Regs.REG_CONFIG: TMP102Regs.CONFIG_VERIFY_MASK}

# Source: tmp102_devNB.ipynb | Cell In[29] | 2026-11-16 13:34:49
def test_por_values_decode_as_the_datasheet_says():
    x = TMP102Regs.field_extract
    assert x(TMP102Regs.CFG_POR, TMP102Regs.F_RES) == 0b11
    assert TMP102Regs.CONV_RATES[x(TMP102Regs.CFG_POR, TMP102Regs.F_CR)] == 4.0
    assert x(TMP102Regs.CFG_POR, TMP102Regs.F_AL) == 1

# Source: tmp102_devNB.ipynb | Cell In[30] | 2026-11-16 13:34:49
def test_masked_write_lands_and_exact_write_is_refused():
    regs = on_fake(TMP102Regs)
    sabotaged = regs.CFG_POR & ~regs.field_mask(regs.F_RES)
    with pytest.raises(ValueError, match="mask 0xffff"):
        regs.reg_set(regs.REG_CONFIG, sabotaged, verify_mask=0xFFFF)
    assert regs.reg_set(regs.REG_CONFIG, sabotaged) == regs.CFG_POR

# Source: tmp102_devNB.ipynb | Cell In[31] | 2026-11-16 13:34:50
def test_address_and_clock_limits():
    assert all(TMP102Regs.addr_ok(a) for a in (0x48, 0x49, 0x4A, 0x4B))
    assert not TMP102Regs.addr_ok(0x70)
    assert TMP102Regs.baud_ok(100_000) and not TMP102Regs.baud_ok(1_000_000)

# Source: tmp102_devNB.ipynb | Cell In[33] | 2026-11-16 13:34:50
CASES_12 = [(127.9375, 0x7FF0), (100.0, 0x6400), (75.0, 0x4B00), (50.0, 0x3200),
            (25.0, 0x1900), (0.25, 0x0040), (0.0, 0x0000), (-0.25, 0xFFC0),
            (-25.0, 0xE700), (-55.0, 0xC900)]  # fmt: skip
CASES_13 = [(150.0, 0x4B00), (128.0, 0x4000), (100.0, 0x3200), (25.0, 0x0C80),
            (0.0, 0x0000), (-25.0, 0xF380), (-55.0, 0xE480)]  # fmt: skip

# Source: tmp102_devNB.ipynb | Cell In[37] | 2026-11-16 13:34:50
@pytest.mark.parametrize("celsius, word", CASES_12)
def test_codec_12bit_matches_datasheet(celsius, word):
    assert TMP102.celsius_to_raw(celsius, False) == word
    assert TMP102.celsius_from_raw(word, False) == celsius

# Source: tmp102_devNB.ipynb | Cell In[38] | 2026-11-16 13:34:50
@pytest.mark.parametrize("celsius, word", CASES_13)
def test_codec_13bit_matches_datasheet(celsius, word):
    assert TMP102.celsius_to_raw(celsius, True) == word
    assert TMP102.celsius_from_raw(word, True) == celsius

# Source: tmp102_devNB.ipynb | Cell In[39] | 2026-11-16 13:34:51
@pytest.mark.parametrize("celsius, em", [(200.0, False), (200.0, True), (-100.0, False),
                                         (-100.0, True)])  # fmt: skip
def test_codec_refuses_out_of_range(celsius, em):
    with pytest.raises(ValueError, match="representable|operating range"):
        TMP102.celsius_to_raw(celsius, em)

# Source: tmp102_devNB.ipynb | Cell In[40] | 2026-11-16 13:34:51
def test_sign_extension_needs_the_shift_first():
    # 0xE700 is -25 C; extending the unshifted word would treat padding as magnitude.
    assert TMP102.celsius_from_raw(0xE700, False) == -25.0
    assert TMP102.sign_extend(0xE700, 12) * TMP102.LSB_C != -25.0

# Source: tmp102_devNB.ipynb | Cell In[45] | 2026-11-16 13:34:51
def test_reads_and_threshold_round_trip():
    tmp = on_fake(TMP102)
    assert tmp.temperature() == 25.0
    assert (tmp.low_limit_get(), tmp.high_limit_get()) == (75.0, 80.0)
    assert tmp.limits_set(20.0, 30.0) == (20.0, 30.0)
    assert abs(tmp.low_limit_set(20.03) - 20.03) <= tmp.LSB_C / 2
    tmp.limits_reset()
    assert tmp.high_limit_get() == 80.0

# Source: tmp102_devNB.ipynb | Cell In[46] | 2026-11-16 13:34:51
def test_crossed_limits_are_refused():
    tmp = on_fake(TMP102)
    with pytest.raises(ValueError, match="must be below"):
        tmp.limits_set(80.0, 20.0)

# Source: tmp102_devNB.ipynb | Cell In[47] | 2026-11-16 13:34:52
def test_config_properties_round_trip():
    tmp = on_fake(TMP102)
    for hz in (0.25, 1.0, 8.0, 4.0):
        tmp.conversion_rate = hz
        assert tmp.conversion_rate == hz
    for n in (1, 2, 4, 6):
        tmp.fault_queue = n
        assert tmp.fault_queue == n
    for attr in ("thermostat_mode", "alert_polarity_high", "shutdown"):
        for state in (True, False):
            setattr(tmp, attr, state)
            assert getattr(tmp, attr) is state

# Source: tmp102_devNB.ipynb | Cell In[48] | 2026-11-16 13:34:52
def test_extended_mode_is_tracked_and_refreshable():
    tmp = on_fake(TMP102)
    tmp.extended_mode = True
    assert tmp.em is True
    tmp.em = False  # deliberately stale
    assert tmp.mode_refresh() is True
    tmp.extended_mode = False
    assert tmp.em is False

# Source: tmp102_devNB.ipynb | Cell In[49] | 2026-11-16 13:34:53
def test_extended_mode_switch_waits_for_the_new_format():
    tmp = on_fake(TMP102)
    before = tmp.temperature()
    tmp.extended_mode = True
    assert tmp.temperature() == before
    tmp.extended_mode = False
    assert tmp.temperature() == before

# Source: tmp102_devNB.ipynb | Cell In[50] | 2026-11-16 13:34:54
def test_extended_mode_switch_in_shutdown_needs_a_one_shot():
    tmp = on_fake(TMP102)
    tmp.shutdown = True
    tmp.extended_mode = True
    with pytest.raises(RuntimeError, match="one_shot"):
        tmp.temperature()
    assert tmp.one_shot() == 25.0
    assert tmp.temperature() == 25.0

# Source: tmp102_devNB.ipynb | Cell In[51] | 2026-11-16 13:34:54
def test_one_shot_needs_shutdown_mode():
    tmp = on_fake(TMP102)
    with pytest.raises(RuntimeError, match="shutdown"):
        tmp.one_shot()
    tmp.shutdown = True
    assert tmp.one_shot() == 25.0

# Source: tmp102_devNB.ipynb | Cell In[53] | 2026-11-16 13:34:54
def test_driver_follows_a_warming_room():
    with SimClock() as clock:
        tmp = on_fake(TMP102, temp_c=ramp(20.0, 26.0, 0, 3600))
        for minute in range(0, 61, 10):
            want = 20.0 + 6.0 * minute / 60
            assert abs(tmp.temperature() - want) <= tmp.LSB_C / 2
            clock.advance(600)

# Source: tmp102_devNB.ipynb | Cell In[54] | 2026-11-16 13:34:54
def test_one_shot_reads_the_room_in_shutdown():
    with SimClock() as clock:
        tmp = on_fake(TMP102, temp_c=ramp(20.0, 26.0, 0, 3600))
        tmp.shutdown = True
        clock.advance(1800)
        assert tmp.temperature() == 20.0
        assert tmp.one_shot() == 23.0
        assert tmp.temperature() == 23.0

# Source: tmp102_devNB.ipynb | Cell In[55] | 2026-11-16 13:34:55
def test_extended_mode_switch_waits_in_simulated_time():
    with SimClock() as clock:
        tmp = on_fake(TMP102)
        start = clock.now_us
        tmp.extended_mode = True
        assert clock.now_us - start >= (250 + tmp.CONV_TIME_MAX_MS) * 1000
        assert tmp.temperature() == 25.0

