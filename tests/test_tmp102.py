import pytest

from micropython_i2c import I2CConnection
from micropython_i2c.drivers.tmp102 import TMP102, TMP102Regs
from micropython_i2c.fakes import FakeBus, FakeTMP102
# Source: drivers_tmp102_devNB.ipynb | Cell In[9] | 2026-11-13 22:11:14
def on_fake(cls, temp_word=0x1900):
    """Bind ``cls`` to a ``FakeTMP102`` at 0x48 on a fresh fake bus (25 C by default)."""
    bus = FakeBus()
    bus.attach(0x48, FakeTMP102(temp_word=temp_word))
    return cls(I2CConnection(bus, 0x48))

# Source: drivers_tmp102_devNB.ipynb | Cell In[12] | 2026-11-13 22:11:14
def test_fields_tile_the_data_bits():
    covered = 0
    for name in ("F_OS", "F_RES", "F_FQ", "F_POL", "F_TM", "F_SD", "F_CR", "F_AL", "F_EM"):
        mask = TMP102Regs.field_mask(getattr(TMP102Regs, name))
        assert covered & mask == 0
        covered |= mask
    assert covered == 0xFFF0  # the low nibble is reserved

# Source: drivers_tmp102_devNB.ipynb | Cell In[13] | 2026-11-13 22:11:14
def test_config_verify_mask_excludes_os_res_al():
    m = TMP102Regs.field_mask
    excluded = m(TMP102Regs.F_OS) | m(TMP102Regs.F_RES) | m(TMP102Regs.F_AL)
    assert 0xFFFF & ~TMP102Regs.CONFIG_VERIFY_MASK == excluded
    assert TMP102Regs.VERIFY_MASKS == {TMP102Regs.REG_CONFIG: TMP102Regs.CONFIG_VERIFY_MASK}

# Source: drivers_tmp102_devNB.ipynb | Cell In[14] | 2026-11-13 22:11:15
def test_por_values_decode_as_the_datasheet_says():
    x = TMP102Regs.field_extract
    assert x(TMP102Regs.CFG_POR, TMP102Regs.F_RES) == 0b11
    assert TMP102Regs.CONV_RATES[x(TMP102Regs.CFG_POR, TMP102Regs.F_CR)] == 4.0
    assert x(TMP102Regs.CFG_POR, TMP102Regs.F_AL) == 1

# Source: drivers_tmp102_devNB.ipynb | Cell In[15] | 2026-11-13 22:11:15
def test_masked_write_lands_and_exact_write_is_refused():
    regs = on_fake(TMP102Regs)
    sabotaged = regs.CFG_POR & ~regs.field_mask(regs.F_RES)
    with pytest.raises(ValueError, match="mask 0xffff"):
        regs.reg_set(regs.REG_CONFIG, sabotaged, verify_mask=0xFFFF)
    assert regs.reg_set(regs.REG_CONFIG, sabotaged) == regs.CFG_POR

# Source: drivers_tmp102_devNB.ipynb | Cell In[16] | 2026-11-13 22:11:15
def test_address_and_clock_limits():
    assert all(TMP102Regs.addr_ok(a) for a in (0x48, 0x49, 0x4A, 0x4B))
    assert not TMP102Regs.addr_ok(0x70)
    assert TMP102Regs.baud_ok(100_000) and not TMP102Regs.baud_ok(1_000_000)

# Source: drivers_tmp102_devNB.ipynb | Cell In[18] | 2026-11-13 22:11:16
CASES_12 = [(127.9375, 0x7FF0), (100.0, 0x6400), (75.0, 0x4B00), (50.0, 0x3200),
            (25.0, 0x1900), (0.25, 0x0040), (0.0, 0x0000), (-0.25, 0xFFC0),
            (-25.0, 0xE700), (-55.0, 0xC900)]  # fmt: skip
CASES_13 = [(150.0, 0x4B00), (128.0, 0x4000), (100.0, 0x3200), (25.0, 0x0C80),
            (0.0, 0x0000), (-25.0, 0xF380), (-55.0, 0xE480)]  # fmt: skip

# Source: drivers_tmp102_devNB.ipynb | Cell In[22] | 2026-11-13 22:11:16
@pytest.mark.parametrize("celsius, word", CASES_12)
def test_codec_12bit_matches_datasheet(celsius, word):
    assert TMP102.celsius_to_raw(celsius, False) == word
    assert TMP102.celsius_from_raw(word, False) == celsius

# Source: drivers_tmp102_devNB.ipynb | Cell In[23] | 2026-11-13 22:11:16
@pytest.mark.parametrize("celsius, word", CASES_13)
def test_codec_13bit_matches_datasheet(celsius, word):
    assert TMP102.celsius_to_raw(celsius, True) == word
    assert TMP102.celsius_from_raw(word, True) == celsius

# Source: drivers_tmp102_devNB.ipynb | Cell In[24] | 2026-11-13 22:11:17
@pytest.mark.parametrize("celsius, em", [(200.0, False), (200.0, True), (-100.0, False),
                                         (-100.0, True)])  # fmt: skip
def test_codec_refuses_out_of_range(celsius, em):
    with pytest.raises(ValueError, match="representable|operating range"):
        TMP102.celsius_to_raw(celsius, em)

# Source: drivers_tmp102_devNB.ipynb | Cell In[25] | 2026-11-13 22:11:17
def test_sign_extension_needs_the_shift_first():
    # 0xE700 is -25 C; extending the unshifted word would treat padding as magnitude.
    assert TMP102.celsius_from_raw(0xE700, False) == -25.0
    assert TMP102.sign_extend(0xE700, 12) * TMP102.LSB_C != -25.0

# Source: drivers_tmp102_devNB.ipynb | Cell In[30] | 2026-11-13 22:11:18
def test_reads_and_threshold_round_trip():
    tmp = on_fake(TMP102)
    assert tmp.temperature() == 25.0
    assert (tmp.low_limit_get(), tmp.high_limit_get()) == (75.0, 80.0)
    assert tmp.limits_set(20.0, 30.0) == (20.0, 30.0)
    assert abs(tmp.low_limit_set(20.03) - 20.03) <= tmp.LSB_C / 2
    tmp.limits_reset()
    assert tmp.high_limit_get() == 80.0

# Source: drivers_tmp102_devNB.ipynb | Cell In[31] | 2026-11-13 22:11:18
def test_crossed_limits_are_refused():
    tmp = on_fake(TMP102)
    with pytest.raises(ValueError, match="must be below"):
        tmp.limits_set(80.0, 20.0)

# Source: drivers_tmp102_devNB.ipynb | Cell In[32] | 2026-11-13 22:11:19
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

# Source: drivers_tmp102_devNB.ipynb | Cell In[33] | 2026-11-13 22:11:19
def test_extended_mode_is_tracked_and_refreshable():
    tmp = on_fake(TMP102)
    tmp.extended_mode = True
    assert tmp.em is True
    tmp.em = False  # deliberately stale
    assert tmp.mode_refresh() is True
    tmp.extended_mode = False
    assert tmp.em is False

# Source: drivers_tmp102_devNB.ipynb | Cell In[34] | 2026-11-13 22:11:20
def test_extended_mode_switch_waits_for_the_new_format():
    tmp = on_fake(TMP102)
    before = tmp.temperature()
    tmp.extended_mode = True
    assert tmp.temperature() == before
    tmp.extended_mode = False
    assert tmp.temperature() == before

# Source: drivers_tmp102_devNB.ipynb | Cell In[35] | 2026-11-13 22:11:21
def test_extended_mode_switch_in_shutdown_needs_a_one_shot():
    tmp = on_fake(TMP102)
    tmp.shutdown = True
    tmp.extended_mode = True
    with pytest.raises(RuntimeError, match="one_shot"):
        tmp.temperature()
    assert tmp.one_shot() == 25.0
    assert tmp.temperature() == 25.0

# Source: drivers_tmp102_devNB.ipynb | Cell In[36] | 2026-11-13 22:11:21
def test_one_shot_needs_shutdown_mode():
    tmp = on_fake(TMP102)
    with pytest.raises(RuntimeError, match="shutdown"):
        tmp.one_shot()
    tmp.shutdown = True
    assert tmp.one_shot() == 25.0

