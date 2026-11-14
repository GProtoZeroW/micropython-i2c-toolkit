import pytest

from micropython_i2c import I2CConnection
from micropython_i2c.drivers.shtc3 import SHTC3, SHTC3Cmds
from micropython_i2c.fakes import FakeBus, FakeRegisterPart, FakeSHTC3
# Source: drivers_shtc3_devNB.ipynb | Cell In[9] | 2026-11-14 14:56:49
def on_fake(cls, **part_kw):
    """``(dev, part)``: ``cls`` bound to a fresh ``FakeSHTC3`` at 0x70 on a fake bus."""
    bus = FakeBus()
    part = bus.attach(0x70, FakeSHTC3(**part_kw))
    return cls(I2CConnection(bus, 0x70)), part

# Source: drivers_shtc3_devNB.ipynb | Cell In[12] | 2026-11-14 14:56:49
def test_crc_vectors_match_datasheet():
    for data, want in SHTC3Cmds.CRC_VECTORS:
        assert SHTC3Cmds.crc(data) == want

# Source: drivers_shtc3_devNB.ipynb | Cell In[13] | 2026-11-14 14:56:50
def test_measurement_command_table():
    assert len(set(SHTC3Cmds.CMD_MEASURE.values())) == 8
    assert SHTC3Cmds.CMD_MEASURE[(False, False, False)] == 0x7866
    assert SHTC3Cmds.CMD_MEASURE[(True, True, True)] == 0x44DE

# Source: drivers_shtc3_devNB.ipynb | Cell In[14] | 2026-11-14 14:56:50
def test_datasheet_example_packet_decodes():
    p = SHTC3Cmds.EXAMPLE_PACKET
    assert SHTC3Cmds.crc(p[0:2]) == p[2] and SHTC3Cmds.crc(p[3:5]) == p[5]
    rh_word, t_word = (p[0] << 8) | p[1], (p[3] << 8) | p[4]
    # Section 5.11's formulas, written out (the driver's codecs are tested below).
    assert 100 * rh_word / 65536 == pytest.approx(63.0, abs=0.05)
    assert -45 + 175 * t_word / 65536 == pytest.approx(23.7, abs=0.05)

# Source: drivers_shtc3_devNB.ipynb | Cell In[15] | 2026-11-14 14:56:50
def test_address_and_clock_limits():
    assert SHTC3Cmds.addr_ok(0x70) and not SHTC3Cmds.addr_ok(0x71)
    assert SHTC3Cmds.baud_ok(1_000_000) and not SHTC3Cmds.baud_ok(3_400_000)

# Source: drivers_shtc3_devNB.ipynb | Cell In[16] | 2026-11-14 14:56:51
def test_words_read_rejects_a_bad_crc():
    cmds, part = on_fake(SHTC3Cmds)
    cmds.cmd_send(cmds.CMD_READ_ID)
    part.corrupt = 1
    with pytest.raises(ValueError, match="word 0"):
        cmds.words_read(1)

# Source: drivers_shtc3_devNB.ipynb | Cell In[18] | 2026-11-14 14:56:51
CODEC_CASES = [(0x0000, -45.0, 0.0), (0x8000, 42.5, 50.0), (0xFFFF, 129.99733, 99.99847),
               (0x648B, 23.73055, 39.27460)]  # fmt: skip

# Source: drivers_shtc3_devNB.ipynb | Cell In[21] | 2026-11-14 14:56:51
@pytest.mark.parametrize("word, t, rh", CODEC_CASES)
def test_codecs(word, t, rh):
    assert SHTC3.celsius_from_raw(word) == pytest.approx(t, abs=1e-4)
    assert SHTC3.humidity_from_raw(word) == pytest.approx(rh, abs=1e-4)
    assert SHTC3.celsius_from_raw(SHTC3.celsius_to_raw(t)) == pytest.approx(t, abs=175 / 65536)
    assert SHTC3.humidity_from_raw(SHTC3.humidity_to_raw(rh)) == pytest.approx(rh, abs=100 / 65536)

# Source: drivers_shtc3_devNB.ipynb | Cell In[22] | 2026-11-14 14:56:52
@pytest.mark.parametrize(
    "fn, value",
    [("celsius_to_raw", -50.0), ("celsius_to_raw", 131.0), ("humidity_to_raw", -1.0),
     ("humidity_to_raw", 101.0)],
)  # fmt: skip
def test_codecs_refuse_out_of_range(fn, value):
    with pytest.raises(ValueError):
        getattr(SHTC3, fn)(value)

# Source: drivers_shtc3_devNB.ipynb | Cell In[25] | 2026-11-14 14:56:52
def test_identify_and_init_sleeps():
    sht, part = on_fake(SHTC3, id_word=0x5887)  # unspecified bits set
    assert sht.ident == 0x5887 and sht.identify()
    assert part.asleep and part.commands[-1] == SHTC3.CMD_SLEEP

# Source: drivers_shtc3_devNB.ipynb | Cell In[26] | 2026-11-14 14:56:52
def test_init_refuses_a_part_that_is_not_an_shtc3():
    with pytest.raises(RuntimeError, match="ID 0x1234"):
        on_fake(SHTC3, id_word=0x1234)
    bus = FakeBus()
    bus.attach(0x70, FakeRegisterPart())
    with pytest.raises(RuntimeError, match="not an SHTC3"):
        SHTC3(I2CConnection(bus, 0x70))

# Source: drivers_shtc3_devNB.ipynb | Cell In[27] | 2026-11-14 14:56:53
def test_measure_normal():
    sht, part = on_fake(SHTC3, temp_c=23.7, rh=63.0)
    t, rh = sht.measure()
    assert t == pytest.approx(23.7, abs=0.01) and rh == pytest.approx(63.0, abs=0.01)
    assert part.commands[-3:] == [SHTC3.CMD_WAKEUP, 0x7866, SHTC3.CMD_SLEEP]
    assert part.asleep

# Source: drivers_shtc3_devNB.ipynb | Cell In[28] | 2026-11-14 14:56:53
def test_measure_low_power():
    sht, part = on_fake(SHTC3, temp_c=-10.0, rh=5.0)
    t, rh = sht.measure(low_power=True)
    assert t == pytest.approx(-10.0, abs=0.01) and rh == pytest.approx(5.0, abs=0.01)
    assert part.commands[-2] == 0x609C

# Source: drivers_shtc3_devNB.ipynb | Cell In[29] | 2026-11-14 14:56:54
def test_measure_waits_long_enough():
    sht, _ = on_fake(SHTC3)
    for _ in range(10):
        assert sht.measure() == pytest.approx((25.0, 50.0), abs=0.01)

# Source: drivers_shtc3_devNB.ipynb | Cell In[30] | 2026-11-14 14:56:54
def test_always_back_to_sleep_even_on_crc_error():
    sht, part = on_fake(SHTC3)
    part.corrupt = 1
    with pytest.raises(ValueError, match="checksum"):
        sht.measure()
    assert part.asleep and part.commands[-1] == SHTC3.CMD_SLEEP

# Source: drivers_shtc3_devNB.ipynb | Cell In[31] | 2026-11-14 14:56:54
def test_commands_while_asleep_are_refused():
    sht, part = on_fake(SHTC3)
    assert part.asleep
    with pytest.raises(OSError):
        sht.cmd_send(SHTC3.CMD_READ_ID)
    with pytest.raises(OSError):  # straight after the refused command
        sht.cmd_send(SHTC3.CMD_WAKEUP)
    assert sht.identify()  # wake() retries

# Source: drivers_shtc3_devNB.ipynb | Cell In[32] | 2026-11-14 14:56:55
def test_soft_reset_leaves_it_asleep():
    sht, part = on_fake(SHTC3)
    sht.soft_reset()
    assert part.commands[-2:] == [SHTC3.CMD_SOFT_RESET, SHTC3.CMD_SLEEP]
    assert part.asleep and sht.identify()

