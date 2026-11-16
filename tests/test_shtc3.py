import pytest

from micropython_i2c import I2CConnection
from micropython_i2c.devices.sensors.environmental.shtc3 import SHTC3, SHTC3Cmds
from micropython_i2c.devices.sensors.environmental.shtc3_fake import FakeSHTC3
from micropython_i2c.sim.bus import FakeBus, FakeRegisterPart
from micropython_i2c.sim.clock import SimClock
from micropython_i2c.sim.signals import ramp
# Source: shtc3_devNB.ipynb | Cell In[11] | 2026-11-16 13:35:04
def send(part, cmd):
    """Write the 16-bit command ``cmd`` to ``part``, MSB first."""
    part.raw_write(bytes([cmd >> 8, cmd & 0xFF]))


def words_of(packet):
    """The 16-bit words of a read, each CRC checked (``AssertionError`` if one is wrong)."""
    words = []
    for i in range(0, len(packet), 3):
        assert FakeSHTC3.crc8(packet[i : i + 2]) == packet[i + 2], "bad CRC at byte {}".format(i)
        words.append((packet[i] << 8) | packet[i + 1])
    return words

# Source: shtc3_devNB.ipynb | Cell In[16] | 2026-11-16 13:35:04
def test_fake_reproduces_figure_7():
    with SimClock() as clock:
        part = FakeSHTC3(temp_c=-45 + 175 * 0x648B / 65536, rh=100 * 0xA133 / 65536)
        send(part, 0x58E0)  # normal, humidity first, no clock stretching
        clock.advance(0.011)
        assert part.raw_read(6) == b"\xa1\x33\x1c\x64\x8b\xc7"

# Source: shtc3_devNB.ipynb | Cell In[17] | 2026-11-16 13:35:05
def test_fake_measurement_commands_and_words():
    with SimClock() as clock:
        part = FakeSHTC3(temp_c=-10.0, rh=5.0)
        for cmd, (_, rh_first, _) in FakeSHTC3.MEASURE.items():
            send(part, cmd)
            clock.advance(0.011)
            words = words_of(part.raw_read(6))
            t_word, rh_word = words[::-1] if rh_first else words
            assert abs(-45 + 175 * t_word / 65536 - -10.0) <= 175 / 65536
            assert abs(100 * rh_word / 65536 - 5.0) <= 100 / 65536

# Source: shtc3_devNB.ipynb | Cell In[18] | 2026-11-16 13:35:05
def test_fake_clock_stretched_read_waits_for_the_result():
    with SimClock() as clock:
        part = FakeSHTC3()
        for cmd, low_power in ((0x7CA2, False), (0x6458, True)):
            start = clock.now_us
            send(part, cmd)
            clock.advance(0.0002)
            words_of(part.raw_read(6))
            assert clock.now_us - start == int(part.MEAS_MS[low_power] * 1000)

# Source: shtc3_devNB.ipynb | Cell In[19] | 2026-11-16 13:35:05
def test_fake_nacks_until_the_measurement_ends():
    with SimClock() as clock:
        part = FakeSHTC3()
        send(part, 0x7866)
        clock.advance(0.0109)
        with pytest.raises(OSError):
            part.raw_read(6)
        with pytest.raises(OSError):  # a command too
            send(part, part.WAKEUP)
        clock.advance(0.0001)
        assert words_of(part.raw_read(6)) == [0x6666, 0x8000]  # 25 C, 50 %RH

# Source: shtc3_devNB.ipynb | Cell In[20] | 2026-11-16 13:35:05
def test_fake_sleep_and_the_refusal_after_a_nack():
    with SimClock() as clock:
        part = FakeSHTC3()
        send(part, part.SLEEP)
        with pytest.raises(OSError):
            send(part, part.READ_ID)
        with pytest.raises(OSError):
            send(part, part.WAKEUP)
        clock.advance(part.REFUSE_MS / 1000)
        send(part, part.WAKEUP)
        assert not part.asleep
        send(part, part.WAKEUP)  # already awake: acknowledged

# Source: shtc3_devNB.ipynb | Cell In[21] | 2026-11-16 13:35:06
def test_fake_result_is_read_once():
    part = FakeSHTC3()
    send(part, part.READ_ID)
    assert words_of(part.raw_read(3)) == [0x0807]
    with pytest.raises(OSError):
        part.raw_read(3)
    send(part, part.READ_ID)
    send(part, part.SOFT_RESET)
    with pytest.raises(OSError):
        part.raw_read(3)

# Source: shtc3_devNB.ipynb | Cell In[22] | 2026-11-16 13:35:06
def test_fake_reads_signals_at_the_measurement_command():
    with SimClock() as clock:
        part = FakeSHTC3(temp_c=ramp(20.0, 30.0, 0, 100), rh=ramp(60.0, 40.0, 0, 100))
        clock.advance(50)
        send(part, 0x7CA2)
        t_word, rh_word = words_of(part.raw_read(6))
        assert abs(-45 + 175 * t_word / 65536 - 25.0) <= 175 / 65536
        assert abs(100 * rh_word / 65536 - 50.0) <= 100 / 65536

# Source: shtc3_devNB.ipynb | Cell In[25] | 2026-11-16 13:35:06
def on_fake(cls, **part_kw):
    """``(dev, part)``: ``cls`` bound to a fresh ``FakeSHTC3`` at 0x70 on a fake bus."""
    bus = FakeBus()
    part = bus.attach(0x70, FakeSHTC3(**part_kw))
    return cls(I2CConnection(bus, 0x70)), part

# Source: shtc3_devNB.ipynb | Cell In[28] | 2026-11-16 13:35:06
def test_crc_vectors_match_datasheet():
    for data, want in SHTC3Cmds.CRC_VECTORS:
        assert SHTC3Cmds.crc(data) == want

# Source: shtc3_devNB.ipynb | Cell In[29] | 2026-11-16 13:35:06
def test_measurement_command_table():
    assert len(set(SHTC3Cmds.CMD_MEASURE.values())) == 8
    assert SHTC3Cmds.CMD_MEASURE[(False, False, False)] == 0x7866
    assert SHTC3Cmds.CMD_MEASURE[(True, True, True)] == 0x44DE

# Source: shtc3_devNB.ipynb | Cell In[30] | 2026-11-16 13:35:07
def test_datasheet_example_packet_decodes():
    p = SHTC3Cmds.EXAMPLE_PACKET
    assert SHTC3Cmds.crc(p[0:2]) == p[2] and SHTC3Cmds.crc(p[3:5]) == p[5]
    rh_word, t_word = (p[0] << 8) | p[1], (p[3] << 8) | p[4]
    # Section 5.11's formulas, written out (the driver's codecs are tested below).
    assert 100 * rh_word / 65536 == pytest.approx(63.0, abs=0.05)
    assert -45 + 175 * t_word / 65536 == pytest.approx(23.7, abs=0.05)

# Source: shtc3_devNB.ipynb | Cell In[31] | 2026-11-16 13:35:07
def test_address_and_clock_limits():
    assert SHTC3Cmds.addr_ok(0x70) and not SHTC3Cmds.addr_ok(0x71)
    assert SHTC3Cmds.baud_ok(1_000_000) and not SHTC3Cmds.baud_ok(3_400_000)

# Source: shtc3_devNB.ipynb | Cell In[32] | 2026-11-16 13:35:07
def test_words_read_rejects_a_bad_crc():
    cmds, part = on_fake(SHTC3Cmds)
    cmds.cmd_send(cmds.CMD_READ_ID)
    part.corrupt = 1
    with pytest.raises(ValueError, match="word 0"):
        cmds.words_read(1)

# Source: shtc3_devNB.ipynb | Cell In[34] | 2026-11-16 13:35:08
CODEC_CASES = [(0x0000, -45.0, 0.0), (0x8000, 42.5, 50.0), (0xFFFF, 129.99733, 99.99847),
               (0x648B, 23.73055, 39.27460)]  # fmt: skip

# Source: shtc3_devNB.ipynb | Cell In[37] | 2026-11-16 13:35:08
@pytest.mark.parametrize("word, t, rh", CODEC_CASES)
def test_codecs(word, t, rh):
    assert SHTC3.celsius_from_raw(word) == pytest.approx(t, abs=1e-4)
    assert SHTC3.humidity_from_raw(word) == pytest.approx(rh, abs=1e-4)
    assert SHTC3.celsius_from_raw(SHTC3.celsius_to_raw(t)) == pytest.approx(t, abs=175 / 65536)
    assert SHTC3.humidity_from_raw(SHTC3.humidity_to_raw(rh)) == pytest.approx(rh, abs=100 / 65536)

# Source: shtc3_devNB.ipynb | Cell In[38] | 2026-11-16 13:35:08
@pytest.mark.parametrize(
    "fn, value",
    [("celsius_to_raw", -50.0), ("celsius_to_raw", 131.0), ("humidity_to_raw", -1.0),
     ("humidity_to_raw", 101.0)],
)  # fmt: skip
def test_codecs_refuse_out_of_range(fn, value):
    with pytest.raises(ValueError):
        getattr(SHTC3, fn)(value)

# Source: shtc3_devNB.ipynb | Cell In[41] | 2026-11-16 13:35:08
def test_identify_and_init_sleeps():
    sht, part = on_fake(SHTC3, id_word=0x5887)  # unspecified bits set
    assert sht.ident == 0x5887 and sht.identify()
    assert part.asleep and part.commands[-1] == SHTC3.CMD_SLEEP

# Source: shtc3_devNB.ipynb | Cell In[42] | 2026-11-16 13:35:08
def test_init_refuses_a_part_that_is_not_an_shtc3():
    with pytest.raises(RuntimeError, match="ID 0x1234"):
        on_fake(SHTC3, id_word=0x1234)
    bus = FakeBus()
    bus.attach(0x70, FakeRegisterPart())
    with pytest.raises(RuntimeError, match="not an SHTC3"):
        SHTC3(I2CConnection(bus, 0x70))

# Source: shtc3_devNB.ipynb | Cell In[43] | 2026-11-16 13:35:09
def test_measure_normal():
    sht, part = on_fake(SHTC3, temp_c=23.7, rh=63.0)
    t, rh = sht.measure()
    assert t == pytest.approx(23.7, abs=0.01) and rh == pytest.approx(63.0, abs=0.01)
    assert part.commands[-3:] == [SHTC3.CMD_WAKEUP, 0x7866, SHTC3.CMD_SLEEP]
    assert part.asleep

# Source: shtc3_devNB.ipynb | Cell In[44] | 2026-11-16 13:35:09
def test_measure_low_power():
    sht, part = on_fake(SHTC3, temp_c=-10.0, rh=5.0)
    t, rh = sht.measure(low_power=True)
    assert t == pytest.approx(-10.0, abs=0.01) and rh == pytest.approx(5.0, abs=0.01)
    assert part.commands[-2] == 0x609C

# Source: shtc3_devNB.ipynb | Cell In[45] | 2026-11-16 13:35:09
def test_measure_waits_long_enough():
    sht, _ = on_fake(SHTC3)
    for _ in range(10):
        assert sht.measure() == pytest.approx((25.0, 50.0), abs=0.01)

# Source: shtc3_devNB.ipynb | Cell In[46] | 2026-11-16 13:35:10
def test_always_back_to_sleep_even_on_crc_error():
    sht, part = on_fake(SHTC3)
    part.corrupt = 1
    with pytest.raises(ValueError, match="checksum"):
        sht.measure()
    assert part.asleep and part.commands[-1] == SHTC3.CMD_SLEEP

# Source: shtc3_devNB.ipynb | Cell In[47] | 2026-11-16 13:35:10
def test_commands_while_asleep_are_refused():
    sht, part = on_fake(SHTC3)
    assert part.asleep
    with pytest.raises(OSError):
        sht.cmd_send(SHTC3.CMD_READ_ID)
    with pytest.raises(OSError):  # straight after the refused command
        sht.cmd_send(SHTC3.CMD_WAKEUP)
    assert sht.identify()  # wake() retries

# Source: shtc3_devNB.ipynb | Cell In[48] | 2026-11-16 13:35:10
def test_soft_reset_leaves_it_asleep():
    sht, part = on_fake(SHTC3)
    sht.soft_reset()
    assert part.commands[-2:] == [SHTC3.CMD_SOFT_RESET, SHTC3.CMD_SLEEP]
    assert part.asleep and sht.identify()

# Source: shtc3_devNB.ipynb | Cell In[50] | 2026-11-16 13:35:11
def test_driver_follows_a_warming_drying_room():
    with SimClock() as clock:
        sht, _ = on_fake(SHTC3, temp_c=ramp(20.0, 26.0, 0, 3600), rh=ramp(60.0, 45.0, 0, 3600))
        for _ in range(7):
            minute = clock.now_s() / 60
            for low_power in (False, True):
                t, rh = sht.measure(low_power=low_power)
                assert abs(t - (20.0 + 6.0 * minute / 60)) <= 0.01
                assert abs(rh - (60.0 - 15.0 * minute / 60)) <= 0.01
            clock.advance(600)

# Source: shtc3_devNB.ipynb | Cell In[51] | 2026-11-16 13:35:11
def test_measure_waits_the_datasheet_time_in_simulated_time():
    with SimClock() as clock:
        sht, part = on_fake(SHTC3)
        for low_power in (False, True):
            start = clock.now_us
            sht.measure(low_power=low_power)
            assert clock.now_us - start >= SHTC3.T_WAKEUP_US + SHTC3.T_MEAS_US[low_power]
            assert part.asleep

