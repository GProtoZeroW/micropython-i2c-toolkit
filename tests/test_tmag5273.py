import math

import pytest

from micropython_i2c import I2CConnection
from micropython_i2c.devices.sensors.magnetic.tmag5273 import TMAG5273, TMAG5273Regs
from micropython_i2c.devices.sensors.magnetic.tmag5273_fake import FakeTMAG5273, crc8_07
from micropython_i2c.sim.bus import FakeBus, FakeMux, FakeRegisterPart
from micropython_i2c.sim.clock import SimClock
from micropython_i2c.sim.signals import ramp, sine
# Source: tmag5273_devNB.ipynb | Cell In[12] | 2026-11-16 13:35:36
def fake_part(addr=0x22, **part_kw):
    """``(bus, part)``: a fresh ``FakeTMAG5273`` at ``addr`` on a fake bus, no mux."""
    bus = FakeBus()
    part = bus.attach(addr, FakeTMAG5273(addr=addr, **part_kw))
    return bus, part

# Source: tmag5273_devNB.ipynb | Cell In[15] | 2026-11-16 13:35:36
@pytest.mark.parametrize(
    "data, want",
    [(b"\x00", 0xF3), (b"\xff", 0x00), (b"\x80", 0x7A), (b"\x4c", 0x10), (b"\xe0", 0x5D),
     (b"\x00\x00\x00\x00", 0xD1), (b"\xff\xff\xff\xff", 0x0F)],
)  # fmt: skip
def test_fake_crc_matches_datasheet(data, want):
    assert crc8_07(data) == want

# Source: tmag5273_devNB.ipynb | Cell In[16] | 2026-11-16 13:35:36
def test_fake_defaults_and_takes_addr():
    part = FakeTMAG5273()
    assert FakeTMAG5273.TAKES_ADDR is True
    assert (part.addr, part.version, part.regs[0x0C]) == (0x35, 1, 0x35 << 1)

# Source: tmag5273_devNB.ipynb | Cell In[17] | 2026-11-16 13:35:37
def test_fake_power_on_registers():
    bus, _ = fake_part(version=2)
    assert bus.readfrom_mem(0x22, 0x0C, 4) == bytes([0x22 << 1, 2, 0x49, 0x54])
    assert bus.readfrom_mem(0x22, 0x18, 1)[0] & 0x10  # POR
    assert bus.readfrom_mem(0x22, 0x1C, 1)[0] == 0x10  # INTB_RB only
    assert bus.readfrom_mem(0x22, 0x00, 0x0C) == bytes(0x0C)

# Source: tmag5273_devNB.ipynb | Cell In[18] | 2026-11-16 13:35:37
def test_fake_read_only_and_reserved_bits():
    bus, _ = fake_part()
    bus.writeto_mem(0x22, 0x03, b"\xff")  # SENSOR_CONFIG_2: bit 7 reserved
    bus.writeto_mem(0x22, 0x08, b"\xff")  # INT_CONFIG_1: bit 1 reserved
    bus.writeto_mem(0x22, 0x0D, b"\x07")  # DEVICE_ID
    bus.writeto_mem(0x22, 0x12, b"\x12\x34")  # X result
    assert bus.readfrom_mem(0x22, 0x03, 1) == b"\x7f"
    assert bus.readfrom_mem(0x22, 0x08, 1) == b"\xfd"
    assert bus.readfrom_mem(0x22, 0x0D, 1) == b"\x01"
    assert bus.readfrom_mem(0x22, 0x12, 2) == b"\x00\x00"

# Source: tmag5273_devNB.ipynb | Cell In[19] | 2026-11-16 13:35:37
def test_fake_status_bits_are_write_1_to_clear():
    bus, part = fake_part()
    part.regs[0x1C] |= 0x0F  # every error flag up
    bus.writeto_mem(0x22, 0x18, b"\x00")
    assert bus.readfrom_mem(0x22, 0x18, 1)[0] & 0x10
    bus.writeto_mem(0x22, 0x18, b"\x10")
    assert not bus.readfrom_mem(0x22, 0x18, 1)[0] & 0x10
    bus.writeto_mem(0x22, 0x1C, b"\xff")
    assert bus.readfrom_mem(0x22, 0x1C, 1) == b"\x10"

# Source: tmag5273_devNB.ipynb | Cell In[20] | 2026-11-16 13:35:37
def test_fake_converts_on_trigger_or_continuously():
    bus, part = fake_part()
    bus.writeto_mem(0x22, 0x02, b"\x70")  # XYZ, standby
    assert bus.readfrom_mem(0x22, 0x12, 2) == b"\x00\x00" and part.conversions == 0
    status = bus.readfrom_mem(0x22, 0x18 | 0x80, 1)[0]  # trigger, then read CONV_STATUS
    assert part.conversions == 1 and status & 0x01 and status >> 5 == 1
    assert bus.readfrom_mem(0x22, 0x12, 2) == b"\x10\x00"
    bus.writeto_mem(0x22, 0x01, b"\x02")  # continuous
    before = part.conversions
    bus.readfrom_mem(0x22, 0x12, 2)
    bus.readfrom_mem(0x22, 0x12, 2)
    assert part.conversions == before + 2

# Source: tmag5273_devNB.ipynb | Cell In[21] | 2026-11-16 13:35:38
def test_fake_result_words_follow_the_equations():
    bus, _ = fake_part(field_mT=(5.0, -5.0, 20.0), temp_c=26.0)
    bus.writeto_mem(0x22, 0x02, b"\x70")  # XYZ
    bus.writeto_mem(0x22, 0x07, b"\x01")  # T_CH_EN
    bus.writeto_mem(0x22, 0x01, b"\x02")  # continuous
    assert bus.readfrom_mem(0x22, 0x10, 8) == bytes.fromhex("449e 1000 f000 4000")
    bus.writeto_mem(0x22, 0x03, b"\x03")  # both ranges +-80 mT
    assert bus.readfrom_mem(0x22, 0x12, 6) == bytes.fromhex("0800 f800 2000")

# Source: tmag5273_devNB.ipynb | Cell In[22] | 2026-11-16 13:35:38
def test_fake_angle_and_offsets_use_the_pair():
    bus, _ = fake_part(field_mT=(5.0, -5.0, 0.0))
    bus.writeto_mem(0x22, 0x02, b"\x70")  # XYZ
    bus.writeto_mem(0x22, 0x03, b"\x04")  # ANGLE_EN = XY
    bus.writeto_mem(0x22, 0x01, b"\x02")  # continuous
    assert bus.readfrom_mem(0x22, 0x19, 2) == (315 << 4).to_bytes(2, "big")
    bus.writeto_mem(0x22, 0x0A, b"\x80")  # X offset -2.5 mT
    assert bus.readfrom_mem(0x22, 0x12, 2) == b"\x08\x00"  # 2.5 mT

# Source: tmag5273_devNB.ipynb | Cell In[23] | 2026-11-16 13:35:38
def test_fake_fast_packet_order_and_crc():
    bus, _ = fake_part(field_mT=(5.0, -5.0, 2.5))
    bus.writeto_mem(0x22, 0x02, b"\x50")  # MAG_CH_EN = ZX
    bus.writeto_mem(0x22, 0x01, b"\x02")  # continuous
    bus.writeto_mem(0x22, 0x00, b"\x81")  # CRC_EN, I2C_RD = fast16
    packet = bus.readfrom(0x22, 6)
    assert packet[:4] == bytes.fromhex("0800 1000")  # Z = 2.5 mT, then X = 5 mT
    assert packet[4] & 0x01  # CONV_STATUS.RESULT_STATUS
    assert packet[5] == crc8_07(bytes([(0x22 << 1) | 1]) + packet[:5])

# Source: tmag5273_devNB.ipynb | Cell In[24] | 2026-11-16 13:35:38
def test_fake_fast_crc_covers_overheard_address_bytes():
    bus = FakeBus()
    mux = bus.attach(0x70, FakeMux())
    mux.state = 0x01
    bus.attach(0x22, FakeTMAG5273(addr=0x22), path=[(0x70, 0)])
    bus.writeto_mem(0x22, 0x02, b"\x30")  # XY
    bus.writeto_mem(0x22, 0x01, b"\x02")  # continuous
    bus.writeto_mem(0x22, 0x00, b"\x81")  # CRC_EN, fast16
    bus.writeto(0x70, b"\x01")  # a channel select the part overhears
    packet = bus.readfrom(0x22, 6)
    assert packet[-1] == crc8_07(bytes([0x70 << 1, (0x22 << 1) | 1]) + packet[:-1])

# Source: tmag5273_devNB.ipynb | Cell In[25] | 2026-11-16 13:35:39
def test_fake_standard_read_crc_is_the_fifth_byte():
    bus, _ = fake_part()
    bus.writeto_mem(0x22, 0x00, b"\x80")  # CRC_EN, standard reads
    data = bus.readfrom_mem(0x22, 0x0C, 5)
    assert data[:4] == bytes([0x22 << 1, 1, 0x49, 0x54])
    assert data[4] == crc8_07(data[:4])
    assert len(bus.readfrom_mem(0x22, 0x0C, 4)) == 4

# Source: tmag5273_devNB.ipynb | Cell In[26] | 2026-11-16 13:35:39
def test_fake_register_read_in_fast_mode_returns_the_packet():
    bus, _ = fake_part()
    bus.writeto_mem(0x22, 0x02, b"\x30")  # XY
    bus.writeto_mem(0x22, 0x01, b"\x02")  # continuous
    bus.writeto_mem(0x22, 0x00, b"\x01")  # fast16, no CRC
    assert bus.readfrom_mem(0x22, 0x0D, 3) == bus.readfrom(0x22, 3) == b"\x10\x00\xf0"

# Source: tmag5273_devNB.ipynb | Cell In[27] | 2026-11-16 13:35:39
@pytest.mark.parametrize("mode", [1, 3])
def test_fake_sleep_nacks_the_waking_transfer(mode):
    bus, part = fake_part()
    bus.writeto_mem(0x22, 0x01, bytes([mode]))
    assert part.asleep
    with pytest.raises(OSError):
        bus.readfrom_mem(0x22, 0x0D, 1)
    assert bus.readfrom_mem(0x22, 0x01, 1)[0] & 0x03 == 0  # standby
    assert bus.readfrom_mem(0x22, 0x0D, 1) == b"\x01"

# Source: tmag5273_devNB.ipynb | Cell In[28] | 2026-11-16 13:35:40
def test_fake_corrupt_flips_one_bit_per_packet():
    bus, part = fake_part()
    bus.writeto_mem(0x22, 0x02, b"\x10")  # X, standby: no conversion between the reads
    bus.readfrom_mem(0x22, 0x18 | 0x80, 1)  # one triggered conversion
    bus.writeto_mem(0x22, 0x00, b"\x01")  # fast16
    good = bus.readfrom(0x22, 3)
    part.corrupt = 1
    bad = bus.readfrom(0x22, 3)
    assert bad[0] == good[0] ^ 0x01 and bad[1:] == good[1:]
    assert bus.readfrom(0x22, 3) == good and part.corrupt == 0

# Source: tmag5273_devNB.ipynb | Cell In[29] | 2026-11-16 13:35:40
def test_fake_signals_are_read_at_conversion_time():
    with SimClock() as clock:
        bus, _ = fake_part(field_mT=(ramp(0.0, 10.0, 0, 10), 0.0, sine(0.0, 5.0, 20)),
                           temp_c=ramp(25.0, 35.0, 0, 10))  # fmt: skip
        bus.writeto_mem(0x22, 0x02, b"\x50")  # ZX, standby
        bus.writeto_mem(0x22, 0x07, b"\x01")  # T_CH_EN
        clock.advance(5)
        bus.readfrom_mem(0x22, 0x18 | 0x80, 1)  # convert at t = 5 s
        at5 = bus.readfrom_mem(0x22, 0x10, 8)
        assert at5[0:2] == (17508 + 5 * 58).to_bytes(2, "big")  # 30 C, half way up
        assert at5[2:4] == b"\x10\x00"  # X = 5 mT, half way up
        assert at5[6:8] == b"\x10\x00"  # Z = 5 sin(pi / 2) = 5 mT
        clock.advance(5)
        assert bus.readfrom_mem(0x22, 0x10, 8) == at5  # no conversion, no change
        bus.readfrom_mem(0x22, 0x18 | 0x80, 1)  # convert at t = 10 s
        at10 = bus.readfrom_mem(0x22, 0x10, 8)
        assert at10[2:4] == b"\x20\x00" and at10[6:8] == b"\x00\x00"  # X 10 mT, Z 0

# Source: tmag5273_devNB.ipynb | Cell In[33] | 2026-11-16 13:35:40
def on_fake(cls, addr=0x22, mux=False, **part_kw):
    """``(dev, part)``: ``cls`` bound to a fresh ``FakeTMAG5273`` at ``addr`` on a fake bus,
    behind channel 0 of a fake mux at 0x70 if ``mux``."""
    bus = FakeBus()
    path = []
    if mux:
        bus.attach(0x70, FakeMux())
        path = [[0x70, 0]]
    part = bus.attach(addr, FakeTMAG5273(addr=addr, **part_kw), path=path)
    return cls(I2CConnection(bus, addr, path)), part

# Source: tmag5273_devNB.ipynb | Cell In[36] | 2026-11-16 13:35:40
FIELDS_BY_REG = {
    "DEVICE_CONFIG_1": ("F_CRC_EN", "F_MAG_TEMPCO", "F_CONV_AVG", "F_I2C_RD"),
    "DEVICE_CONFIG_2": ("F_THR_HYST", "F_LP_LN", "F_GLITCH_FILTER", "F_TRIGGER_MODE",
                        "F_OPERATING_MODE"),
    "SENSOR_CONFIG_1": ("F_MAG_CH_EN", "F_SLEEPTIME"),
    "SENSOR_CONFIG_2": ("F_THRX_COUNT", "F_MAG_THR_DIR", "F_MAG_GAIN_CH", "F_ANGLE_EN",
                        "F_X_Y_RANGE", "F_Z_RANGE"),
    "T_CONFIG": ("F_T_THR_CONFIG", "F_T_CH_EN"),
    "INT_CONFIG_1": ("F_RSLT_INT", "F_THRSLD_INT", "F_INT_STATE", "F_INT_MODE", "F_MASK_INTB"),
    "CONV_STATUS": ("F_SET_COUNT", "F_POR", "F_DIAG_STATUS", "F_RESULT_STATUS"),
    "DEVICE_STATUS": ("F_INTB_RB", "F_OSC_ER", "F_INT_ER", "F_OTP_CRC_ER", "F_VCC_UV_ER"),
}  # fmt: skip


@pytest.mark.parametrize("reg", sorted(FIELDS_BY_REG))
def test_fields_do_not_overlap(reg):
    covered = 0
    for name in FIELDS_BY_REG[reg]:
        mask = TMAG5273Regs.field_mask(getattr(TMAG5273Regs, name))
        assert covered & mask == 0 and mask <= 0xFF
        covered |= mask

# Source: tmag5273_devNB.ipynb | Cell In[37] | 2026-11-16 13:35:40
def test_crc_vectors_match_datasheet():
    assert len(TMAG5273Regs.CRC_VECTORS) == 7
    for data, want in TMAG5273Regs.CRC_VECTORS:
        assert TMAG5273Regs.crc(data) == want

# Source: tmag5273_devNB.ipynb | Cell In[38] | 2026-11-16 13:35:41
def test_address_and_clock_limits():
    assert all(TMAG5273Regs.addr_ok(a) for a in TMAG5273Regs.FACTORY_ADDRS.values())
    assert not TMAG5273Regs.addr_ok(0x07) and not TMAG5273Regs.addr_ok(0x79)
    assert TMAG5273Regs.baud_ok(1_000_000) and not TMAG5273Regs.baud_ok(3_400_000)

# Source: tmag5273_devNB.ipynb | Cell In[39] | 2026-11-16 13:35:41
def test_auto_increment_reads_register_pairs():
    regs, _ = on_fake(TMAG5273Regs)
    lsb = regs.reg_read(regs.REG_MANUFACTURER_ID_LSB)
    msb = regs.reg_read(regs.REG_MANUFACTURER_ID_MSB)
    assert regs.reg_read(regs.REG_MANUFACTURER_ID_LSB, nbytes=2) == (lsb << 8) | msb == 0x4954
    assert regs.reg_get(regs.REG_MANUFACTURER_ID_LSB) == lsb
    assert regs.reg_get(regs.REG_MANUFACTURER_ID_LSB, nbytes=2) == 0x4954

# Source: tmag5273_devNB.ipynb | Cell In[40] | 2026-11-16 13:35:41
def test_status_bits_are_write_1_to_clear():
    regs, _ = on_fake(TMAG5273Regs)
    por = regs.field_mask(regs.F_POR)
    assert regs.field_get(regs.REG_CONV_STATUS, regs.F_POR) == 1
    with pytest.raises(ValueError):
        regs.reg_set(regs.REG_CONV_STATUS, por)
    regs.reg_write(regs.REG_CONV_STATUS, por)
    assert regs.field_get(regs.REG_CONV_STATUS, regs.F_POR) == 0

# Source: tmag5273_devNB.ipynb | Cell In[42] | 2026-11-16 13:35:41
MAG_CASES = [(0x0000, 40.0, 0.0), (0x1000, 40.0, 5.0), (0xF000, 40.0, -5.0), (0x0800, 40.0, 2.5),
             (0x7FFF, 40.0, 39.99878), (0x8000, 40.0, -40.0), (0x1000, 80.0, 10.0),
             (0x8000, 80.0, -80.0)]  # fmt: skip
PACKET_CASES = [(1, "XYZ", False, False, 7), (1, "XYZ", True, False, 9), (1, "X", False, True, 4),
                (1, "XY", True, True, 8), (2, "XYZ", True, False, 5), (2, "XYZ", True, True, 6),
                (2, "XY", False, True, 4)]  # fmt: skip
CONV_TIME_CASES = [(1, 1, 50), (1, 32, 825), (2, 1, 75), (2, 32, 1625), (3, 1, 100)]

# Source: tmag5273_devNB.ipynb | Cell In[45] | 2026-11-16 13:35:41
@pytest.mark.parametrize("word, br, want", MAG_CASES)
def test_mag_decode(word, br, want):
    assert TMAG5273.mag_from_raw(word, br) == pytest.approx(want, abs=1e-4)

# Source: tmag5273_devNB.ipynb | Cell In[46] | 2026-11-16 13:35:42
@pytest.mark.parametrize("br, stated", [(40.0, 820), (80.0, 410)])
def test_sensitivity_matches_datasheet(br, stated):
    assert 1 / TMAG5273.mag_from_raw(1, br) == pytest.approx(stated, abs=1.0)

# Source: tmag5273_devNB.ipynb | Cell In[47] | 2026-11-16 13:35:42
@pytest.mark.parametrize("br", [40.0, 80.0])
def test_mag_round_trip(br):
    for mT in (0.0, 5.0, -5.0, 0.001221, br * 0.99, -br * 0.99):
        w = TMAG5273.mag_to_raw(mT, br)
        assert TMAG5273.mag_from_raw(w, br) == pytest.approx(mT, abs=2 * br / 65536)

# Source: tmag5273_devNB.ipynb | Cell In[48] | 2026-11-16 13:35:42
@pytest.mark.parametrize("mT, br", [(50.0, 40.0), (-50.0, 40.0), (100.0, 80.0)])
def test_mag_out_of_range(mT, br):
    with pytest.raises(ValueError):
        TMAG5273.mag_to_raw(mT, br)

# Source: tmag5273_devNB.ipynb | Cell In[49] | 2026-11-16 13:35:42
def test_temperature_codec():
    assert TMAG5273.celsius_from_raw(17508) == 25.0
    assert TMAG5273.celsius_from_raw(17508 + 58) == 26.0
    for c in (-40.0, 0.0, 25.0, 100.0, 170.0):
        assert TMAG5273.celsius_from_raw(TMAG5273.celsius_to_raw(c)) == pytest.approx(c, abs=0.02)
    assert TMAG5273.celsius_to_raw(TMAG5273.TEMP_MAX) < 0x8000
    signed = 25.0 + (TMAG5273.sign_extend(0x8000, 16) - 17508) / 58
    assert abs(TMAG5273.celsius_from_raw(0x8000) - signed) > 1000

# Source: tmag5273_devNB.ipynb | Cell In[50] | 2026-11-16 13:35:43
def test_angle_codec():
    assert TMAG5273.angle_from_raw(0x1628) == 354.5
    assert TMAG5273.angle_from_raw(0x0114) == 17.25
    for deg in (0.0, 17.25, 90.0, 180.0, 354.5, 359.9375):
        assert TMAG5273.angle_from_raw(TMAG5273.angle_to_raw(deg)) == deg
    assert TMAG5273.angle_to_raw(359.9375) <= 0x1FFF

# Source: tmag5273_devNB.ipynb | Cell In[51] | 2026-11-16 13:35:43
def test_offset_codec():
    assert TMAG5273.offset_from_raw(0b10000000, 40.0) == -2.5
    assert TMAG5273.offset_from_raw(0b00010000, 40.0) == pytest.approx(0.3125)
    for br in (40.0, 80.0):
        limit = TMAG5273.offset_limit(br)
        for mT in (0.0, limit / 8, -limit / 8, -limit, limit * 127 / 128):
            assert TMAG5273.offset_from_raw(TMAG5273.offset_to_raw(mT, br), br) == pytest.approx(mT)
    with pytest.raises(ValueError, match="representable"):
        TMAG5273.offset_to_raw(5.0, 40.0)

# Source: tmag5273_devNB.ipynb | Cell In[52] | 2026-11-16 13:35:43
@pytest.mark.parametrize("mode, chans, temp, crc, want", PACKET_CASES)
def test_packet_length(mode, chans, temp, crc, want):
    assert TMAG5273.packet_length(mode, tuple(chans), temp, crc) == want

# Source: tmag5273_devNB.ipynb | Cell In[53] | 2026-11-16 13:35:44
@pytest.mark.parametrize("mode", [0, 3])
def test_packet_length_rejects_standard(mode):
    with pytest.raises(ValueError):
        TMAG5273.packet_length(mode, ("X",), False, False)

# Source: tmag5273_devNB.ipynb | Cell In[54] | 2026-11-16 13:35:44
def test_streamed_axes_follow_the_name_once_each():
    assert TMAG5273.MAG_CH[0x7] == "XYZ" and TMAG5273.MAG_CH[0xB] == "XZX"
    assert TMAG5273.CH_SEQUENCE[0x5] == ("Z", "X")
    assert TMAG5273.CH_SEQUENCE[0x8] == ("X", "Y")
    assert TMAG5273.CH_SEQUENCE[0xB] == ("X", "Z")
    for code, name in TMAG5273.MAG_CH.items():
        seq = TMAG5273.CH_SEQUENCE[code]
        assert len(set(seq)) == len(seq) and "".join(seq) == "".join(
            sorted(set(name), key=name.index)
        )

# Source: tmag5273_devNB.ipynb | Cell In[55] | 2026-11-16 13:35:44
@pytest.mark.parametrize("channels, averaging, want", CONV_TIME_CASES)
def test_conversion_time(channels, averaging, want):
    assert TMAG5273.conversion_time_us(channels, averaging) == want

# Source: tmag5273_devNB.ipynb | Cell In[59] | 2026-11-16 13:35:44
def test_identify_and_variant():
    mag, _ = on_fake(TMAG5273)
    assert mag.identify() == (0x5449, 1)
    assert mag.variant() == "B1"
    assert mag.ranges == {0: 40.0, 1: 80.0}
    assert on_fake(TMAG5273, addr=0x35, version=2)[0].variant() == "A2"

# Source: tmag5273_devNB.ipynb | Cell In[60] | 2026-11-16 13:35:45
def test_init_refuses_a_part_that_is_not_a_tmag5273():
    bus = FakeBus()
    bus.attach(0x22, FakeRegisterPart())
    with pytest.raises(RuntimeError, match="not a TMAG5273"):
        TMAG5273(I2CConnection(bus, 0x22))
    assert not any(entry[0] in ("w", "wm") for entry in bus.log)

# Source: tmag5273_devNB.ipynb | Cell In[61] | 2026-11-16 13:35:45
def test_configure_sets_everything():
    mag, _ = on_fake(TMAG5273)
    mag.configure(channels="XYZ", temperature=True, averaging=4, xy_range=80.0)
    assert (mag.channels, mag.temperature_enabled, mag.averaging) == ("XYZ", True, 4)
    assert mag.operating_mode == "continuous"
    assert (mag.xy_range, mag.z_range) == (80.0, 40.0)
    with pytest.raises(ValueError):
        mag.channels = "QQ"

# Source: tmag5273_devNB.ipynb | Cell In[62] | 2026-11-16 13:35:45
def test_reads_results():
    mag, _ = on_fake(TMAG5273, field_mT=(5.0, -5.0, 0.0), temp_c=25.0)
    mag.configure(channels="XYZ", temperature=True)
    assert mag.all_read() == (25.0, 5.0, -5.0, 0.0)
    assert mag.magnetic() == (5.0, -5.0, 0.0)
    assert mag.temperature() == 25.0
    assert mag.magnetic_raw() == (0x1000, 0xF000, 0x0000)

# Source: tmag5273_devNB.ipynb | Cell In[63] | 2026-11-16 13:35:45
def test_range_switch_reports_the_same_field():
    mag, _ = on_fake(TMAG5273, field_mT=(5.0, -5.0, 12.0))
    mag.configure(channels="XYZ")
    at40 = mag.magnetic()
    assert mag.ranges_set(80.0, 80.0) == (80.0, 80.0)
    at80 = mag.magnetic()
    assert at40 == pytest.approx(at80, abs=2 * 80.0 / 65536)

# Source: tmag5273_devNB.ipynb | Cell In[64] | 2026-11-16 13:35:46
def test_crc_on_standard_reads_stay_within_four_bytes():
    mag, _ = on_fake(TMAG5273, field_mT=(5.0, -5.0, 2.5))
    mag.configure(channels="XYZ", temperature=True)
    mag.crc_enabled = True
    bus = mag.conn.bus
    start = len(bus.log)
    assert mag.all_read() == (25.0, 5.0, -5.0, 2.5)
    assert mag.magnetic() == (5.0, -5.0, 2.5)
    reads = [e for e in bus.log[start:] if e[0] == "rm"]
    assert reads and all(e[3] <= 4 for e in reads)

# Source: tmag5273_devNB.ipynb | Cell In[65] | 2026-11-16 13:35:46
def test_fast16_read_with_crc_is_one_transfer():
    mag, _ = on_fake(TMAG5273, field_mT=(5.0, -5.0, 2.5))
    mag.configure(channels="XYZ")
    mag.crc_enabled = True
    mag.read_mode = "fast16"
    bus = mag.conn.bus
    start = len(bus.log)
    readings, status = mag.fast_read()
    assert readings == {"X": 5.0, "Y": -5.0, "Z": 2.5}
    assert status & 0x01  # RESULT_STATUS
    assert [e[0] for e in bus.log[start:]] == ["r"]

# Source: tmag5273_devNB.ipynb | Cell In[66] | 2026-11-16 13:35:46
def test_fast8_read_quantises():
    mag, _ = on_fake(TMAG5273, field_mT=(5.1, -5.1, 0.2), temp_c=27.0)
    mag.configure(channels="XYZ", temperature=True)
    mag.read_mode = "fast8"
    readings, _ = mag.fast_read()
    for ax, want in zip("XYZ", (5.1, -5.1, 0.2)):
        assert abs(readings[ax] - want) <= 2 * 40.0 / 256
    assert 27.0 - TMAG5273.TEMP_LSB_STEP_C <= readings["T"] <= 27.0

# Source: tmag5273_devNB.ipynb | Cell In[67] | 2026-11-16 13:35:47
def test_fast_read_detects_corruption():
    mag, part = on_fake(TMAG5273)
    mag.configure(channels="XY")
    mag.crc_enabled = True
    mag.read_mode = "fast16"
    part.corrupt = 2  # the read and its retry
    with pytest.raises(ValueError, match="checksum"):
        mag.fast_read()
    part.corrupt = 1
    mag.fast_read(verify=False)

# Source: tmag5273_devNB.ipynb | Cell In[68] | 2026-11-16 13:35:47
def test_fast_read_retries_once():
    mag, part = on_fake(TMAG5273)
    mag.configure(channels="XY")
    mag.crc_enabled = True
    mag.read_mode = "fast16"
    part.corrupt = 1
    bus = mag.conn.bus
    start = len(bus.log)
    assert mag.fast_read()[0] == {"X": 5.0, "Y": -5.0}
    assert [e[0] for e in bus.log[start:]] == ["r", "r"]

# Source: tmag5273_devNB.ipynb | Cell In[69] | 2026-11-16 13:35:47
def test_fast_read_crc_covers_the_mux_byte():
    mag, _ = on_fake(TMAG5273, mux=True)
    mag.configure(channels="XY")
    mag.crc_enabled = True
    mag.read_mode = "fast16"
    bus = mag.conn.bus
    start = len(bus.log)
    assert mag.fast_read()[0] == {"X": 5.0, "Y": -5.0}
    assert [e[0] for e in bus.log[start:]] == ["w", "r"]
    raw = mag.read_raw(mag.fast_read_length())
    tail = bytes([(0x22 << 1) | 1]) + raw[:-1]
    assert TMAG5273.crc(bytes([0x70 << 1]) + tail) == raw[-1] != TMAG5273.crc(tail)

# Source: tmag5273_devNB.ipynb | Cell In[70] | 2026-11-16 13:35:47
def test_fast16_crc_refuses_four_channels():
    mag, _ = on_fake(TMAG5273)
    mag.configure(channels="XYZ", temperature=True)
    mag.crc_enabled = True
    mag.read_mode = "fast16"
    start = len(mag.conn.bus.log)
    with pytest.raises(ValueError, match="at most 3"):
        mag.fast_read()
    assert len(mag.conn.bus.log) == start

# Source: tmag5273_devNB.ipynb | Cell In[71] | 2026-11-16 13:35:48
def test_pseudo_simultaneous_axes_stream_once():
    mag, _ = on_fake(TMAG5273, field_mT=(5.0, -5.0, 2.5))
    for name, want in (("XZX", ["X", "Z"]), ("ZX", ["Z", "X"]), ("YXY", ["Y", "X"])):
        mag.configure(channels=name)
        mag.read_mode = "fast16"
        words, _, _ = mag.fast_read_raw()
        assert [n for n, _ in words] == want
        field = dict(zip("XYZ", (5.0, -5.0, 2.5)))
        assert mag.fast_read()[0] == {a: field[a] for a in want}
        mag.read_mode = "standard"

# Source: tmag5273_devNB.ipynb | Cell In[72] | 2026-11-16 13:35:48
def test_registers_unreadable_in_fast_mode():
    mag, _ = on_fake(TMAG5273)
    mag.configure(channels="XYZ")
    mag.read_mode = "fast16"
    with pytest.raises(RuntimeError, match="standard"):
        mag.conv_status()
    with pytest.raises(RuntimeError, match="standard"):
        mag.channels = "XY"
    assert (mag.channels, mag.read_mode) == ("XYZ", "fast16")
    mag.read_mode = "standard"
    assert mag.conv_status()["ready"]
    assert mag.read_mode == "standard"

# Source: tmag5273_devNB.ipynb | Cell In[73] | 2026-11-16 13:35:48
def test_offsets_and_angle():
    mag, _ = on_fake(TMAG5273, field_mT=(5.0, -5.0, 0.0))
    mag.configure(channels="XYZ")
    mag.angle_pair = "XY"
    assert mag.angle() == pytest.approx(315.0, abs=1 / 16)
    assert mag.offsets_set(-2.5, 1.25) == (-2.5, 1.25)
    x, y, z = mag.magnetic()
    assert (x, y, z) == pytest.approx((2.5, -3.75, 0.0), abs=1e-3)
    want = math.degrees(math.atan2(y, x)) % 360.0
    assert mag.angle() == pytest.approx(want, abs=1 / 16)

# Source: tmag5273_devNB.ipynb | Cell In[74] | 2026-11-16 13:35:48
def test_trigger_converts_in_standby():
    mag, _ = on_fake(TMAG5273)
    mag.configure(channels="XYZ", mode="standby")
    first = mag.conv_status()["set_count"]
    assert mag.conv_status()["set_count"] == first
    mag.trigger()
    assert mag.conv_status()["set_count"] == (first + 1) % 8
    assert mag.magnetic() == (5.0, -5.0, 0.0)

# Source: tmag5273_devNB.ipynb | Cell In[75] | 2026-11-16 13:35:49
def test_sleep_and_wake():
    mag, _ = on_fake(TMAG5273)
    mag.operating_mode = "sleep"
    with pytest.raises(OSError):
        mag.device_status()
    mag.operating_mode = "sleep"
    mag.wake()
    assert mag.operating_mode == "standby"
    assert not mag.device_status()["vcc_uv_error"]

# Source: tmag5273_devNB.ipynb | Cell In[76] | 2026-11-16 13:35:49
def test_status_clear_clears_por():
    mag, _ = on_fake(TMAG5273)
    assert mag.conv_status()["por"]
    mag.status_clear()
    assert not mag.conv_status()["por"]
    assert not any(v for k, v in mag.device_status().items() if k.endswith("error"))

# Source: tmag5273_devNB.ipynb | Cell In[77] | 2026-11-16 13:35:49
def test_reset_config_restores_por():
    mag, part = on_fake(TMAG5273)
    mag.configure(channels="XY", temperature=True, xy_range=80.0)
    mag.read_mode = "fast8"
    assert mag.reset_config() == (40.0, 40.0)
    assert all(part.regs[r] == 0 for r in TMAG5273.CONFIG_REGS)
    assert part.regs[0x0C] == 0x22 << 1
    assert mag.read_mode == "standard"

# Source: tmag5273_devNB.ipynb | Cell In[78] | 2026-11-16 13:35:49
def test_reset_flag_recovers_a_part_left_in_fast_mode():
    _, part = on_fake(TMAG5273)
    part.regs[0x00] = 0x01  # left in fast16 by an earlier session
    bus = FakeBus()
    bus.attach(0x22, part)
    with pytest.raises(RuntimeError, match="reset=True"):
        TMAG5273(I2CConnection(bus, 0x22))
    mag = TMAG5273(I2CConnection(bus, 0x22), reset=True)
    assert mag.read_mode == "standard" and mag.identify() == (0x5449, 1)

# Source: tmag5273_devNB.ipynb | Cell In[79] | 2026-11-16 13:35:50
TURN_S = 60.0  # one turn of the turntable
LOWER_S = 120.0  # time to lower the magnet from 0.5 mT to 30 mT on Z


def turntable_magnet(radius_mT=20.0):
    """(X, Y, Z) signals: a magnet turning once per ``TURN_S`` in the XY plane, lowered on Z."""
    x = sine(0.0, radius_mT, TURN_S, phase_s=-TURN_S / 4)  # a cosine
    y = sine(0.0, radius_mT, TURN_S)
    z = ramp(0.5, 30.0, 0.0, LOWER_S)
    return (x, y, z)


def turntable_angle(t_s):
    """Where the magnet points at ``t_s``, in degrees: one turn per ``TURN_S``."""
    return (360.0 * t_s / TURN_S) % 360.0


def angle_error(a, b):
    """The difference between two angles in degrees, across the 0/360 wrap."""
    return abs((a - b + 180.0) % 360.0 - 180.0)

# Source: tmag5273_devNB.ipynb | Cell In[82] | 2026-11-16 13:35:50
def test_angle_follows_a_turning_magnet():
    with SimClock() as clock:
        mag, _ = on_fake(TMAG5273, field_mT=turntable_magnet())
        mag.configure(channels="XYZ")
        mag.angle_pair = "XY"
        for _ in range(24):
            assert angle_error(mag.angle(), turntable_angle(clock.now_s())) < 0.1
            clock.advance(TURN_S / 24 + 0.3)

# Source: tmag5273_devNB.ipynb | Cell In[83] | 2026-11-16 13:35:50
def test_fast_read_follows_an_approaching_magnet():
    with SimClock() as clock:
        mag, _ = on_fake(TMAG5273, field_mT=turntable_magnet())
        mag.configure(channels="XYZ")
        mag.crc_enabled = True
        mag.read_mode = "fast16"
        for _ in range(7):
            want = 0.5 + (30.0 - 0.5) * min(clock.now_s(), LOWER_S) / LOWER_S
            assert mag.fast_read()[0]["Z"] == pytest.approx(want, abs=2 * 40.0 / 65536)
            clock.advance(20)

# Source: tmag5273_devNB.ipynb | Cell In[84] | 2026-11-16 13:35:50
def test_triggered_temperature_is_the_value_at_the_trigger():
    with SimClock() as clock:
        mag, _ = on_fake(TMAG5273, temp_c=ramp(25.0, 35.0, 0.0, 10.0))
        mag.configure(channels="X", temperature=True, mode="standby")
        clock.advance(5)
        mag.trigger()
        assert mag.temperature() == pytest.approx(30.0, abs=0.02)
        clock.advance(5)
        assert mag.temperature() == pytest.approx(30.0, abs=0.02)
        mag.trigger()
        assert mag.temperature() == pytest.approx(35.0, abs=0.02)

