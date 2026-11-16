import math

import pytest

from micropython_i2c import I2CConnection
from micropython_i2c.devices.sensors.magnetic.tmag5273 import TMAG5273, TMAG5273Regs
from micropython_i2c.fakes import FakeBus, FakeMux, FakeRegisterPart, FakeTMAG5273
# Source: tmag5273_devNB.ipynb | Cell In[10] | 2026-11-14 12:39:49
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

# Source: tmag5273_devNB.ipynb | Cell In[13] | 2026-11-14 12:39:49
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

# Source: tmag5273_devNB.ipynb | Cell In[14] | 2026-11-14 12:39:49
def test_crc_vectors_match_datasheet():
    assert len(TMAG5273Regs.CRC_VECTORS) == 7
    for data, want in TMAG5273Regs.CRC_VECTORS:
        assert TMAG5273Regs.crc(data) == want

# Source: tmag5273_devNB.ipynb | Cell In[15] | 2026-11-14 12:39:50
def test_address_and_clock_limits():
    assert all(TMAG5273Regs.addr_ok(a) for a in TMAG5273Regs.FACTORY_ADDRS.values())
    assert not TMAG5273Regs.addr_ok(0x07) and not TMAG5273Regs.addr_ok(0x79)
    assert TMAG5273Regs.baud_ok(1_000_000) and not TMAG5273Regs.baud_ok(3_400_000)

# Source: tmag5273_devNB.ipynb | Cell In[16] | 2026-11-14 12:39:50
def test_auto_increment_reads_register_pairs():
    regs, _ = on_fake(TMAG5273Regs)
    lsb = regs.reg_read(regs.REG_MANUFACTURER_ID_LSB)
    msb = regs.reg_read(regs.REG_MANUFACTURER_ID_MSB)
    assert regs.reg_read(regs.REG_MANUFACTURER_ID_LSB, nbytes=2) == (lsb << 8) | msb == 0x4954
    assert regs.reg_get(regs.REG_MANUFACTURER_ID_LSB) == lsb
    assert regs.reg_get(regs.REG_MANUFACTURER_ID_LSB, nbytes=2) == 0x4954

# Source: tmag5273_devNB.ipynb | Cell In[17] | 2026-11-14 12:39:50
def test_status_bits_are_write_1_to_clear():
    regs, _ = on_fake(TMAG5273Regs)
    por = regs.field_mask(regs.F_POR)
    assert regs.field_get(regs.REG_CONV_STATUS, regs.F_POR) == 1
    with pytest.raises(ValueError):
        regs.reg_set(regs.REG_CONV_STATUS, por)
    regs.reg_write(regs.REG_CONV_STATUS, por)
    assert regs.field_get(regs.REG_CONV_STATUS, regs.F_POR) == 0

# Source: tmag5273_devNB.ipynb | Cell In[19] | 2026-11-14 12:39:51
MAG_CASES = [(0x0000, 40.0, 0.0), (0x1000, 40.0, 5.0), (0xF000, 40.0, -5.0), (0x0800, 40.0, 2.5),
             (0x7FFF, 40.0, 39.99878), (0x8000, 40.0, -40.0), (0x1000, 80.0, 10.0),
             (0x8000, 80.0, -80.0)]  # fmt: skip
PACKET_CASES = [(1, "XYZ", False, False, 7), (1, "XYZ", True, False, 9), (1, "X", False, True, 4),
                (1, "XY", True, True, 8), (2, "XYZ", True, False, 5), (2, "XYZ", True, True, 6),
                (2, "XY", False, True, 4)]  # fmt: skip
CONV_TIME_CASES = [(1, 1, 50), (1, 32, 825), (2, 1, 75), (2, 32, 1625), (3, 1, 100)]

# Source: tmag5273_devNB.ipynb | Cell In[22] | 2026-11-14 12:39:51
@pytest.mark.parametrize("word, br, want", MAG_CASES)
def test_mag_decode(word, br, want):
    assert TMAG5273.mag_from_raw(word, br) == pytest.approx(want, abs=1e-4)

# Source: tmag5273_devNB.ipynb | Cell In[23] | 2026-11-14 12:39:51
@pytest.mark.parametrize("br, stated", [(40.0, 820), (80.0, 410)])
def test_sensitivity_matches_datasheet(br, stated):
    assert 1 / TMAG5273.mag_from_raw(1, br) == pytest.approx(stated, abs=1.0)

# Source: tmag5273_devNB.ipynb | Cell In[24] | 2026-11-14 12:39:52
@pytest.mark.parametrize("br", [40.0, 80.0])
def test_mag_round_trip(br):
    for mT in (0.0, 5.0, -5.0, 0.001221, br * 0.99, -br * 0.99):
        w = TMAG5273.mag_to_raw(mT, br)
        assert TMAG5273.mag_from_raw(w, br) == pytest.approx(mT, abs=2 * br / 65536)

# Source: tmag5273_devNB.ipynb | Cell In[25] | 2026-11-14 12:39:52
@pytest.mark.parametrize("mT, br", [(50.0, 40.0), (-50.0, 40.0), (100.0, 80.0)])
def test_mag_out_of_range(mT, br):
    with pytest.raises(ValueError):
        TMAG5273.mag_to_raw(mT, br)

# Source: tmag5273_devNB.ipynb | Cell In[26] | 2026-11-14 12:39:53
def test_temperature_codec():
    assert TMAG5273.celsius_from_raw(17508) == 25.0
    assert TMAG5273.celsius_from_raw(17508 + 58) == 26.0
    for c in (-40.0, 0.0, 25.0, 100.0, 170.0):
        assert TMAG5273.celsius_from_raw(TMAG5273.celsius_to_raw(c)) == pytest.approx(c, abs=0.02)
    assert TMAG5273.celsius_to_raw(TMAG5273.TEMP_MAX) < 0x8000
    signed = 25.0 + (TMAG5273.sign_extend(0x8000, 16) - 17508) / 58
    assert abs(TMAG5273.celsius_from_raw(0x8000) - signed) > 1000

# Source: tmag5273_devNB.ipynb | Cell In[27] | 2026-11-14 12:39:53
def test_angle_codec():
    assert TMAG5273.angle_from_raw(0x1628) == 354.5
    assert TMAG5273.angle_from_raw(0x0114) == 17.25
    for deg in (0.0, 17.25, 90.0, 180.0, 354.5, 359.9375):
        assert TMAG5273.angle_from_raw(TMAG5273.angle_to_raw(deg)) == deg
    assert TMAG5273.angle_to_raw(359.9375) <= 0x1FFF

# Source: tmag5273_devNB.ipynb | Cell In[28] | 2026-11-14 12:39:53
def test_offset_codec():
    assert TMAG5273.offset_from_raw(0b10000000, 40.0) == -2.5
    assert TMAG5273.offset_from_raw(0b00010000, 40.0) == pytest.approx(0.3125)
    for br in (40.0, 80.0):
        limit = TMAG5273.offset_limit(br)
        for mT in (0.0, limit / 8, -limit / 8, -limit, limit * 127 / 128):
            assert TMAG5273.offset_from_raw(TMAG5273.offset_to_raw(mT, br), br) == pytest.approx(mT)
    with pytest.raises(ValueError, match="representable"):
        TMAG5273.offset_to_raw(5.0, 40.0)

# Source: tmag5273_devNB.ipynb | Cell In[29] | 2026-11-14 12:39:54
@pytest.mark.parametrize("mode, chans, temp, crc, want", PACKET_CASES)
def test_packet_length(mode, chans, temp, crc, want):
    assert TMAG5273.packet_length(mode, tuple(chans), temp, crc) == want

# Source: tmag5273_devNB.ipynb | Cell In[30] | 2026-11-14 12:39:54
@pytest.mark.parametrize("mode", [0, 3])
def test_packet_length_rejects_standard(mode):
    with pytest.raises(ValueError):
        TMAG5273.packet_length(mode, ("X",), False, False)

# Source: tmag5273_devNB.ipynb | Cell In[31] | 2026-11-14 12:39:54
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

# Source: tmag5273_devNB.ipynb | Cell In[32] | 2026-11-14 12:39:55
@pytest.mark.parametrize("channels, averaging, want", CONV_TIME_CASES)
def test_conversion_time(channels, averaging, want):
    assert TMAG5273.conversion_time_us(channels, averaging) == want

# Source: tmag5273_devNB.ipynb | Cell In[36] | 2026-11-14 12:39:55
def test_identify_and_variant():
    mag, _ = on_fake(TMAG5273)
    assert mag.identify() == (0x5449, 1)
    assert mag.variant() == "B1"
    assert mag.ranges == {0: 40.0, 1: 80.0}
    assert on_fake(TMAG5273, addr=0x35, version=2)[0].variant() == "A2"

# Source: tmag5273_devNB.ipynb | Cell In[37] | 2026-11-14 12:39:56
def test_init_refuses_a_part_that_is_not_a_tmag5273():
    bus = FakeBus()
    bus.attach(0x22, FakeRegisterPart())
    with pytest.raises(RuntimeError, match="not a TMAG5273"):
        TMAG5273(I2CConnection(bus, 0x22))
    assert not any(entry[0] in ("w", "wm") for entry in bus.log)

# Source: tmag5273_devNB.ipynb | Cell In[38] | 2026-11-14 12:39:56
def test_configure_sets_everything():
    mag, _ = on_fake(TMAG5273)
    mag.configure(channels="XYZ", temperature=True, averaging=4, xy_range=80.0)
    assert (mag.channels, mag.temperature_enabled, mag.averaging) == ("XYZ", True, 4)
    assert mag.operating_mode == "continuous"
    assert (mag.xy_range, mag.z_range) == (80.0, 40.0)
    with pytest.raises(ValueError):
        mag.channels = "QQ"

# Source: tmag5273_devNB.ipynb | Cell In[39] | 2026-11-14 12:39:56
def test_reads_results():
    mag, _ = on_fake(TMAG5273, field_mT=(5.0, -5.0, 0.0), temp_c=25.0)
    mag.configure(channels="XYZ", temperature=True)
    assert mag.all_read() == (25.0, 5.0, -5.0, 0.0)
    assert mag.magnetic() == (5.0, -5.0, 0.0)
    assert mag.temperature() == 25.0
    assert mag.magnetic_raw() == (0x1000, 0xF000, 0x0000)

# Source: tmag5273_devNB.ipynb | Cell In[40] | 2026-11-14 12:39:57
def test_range_switch_reports_the_same_field():
    mag, _ = on_fake(TMAG5273, field_mT=(5.0, -5.0, 12.0))
    mag.configure(channels="XYZ")
    at40 = mag.magnetic()
    assert mag.ranges_set(80.0, 80.0) == (80.0, 80.0)
    at80 = mag.magnetic()
    assert at40 == pytest.approx(at80, abs=2 * 80.0 / 65536)

# Source: tmag5273_devNB.ipynb | Cell In[41] | 2026-11-14 12:39:57
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

# Source: tmag5273_devNB.ipynb | Cell In[42] | 2026-11-14 12:39:57
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

# Source: tmag5273_devNB.ipynb | Cell In[43] | 2026-11-14 12:39:58
def test_fast8_read_quantises():
    mag, _ = on_fake(TMAG5273, field_mT=(5.1, -5.1, 0.2), temp_c=27.0)
    mag.configure(channels="XYZ", temperature=True)
    mag.read_mode = "fast8"
    readings, _ = mag.fast_read()
    for ax, want in zip("XYZ", (5.1, -5.1, 0.2)):
        assert abs(readings[ax] - want) <= 2 * 40.0 / 256
    assert 27.0 - TMAG5273.TEMP_LSB_STEP_C <= readings["T"] <= 27.0

# Source: tmag5273_devNB.ipynb | Cell In[44] | 2026-11-14 12:39:58
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

# Source: tmag5273_devNB.ipynb | Cell In[45] | 2026-11-14 12:39:58
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

# Source: tmag5273_devNB.ipynb | Cell In[46] | 2026-11-14 12:39:59
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

# Source: tmag5273_devNB.ipynb | Cell In[47] | 2026-11-14 12:39:59
def test_fast16_crc_refuses_four_channels():
    mag, _ = on_fake(TMAG5273)
    mag.configure(channels="XYZ", temperature=True)
    mag.crc_enabled = True
    mag.read_mode = "fast16"
    start = len(mag.conn.bus.log)
    with pytest.raises(ValueError, match="at most 3"):
        mag.fast_read()
    assert len(mag.conn.bus.log) == start

# Source: tmag5273_devNB.ipynb | Cell In[48] | 2026-11-14 12:39:59
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

# Source: tmag5273_devNB.ipynb | Cell In[49] | 2026-11-14 12:40:00
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

# Source: tmag5273_devNB.ipynb | Cell In[50] | 2026-11-14 12:40:00
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

# Source: tmag5273_devNB.ipynb | Cell In[51] | 2026-11-14 12:40:01
def test_trigger_converts_in_standby():
    mag, _ = on_fake(TMAG5273)
    mag.configure(channels="XYZ", mode="standby")
    first = mag.conv_status()["set_count"]
    assert mag.conv_status()["set_count"] == first
    mag.trigger()
    assert mag.conv_status()["set_count"] == (first + 1) % 8
    assert mag.magnetic() == (5.0, -5.0, 0.0)

# Source: tmag5273_devNB.ipynb | Cell In[52] | 2026-11-14 12:40:01
def test_sleep_and_wake():
    mag, _ = on_fake(TMAG5273)
    mag.operating_mode = "sleep"
    with pytest.raises(OSError):
        mag.device_status()
    mag.operating_mode = "sleep"
    mag.wake()
    assert mag.operating_mode == "standby"
    assert not mag.device_status()["vcc_uv_error"]

# Source: tmag5273_devNB.ipynb | Cell In[53] | 2026-11-14 12:40:01
def test_status_clear_clears_por():
    mag, _ = on_fake(TMAG5273)
    assert mag.conv_status()["por"]
    mag.status_clear()
    assert not mag.conv_status()["por"]
    assert not any(v for k, v in mag.device_status().items() if k.endswith("error"))

# Source: tmag5273_devNB.ipynb | Cell In[54] | 2026-11-14 12:40:02
def test_reset_config_restores_por():
    mag, part = on_fake(TMAG5273)
    mag.configure(channels="XY", temperature=True, xy_range=80.0)
    mag.read_mode = "fast8"
    assert mag.reset_config() == (40.0, 40.0)
    assert all(part.regs[r] == 0 for r in TMAG5273.CONFIG_REGS)
    assert part.regs[0x0C] == 0x22 << 1
    assert mag.read_mode == "standard"

# Source: tmag5273_devNB.ipynb | Cell In[55] | 2026-11-14 12:40:02
def test_reset_flag_recovers_a_part_left_in_fast_mode():
    _, part = on_fake(TMAG5273)
    part.regs[0x00] = 0x01  # left in fast16 by an earlier session
    bus = FakeBus()
    bus.attach(0x22, part)
    with pytest.raises(RuntimeError, match="reset=True"):
        TMAG5273(I2CConnection(bus, 0x22))
    mag = TMAG5273(I2CConnection(bus, 0x22), reset=True)
    assert mag.read_mode == "standard" and mag.identify() == (0x5449, 1)

