import pytest

from micropython_i2c.connection import I2CConnection
from micropython_i2c.node import I2CDevice
from micropython_i2c.protocol import CommandI2CBase, RegisterI2CBase
from micropython_i2c.sim.bus import FakeBus, FakeRegisterPart, FakeSensirion
# Source: protocol_devNB.ipynb | Cell In[10] | 2026-11-16 13:37:14
class DemoRegs(RegisterI2CBase, I2CDevice):
    """A TMP102-shaped register map on a generic register part."""

    ALLOWED_ADDRS = (0x48,)
    REG_TEMP = 0x00
    REG_CONFIG = 0x02
    REG_THIGH = 0x04
    F_OS = (15, 1)  # status: reads 0 here, ignored by the verify mask
    F_RES = (13, 2)  # read-only, always 0b11
    F_SD = (8, 1)
    F_CR = (6, 2)
    F_AL = (5, 1)  # read-only
    CONV_RATES = {0: 0.25, 1: 1.0, 2: 4.0, 3: 8.0}
    CFG_POR = 0x60A0
    CONFIG_VERIFY_MASK = 0xFFFF & ~((1 << 15) | (3 << 13) | (1 << 5))
    VERIFY_MASKS = {REG_CONFIG: CONFIG_VERIFY_MASK}


def demo_part():
    """The register file behind ``DemoRegs``: TEMP 0x1900, CONFIG 0x60A0, THIGH 0x5000."""
    regs = {0x00: 0x19, 0x01: 0x00, 0x02: 0x60, 0x03: 0xA0, 0x04: 0x50, 0x05: 0x00}
    return FakeRegisterPart(regs, read_only={0x02: 0xE0, 0x03: 0x20})


@pytest.fixture
def proto():
    bus = FakeBus()
    bus.attach(0x48, demo_part())
    return DemoRegs(I2CConnection(bus, 0x48))

# Source: protocol_devNB.ipynb | Cell In[15] | 2026-11-16 13:37:14
def test_reg_read_assembles_big_endian(proto):
    assert proto.reg_read(proto.REG_CONFIG) == 0x60A0
    assert proto.reg_read(proto.REG_TEMP) == 0x1900
    assert proto.reg_read(proto.REG_TEMP) & 0x000F == 0

# Source: protocol_devNB.ipynb | Cell In[16] | 2026-11-16 13:37:15
def test_cache_miss_hit_refresh(proto):
    assert proto.cached(proto.REG_TEMP) is None
    first = proto.reg_get(proto.REG_TEMP)
    proto.conn.bus.nodes[0][2].regs[0] = 0x1A  # the part converts again (26 C)
    assert proto.reg_get(proto.REG_TEMP) == first  # stale by design
    assert proto.reg_get(proto.REG_TEMP, refresh=True) == 0x1A00

# Source: protocol_devNB.ipynb | Cell In[17] | 2026-11-16 13:37:15
def test_cache_hit_costs_no_bus_traffic(proto):
    proto.reg_get(proto.REG_CONFIG)
    n = len(proto.conn.bus.log)
    proto.reg_get(proto.REG_CONFIG)
    assert len(proto.conn.bus.log) == n

# Source: protocol_devNB.ipynb | Cell In[18] | 2026-11-16 13:37:15
def test_cache_respects_width(proto):
    narrow = proto.reg_get(proto.REG_CONFIG, nbytes=1)
    wide = proto.reg_get(proto.REG_CONFIG, nbytes=2)
    assert (narrow, wide) == (0x60, 0x60A0)

# Source: protocol_devNB.ipynb | Cell In[19] | 2026-11-16 13:37:15
def test_cache_clear(proto):
    for reg in (proto.REG_TEMP, proto.REG_CONFIG, proto.REG_THIGH):
        proto.reg_get(reg)
    proto.cache_clear(proto.REG_TEMP)
    assert proto.REG_TEMP not in proto.reg_cache and proto.REG_CONFIG in proto.reg_cache
    proto.cache_clear(0x7F)  # not cached: no error
    proto.cache_clear()
    assert not proto.reg_cache

# Source: protocol_devNB.ipynb | Cell In[20] | 2026-11-16 13:37:16
def test_config_verify_mask_excludes_os_res_al(proto):
    excluded = (
        proto.field_mask(proto.F_OS) | proto.field_mask(proto.F_RES) | proto.field_mask(proto.F_AL)
    )
    assert proto.CONFIG_VERIFY_MASK & excluded == 0
    assert 0xFFFF & ~proto.CONFIG_VERIFY_MASK == excluded
    assert proto.verify_mask_for(proto.REG_CONFIG) == proto.CONFIG_VERIFY_MASK
    assert proto.verify_mask_for(proto.REG_THIGH) is None

# Source: protocol_devNB.ipynb | Cell In[21] | 2026-11-16 13:37:16
def test_chip_wide_verify_mask_is_the_fallback(proto):
    proto.VERIFY_MASK = 0x0FFF  # set on the instance, so the class stays untouched
    try:
        assert proto.verify_mask_for(proto.REG_THIGH) == 0x0FFF
        assert proto.verify_mask_for(proto.REG_CONFIG) == proto.CONFIG_VERIFY_MASK
    finally:
        del proto.VERIFY_MASK
    assert DemoRegs.VERIFY_MASK is None

# Source: protocol_devNB.ipynb | Cell In[22] | 2026-11-16 13:37:16
def test_reg_set_masked_vs_exact(proto):
    original = proto.reg_read(proto.REG_CONFIG)
    sabotaged = original & ~proto.field_mask(proto.F_RES)
    with pytest.raises(ValueError, match="exact|mask 0xffff"):
        proto.reg_set(proto.REG_CONFIG, sabotaged, verify_mask=0xFFFF)
    assert proto.reg_set(proto.REG_CONFIG, sabotaged) == original  # RES reads back 11
    assert proto.cached(proto.REG_CONFIG) == original

# Source: protocol_devNB.ipynb | Cell In[23] | 2026-11-16 13:37:16
def test_failed_reg_set_leaves_the_cache_alone(proto):
    proto.reg_get(proto.REG_CONFIG)
    expected = r"reg 0x02: wrote 0xa0, device holds 0x60a0 \(mask 0xffff\)"
    with pytest.raises(ValueError, match=expected):
        proto.reg_set(proto.REG_CONFIG, 0x00A0, verify_mask=0xFFFF)  # RES is read-only
    assert proto.cached(proto.REG_CONFIG) == 0x60A0

# Source: protocol_devNB.ipynb | Cell In[24] | 2026-11-16 13:37:16
def test_field_set_leaves_neighbours(proto):
    before = proto.reg_read(proto.REG_CONFIG)
    proto.field_set(proto.REG_CONFIG, proto.F_SD, 1)
    after = proto.reg_read(proto.REG_CONFIG)
    # Only the bits inside the verify mask are the register's settings (OS is status).
    assert (before ^ after) & proto.CONFIG_VERIFY_MASK == proto.field_mask(proto.F_SD)

# Source: protocol_devNB.ipynb | Cell In[25] | 2026-11-16 13:37:17
def test_flag_round_trip(proto):
    assert proto.flag_get(proto.REG_CONFIG, proto.F_SD) is False
    proto.flag_set(proto.REG_CONFIG, proto.F_SD, "yes")
    assert proto.flag_get(proto.REG_CONFIG, proto.F_SD) is True
    proto.flag_set(proto.REG_CONFIG, proto.F_SD, 0)
    assert proto.flag_get(proto.REG_CONFIG, proto.F_SD) is False

# Source: protocol_devNB.ipynb | Cell In[26] | 2026-11-16 13:37:17
def test_coded_round_trip_and_reject(proto):
    for hz in (0.25, 1.0, 8.0, 4.0):
        proto.coded_set(proto.REG_CONFIG, proto.F_CR, proto.CONV_RATES, hz)
        assert proto.coded_get(proto.REG_CONFIG, proto.F_CR, proto.CONV_RATES) == hz
    with pytest.raises(ValueError, match="not in"):
        proto.coded_set(proto.REG_CONFIG, proto.F_CR, proto.CONV_RATES, 2.0)

# Source: protocol_devNB.ipynb | Cell In[27] | 2026-11-16 13:37:17
def test_validator_runs_before_bus_traffic(proto):
    def under_100c(v):
        if v > 0x6400:
            raise ValueError("too hot")

    n = len(proto.conn.bus.log)
    with pytest.raises(ValueError, match="too hot"):
        proto.reg_set(proto.REG_THIGH, 0x7000, validator=under_100c)
    assert len(proto.conn.bus.log) == n
    assert proto.reg_set(proto.REG_THIGH, 0x1E00, validator=under_100c) == 0x1E00

# Source: protocol_devNB.ipynb | Cell In[28] | 2026-11-16 13:37:17
def test_tolerance(proto):
    sabotaged = proto.CFG_POR & ~proto.field_mask(proto.F_RES)
    proto.reg_write(proto.REG_CONFIG, sabotaged)
    delta = abs(proto.reg_read(proto.REG_CONFIG) - sabotaged)
    with pytest.raises(ValueError, match="tol"):
        proto.reg_set(proto.REG_CONFIG, sabotaged, tolerance=delta - 1)
    landed = proto.reg_set(proto.REG_CONFIG, sabotaged, tolerance=delta)
    assert proto.cached(proto.REG_CONFIG) == landed

# Source: protocol_devNB.ipynb | Cell In[29] | 2026-11-16 13:37:18
def test_reg_write_leaves_cache_stale_and_truncates(proto):
    proto.reg_get(proto.REG_THIGH)
    proto.reg_write(proto.REG_THIGH, 0x2800)
    assert proto.reg_get(proto.REG_THIGH) != 0x2800
    proto.reg_write(proto.REG_THIGH, 0xABCDEF)
    assert proto.reg_read(proto.REG_THIGH) == 0xCDEF

# Source: protocol_devNB.ipynb | Cell In[30] | 2026-11-16 13:37:18
def test_endianness_flip_on_instance(proto):
    normal = proto.reg_read(proto.REG_CONFIG)
    proto.BIG_ENDIAN = False
    swapped = proto.reg_read(proto.REG_CONFIG)
    del proto.BIG_ENDIAN
    assert swapped == ((normal & 0xFF) << 8) | (normal >> 8)
    assert DemoRegs.BIG_ENDIAN is True

# Source: protocol_devNB.ipynb | Cell In[32] | 2026-11-16 13:37:18
class FakeGasSensor(CommandI2CBase, I2CDevice):
    ALLOWED_ADDRS = (0x62,)


def gas_sensor():
    """A ``FakeGasSensor`` on a bus with a ``FakeSensirion`` answering 0xBEEF, 0x1234."""
    bus = FakeBus()
    part = bus.attach(0x62, FakeSensirion([0xBEEF, 0x1234], CommandI2CBase.crc))
    return FakeGasSensor(I2CConnection(bus, 0x62)), part

# Source: protocol_devNB.ipynb | Cell In[35] | 2026-11-16 13:37:18
def test_words_read_verifies_each_word():
    bus = FakeBus()
    part = bus.attach(0x62, FakeSensirion([0xBEEF, 0x1234], CommandI2CBase.crc))
    dev = FakeGasSensor(I2CConnection(bus, 0x62))
    dev.cmd_send(0x2C06)
    assert part.commands == [0x2C06]
    assert dev.words_read(2) == [0xBEEF, 0x1234]

    part.crc_fn = lambda pair: CommandI2CBase.crc(pair) ^ 1
    with pytest.raises(ValueError, match="word 0"):
        dev.words_read(2)
    assert dev.words_read(2, check=False) == [0xBEEF, 0x1234]

# Source: protocol_devNB.ipynb | Cell In[36] | 2026-11-16 13:37:18
def test_bad_crc_on_a_later_word_names_that_word():
    dev, part = gas_sensor()
    part.crc_fn = lambda pair: CommandI2CBase.crc(pair) ^ (pair == b"\x12\x34")
    assert dev.words_read(1) == [0xBEEF]
    with pytest.raises(ValueError, match="word 1"):
        dev.words_read(2)

# Source: protocol_devNB.ipynb | Cell In[37] | 2026-11-16 13:37:18
def test_words_read_is_one_transfer_of_three_bytes_per_word():
    dev, _ = gas_sensor()
    dev.words_read(2)
    assert dev.conn.bus.log == [("r", 0x62, 6)]

# Source: protocol_devNB.ipynb | Cell In[38] | 2026-11-16 13:37:19
def test_cmd_send_width_follows_cmd_bytes():
    dev, part = gas_sensor()
    dev.cmd_send(0x2C06)
    dev.CMD_BYTES = 1  # an 8-bit opcode part
    dev.cmd_send(0x1F)
    assert dev.conn.bus.log == [("w", 0x62, b"\x2c\x06"), ("w", 0x62, b"\x1f")]
    assert part.commands == [0x2C06, 0x1F]

# Source: protocol_devNB.ipynb | Cell In[39] | 2026-11-16 13:37:19
def test_sensirion_crc_defaults():
    assert (CommandI2CBase.CRC_POLY, CommandI2CBase.CRC_INIT) == (0x31, 0xFF)
    assert CommandI2CBase.crc(b"\xbe\xef") == 0x92  # Sensirion datasheet example

