"""RegisterI2CBase and CommandI2CBase against fake parts (ported from section 7)."""

import pytest

from micropython_i2c import CommandI2CBase, I2CConnection, I2CDevice
from micropython_i2c.drivers.tmp102 import TMP102Regs
from micropython_i2c.fakes import FakeBus, FakeSensirion, FakeTMP102


@pytest.fixture
def proto():
    bus = FakeBus()
    bus.attach(0x48, FakeTMP102())
    return TMP102Regs(I2CConnection(bus, 0x48))


def test_reg_read_assembles_big_endian(proto):
    assert proto.reg_read(proto.REG_CONFIG) == 0x60A0
    assert proto.reg_read(proto.REG_TEMP) & 0x000F == 0


def test_cache_miss_hit_refresh(proto):
    assert proto.cached(proto.REG_TEMP) is None
    first = proto.reg_get(proto.REG_TEMP)
    proto.conn.bus.nodes[0][2].counts = 0x1A0  # the part converts again (26 C)
    assert proto.reg_get(proto.REG_TEMP) == first  # stale by design
    assert proto.reg_get(proto.REG_TEMP, refresh=True) == 0x1A00


def test_cache_respects_width(proto):
    narrow = proto.reg_get(proto.REG_CONFIG, nbytes=1)
    wide = proto.reg_get(proto.REG_CONFIG, nbytes=2)
    assert (narrow, wide) == (0x60, 0x60A0)


def test_cache_clear(proto):
    for reg in (proto.REG_TEMP, proto.REG_CONFIG, proto.REG_THIGH):
        proto.reg_get(reg)
    proto.cache_clear(proto.REG_TEMP)
    assert proto.REG_TEMP not in proto.reg_cache and proto.REG_CONFIG in proto.reg_cache
    proto.cache_clear()
    assert not proto.reg_cache


def test_config_verify_mask_excludes_os_res_al(proto):
    excluded = (
        proto.field_mask(proto.F_OS) | proto.field_mask(proto.F_RES) | proto.field_mask(proto.F_AL)
    )
    assert proto.CONFIG_VERIFY_MASK & excluded == 0
    assert 0xFFFF & ~proto.CONFIG_VERIFY_MASK == excluded
    assert proto.verify_mask_for(proto.REG_CONFIG) == proto.CONFIG_VERIFY_MASK
    assert proto.verify_mask_for(proto.REG_THIGH) is None


def test_reg_set_masked_vs_exact(proto):
    original = proto.reg_read(proto.REG_CONFIG)
    sabotaged = original & ~proto.field_mask(proto.F_RES)
    with pytest.raises(ValueError, match="exact|mask 0xffff"):
        proto.reg_set(proto.REG_CONFIG, sabotaged, verify_mask=0xFFFF)
    assert proto.reg_set(proto.REG_CONFIG, sabotaged) == original  # RES reads back 11
    assert proto.cached(proto.REG_CONFIG) == original


def test_field_set_leaves_neighbours(proto):
    before = proto.reg_read(proto.REG_CONFIG)
    proto.field_set(proto.REG_CONFIG, proto.F_SD, 1)
    after = proto.reg_read(proto.REG_CONFIG)
    # OS is a status bit (it reads 1 once shutdown has finished the conversion): only the
    # bits inside the verify mask are the register's settings.
    assert (before ^ after) & proto.CONFIG_VERIFY_MASK == proto.field_mask(proto.F_SD)


def test_coded_round_trip_and_reject(proto):
    for hz in (0.25, 1.0, 8.0, 4.0):
        proto.coded_set(proto.REG_CONFIG, proto.F_CR, proto.CONV_RATES, hz)
        assert proto.coded_get(proto.REG_CONFIG, proto.F_CR, proto.CONV_RATES) == hz
    with pytest.raises(ValueError, match="not in"):
        proto.coded_set(proto.REG_CONFIG, proto.F_CR, proto.CONV_RATES, 2.0)


def test_validator_runs_before_bus_traffic(proto):
    def under_100c(v):
        if v > 0x6400:
            raise ValueError("too hot")

    n = len(proto.conn.bus.log)
    with pytest.raises(ValueError, match="too hot"):
        proto.reg_set(proto.REG_THIGH, 0x7000, validator=under_100c)
    assert len(proto.conn.bus.log) == n
    assert proto.reg_set(proto.REG_THIGH, 0x1E00, validator=under_100c) == 0x1E00


def test_tolerance(proto):
    sabotaged = proto.CFG_POR & ~proto.field_mask(proto.F_RES)
    proto.reg_write(proto.REG_CONFIG, sabotaged)
    delta = abs(proto.reg_read(proto.REG_CONFIG) - sabotaged)
    with pytest.raises(ValueError, match="tol"):
        proto.reg_set(proto.REG_CONFIG, sabotaged, tolerance=delta - 1)
    landed = proto.reg_set(proto.REG_CONFIG, sabotaged, tolerance=delta)
    assert proto.cached(proto.REG_CONFIG) == landed


def test_reg_write_leaves_cache_stale_and_truncates(proto):
    proto.reg_get(proto.REG_THIGH)
    proto.reg_write(proto.REG_THIGH, 0x2800)
    assert proto.reg_get(proto.REG_THIGH) != 0x2800
    proto.reg_write(proto.REG_THIGH, 0xABCDEF)
    assert proto.reg_read(proto.REG_THIGH) == 0xCDEF


def test_endianness_flip_on_instance(proto):
    normal = proto.reg_read(proto.REG_CONFIG)
    proto.BIG_ENDIAN = False
    swapped = proto.reg_read(proto.REG_CONFIG)
    del proto.BIG_ENDIAN
    assert swapped == ((normal & 0xFF) << 8) | (normal >> 8)
    assert TMP102Regs.BIG_ENDIAN is True


class FakeGasSensor(CommandI2CBase, I2CDevice):
    ALLOWED_ADDRS = (0x62,)


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
