"""CRCMixin, offline. Ported from the old notebook's section 10.5-10.11.

Vectors come from three independent sources: the SHTC3 datasheet (Table 16), the
TMAG5273 datasheet (section 7.4.1, clean page render) and the standard ``123456789``
check values for four catalogued variants spanning both widths and both reflections.
"""

import pytest

from micropython_i2c import BitTools, CommandI2CBase, CRCMixin


def crc_class(name="C", **params):
    return type(name, (CRCMixin,), params)


SHTC3 = crc_class("SHTC3", CRC_POLY=0x31, CRC_INIT=0xFF)
TMAG = crc_class("TMAG", CRC_POLY=0x07, CRC_INIT=0xFF)

PARAM_SETS = {
    "CRC-8/SMBUS": dict(CRC_WIDTH=8, CRC_POLY=0x07, CRC_INIT=0x00),
    "SHTC3": dict(CRC_WIDTH=8, CRC_POLY=0x31, CRC_INIT=0xFF),
    "TMAG5273": dict(CRC_WIDTH=8, CRC_POLY=0x07, CRC_INIT=0xFF),
    "CRC-8/MAXIM": dict(CRC_WIDTH=8, CRC_POLY=0x31, CRC_INIT=0x00,
                        CRC_REFLECT_IN=True, CRC_REFLECT_OUT=True),
    "CRC-16/CCITT-FALSE": dict(CRC_WIDTH=16, CRC_POLY=0x1021, CRC_INIT=0xFFFF),
    "CRC-16/ARC": dict(CRC_WIDTH=16, CRC_POLY=0x8005, CRC_INIT=0x0000,
                       CRC_REFLECT_IN=True, CRC_REFLECT_OUT=True),
}  # fmt: skip
CHECK_VALUES = {"CRC-8/SMBUS": 0xF4, "CRC-8/MAXIM": 0xA1, "CRC-16/CCITT-FALSE": 0x29B1,
                "CRC-16/ARC": 0xBB3D}  # fmt: skip
CORPUS = (b"\x00", b"\xff", b"\x12\x34", b"\xbe\xef", b"123456789", b"\x00" * 4, b"\xff" * 4,
          bytes(range(32)), bytes(range(255, 223, -1)))  # fmt: skip


def test_unset_polynomial_raises_everywhere():
    unset = crc_class("Unconfigured")
    for fn in (lambda: unset.crc(b"\x00"), lambda: unset.crc_ok(b"\x00", 0),
               lambda: unset.crc_verify(b"\x00", 0), unset.crc_table):  # fmt: skip
        with pytest.raises(NotImplementedError, match="Unconfigured"):
            fn()


@pytest.mark.parametrize("data, want", [(b"\x00", 0xAC), (b"\xbe\xef", 0x92)])
def test_shtc3_vectors(data, want):
    assert SHTC3.crc(data) == want
    assert CommandI2CBase.crc(data) == want  # Sensirion defaults


@pytest.mark.parametrize(
    "data, want", [(b"\xe0", 0x5D), (b"\x00\x00\x00\x00", 0xD1), (b"\xff\xff\xff\xff", 0x0F)]
)
def test_tmag5273_vectors(data, want):
    assert TMAG.crc(data) == want


@pytest.mark.parametrize("name", sorted(CHECK_VALUES))
def test_standard_check_value(name):
    assert crc_class(**PARAM_SETS[name]).crc(b"123456789") == CHECK_VALUES[name]


def test_width_masks():
    c8, c16 = crc_class(**PARAM_SETS["CRC-8/SMBUS"]), crc_class(**PARAM_SETS["CRC-16/ARC"])
    assert (c8.crc_mask(), c16.crc_mask()) == (0xFF, 0xFFFF)
    assert (c8.crc_topbit(), c16.crc_topbit()) == (0x80, 0x8000)


def test_parameters_in_isolation():
    base_params = dict(CRC_POLY=0x07, CRC_INIT=0xFF)
    data = b"\x12\x34\x56"
    base = crc_class(**base_params).crc(data)
    assert crc_class(**base_params, CRC_XOR_OUT=0xFF).crc(data) == base ^ 0xFF
    assert crc_class(**base_params, CRC_REFLECT_OUT=True).crc(data) == BitTools.bit_reverse(base)
    assert crc_class(**base_params, CRC_REFLECT_IN=True).crc(data) != base


def test_init_override_and_resume():
    data = b"\x12\x34\x56"
    whole = TMAG.crc(data)
    assert TMAG.crc(data, init=0x00) != whole
    assert TMAG.crc(data[2:], init=TMAG.crc(data[:2])) == whole


@pytest.mark.parametrize("name", sorted(PARAM_SETS))
def test_table_matches_bitwise(name):
    bitwise = crc_class(**PARAM_SETS[name])
    tabled = crc_class(**PARAM_SETS[name], CRC_TABLE_ENABLE=True)
    for data in CORPUS:
        assert bitwise.crc(data) == tabled.crc(data)


def test_table_lazy_cached_and_clearable():
    tbl = crc_class(CRC_POLY=0x07, CRC_TABLE_ENABLE=True)
    assert "_crc_tbl" not in tbl.__dict__
    tbl.crc(b"\x00")
    assert len(tbl.crc_table()) == 256
    assert tbl.crc_table() is tbl.crc_table()
    tbl.crc_table_clear()
    assert "_crc_tbl" not in tbl.__dict__


def test_subclass_builds_own_table():
    parent = crc_class("Parent", CRC_POLY=0x07, CRC_INIT=0xFF, CRC_TABLE_ENABLE=True)
    child = type("Child", (parent,), {"CRC_POLY": 0x31})
    parent.crc(b"\x00")
    assert child.crc(b"\x00") == 0xAC
    assert parent.crc(b"\xe0") == 0x5D
    assert parent.crc_table() is not child.crc_table()


def test_ok_and_verify():
    good = SHTC3.crc(b"\xbe\xef")
    assert SHTC3.crc_ok(b"\xbe\xef", good)
    assert not SHTC3.crc_ok(b"\xbe\xef", good ^ 0xFF)
    assert SHTC3.crc_verify(b"\xbe\xef", good) == good
    with pytest.raises(ValueError, match="word 3"):
        SHTC3.crc_verify(b"\xbe\xef", 0x00, context="word 3")


def test_every_single_bit_flip_caught():
    good = SHTC3.crc(b"\xbe\xef")
    for bit in range(16):
        corrupt = (0xBEEF ^ (1 << bit)).to_bytes(2, "big")
        assert not SHTC3.crc_ok(corrupt, good)


def test_tmag_scope_includes_command_byte():
    cmd = bytes([(0x35 << 1) | 1])
    payload = bytes([0x12, 0x34, 0x56, 0x78, 0x10])
    crc = TMAG.crc(cmd + payload)
    assert TMAG.crc_ok(cmd + payload, crc)
    assert not TMAG.crc_ok(payload, crc)
