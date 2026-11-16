import pytest

from micropython_i2c.bits import BitTools
# Source: bits_devNB.ipynb | Cell In[11] | 2026-11-16 13:29:36
@pytest.mark.parametrize("bits", [8, 12, 13, 16, 24])
def test_sign_extend_boundaries(bits):
    hi = (1 << (bits - 1)) - 1
    assert BitTools.sign_extend(hi, bits) == hi
    assert BitTools.sign_extend(1 << (bits - 1), bits) == -(1 << (bits - 1))
    assert BitTools.sign_extend((1 << bits) - 1, bits) == -1
    assert BitTools.sign_extend(0, bits) == 0

# Source: bits_devNB.ipynb | Cell In[12] | 2026-11-16 13:29:37
def test_sign_extend_masks_oversized_input():
    assert BitTools.sign_extend(0xFFFFF, 12) == BitTools.sign_extend(0xFFF, 12) == -1

# Source: bits_devNB.ipynb | Cell In[13] | 2026-11-16 13:29:37
def test_sign_extend_exhaustive_8_bit():
    for v in range(256):
        assert BitTools.sign_extend(v, 8) == (v - 256 if v & 0x80 else v)

# Source: bits_devNB.ipynb | Cell In[14] | 2026-11-16 13:29:37
@pytest.mark.parametrize(
    "word, em, counts",
    [(0xE700, False, -400), (0x3200, False, 800), (0xF380, True, -400), (0x3200, True, 1600)],
)
def test_sign_extend_after_shift_tmp102(word, em, counts):
    shift, bits = (3, 13) if em else (4, 12)
    assert BitTools.sign_extend(word >> shift, bits) == counts

# Source: bits_devNB.ipynb | Cell In[15] | 2026-11-16 13:29:37
def test_sign_extend_unshifted_is_wrong():
    assert BitTools.sign_extend(0xE700, 12) != -400

# Source: bits_devNB.ipynb | Cell In[16] | 2026-11-16 13:29:38
TMP102_FIELDS = (
    ("OS", (15, 1), 0x8000),
    ("RES", (13, 2), 0x6000),
    ("FQ", (11, 2), 0x1800),
    ("POL", (10, 1), 0x0400),
    ("TM", (9, 1), 0x0200),
    ("SD", (8, 1), 0x0100),
    ("CR", (6, 2), 0x00C0),
    ("AL", (5, 1), 0x0020),
    ("EM", (4, 1), 0x0010),
)
CFG_POR = 0x60A0

# Source: bits_devNB.ipynb | Cell In[19] | 2026-11-16 13:29:38
@pytest.mark.parametrize("name, field, mask", TMP102_FIELDS)
def test_field_mask(name, field, mask):
    assert BitTools.field_mask(field) == mask

# Source: bits_devNB.ipynb | Cell In[20] | 2026-11-16 13:29:38
def test_fields_tile_without_overlap():
    covered = 0
    for _, field, _ in TMP102_FIELDS:
        m = BitTools.field_mask(field)
        assert covered & m == 0
        covered |= m
    assert covered == 0xFFF0

# Source: bits_devNB.ipynb | Cell In[21] | 2026-11-16 13:29:38
def test_field_extract_por():
    assert BitTools.field_extract(CFG_POR, (13, 2)) == 0b11
    assert BitTools.field_extract(CFG_POR, (6, 2)) == 0b10
    assert BitTools.field_extract(CFG_POR, (5, 1)) == 1

# Source: bits_devNB.ipynb | Cell In[22] | 2026-11-16 13:29:39
@pytest.mark.parametrize("name, field, mask", TMP102_FIELDS)
def test_field_insert_round_trip_and_isolation(name, field, mask):
    for v in range(1 << field[1]):
        out = BitTools.field_insert(CFG_POR, field, v)
        assert BitTools.field_extract(out, field) == v
        assert out & ~mask == CFG_POR & ~mask

# Source: bits_devNB.ipynb | Cell In[23] | 2026-11-16 13:29:39
def test_field_insert_clamps():
    assert BitTools.field_insert(0x0000, (8, 1), 0xFF) == 0x0100

# Source: bits_devNB.ipynb | Cell In[26] | 2026-11-16 13:29:39
@pytest.mark.parametrize(
    "value, bits, want",
    [(0x80, 8, 0x01), (0x01, 8, 0x80), (0xD2, 8, 0x4B), (0x0001, 16, 0x8000),
     (0x8000, 16, 0x0001), (0xFFFF, 16, 0xFFFF), (0, 8, 0), (0x99, 8, 0x99)],
)  # fmt: skip
def test_bit_reverse(value, bits, want):
    assert BitTools.bit_reverse(value, bits) == want

# Source: bits_devNB.ipynb | Cell In[27] | 2026-11-16 13:29:40
def test_bit_reverse_involution():
    assert all(BitTools.bit_reverse(BitTools.bit_reverse(v, 8), 8) == v for v in range(256))

# Source: bits_devNB.ipynb | Cell In[30] | 2026-11-16 13:29:40
@pytest.mark.parametrize(
    "data, be, want",
    [(b"\x12\x34", True, 0x1234), (b"\x12\x34", False, 0x3412), (b"\xff", True, 0xFF),
     (b"\x12\x34\x56", True, 0x123456), (b"\x12\x34\x56", False, 0x563412),
     (b"\x00\x00", True, 0)],
)  # fmt: skip
def test_bytes_to_int(data, be, want):
    assert BitTools.bytes_to_int(data, be) == want

# Source: bits_devNB.ipynb | Cell In[31] | 2026-11-16 13:29:40
@pytest.mark.parametrize(
    "value, n, be, want",
    [(0x1234, 2, True, b"\x12\x34"), (0x1234, 2, False, b"\x34\x12"), (0xFF, 1, True, b"\xff"),
     (0x123456, 3, True, b"\x12\x34\x56"), (0x123456, 3, False, b"\x56\x34\x12")],
)  # fmt: skip
def test_int_to_bytes(value, n, be, want):
    assert BitTools.int_to_bytes(value, n, be) == want

# Source: bits_devNB.ipynb | Cell In[32] | 2026-11-16 13:29:41
def test_int_to_bytes_truncates():
    assert BitTools.int_to_bytes(0xABCDEF, 2) == b"\xcd\xef"
    assert BitTools.int_to_bytes(-1, 1) == b"\xff"

# Source: bits_devNB.ipynb | Cell In[33] | 2026-11-16 13:29:41
@pytest.mark.parametrize("n", [1, 2, 3, 4])
@pytest.mark.parametrize("be", [True, False])
def test_byte_round_trip(n, be):
    for value in (0, 1, (1 << (8 * n)) - 1, (1 << (8 * n)) // 3):
        assert BitTools.bytes_to_int(BitTools.int_to_bytes(value, n, be), be) == value

