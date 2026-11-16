import pytest

from micropython_i2c.debug import (
    bin_bytes,
    check,
    check_counts,
    check_raises,
    check_reset,
    check_summary,
    info,
    show,
    show_reg,
)
# Source: debug_devNB.ipynb | Cell In[10] | 2026-11-16 13:35:24
class StubRegisterDevice:
    """Just enough of a register driver for ``show_reg``: ``REG_BYTES`` and ``reg_read``."""

    REG_BYTES = 2

    def __init__(self, regs):
        self.regs = regs
        self.reads = []

    def reg_read(self, reg, nbytes):
        self.reads.append((reg, nbytes))
        return self.regs[reg]

# Source: debug_devNB.ipynb | Cell In[14] | 2026-11-16 13:35:24
def test_bin_bytes_groups_msb_first():
    assert bin_bytes(0x60A0) == "01100000 10100000"
    assert bin_bytes(0x15, 1) == "00010101"
    assert bin_bytes(0x0102, 3) == "00000000 00000001 00000010"

# Source: debug_devNB.ipynb | Cell In[15] | 2026-11-16 13:35:25
def test_show_line_format(capsys):
    show(0x60A0, "config")
    show(0x15, "DEVICE_STATUS", 1)
    assert capsys.readouterr().out == (
        "config           0x60a0  01100000 10100000  24736\n"
        "DEVICE_STATUS    0x15  00010101  21\n"
    )

# Source: debug_devNB.ipynb | Cell In[16] | 2026-11-16 13:35:25
def test_show_pads_a_zero_high_byte(capsys):
    show(0x00A0)
    assert capsys.readouterr().out.split()[0] == "0x00a0"

# Source: debug_devNB.ipynb | Cell In[17] | 2026-11-16 13:35:25
def test_show_refuses_a_value_that_does_not_fit(capsys):
    with pytest.raises(ValueError, match="0x1ff does not fit in 1 byte"):
        show(0x1FF, nbytes=1)
    with pytest.raises(ValueError, match="does not fit in 2 byte"):
        show(-1)
    assert capsys.readouterr().out == ""

# Source: debug_devNB.ipynb | Cell In[18] | 2026-11-16 13:35:25
def test_show_reg_uses_reg_bytes_and_the_address_label(capsys):
    dev = StubRegisterDevice({0x01: 0x60A0})
    show_reg(dev, 0x01)
    assert dev.reads == [(0x01, 2)]
    assert capsys.readouterr().out.startswith("reg 0x01         0x60a0  ")

# Source: debug_devNB.ipynb | Cell In[19] | 2026-11-16 13:35:26
def test_show_reg_width_and_label_override(capsys):
    dev = StubRegisterDevice({0x0D: 0x0F})
    show_reg(dev, 0x0D, "CONV_STATUS", 1)
    assert dev.reads == [(0x0D, 1)]
    assert capsys.readouterr().out == "CONV_STATUS      0x0f  00001111  15\n"

# Source: debug_devNB.ipynb | Cell In[30] | 2026-11-16 13:35:26
def test_check_counts_passes_and_fails_and_returns_the_result():
    check_reset()
    assert check("a", True) is True
    assert check("b", False) is False
    assert check("c", 1) is True
    assert check_counts() == (2, 1)
    check_reset()

# Source: debug_devNB.ipynb | Cell In[31] | 2026-11-16 13:35:26
def test_check_coerces_the_result_to_bool():
    check_reset()
    assert check("non-empty list", [0]) is True
    assert check("zero", 0) is False
    assert check("None", None) is False
    assert check_counts() == (1, 2)
    check_reset()

# Source: debug_devNB.ipynb | Cell In[32] | 2026-11-16 13:35:27
def test_check_line_format(capsys):
    check_reset()
    check("WHO_AM_I", True, "0x35")
    check("ready", False)
    assert capsys.readouterr().out == (
        "  [PASS] WHO_AM_I" + " " * 28 + " 0x35\n"
        "  [FAIL] ready" + " " * 31 + " \n"
    )
    check_reset()

# Source: debug_devNB.ipynb | Cell In[33] | 2026-11-16 13:35:27
def test_check_line_keeps_a_long_label_whole(capsys):
    check_reset()
    label = "x" * 40
    check(label, True, "d")
    assert capsys.readouterr().out == "  [PASS] " + label + " d\n"
    check_reset()

# Source: debug_devNB.ipynb | Cell In[34] | 2026-11-16 13:35:27
def test_check_raises_passes_and_returns_the_exception(capsys):
    check_reset()
    err = check_raises("bad int", ValueError, int, "x")
    assert isinstance(err, ValueError)
    assert check_counts() == (1, 0)
    out = capsys.readouterr().out
    assert out == "  [PASS] bad int" + " " * 29 + " \n" + " " * 10 + str(err) + "\n"
    check_reset()

# Source: debug_devNB.ipynb | Cell In[35] | 2026-11-16 13:35:28
def test_check_raises_fails_when_nothing_is_raised(capsys):
    check_reset()
    assert check_raises("int('7')", ValueError, int, "7") is None
    assert check_counts() == (0, 1)
    assert capsys.readouterr().out == "  [FAIL] int('7')" + " " * 28 + " it did not raise\n"
    check_reset()

# Source: debug_devNB.ipynb | Cell In[36] | 2026-11-16 13:35:28
def test_check_raises_lets_another_exception_through():
    check_reset()
    with pytest.raises(ValueError):
        check_raises("wrong type", KeyError, int, "x")
    assert check_counts() == (0, 0)

# Source: debug_devNB.ipynb | Cell In[37] | 2026-11-16 13:35:28
def test_check_raises_passes_args_and_kwargs():
    check_reset()
    seen = []

    def refuse(*args, **kwargs):
        seen.append((args, kwargs))
        raise OSError(19, "ENODEV")

    err = check_raises("refused", OSError, refuse, 1, 2, addr=0x48)
    assert seen == [((1, 2), {"addr": 0x48})]
    assert err.args == (19, "ENODEV")
    check_reset()

# Source: debug_devNB.ipynb | Cell In[38] | 2026-11-16 13:35:28
def test_info_is_printed_but_not_counted(capsys):
    check_reset()
    info("temperature 22.9 C")
    assert capsys.readouterr().out == "  [info] temperature 22.9 C\n"
    assert check_counts() == (0, 0)

# Source: debug_devNB.ipynb | Cell In[39] | 2026-11-16 13:35:29
def test_check_reset_zeroes_the_tally():
    check("a", True)
    check("b", False)
    check_reset()
    assert check_counts() == (0, 0)

# Source: debug_devNB.ipynb | Cell In[40] | 2026-11-16 13:35:29
def test_check_counts_prints_nothing(capsys):
    check_reset()
    check("a", True)
    capsys.readouterr()
    assert check_counts() == (1, 0)
    assert capsys.readouterr().out == ""
    check_reset()

# Source: debug_devNB.ipynb | Cell In[41] | 2026-11-16 13:35:29
def test_check_summary_prints_and_returns_the_tally(capsys):
    check_reset()
    check("a", True)
    check("b", True)
    check("c", False)
    capsys.readouterr()
    assert check_summary() == (2, 1)
    assert capsys.readouterr().out == "2 passed, 1 failed\n"
    assert check_counts() == (2, 1)  # the summary does not reset
    check_reset()

