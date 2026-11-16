import pytest

from micropython_i2c.boards import BOARDS, board_get, bus_open
# Source: boards_devNB.ipynb | Cell In[11] | 2026-11-16 13:29:33
def test_board_get_by_name_and_by_dict():
    assert board_get("feathers3") is BOARDS["feathers3"]
    own = {"buses": {}}
    assert board_get(own) is own

# Source: boards_devNB.ipynb | Cell In[12] | 2026-11-16 13:29:33
def test_unknown_board_names_the_known_ones():
    with pytest.raises(ValueError, match="unknown board 'pico'; known: \\['feathers3'\\]"):
        board_get("pico")

# Source: boards_devNB.ipynb | Cell In[13] | 2026-11-16 13:29:33
def test_every_bus_has_pins_and_a_clock_cap():
    for name, board in BOARDS.items():
        for bus_id, cfg in board["buses"].items():
            assert {"scl", "sda", "baud_max"} <= set(cfg), (name, bus_id)
            assert 10_000 <= cfg["baud_max"] <= 1_000_000

# Source: boards_devNB.ipynb | Cell In[16] | 2026-11-16 13:29:34
def test_bus_open_needs_the_board():
    with pytest.raises(ImportError):
        bus_open("feathers3", 0)

