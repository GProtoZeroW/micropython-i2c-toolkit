import pytest

from micropython_i2c.sim.bus import FakeBus, FakeMux, FakeRegisterPart, FakeSensirion
# Source: sim_bus_devNB.ipynb | Cell In[11] | 2026-11-16 13:24:20
class Listener(FakeRegisterPart):
    """A register part that records the address bytes it hears."""

    def __init__(self):
        super().__init__()
        self.heard = []

    def hear(self, byte):
        self.heard.append(byte)

# Source: sim_bus_devNB.ipynb | Cell In[16] | 2026-11-16 13:24:20
def test_part_behind_a_mux_answers_only_while_its_channel_is_on():
    bus = FakeBus()
    bus.attach(0x70, FakeMux())
    bus.attach(0x48, FakeRegisterPart({0: 0x42}), path=[(0x70, 2)])
    with pytest.raises(OSError):
        bus.readfrom_mem(0x48, 0, 1)
    bus.writeto(0x70, bytes([0b100]))
    assert bus.readfrom_mem(0x48, 0, 1) == b"\x42"
    bus.writeto(0x70, bytes([0b001]))
    assert bus.scan() == [0x70]

# Source: sim_bus_devNB.ipynb | Cell In[17] | 2026-11-16 13:24:20
def test_nested_muxes_need_every_hop_on():
    bus = FakeBus()
    bus.attach(0x70, FakeMux())
    bus.attach(0x71, FakeMux(), path=[(0x70, 1)])
    bus.attach(0x48, FakeRegisterPart(), path=[(0x70, 1), (0x71, 3)])
    bus.writeto(0x70, bytes([0b10]))
    bus.writeto(0x71, bytes([0b1000]))
    assert 0x48 in bus.scan()
    bus.writeto(0x70, bytes([0]))
    assert bus.scan() == [0x70]

# Source: sim_bus_devNB.ipynb | Cell In[18] | 2026-11-16 13:24:21
def test_two_parts_at_one_address_collide_on_read_but_both_get_a_write():
    bus = FakeBus()
    a = bus.attach(0x48, FakeSensirion([1], lambda b: 0))
    b = bus.attach(0x48, FakeSensirion([2], lambda b: 0))
    bus.writeto(0x48, b"\x12\x34")
    assert a.commands == b.commands == [0x1234]
    with pytest.raises(RuntimeError, match="collision at 0x48"):
        bus.readfrom(0x48, 3)

# Source: sim_bus_devNB.ipynb | Cell In[19] | 2026-11-16 13:24:21
def test_a_part_with_bus_addr_answers_there():
    bus = FakeBus()
    part = bus.attach(0x29, FakeRegisterPart())
    part.bus_addr = 0x30
    assert bus.scan() == [0x30]
    with pytest.raises(OSError):
        bus.readfrom(0x29, 1)
    bus.readfrom(0x30, 1)

# Source: sim_bus_devNB.ipynb | Cell In[20] | 2026-11-16 13:24:21
def test_visible_parts_hear_every_address_byte_first():
    bus = FakeBus()
    ear = bus.attach(0x22, Listener())
    bus.attach(0x48, FakeRegisterPart())
    bus.readfrom_mem(0x48, 0, 1)
    bus.writeto(0x48, b"\x00")
    assert ear.heard == [0x90, 0x91, 0x90]  # write + repeated-start read, then a write

# Source: sim_bus_devNB.ipynb | Cell In[21] | 2026-11-16 13:24:21
def test_register_part_auto_increments_and_keeps_read_only_bits():
    part = FakeRegisterPart({0xFF: 0xAA, 0: 0xBB}, read_only={1: 0xF0}, size=256)
    assert part.mem_read(0xFF, 2) == b"\xaa\xbb"  # wraps at size
    part.mem_write(1, b"\xff")
    assert part.regs[1] == 0x0F

# Source: sim_bus_devNB.ipynb | Cell In[22] | 2026-11-16 13:24:21
def test_sensirion_part_sends_a_crc_after_each_word():
    part = FakeSensirion([0xBEEF, 0x0001], lambda pair: pair[0] ^ pair[1])
    assert part.raw_read(6) == bytes([0xBE, 0xEF, 0xBE ^ 0xEF, 0x00, 0x01, 0x01])
    assert part.raw_read(2) == b"\xbe\xef"

# Source: sim_bus_devNB.ipynb | Cell In[23] | 2026-11-16 13:24:22
def test_log_records_every_transfer():
    bus = FakeBus()
    bus.attach(0x48, FakeRegisterPart())
    bus.writeto_mem(0x48, 1, b"\x60")
    bus.readfrom_mem(0x48, 1, 1)
    bus.writeto(0x48, b"\x00")
    bus.readfrom(0x48, 2)
    assert bus.log == [
        ("wm", 0x48, 1, b"\x60"),
        ("rm", 0x48, 1, 1),
        ("w", 0x48, b"\x00"),
        ("r", 0x48, 2),
    ]

