import pytest

from micropython_i2c import CommandI2CBase, NullLock
from micropython_i2c.devices import registry
from micropython_i2c.manager import I2CManager, scan_check
from micropython_i2c.routing import RoutingTable
from micropython_i2c.sim.bus import FakeBus, FakeMux, FakeRegisterPart, FakeSensirion
# Source: manager_devNB.ipynb | Cell In[8] | 2026-11-16 13:29:45
REG = registry(["TCA9548A", "SHTC3", "TMAG5273"])


def config(**overrides):
    """The table the fake topology implements."""
    cfg = {
        "buses": [0],
        "muxes": {"mux_a": {"bus": 0, "address": "0x71", "driver": "TCA9548A"}},
        "devices": [
            {"name": "humidity", "bus": 0, "address": "0x70", "driver": "SHTC3"},
            {"name": "hall", "bus": 0, "address": "0x35", "driver": "TMAG5273",
             "path": [["mux_a", 3]]},
        ],
    }  # fmt: skip
    cfg.update(overrides)
    return cfg


def fake_topology():
    """A FakeBus wired as ``config()`` says."""
    bus = FakeBus()
    bus.attach(0x71, FakeMux())
    bus.attach(0x70, FakeSensirion([0x0807], CommandI2CBase.crc))
    bus.attach(0x35, FakeRegisterPart({0x0E: 0x49, 0x0F: 0x54, 0x0D: 0x01}), path=[(0x71, 3)])
    return bus

# Source: manager_devNB.ipynb | Cell In[12] | 2026-11-16 13:29:45
def test_scan_check_finds_everything_and_releases_mux():
    bus = fake_topology()
    table = RoutingTable.loads(config()).validate(REG)
    ok, results = scan_check(table, {0: bus}, verbose=False)
    assert ok and len(results) == 3
    assert bus.nodes[0][2].state == 0  # mux left deselected

# Source: manager_devNB.ipynb | Cell In[13] | 2026-11-16 13:29:46
def test_scan_check_reports_missing():
    bus = fake_topology()
    cfg = config()
    cfg["devices"][1]["address"] = "0x36"
    table = RoutingTable.loads(cfg).validate()
    ok, results = scan_check(table, {0: bus}, verbose=False)
    assert not ok
    assert [r["label"] for r in results if not r["present"]] == ["hall"]

# Source: manager_devNB.ipynb | Cell In[19] | 2026-11-16 13:29:46
def test_manager_builds_and_talks():
    bus = fake_topology()
    table = RoutingTable.loads(config()).validate(REG)
    mgr = I2CManager(table, REG, "feathers3")
    mgr.buses, mgr.locks = {0: bus}, {0: NullLock()}  # stand in for buses_build()
    mgr.build()
    assert mgr["hall"].identify() == (0x5449, 1)
    assert mgr["humidity"].identify()
    mgr["mux_a"].channel_select(2)
    assert mgr["mux_a"].control_read() == 0x04
    with pytest.raises(KeyError):
        mgr["nope"]

# Source: manager_devNB.ipynb | Cell In[20] | 2026-11-16 13:29:46
def test_bus_factory_builds_each_bus_at_its_clock():
    calls = []

    def factory(bus_id, cfg, freq):
        calls.append((bus_id, cfg["scl"], freq))
        return fake_topology()

    mgr = I2CManager(RoutingTable.loads(config()).validate(REG), REG, "feathers3", factory)
    mgr.build()
    assert calls == [(0, 9, 400_000)]
    assert set(mgr.locks) == {0}
    assert mgr["hall"].identify() == (0x5449, 1)

# Source: manager_devNB.ipynb | Cell In[21] | 2026-11-16 13:29:47
def test_build_skips_stages_already_built():
    mgr = I2CManager(RoutingTable.loads(config()).validate(REG), REG, "feathers3", lambda *a: 1 / 0)
    mgr.buses, mgr.locks = {0: fake_topology()}, {0: NullLock()}
    mgr.build()  # the factory would raise if buses_build ran again
    assert set(mgr.devices) == {"humidity", "hall"}

# Source: manager_devNB.ipynb | Cell In[22] | 2026-11-16 13:29:47
def test_bus_freq_is_lowest_ceiling():
    table = RoutingTable.loads(config()).validate(REG)
    mgr = I2CManager(table, REG, "feathers3")
    assert mgr.bus_freq(0) == 400_000  # TCA9548A and the board both cap at 400 kHz

# Source: manager_devNB.ipynb | Cell In[23] | 2026-11-16 13:29:47
def test_bus_freq_refuses_a_cap_below_every_part():
    slow = {"buses": {0: {"scl": 0, "sda": 0, "baud_max": 5_000}}}
    mgr = I2CManager(RoutingTable.loads(config()).validate(REG), REG, slow)
    with pytest.raises(ValueError, match="no clock satisfies every part"):
        mgr.bus_freq(0)

# Source: manager_devNB.ipynb | Cell In[24] | 2026-11-16 13:29:48
def test_without_a_factory_the_host_cannot_build_buses():
    mgr = I2CManager(RoutingTable.loads(config()).validate(REG), REG, "feathers3")
    with pytest.raises(ImportError):
        mgr.buses_build()

