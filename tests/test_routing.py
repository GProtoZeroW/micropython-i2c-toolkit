"""RoutingTable validation, connection routing, scan_check and I2CManager on a fake bus."""

import pytest

from micropython_i2c import CommandI2CBase, I2CConnection, NullLock
from micropython_i2c.drivers import registry
from micropython_i2c.fakes import FakeBus, FakeMux, FakeRegisterPart, FakeSensirion
from micropython_i2c.manager import I2CManager, scan_check
from micropython_i2c.routing import ConfigError, RoutingTable

REG = registry()


def config(**overrides):
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


def problems(cfg, registry=REG):
    with pytest.raises(ConfigError) as info:
        RoutingTable.loads(cfg).validate(registry)
    return "\n".join(info.value.problems)


def test_valid_config_passes_and_parses_hex():
    table = RoutingTable.loads(config()).validate(REG)
    assert table.muxes["mux_a"]["address"] == 0x71
    assert table.resolve_path(table.devices[1]["path"]) == [[0x71, 3]]


def test_json_text_accepted():
    RoutingTable.loads('{"buses": [0], "devices": []}').validate()


def test_unknown_mux_and_undeclared_bus():
    cfg = config(buses=[1])
    cfg["devices"][1]["path"] = [["nope", 0]]
    text = problems(cfg)
    assert "unknown mux 'nope'" in text and "not in declared buses" in text


def test_duplicate_names():
    cfg = config()
    cfg["devices"][1]["name"] = "humidity"
    assert "duplicate device name" in problems(cfg)


def test_path_inconsistent_with_mux_declaration():
    cfg = config()
    cfg["muxes"]["mux_b"] = {"bus": 0, "address": "0x72", "driver": "TCA9548A",
                             "path": [["mux_a", 1]]}  # fmt: skip
    cfg["devices"][1]["path"] = [["mux_b", 0]]  # skips mux_a
    assert "declared at" in problems(cfg)


def test_mux_cycle_rejected():
    cfg = config()
    cfg["muxes"]["mux_a"]["path"] = [["mux_a", 0]]
    assert "mux 'mux_a'" in problems(cfg)


def test_collision_on_same_segment():
    cfg = config()
    cfg["muxes"]["mux_a"]["address"] = "0x70"
    assert "collision" in problems(cfg)


def test_collision_seen_through_upstream_segment():
    cfg = config()
    cfg["devices"][1]["address"] = "0x70"  # behind mux_a, but humidity at 0x70 upstream
    cfg["devices"][1]["driver"] = "SHTC3"
    assert "humidity, hall" in problems(cfg) or "hall, humidity" in problems(cfg)


def test_same_address_on_sibling_channels_is_fine():
    cfg = config()
    cfg["devices"].append({"name": "hall2", "bus": 0, "address": "0x35", "driver": "TMAG5273",
                           "path": [["mux_a", 4]]})  # fmt: skip
    RoutingTable.loads(cfg).validate(REG)


def test_driver_aware_checks():
    cfg = config()
    cfg["devices"][0]["address"] = "0x44"
    cfg["devices"][1]["path"] = [["mux_a", 9]]
    cfg["devices"].append({"name": "x", "bus": 0, "address": 1, "driver": "NOPE"})
    text = problems(cfg)
    assert "not valid for SHTC3" in text
    assert "channel 9 out of range 0..7" in text
    assert "'NOPE' not in registry" in text


def test_driver_checks_skipped_without_registry():
    cfg = config()
    cfg["devices"][0]["address"] = "0x44"
    RoutingTable.loads(cfg).validate()


# --- routing on a fake bus ----------------------------------------------------------------


def fake_topology():
    bus = FakeBus()
    bus.attach(0x71, FakeMux())
    bus.attach(0x70, FakeSensirion([0x0807], CommandI2CBase.crc))
    bus.attach(0x35, FakeRegisterPart({0x0E: 0x49, 0x0F: 0x54, 0x0D: 0x01}), path=[(0x71, 3)])
    return bus


def test_connection_selects_path_each_transfer():
    bus = fake_topology()
    conn = I2CConnection(bus, 0x35, [[0x71, 3]], NullLock())
    with pytest.raises(OSError):
        bus.readfrom_mem(0x35, 0x0E, 1)  # unreachable until the mux is switched
    assert conn.readfrom_mem(0x0E, 2) == b"\x49\x54"
    assert bus.log[-2] == ("w", 0x71, b"\x08")


def test_scan_check_finds_everything_and_releases_mux():
    bus = fake_topology()
    table = RoutingTable.loads(config()).validate(REG)
    ok, results = scan_check(table, {0: bus}, verbose=False)
    assert ok and len(results) == 3
    assert bus.nodes[0][2].state == 0  # mux left deselected


def test_scan_check_reports_missing():
    bus = fake_topology()
    cfg = config()
    cfg["devices"][1]["address"] = "0x36"
    table = RoutingTable.loads(cfg).validate()
    ok, results = scan_check(table, {0: bus}, verbose=False)
    assert not ok
    assert [r["label"] for r in results if not r["present"]] == ["hall"]


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


def test_bus_freq_is_lowest_ceiling():
    table = RoutingTable.loads(config()).validate(REG)
    mgr = I2CManager(table, REG, "feathers3")
    assert mgr.bus_freq(0) == 400_000  # TCA9548A and the board both cap at 400 kHz


# --- the bench as wired: mux on bus 0, VL53L4CD + relay on bus 1 -------------------------

BENCH = {
    "buses": [0, 1],
    "muxes": {"mux": {"bus": 0, "address": "0x71", "driver": "TCA9548A"}},
    "devices": [
        {"name": "humidity", "bus": 0, "address": "0x70", "driver": "SHTC3",
         "path": [["mux", 0]]},
        {"name": "temp", "bus": 0, "address": "0x48", "driver": "TMP102",
         "path": [["mux", 2]]},
        {"name": "tof", "bus": 1, "address": "0x29", "driver": "VL53L4CD"},
        {"name": "relay", "bus": 1, "address": "0x18", "driver": "QwiicRelay"},
    ],
}  # fmt: skip


def test_bench_config_validates():
    RoutingTable.loads(BENCH).validate(REG)


def test_bench_mux_at_default_address_collides_with_shtc3():
    cfg = {**BENCH, "muxes": {"mux": {"bus": 0, "address": "0x70", "driver": "TCA9548A"}}}
    assert "collision on bus 0 at 0x70" in problems(cfg)


def test_bench_builds_on_fake_buses():
    from micropython_i2c.fakes import FakeRelay, FakeSHTC3, FakeTMP102, fake_vl53l4cd

    bus0, bus1 = FakeBus(), FakeBus()
    bus0.attach(0x71, FakeMux())
    bus0.attach(0x70, FakeSHTC3(rh=50.0), path=[(0x71, 0)])
    bus0.attach(0x48, FakeTMP102(), path=[(0x71, 2)])
    bus1.attach(0x29, fake_vl53l4cd())
    bus1.attach(0x18, FakeRelay())

    table = RoutingTable.loads(BENCH).validate(REG)
    ok, _ = scan_check(table, {0: bus0, 1: bus1}, verbose=False)
    assert ok
    mgr = I2CManager(table, REG, "feathers3")
    mgr.buses, mgr.locks = {0: bus0, 1: bus1}, {0: NullLock(), 1: NullLock()}
    mgr.build()
    assert mgr["temp"].temperature() == 25.0
    assert mgr["humidity"].measure()[1] == 50.0
    mgr["relay"].on()
    assert mgr["relay"].state
    assert mgr.bus_freq(1) == 400_000
