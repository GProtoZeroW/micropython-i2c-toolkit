import pickle

import pytest

from micropython_i2c.devices import registry
from micropython_i2c.routing import ConfigError, RoutingTable
# Source: routing_devNB.ipynb | Cell In[10] | 2026-11-16 14:35:10
def test_config_error_lists_every_problem():
    err = ConfigError(("a: first", "b: second"))
    assert err.problems == ["a: first", "b: second"]
    assert str(err) == "routing table invalid:\n  a: first\n  b: second"
    assert isinstance(err, Exception)

# Source: routing_devNB.ipynb | Cell In[11] | 2026-11-16 14:35:11
def test_config_error_survives_a_pickle_round_trip():
    err = pickle.loads(pickle.dumps(ConfigError(["a: first", "b: second"])))
    assert err.problems == ["a: first", "b: second"]
    assert str(err) == "routing table invalid:\n  a: first\n  b: second"

# Source: routing_devNB.ipynb | Cell In[17] | 2026-11-16 14:35:11
class StandInPart:
    """A driver class reduced to what ``RoutingTable`` reads."""

    ALLOWED_ADDRS = ()

    @classmethod
    def addr_ok(cls, addr):
        return addr in cls.ALLOWED_ADDRS


def stand_in(name, addrs, channels=None):
    """A ``StandInPart`` subclass called ``name``; a mux when ``channels`` is given."""
    attrs = {"ALLOWED_ADDRS": tuple(addrs)}
    if channels is not None:
        attrs["CHANNELS"] = channels
    return type(name, (StandInPart,), attrs)

# Source: routing_devNB.ipynb | Cell In[18] | 2026-11-16 14:35:11
REG = {
    "SHTC3": stand_in("SHTC3", [0x70]),
    "TMAG5273": stand_in("TMAG5273", range(0x08, 0x79)),
    "TCA9548A": stand_in("TCA9548A", range(0x70, 0x78), channels=8),
    "TMP102": stand_in("TMP102", [0x48, 0x49, 0x4A, 0x4B]),
    "VL53L4CD": stand_in("VL53L4CD", range(0x08, 0x78)),
    "QwiicRelay": stand_in("QwiicRelay", [0x18, 0x19]),
}

# Source: routing_devNB.ipynb | Cell In[19] | 2026-11-16 14:35:11
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

# Source: routing_devNB.ipynb | Cell In[20] | 2026-11-16 14:35:11
def problems(cfg, registry=REG):
    with pytest.raises(ConfigError) as info:
        RoutingTable.loads(cfg).validate(registry)
    return "\n".join(info.value.problems)

# Source: routing_devNB.ipynb | Cell In[21] | 2026-11-16 14:35:11
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

# Source: routing_devNB.ipynb | Cell In[24] | 2026-11-16 14:35:11
def test_valid_config_passes_and_parses_hex():
    table = RoutingTable.loads(config()).validate(REG)
    assert table.muxes["mux_a"]["address"] == 0x71
    assert table.resolve_path(table.devices[1]["path"]) == [[0x71, 3]]

# Source: routing_devNB.ipynb | Cell In[25] | 2026-11-16 14:35:12
def test_json_text_accepted():
    RoutingTable.loads('{"buses": [0], "devices": []}').validate()

# Source: routing_devNB.ipynb | Cell In[26] | 2026-11-16 14:35:12
def test_addresses_parse_from_hex_decimal_and_int():
    cfg = config()
    cfg["devices"][0]["address"] = "112"
    cfg["devices"][1]["address"] = 0x35
    cfg["muxes"]["mux_a"]["address"] = "0X71"
    table = RoutingTable.loads(cfg)
    assert [e["address"] for _, e in table.entries()] == [0x71, 0x70, 0x35]

# Source: routing_devNB.ipynb | Cell In[27] | 2026-11-16 14:35:12
def test_paths_become_tuples_and_defaults_fill_in():
    table = RoutingTable.loads(config())
    assert table.devices[1]["path"] == [("mux_a", 3)]
    assert table.devices[0]["path"] == []
    assert table.devices[0]["init_kwargs"] == {}
    assert table.muxes["mux_a"]["reset_pin"] is None
    assert table.muxes["mux_a"]["path"] == []

# Source: routing_devNB.ipynb | Cell In[28] | 2026-11-16 14:35:13
def test_entries_label_muxes_and_segments_group_by_route():
    table = RoutingTable.loads(config())
    assert [label for label, _ in table.entries()] == ["mux:mux_a", "humidity", "hall"]
    assert table.segments() == {
        (0, ()): [(0x71, "mux:mux_a"), (0x70, "humidity")],
        (0, (("mux_a", 3),)): [(0x35, "hall")],
    }

# Source: routing_devNB.ipynb | Cell In[29] | 2026-11-16 14:35:13
def test_missing_keys_and_bad_addresses_are_listed_together():
    cfg = config()
    del cfg["devices"][0]["driver"]
    del cfg["devices"][1]["name"]
    cfg["muxes"]["mux_a"]["address"] = "zz"
    with pytest.raises(ConfigError) as info:
        RoutingTable.loads(cfg)
    assert info.value.problems == [
        "mux:mux_a: address 'zz' does not parse",
        "humidity: missing required key 'driver'",
        "devices[1]: missing required key 'name'",
    ]

# Source: routing_devNB.ipynb | Cell In[30] | 2026-11-16 14:35:13
def test_a_malformed_path_is_reported():
    cfg = config()
    cfg["devices"][1]["path"] = [["mux_a"]]  # a hop needs a mux and a channel
    with pytest.raises(ConfigError, match="hall: path .* does not parse"):
        RoutingTable.loads(cfg)

# Source: routing_devNB.ipynb | Cell In[36] | 2026-11-16 14:35:14
def test_unknown_mux_and_undeclared_bus():
    cfg = config(buses=[1])
    cfg["devices"][1]["path"] = [["nope", 0]]
    text = problems(cfg)
    assert "unknown mux 'nope'" in text and "not in declared buses" in text

# Source: routing_devNB.ipynb | Cell In[37] | 2026-11-16 14:35:14
def test_duplicate_names():
    cfg = config()
    cfg["devices"][1]["name"] = "humidity"
    assert "duplicate device name" in problems(cfg)

# Source: routing_devNB.ipynb | Cell In[38] | 2026-11-16 14:35:14
def test_name_shared_by_a_device_and_a_mux():
    cfg = config()
    cfg["devices"][0]["name"] = "mux_a"
    assert "name 'mux_a' used by both a device and a mux" in problems(cfg)

# Source: routing_devNB.ipynb | Cell In[39] | 2026-11-16 14:35:15
def test_negative_or_non_int_channel_rejected():
    cfg = config()
    cfg["devices"][1]["path"] = [["mux_a", -1]]
    assert "bad channel -1 on mux 'mux_a'" in problems(cfg)
    cfg["devices"][1]["path"] = [["mux_a", "3"]]
    assert "bad channel '3' on mux 'mux_a'" in problems(cfg)
    cfg["devices"][1]["path"] = [["mux_a", True]]  # a bool is not a channel number
    assert "bad channel True on mux 'mux_a'" in problems(cfg)
    cfg["devices"][1]["path"] = [["mux_a", [3]]]  # unhashable: used to crash segments()
    assert "bad channel [3] on mux 'mux_a'" in problems(cfg)

# Source: routing_devNB.ipynb | Cell In[40] | 2026-11-16 14:35:15
def test_path_inconsistent_with_mux_declaration():
    cfg = config()
    cfg["muxes"]["mux_b"] = {"bus": 0, "address": "0x72", "driver": "TCA9548A",
                             "path": [["mux_a", 1]]}  # fmt: skip
    cfg["devices"][1]["path"] = [["mux_b", 0]]  # skips mux_a
    assert "declared at" in problems(cfg)

# Source: routing_devNB.ipynb | Cell In[41] | 2026-11-16 14:35:15
def test_path_through_a_mux_on_another_bus():
    cfg = config(buses=[0, 1])
    cfg["devices"][1]["bus"] = 1
    assert "hall: on bus 1 but its path enters mux 'mux_a' on bus 0" in problems(cfg)

# Source: routing_devNB.ipynb | Cell In[42] | 2026-11-16 14:35:16
def test_mux_cycle_rejected():
    cfg = config()
    cfg["muxes"]["mux_a"]["path"] = [["mux_a", 0]]
    assert "mux 'mux_a'" in problems(cfg)

# Source: routing_devNB.ipynb | Cell In[46] | 2026-11-16 14:35:16
def test_collision_on_same_segment():
    cfg = config()
    cfg["muxes"]["mux_a"]["address"] = "0x70"
    assert "collision" in problems(cfg)

# Source: routing_devNB.ipynb | Cell In[47] | 2026-11-16 14:35:16
def test_collision_seen_through_upstream_segment():
    cfg = config()
    cfg["devices"][1]["address"] = "0x70"  # behind mux_a, but humidity at 0x70 upstream
    cfg["devices"][1]["driver"] = "SHTC3"
    assert "humidity, hall" in problems(cfg) or "hall, humidity" in problems(cfg)

# Source: routing_devNB.ipynb | Cell In[48] | 2026-11-16 14:35:17
def test_same_address_on_sibling_channels_is_fine():
    cfg = config()
    cfg["devices"].append({"name": "hall2", "bus": 0, "address": "0x35", "driver": "TMAG5273",
                           "path": [["mux_a", 4]]})  # fmt: skip
    RoutingTable.loads(cfg).validate(REG)

# Source: routing_devNB.ipynb | Cell In[49] | 2026-11-16 14:35:17
def test_same_address_on_different_buses_is_fine():
    cfg = config(buses=[0, 1])
    cfg["devices"].append({"name": "humidity2", "bus": 1, "address": "0x70", "driver": "SHTC3"})
    RoutingTable.loads(cfg).validate(REG)

# Source: routing_devNB.ipynb | Cell In[50] | 2026-11-16 14:35:17
def test_each_collision_reported_once():
    cfg = config()
    cfg["muxes"]["mux_a"]["address"] = "0x70"
    found = problems(cfg).splitlines()
    assert found == ["address collision on bus 0 at 0x70: mux:mux_a, humidity"]

# Source: routing_devNB.ipynb | Cell In[53] | 2026-11-16 14:35:18
def test_driver_aware_checks():
    cfg = config()
    cfg["devices"][0]["address"] = "0x44"
    cfg["devices"][1]["path"] = [["mux_a", 9]]
    cfg["devices"].append({"name": "x", "bus": 0, "address": 1, "driver": "NOPE"})
    text = problems(cfg)
    assert "not valid for SHTC3" in text
    assert "channel 9 out of range 0..7" in text
    assert "'NOPE' not in registry" in text

# Source: routing_devNB.ipynb | Cell In[54] | 2026-11-16 14:35:18
def test_driver_checks_skipped_without_registry():
    cfg = config()
    cfg["devices"][0]["address"] = "0x44"
    RoutingTable.loads(cfg).validate()

# Source: routing_devNB.ipynb | Cell In[55] | 2026-11-16 14:35:18
def test_mux_without_driver_reported_with_registry():
    cfg = config()
    del cfg["muxes"]["mux_a"]["driver"]
    assert "mux:mux_a: no driver given" in problems(cfg)
    RoutingTable.loads(cfg).validate()  # structural checks do not need a driver

# Source: routing_devNB.ipynb | Cell In[56] | 2026-11-16 14:35:19
def test_a_device_driver_used_as_a_mux_is_reported():
    cfg = config()
    cfg["muxes"]["mux_a"]["driver"] = "TMAG5273"  # no CHANNELS: not a mux
    assert "mux:mux_a: driver 'TMAG5273' is not a mux driver (no CHANNELS)" in problems(cfg)

# Source: routing_devNB.ipynb | Cell In[59] | 2026-11-16 14:35:19
def test_bench_config_validates():
    RoutingTable.loads(BENCH).validate(REG)

# Source: routing_devNB.ipynb | Cell In[60] | 2026-11-16 14:35:19
def test_bench_mux_at_default_address_collides_with_shtc3():
    cfg = {**BENCH, "muxes": {"mux": {"bus": 0, "address": "0x70", "driver": "TCA9548A"}}}
    assert "collision on bus 0 at 0x70" in problems(cfg)

# Source: routing_devNB.ipynb | Cell In[61] | 2026-11-16 14:35:20
def test_bench_validates_with_the_real_drivers():
    real = registry(["SHTC3", "TMAG5273", "TCA9548A", "TMP102", "VL53L4CD", "QwiicRelay"])
    RoutingTable.loads(BENCH).validate(real)
    RoutingTable.loads(config()).validate(real)
    cfg = config()
    cfg["devices"][1]["path"] = [["mux_a", 8]]
    assert "channel 8 out of range 0..7" in problems(cfg, real)

