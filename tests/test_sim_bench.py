import pytest

from micropython_i2c.compat import sleep_ms
from micropython_i2c.devices.drivers.qwiic_relay_fake import FakeQwiicRelay
from micropython_i2c.routing import ConfigError
from micropython_i2c.sim.bench import SimBench
from micropython_i2c.sim.bus import FakeRegisterPart
from micropython_i2c.sim.clock import SimClock
from micropython_i2c.sim.signals import lag, noisy, ramp, sequence, sine
# Source: sim_bench_devNB.ipynb | Cell In[9] | 2026-11-16 13:46:57
BENCH = {
    "buses": [0, 1],
    "muxes": {"mux": {"bus": 1, "address": "0x70", "driver": "TCA9548A"}},
    "devices": [
        {"name": "air", "bus": 0, "address": "0x70", "driver": "SHTC3"},
        {"name": "heater", "bus": 0, "address": "0x18", "driver": "QwiicRelay"},
        {"name": "door", "bus": 1, "address": "0x22", "driver": "TMAG5273", "path": [["mux", 0]]},
        {"name": "soil", "bus": 1, "address": "0x48", "driver": "TMP102", "path": [["mux", 2]]},
        {"name": "tof", "bus": 1, "address": "0x29", "driver": "VL53L1X", "path": [["mux", 7]]},
    ],
}

# Source: sim_bench_devNB.ipynb | Cell In[13] | 2026-11-16 13:46:57
def test_bench_builds_and_every_part_answers():
    with SimClock():
        bench = SimBench(BENCH)
        ok, results = bench.scan(verbose=False)
        assert ok and len(results) == 6
        bench.build()
        assert set(bench.manager.devices) == {"air", "heater", "door", "soil", "tof"}

# Source: sim_bench_devNB.ipynb | Cell In[14] | 2026-11-16 13:46:57
def test_parts_set_the_fakes_and_drivers_read_them():
    with SimClock():
        bench = SimBench(BENCH, parts={"soil": {"temp_c": 21.5}, "air": {"rh": 55.0}}).build()
        assert bench["soil"].temperature() == pytest.approx(21.5, abs=0.07)
        assert bench["air"].measure()[1] == pytest.approx(55.0, abs=0.01)
        bench["heater"].on()
        assert bench.fake("heater").closed

# Source: sim_bench_devNB.ipynb | Cell In[15] | 2026-11-16 13:46:58
def test_an_invalid_config_is_refused_with_every_problem():
    extra = {"name": "air2", "bus": 1, "address": "0x70", "driver": "SHTC3"}
    bad = dict(BENCH, devices=BENCH["devices"] + [extra])
    with pytest.raises(ConfigError, match="collision on bus 1 at 0x70"):
        SimBench(bad)

# Source: sim_bench_devNB.ipynb | Cell In[16] | 2026-11-16 13:46:58
def test_a_ready_made_fake_is_attached_as_given():
    part = FakeRegisterPart()
    one = {"name": "x", "bus": 0, "address": "0x48", "driver": "TMP102"}
    bench = SimBench({"buses": [0], "devices": [one]}, parts={"x": part})
    assert bench.fake("x") is part
    assert bench.buses[0].scan() == [0x48]

# Source: sim_bench_devNB.ipynb | Cell In[17] | 2026-11-16 13:46:58
def test_a_ramp_shows_up_through_the_whole_stack():
    with SimClock() as clock:
        bench = SimBench(BENCH, parts={"soil": {"temp_c": ramp(20.0, 26.0, 0, 3600)}}).build()
        clock.advance(1800)
        assert bench["soil"].temperature() == pytest.approx(23.0, abs=0.1)

# Source: sim_bench_devNB.ipynb | Cell In[18] | 2026-11-16 13:46:58
def greenhouse(bench, minutes):
    """The application: one pass a minute. Returns one log row per minute."""
    heater, soil, air, door, tof = (bench[n] for n in ("heater", "soil", "air", "door", "tof"))
    door.configure(channels="XYZ")
    log = []
    for minute in range(minutes):
        t_soil = soil.temperature()
        if t_soil < 20.0:
            heater.on()
        elif t_soil > 22.0:
            heater.off()
        t_air, rh = air.measure()
        door_open = abs(door.magnetic()[2]) < 5.0  # the door magnet holds Z near 25 mT
        visitor = tof.measure().distance_mm < 800
        log.append((minute, t_soil, heater.state, t_air, rh, door_open, visitor))
        sleep_ms(60_000)
    return log


def greenhouse_world():
    """The scenario: the parts dict for SimBench, and the heater's fake."""
    heater = FakeQwiicRelay()
    parts = {
        "heater": heater,
        "soil": {"temp_c": lag(lambda t: 30.0 if heater.closed else 12.0, 900, initial=16.0)},
        "air": {"temp_c": sine(18.0, 3.0, 4 * 3600), "rh": noisy(55.0, 1.0, seed=1)},
        "door": {"field_mT": (0.0, 0.0, lambda t: 0.5 if 3600 <= t < 4800 else 25.0)},
        "tof": {"distance_mm": sequence([(0, 3000), (5400, 3000), (5700, 400), (6000, 400),
                                         (6300, 3000)])},
    }  # fmt: skip
    return parts, heater

# Source: sim_bench_devNB.ipynb | Cell In[20] | 2026-11-16 13:46:58
def test_greenhouse_holds_its_band_and_sees_the_door_and_the_visitor():
    with SimClock():
        parts, heater = greenhouse_world()
        log = greenhouse(SimBench(BENCH, parts=parts).build(), 180)
    soil = [row[1] for row in log if row[0] >= 30]
    assert 19.0 < min(soil) and max(soil) < 23.0
    assert [r[0] for r in log if r[5]] == list(range(60, 80))
    visitor = [r[0] for r in log if r[6]]
    assert visitor and 92 <= visitor[0] and visitor[-1] <= 104
    assert any(r[2] for r in log) and not all(r[2] for r in log)  # the heater cycles

