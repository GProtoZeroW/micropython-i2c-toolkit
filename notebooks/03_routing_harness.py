# ---
# jupyter:
#   jupytext:
#     formats: ipynb,py:percent
#     text_representation:
#       extension: .py
#       format_name: percent
#       format_version: '1.3'
#       jupytext_version: 1.19.6
#   kernelspec:
#     display_name: MicroPython (micropylab)
#     language: python
#     name: micropylab
# ---

# %% [markdown]
# # Routing harness: config -> validate -> scan -> build -> talk
#
# The whole bench through the routing layer, first rehearsed on the host in simulation, then on
# the real board. Both read the same config, `notebooks/bench.json`: the bench as wired (README
# "The bench", 2026-11-14).
#
# ```
# I2C 1 (scl=15 sda=16)
#   SparkFun Qwiic Mux (TCA9548A) @ 0x70   (address jumpers open)
#     ch 0: SparkFun TMAG5273 (B1) @ 0x22   hall
#     ch 2: TMP102                 @ 0x48   temp
#     ch 7: Adafruit VL53L1X       @ 0x29   tof
# ```
#
# **Where each cell runs.** Sections 1 and 2 run in ordinary Python (CPython) on this computer,
# with no board. `%board connect` starts section 3: from there on cells run in MicroPython on
# the board, except cells that start with `%%host`.
#
# > **Do not run sections 3 onward unattended.** They talk to the real bench; section 7 asks you
# > to hold a hand over the distance sensor. Run them one cell at a time.
#
# The flow:
#
# 1. `RoutingTable.loads()` parses the JSON config.
# 2. `validate(REGISTRY)` checks structure (unknown muxes, undeclared buses, path consistency,
#    address collisions including ones visible through upstream segments) and the parts
#    themselves (driver exists, address legal, channel within the mux).
# 3. `I2CManager.buses_build()` opens each bus at the lowest `BAUD_MAX` of everything on it,
#    with one lock per bus.
# 4. `scan()` selects each segment's mux path, scans, and releases it. An ACK means *something*
#    is at the address; the identity reads in section 6 confirm the chip.
# 5. `build()` creates the mux, the device connections, then the driver instances.
#
# Rewired the bench? Change `bench.json`, run sections 1 and 2 (the simulation catches a bad
# config or a collision without the board), then the rest.

# %% [markdown]
# ## 1. Rehearse on the host
# The same config on `SimBench`: fake buses with each part's fake at its address and path,
# built through the real `I2CManager` and drivers, in simulated time. If this fails, the board
# run would fail the same way.

# %%
import json
from pathlib import Path

from micropython_i2c.sim.bench import SimBench
from micropython_i2c.sim.clock import SimClock
from micropython_i2c.sim.signals import ramp

CONFIG_PATH = next(p for p in (Path("bench.json"), Path("notebooks/bench.json")) if p.is_file())
CONFIG = json.loads(CONFIG_PATH.read_text())

with SimClock() as clock:
    sim = SimBench(CONFIG, parts={"temp": {"temp_c": 22.5}, "tof": {"distance_mm": ramp(1200, 150, 0, 5)}})
    ok, _ = sim.scan()
    sim.build()
    print("hall identify:", sim["hall"].identify())
    print("temp: {:.2f} C".format(sim["temp"].temperature()))
    for _ in range(3):
        print("tof:", sim["tof"].measure())
        clock.advance(2)

# %% [markdown]
# ## 2. A bad config, caught on the host
# Every problem at once: the mux moved to `0x48` (where the TMP102 answers once channel 2 is
# on) and the distance sensor on channel 9 (the TCA9548A has 0..7).

# %%
from micropython_i2c.routing import ConfigError

bad = json.loads(CONFIG_PATH.read_text())
bad["muxes"]["mux"]["address"] = "0x48"
bad["devices"][2]["path"] = [["mux", 9]]
try:
    SimBench(bad)
except ConfigError as e:
    print(e)

# %% [markdown]
# ## 3. Connect and upload the package and the config
# Everything below runs on the board.

# %%
# %board connect

# %%
# %%host
pkg = next(p for p in (Path("src"), Path("../src")) if p.is_dir()) / "micropython_i2c"
# %files copy -r {pkg} :/lib/micropython_i2c
# %files copy {CONFIG_PATH} :/bench.json

# %%
import json
import sys

for name in [m for m in sys.modules if m.startswith("micropython_i2c")]:
    del sys.modules[name]

from micropython_i2c.compat import sleep_ms
from micropython_i2c.devices import registry
from micropython_i2c.manager import I2CManager
from micropython_i2c.routing import ConfigError, RoutingTable

with open("/bench.json") as f:
    CONFIG = json.load(f)
REGISTRY = registry(["TCA9548A", "TMAG5273", "TMP102", "VL53L1X"])

# %% [markdown]
# ## 4. Validate on the board
# The same checks as section 2, on MicroPython.

# %%
table = RoutingTable.loads(CONFIG).validate(REGISTRY)
print("validation passed")

bad = RoutingTable.loads(CONFIG)
bad.muxes["mux"]["address"] = 0x48
bad.devices[2]["path"] = [("mux", 9)]
try:
    bad.validate(REGISTRY)
except ConfigError as e:
    print(e)

# %% [markdown]
# ## 5. Open the bus and probe
#
# Expect `OK` for the mux on the root of bus 1 and each part via its channel.

# %%
mgr = I2CManager(table, REGISTRY, "feathers3")
mgr.buses_build()
for bus_id in table.buses:
    print("bus {} clock: {} Hz".format(bus_id, mgr.bus_freq(bus_id)))
ok, results = mgr.scan()

# %% [markdown]
# ## 6. Build and talk to the parts
#
# Building runs each driver's init: the TMP102 reads its mode, and the VL53L1X boots and loads
# its configuration. The identity reads confirm what the scan could not: that the right chip
# is at each address.

# %%
mgr.build()
print("devices:", list(mgr.devices), " muxes:", list(mgr.muxes))

hall, temp, tof = mgr["hall"], mgr["temp"], mgr["tof"]

print("hall  TMAG5273 identify: {}".format(hall.identify()))
hall.configure(channels="XYZ")
print("hall  field (mT): {}".format(tuple(round(v, 3) for v in hall.magnetic())))
t_tmp = temp.temperature()
print("temp  {:.4f} C (R1:R0 = {:02b})".format(t_tmp, temp.field_get(temp.REG_CONFIG, temp.F_RES)))
print("mux   control {:#04x}".format(mgr["mux"].control_read()))

# %% [markdown]
# ## 7. Ranging
#
# Ten readings; hold a hand 5-50 cm above the sensor. Only `valid` readings can be trusted.

# %%
for _ in range(10):
    r = tof.measure()
    print("{:5d} mm  {}{}".format(r.distance_mm, r.status_name, "" if r.valid else "  (not valid)"))
    sleep_ms(200)

# %%
# %board disconnect
