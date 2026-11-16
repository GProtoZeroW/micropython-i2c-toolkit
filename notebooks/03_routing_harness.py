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
# The bench as wired:
#
# ```
# I2C 0 (STEMMA QT, scl=9 sda=8)
#   SparkFun Qwiic Mux (TCA9548A) @ 0x71
#     ch 0: SHTC3    @ 0x70   humidity + temperature
#     ch 2: TMP102   @ 0x48   temperature
# I2C 1 (pins to confirm in section 2)
#   VL53L4CD           @ 0x29   time of flight
#   Qwiic Single Relay @ 0x18   daisy-chained after the VL53L4CD
# ```
#
# The flow:
#
# 1. `RoutingTable.loads()` parses the JSON config.
# 2. `validate(REGISTRY)` checks structure (unknown muxes, undeclared buses, path
#    consistency, address collisions including ones visible through upstream segments)
#    and the parts themselves (driver exists, address legal, channel within the mux).
# 3. `I2CManager.buses_build()` opens each bus at the lowest `BAUD_MAX` of everything on
#    it, with one lock per bus.
# 4. `scan()` selects each segment's mux path, scans, and releases it. An ACK means
#    *something* is at the address; the identity reads in section 6 confirm the chip.
# 5. `build()` creates the mux, the device connections, then the driver instances.

# %% [markdown]
# ## 1. Connect and upload the package

# %%
# %board connect

# %%
# %%host
from pathlib import Path

pkg = next(p for p in (Path("src"), Path("../src")) if p.is_dir()) / "micropython_i2c"
# %files copy -r {pkg} :/lib/micropython_i2c

# %%
import sys

for name in [m for m in sys.modules if m.startswith("micropython_i2c")]:
    del sys.modules[name]

from micropython_i2c.boards import board_get
from micropython_i2c.compat import sleep_ms
from micropython_i2c.devices import registry
from micropython_i2c.manager import I2CManager
from micropython_i2c.routing import ConfigError, RoutingTable

REGISTRY = registry(["TCA9548A", "SHTC3", "TMP102", "VL53L4CD", "QwiicRelay"])
BOARD = board_get("feathers3")

# %% [markdown]
# ## 2. Find the second bus's pins
#
# `boards.py` has scl=15, sda=16 for bus 1 as a guess. This cell scans each candidate
# pin pair and reports which one sees the VL53L4CD (0x29) and the relay (0x18). Add the
# GPIOs your Qwiic cable is actually wired to as another `(scl, sda)` pair. Avoid
# GPIO 19/20 (USB) and 26-37 (flash and PSRAM on the FeatherS3).
#
# A pair that works is used for the rest of this session; copy it into
# `src/micropython_i2c/boards.py` to make it permanent.

# %%
from machine import I2C, Pin

BUS1_CANDIDATES = [(15, 16)]  # (scl, sda): add yours here
WANT = {0x29, 0x18}

found_pins = None
for scl, sda in BUS1_CANDIDATES:
    try:
        seen = set(I2C(1, scl=Pin(scl), sda=Pin(sda), freq=100_000).scan())
    except Exception as e:  # a bad pin can raise rather than scan empty
        print("scl={} sda={}: {}".format(scl, sda, e))
        continue
    print("scl={} sda={}: {}".format(scl, sda, [hex(a) for a in sorted(seen)]))
    if WANT <= seen and found_pins is None:
        found_pins = (scl, sda)

if found_pins:
    BOARD["buses"][1]["scl"], BOARD["buses"][1]["sda"] = found_pins
    print("bus 1 -> scl={}, sda={}".format(*found_pins))
else:
    print("no candidate sees both 0x29 and 0x18: add the wired pins to BUS1_CANDIDATES")

# %% [markdown]
# ## 3. The topology

# %%
CONFIG = """
{
  "buses": [0, 1],
  "muxes": {
    "mux": {"bus": 0, "address": "0x71", "path": [], "driver": "TCA9548A"}
  },
  "devices": [
    {"name": "humidity", "bus": 0, "address": "0x70", "path": [["mux", 0]], "driver": "SHTC3"},
    {"name": "temp",     "bus": 0, "address": "0x48", "path": [["mux", 2]], "driver": "TMP102"},
    {"name": "tof",      "bus": 1, "address": "0x29", "path": [],           "driver": "VL53L4CD"},
    {"name": "relay",    "bus": 1, "address": "0x18", "path": [],           "driver": "QwiicRelay"}
  ]
}
"""

# %% [markdown]
# ## 4. Validate
#
# The second half shows what a bad config reports: every problem at once. With the mux
# left at its default 0x70, the SHTC3 on channel 0 would answer at the same address as
# the mux whenever channel 0 is on, which is why the jumper is set.

# %%
table = RoutingTable.loads(CONFIG).validate(REGISTRY)
print("validation passed")

bad = RoutingTable.loads(CONFIG)
bad.muxes["mux"]["address"] = 0x70  # the mux's default address
bad.devices[1]["path"] = [("mux", 9)]  # TCA9548A has channels 0..7
try:
    bad.validate(REGISTRY)
except ConfigError as e:
    print(e)

# %% [markdown]
# ## 5. Open the buses and probe
#
# Expect `OK` for the mux on the root of bus 0, the SHTC3 via channel 0, the TMP102 via
# channel 2, and both parts on bus 1.

# %%
mgr = I2CManager(table, REGISTRY, BOARD)
mgr.buses_build()
for bus_id in table.buses:
    print("bus {} clock: {} Hz".format(bus_id, mgr.bus_freq(bus_id)))
ok, results = mgr.scan()

# %% [markdown]
# ## 6. Build and talk to the parts
#
# Building runs each driver's init: the TMP102 reads its mode, and the VL53L4CD boots,
# loads its init table and calibrates (about a second). The identity reads confirm what
# the scan could not: that the right chip is at each address.

# %%
mgr.build()
print("devices:", list(mgr.devices), " muxes:", list(mgr.muxes))

humidity, temp, tof, relay = mgr["humidity"], mgr["temp"], mgr["tof"], mgr["relay"]

print("humidity  SHTC3 id ok: {}".format(humidity.identify()))
t_sht, rh = humidity.measure()
print("humidity  {:.2f} C, {:.1f} %RH".format(t_sht, rh))
t_tmp = temp.temperature()
print(
    "temp      {:.4f} C (R1:R0 = {:02b})".format(t_tmp, temp.field_get(temp.REG_CONFIG, temp.F_RES))
)
print("          the two sensors differ by {:.2f} C".format(abs(t_sht - t_tmp)))
print("tof       model {:#06x}, budget {} ms".format(tof.model_id(), tof.timing_budget))
print("relay     firmware {:#04x}, closed: {}".format(relay.firmware_version(), relay.state))
print("mux       control {:#04x}".format(mgr["mux"].control_read()))

# %% [markdown]
# ## 7. Ranging
#
# Ten readings; hold a hand 5-50 cm above the sensor. Only `valid` readings can be
# trusted (open air past about 1.3 m reads "sigma warning" or "invalid phase").

# %%
tof.start()
try:
    for _ in range(10):
        mm = tof.read()
        print("{:5d} mm  {}".format(mm, tof.status_name))
        sleep_ms(200)
finally:
    tof.stop()

# %% [markdown]
# ## 8. The relay
#
# This switches whatever is wired to the relay's contacts, so it only runs when you set
# `RELAY_TEST = True`. It closes the relay for one second, then opens it, and reads the
# state back each time.

# %%
RELAY_TEST = False

if RELAY_TEST:
    relay.on()
    print("closed:", relay.state)
    sleep_ms(1000)
    relay.off()
    print("closed:", relay.state)
else:
    print("skipped; set RELAY_TEST = True to click the relay")

# %%
# %board disconnect
