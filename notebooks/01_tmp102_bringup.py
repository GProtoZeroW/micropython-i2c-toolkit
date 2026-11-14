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
# # TMP102 bring-up: the register mixin, then the driver
#
# Hardware checks for `RegisterI2CBase` and the `TMP102` driver on a real part. The
# arithmetic (bit helpers, CRC, codecs) is covered offline by `uv run pytest`; this
# notebook covers only what needs silicon.
#
# Order matters for diagnosis:
#
# 1. Section 3 drives the mixin directly through `TMP102Regs`, a class with datasheet
#    facts and **no methods**. A failure there is the mixin, the connection or the wiring.
# 2. Section 4 then tests the `TMP102` driver. With section 3 passing, a failure here is
#    driver code.
#
# Hardware: FeatherS3, SparkFun Qwiic Mux (TCA9548A, jumpered to 0x71) on the STEMMA QT
# port (I2C 0, scl=9, sda=8), TMP102 (0x48) on mux **channel 2**. So every transfer here
# also exercises mux routing. Every cell restores what it changed, so cells can be re-run
# in any order after the first pass. Expected values assume a part near room temperature.

# %% [markdown]
# ## 1. Connect and upload the package
#
# `%board connect` finds a single USB board by itself; give the port if several are
# plugged in (`%board connect /dev/ttyACM0`). The host cell copies `src/micropython_i2c`
# to `/lib` on the board. Re-run it after editing the package, then re-run the import
# cell, which drops the old modules so the new files load.

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

from micropython_i2c import I2CConnection, default_lock
from micropython_i2c.boards import bus_open
from micropython_i2c.debug import (
    check,
    check_raises,
    check_reset,
    check_summary,
    info,
    show,
    show_reg,
)
from micropython_i2c.drivers.tca9548a import TCA9548A
from micropython_i2c.drivers.tmp102 import TMP102, TMP102Regs

check_reset()

# %% [markdown]
# ## 2. The bus and the mux
#
# With every mux channel off, only the mux itself (0x71) should answer. With channel 2
# on, the TMP102 joins it. If the TMP102 is missing, check the Qwiic cable goes to the
# mux's port 2, not 0 (port 0 is the SHTC3). `addr_ok` and `baud_ok` need no bus
# traffic; that is the point of keeping datasheet facts on the class.

# %%
MUX_ADDR = 0x71  # SparkFun Qwiic Mux, address jumper set (0x70 would clash with the SHTC3)
TMP_ADDR = 0x48  # ADD0 -> GND
TMP_CHANNEL = 2
TMP_PATH = [[MUX_ADDR, TMP_CHANNEL]]

bus = bus_open("feathers3", 0, freq=100_000)
bus_lock = default_lock()  # one lock per bus; the manager does this in notebook 03
mux = TCA9548A(bus, MUX_ADDR, lock=bus_lock)

mux.channel_deselect()
root = bus.scan()
print("all channels off:", [hex(a) for a in root])
check("mux ACKs", MUX_ADDR in root, hex(MUX_ADDR))
check("TMP102 hidden behind the mux", TMP_ADDR not in root)

mux.channel_select(TMP_CHANNEL)
on_ch = bus.scan()
print("channel {} on:     ".format(TMP_CHANNEL), [hex(a) for a in on_ch])
check("TMP102 ACKs on channel {}".format(TMP_CHANNEL), TMP_ADDR in on_ch, hex(TMP_ADDR))
check("control register reads back", mux.control_read() == 1 << TMP_CHANNEL)
mux.channel_deselect()

check("address legal for the part", TMP102Regs.addr_ok(TMP_ADDR))
check("100 kHz within the part's limits", TMP102Regs.baud_ok(100_000))

# The connection re-selects channel 2 on every transfer, so the mux state left above
# does not matter.
regs = TMP102Regs(I2CConnection(bus, TMP_ADDR, TMP_PATH, bus_lock))
show_reg(regs, regs.REG_CONFIG, "config")
show_reg(regs, regs.REG_TEMP, "temp")

# %% [markdown]
# ## 3. `RegisterI2CBase` through `TMP102Regs`
#
# ### 3.1 The layer stack
#
# The same register through each layer, innermost first. The first disagreeing line
# names the faulty layer. The bare `machine.I2C` read has to switch the mux by hand,
# which is exactly the step the connection does for you.

# %%
mux.channel_select(TMP_CHANNEL)
raw_bus = bus.readfrom_mem(TMP_ADDR, regs.REG_TEMP, 2)
mux.channel_deselect()
raw_conn = regs.conn.readfrom_mem(regs.REG_TEMP, 2)
raw_dev = regs.read_mem(regs.REG_TEMP, 2)
print("machine.I2C  ", raw_bus)
print("I2CConnection", raw_conn)
print("I2CNode      ", raw_dev)
check("all three return 2 bytes", len(raw_bus) == len(raw_conn) == len(raw_dev) == 2)
check("mixin assembles MSB first", regs.reg_read(regs.REG_TEMP) >> 8 == raw_dev[0])
check("temp low nibble is padding", regs.reg_read(regs.REG_TEMP) & 0x000F == 0)
check("R1:R0 read 11", regs.field_get(regs.REG_CONFIG, regs.F_RES) == 0b11,
      "read-only; the best identification this part allows")  # fmt: skip
cfg = regs.reg_read(regs.REG_CONFIG)
if cfg != regs.CFG_POR:
    info("config {:#06x}, POR is {:#06x}: configured by an earlier run".format(cfg, regs.CFG_POR))

# %% [markdown]
# ### 3.2 The shadow cache
#
# Fill on a miss, serve on a hit, go stale against a volatile register (that is the
# behaviour to recognise, not a bug). The cache records the width, so a 1-byte and a
# 2-byte read of the same register never mix.

# %%
regs.cache_clear()
first = regs.reg_get(regs.REG_TEMP)
check("hit returns the cached value", regs.reg_get(regs.REG_TEMP) == first)
show(regs.reg_get(regs.REG_TEMP, refresh=True), "refresh")
check("width-aware", regs.reg_get(regs.REG_CONFIG, nbytes=1) == regs.reg_get(regs.REG_CONFIG) >> 8)

regs.cache_clear(regs.REG_TEMP)
check("selective clear", regs.REG_TEMP not in regs.reg_cache and regs.REG_CONFIG in regs.reg_cache)
regs.cache_clear()
check("full clear", not regs.reg_cache)

# %% [markdown]
# ### 3.3 Verified writes and why the mask matters
#
# Writing the config back unchanged runs the whole verify path. The same write with the
# read-only R1:R0 bits cleared *must* fail an exact check (they always read 11) and pass
# under the register's verify mask.

# %%
original = regs.reg_read(regs.REG_CONFIG)
check("unchanged write verifies", regs.reg_set(regs.REG_CONFIG, original) == original)
check("confirmed value cached", regs.cached(regs.REG_CONFIG) == original)

sabotaged = original & ~regs.field_mask(regs.F_RES)
check_raises("exact verify fails", ValueError, regs.reg_set, regs.REG_CONFIG, sabotaged,
             verify_mask=0xFFFF)  # fmt: skip
check("masked verify passes", regs.reg_set(regs.REG_CONFIG, sabotaged) == original)

before = regs.reg_read(regs.REG_CONFIG)
regs.field_set(regs.REG_CONFIG, regs.F_SD, 1)
changed = before ^ regs.reg_read(regs.REG_CONFIG)
check("field_set moved only SD", changed == regs.field_mask(regs.F_SD), "{:#06x}".format(changed))
regs.field_set(regs.REG_CONFIG, regs.F_SD, 0)
check("restored", regs.reg_read(regs.REG_CONFIG) == before)

# %% [markdown]
# ### 3.4 Coded fields, validator, tolerance, raw writes

# %%
original = regs.reg_read(regs.REG_CONFIG)
for hz in (0.25, 1.0, 8.0, 4.0):
    regs.coded_set(regs.REG_CONFIG, regs.F_CR, regs.CONV_RATES, hz)
    check(
        "rate {} Hz".format(hz), regs.coded_get(regs.REG_CONFIG, regs.F_CR, regs.CONV_RATES) == hz
    )
check_raises("2.0 Hz rejected", ValueError, regs.coded_set, regs.REG_CONFIG, regs.F_CR,
             regs.CONV_RATES, 2.0)  # fmt: skip
regs.reg_set(regs.REG_CONFIG, original)


def under_100c(v):
    if v > 0x6400:
        raise ValueError("{:#06x} exceeds 100 C".format(v))


thigh = regs.reg_read(regs.REG_THIGH)
check_raises("validator fires", ValueError, regs.reg_set, regs.REG_THIGH, 0x7000,
             validator=under_100c)  # fmt: skip
check("validator ran before any write", regs.reg_read(regs.REG_THIGH) == thigh)
check("threshold verifies on every bit", regs.reg_set(regs.REG_THIGH, 0x1E00) == 0x1E00)

# Tolerance: R1:R0 make the config read back differently from a sabotaged request, so
# the exact delta is measured, then a tolerance just under must fail and just over pass.
sabotaged = regs.CFG_POR & ~regs.field_mask(regs.F_RES)
regs.reg_write(regs.REG_CONFIG, sabotaged)
delta = abs(regs.reg_read(regs.REG_CONFIG) - sabotaged)
check_raises("tolerance just under fails", ValueError, regs.reg_set, regs.REG_CONFIG,
             sabotaged, tolerance=delta - 1)  # fmt: skip
check("tolerance just over passes", regs.reg_set(regs.REG_CONFIG, sabotaged, tolerance=delta) >= 0)

# Raw writes bypass the cache, which then goes stale: the trap reg_set avoids.
regs.reg_get(regs.REG_THIGH, refresh=True)
regs.reg_write(regs.REG_THIGH, 0x2800)
check("raw write leaves the cache stale", regs.reg_get(regs.REG_THIGH) != 0x2800)
check("refresh fixes it", regs.reg_get(regs.REG_THIGH, refresh=True) == 0x2800)

regs.reg_set(regs.REG_CONFIG, regs.CFG_POR)
regs.reg_set(regs.REG_THIGH, regs.THIGH_POR)
regs.reg_set(regs.REG_TLOW, regs.TLOW_POR)
regs.cache_clear()
show_reg(regs, regs.REG_CONFIG, "config restored")

# %% [markdown]
# ## 4. The `TMP102` driver
#
# ### 4.1 Bind and read

# %%
tmp = TMP102(I2CConnection(bus, TMP_ADDR, TMP_PATH, bus_lock))
t = tmp.temperature()
print("extended mode:", tmp.extended_mode)
print("temperature:   {:.4f} C".format(t))
check("plausible indoors", 5.0 <= t <= 45.0, "{:.4f} C".format(t))
pad = 0b111 if tmp.extended_mode else 0b1111
check("temp word zero-padded", tmp.temperature_raw() & pad == 0)

# %% [markdown]
# ### 4.2 Thresholds and configuration properties
#
# Values are exact multiples of the LSB, so the round trip can be compared exactly.

# %%
for target in (30.0, 0.0, -25.0, 100.0):
    check("T_HIGH = {} C".format(target), tmp.high_limit_set(target) == target)
landed = tmp.low_limit_set(20.03)
check("rounds to the nearest LSB", abs(landed - 20.03) <= tmp.LSB_C / 2, "{:.4f}".format(landed))
check_raises("crossed limits rejected", ValueError, tmp.limits_set, 80.0, 20.0)
tmp.limits_reset()
check("limits back at POR", (tmp.low_limit_get(), tmp.high_limit_get()) == (75.0, 80.0))

saved = tmp.config_get()
for hz in (0.25, 1.0, 8.0, 4.0):
    tmp.conversion_rate = hz
    check("rate {} Hz".format(hz), tmp.conversion_rate == hz)
for n in (1, 2, 4, 6):
    tmp.fault_queue = n
    check("fault queue {}".format(n), tmp.fault_queue == n)
for attr in ("thermostat_mode", "alert_polarity_high", "shutdown"):
    for state in (True, False):
        setattr(tmp, attr, state)
        check("{} = {}".format(attr, state), getattr(tmp, attr) is state)
tmp.config_set(saved)

# %% [markdown]
# ### 4.3 Extended mode
#
# The one setting that changes how every read is *interpreted*. Reading the same
# temperature through both decodes must agree; a stale flag would show up as a factor
# of two, not half a degree.

# %%
saved = tmp.config_get()
t12 = tmp.temperature()
tmp.extended_mode = True
check("driver tracked EM", tmp.em is True)
raw13 = tmp.temperature_raw()
t13 = tmp.temperature()
check("both decodes agree", abs(t13 - t12) < 0.5, "{:.4f} vs {:.4f} C".format(t12, t13))
check("EM indicator bit D0 set", raw13 & 1 == 1, "{:#06x}".format(raw13))
tmp.extended_mode = False
tmp.em = True  # deliberately stale
tmp.mode_refresh()
check("mode_refresh corrects a stale flag", tmp.em is False)
tmp.config_set(saved)

# %% [markdown]
# ### 4.4 One-shot conversion
#
# The OS handshake, the one place the driver deliberately uses a raw write. Outside
# shutdown mode it must refuse rather than return a stale continuous reading.

# %%
from micropython_i2c.compat import ticks_diff, ticks_ms

saved = tmp.config_get()
tmp.shutdown = False
check_raises("refused in continuous mode", RuntimeError, tmp.one_shot)
tmp.shutdown = True
check("OS reads 1 when idle", tmp.conversion_ready is True)
for i in range(3):
    start = ticks_ms()
    t = tmp.one_shot()
    check("one-shot #{}".format(i + 1), tmp.TEMP_MIN <= t <= tmp.TEMP_MAX,
          "{:.4f} C in {} ms".format(t, ticks_diff(ticks_ms(), start)))  # fmt: skip
tmp.config_set(saved)
tmp.shutdown = False

# %% [markdown]
# ### 4.5 It tracks reality, and ALERT follows the thresholds
#
# Pinch the package during the 5 s read and the number should climb. Then T_HIGH is
# dropped below ambient (AL must assert) and T_LOW raised above it (AL must release).
# With POL=0, AL reads **1 when not alerting**.

# %%
from micropython_i2c.compat import sleep_ms

tmp.conversion_rate = 8.0
for _ in range(20):
    print("  {:7.4f} C   raw {:#06x}".format(tmp.temperature(), tmp.temperature_raw()))
    sleep_ms(250)

tmp.limits_reset()
tmp.fault_queue = 1
t_now = tmp.temperature()
check("no alert at POR limits", tmp.alert is True)
tmp.limits_set(t_now - 20.0, t_now - 10.0)
sleep_ms(500)
check("alert asserts", tmp.alert is False, "T_HIGH {:.2f} C".format(tmp.high_limit_get()))
tmp.limits_set(t_now + 10.0, t_now + 20.0)
sleep_ms(500)
check("alert releases", tmp.alert is True, "T_LOW {:.2f} C".format(tmp.low_limit_get()))

tmp.limits_reset()
tmp.config_reset()
tmp.cache_clear()
show(tmp.config_get(), "config at POR")

# %% [markdown]
# ## 5. Summary

# %%
check_summary()

# %%
# %board disconnect
