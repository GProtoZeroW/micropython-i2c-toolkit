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
# # TMAG5273 bring-up: both mixins, and the first CRC on silicon
#
# The TMAG5273 needs `RegisterI2CBase` for configuration *and* `CRCMixin` on the
# fast-read data path, and unlike the TMP102 it has a real ID. Codecs (Equations 10, 12,
# 16, the angle format, packet lengths) are checked offline by `uv run pytest`; this
# notebook covers what needs the part.
#
# Hardware: FeatherS3, SparkFun Qwiic TMAG5273 (A1) on I2C 0. The factory address is
# 0x35; this board answers at **0x22**, so change `TMAG_ADDR` if yours differs. Every
# live section restores the configuration it changed.

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
import math
import sys

for name in [m for m in sys.modules if m.startswith("micropython_i2c")]:
    del sys.modules[name]

from micropython_i2c import I2CConnection, default_lock
from micropython_i2c.boards import bus_open
from micropython_i2c.compat import sleep_ms
from micropython_i2c.debug import check, check_raises, check_reset, check_summary, info, show
from micropython_i2c.drivers.tmag5273 import TMAG5273, TMAG5273Regs

check_reset()

# %% [markdown]
# ## 2. Bind and identify (`TMAG5273Regs`, no driver logic)
#
# `MANUFACTURER_ID` is fixed ("TI", 0x5449) and `DEVICE_ID.VER` names the variant. The
# ID's LSB sits at the *lower* address, so it is assembled by hand rather than by
# flipping `BIG_ENDIAN` for the whole chip.

# %%
TMAG_ADDR = 0x22  # factory default is TMAG5273Regs.ADDR_DEFAULT (0x35)

bus = bus_open("feathers3", 0, freq=100_000)
bus_lock = default_lock()
print("bus scan:", [hex(a) for a in bus.scan()])
check("part ACKs", TMAG_ADDR in bus.scan(), hex(TMAG_ADDR))

regs = TMAG5273Regs(I2CConnection(bus, TMAG_ADDR, lock=bus_lock))
mid = (regs.reg_read(regs.REG_MANUFACTURER_ID_MSB) << 8) | regs.reg_read(
    regs.REG_MANUFACTURER_ID_LSB
)
check("manufacturer ID", mid == regs.MANUFACTURER_ID, "{:#06x}".format(mid))
ver = regs.field_get(regs.REG_DEVICE_ID, regs.F_VER)
check("variant A1", ver == regs.VER_A1, "VER = {}".format(ver))
addr_reg = regs.reg_read(regs.REG_I2C_ADDRESS)
show(addr_reg, "I2C_ADDRESS", 1)
if addr_reg >> 1 != TMAG_ADDR:
    info("I2C_ADDRESS register says {:#04x}; the OTP address differs".format(addr_reg >> 1))
status = regs.reg_read(regs.REG_DEVICE_STATUS)
for name, field in (("VCC undervoltage", regs.F_VCC_UV_ER), ("OTP CRC error", regs.F_OTP_CRC_ER),
                    ("INT pin error", regs.F_INT_ER), ("oscillator error", regs.F_OSC_ER)):  # fmt: skip
    check(name + " clear", regs.field_extract(status, field) == 0)

# %% [markdown]
# ## 3. `RegisterI2CBase` on a differently shaped part
#
# 8-bit registers (`REG_BYTES = 1`) with 16-bit results across adjacent pairs, read with
# a per-call `nbytes=2` and the part's auto-increment. `POR` in `CONV_STATUS` is
# write-1-to-clear, so it can only be cleared with a raw write.

# %%
for name, reg in (("X", regs.REG_X_MSB_RESULT), ("Y", regs.REG_Y_MSB_RESULT),
                  ("Z", regs.REG_Z_MSB_RESULT), ("T", regs.REG_T_MSB_RESULT)):  # fmt: skip
    show(regs.reg_read(reg, nbytes=2), name + " word")

# Checked on the constant ID pair: result registers move between transfers.
lsb = regs.reg_read(regs.REG_MANUFACTURER_ID_LSB)
msb = regs.reg_read(regs.REG_MANUFACTURER_ID_MSB)
check(
    "auto-increment assembles",
    regs.reg_read(regs.REG_MANUFACTURER_ID_LSB, nbytes=2) == (lsb << 8) | msb,
)

regs.cache_clear()
narrow = regs.reg_get(regs.REG_MANUFACTURER_ID_LSB)
wide = regs.reg_get(regs.REG_MANUFACTURER_ID_LSB, nbytes=2)
check("cache keeps widths apart", wide == (lsb << 8) | msb and narrow == lsb)

original = regs.reg_read(regs.REG_SENSOR_CONFIG_2)
check("8-bit exact verify", regs.reg_set(regs.REG_SENSOR_CONFIG_2, 0x03) == 0x03)
before = regs.reg_read(regs.REG_SENSOR_CONFIG_2)
regs.field_set(regs.REG_SENSOR_CONFIG_2, regs.F_ANGLE_EN, 1)
changed = before ^ regs.reg_read(regs.REG_SENSOR_CONFIG_2)
check("only ANGLE_EN moved", changed & ~regs.field_mask(regs.F_ANGLE_EN) == 0)
regs.reg_set(regs.REG_SENSOR_CONFIG_2, original)

if regs.field_get(regs.REG_CONV_STATUS, regs.F_POR):
    regs.reg_write(regs.REG_CONV_STATUS, regs.field_mask(regs.F_POR))
    check("W1C cleared POR", regs.field_get(regs.REG_CONV_STATUS, regs.F_POR) == 0)
else:
    info("POR already clear, nothing to demonstrate")

# %% [markdown]
# ## 4. The driver: live readings
#
# A bench magnetometer reads the Earth's field (25-65 uT) plus clutter, which is a weak
# check. `SET_COUNT` advancing is the real liveness test. Wave a magnet during the
# 20-sample sweep to see the signal path move.

# %%
tmag = TMAG5273(I2CConnection(bus, TMAG_ADDR, lock=bus_lock))
tmag.configure(channels="XYZ", temperature=True, averaging=4, mode="continuous")
sleep_ms(10)
print("ranges X/Y +-{:g} mT, Z +-{:g} mT, conversion ~{} us".format(
    tmag.xy_range, tmag.z_range, tmag.conversion_time_us()))  # fmt: skip

st = tmag.conv_status()
check("conversion data ready", st["ready"])
check("no diagnostic failure", not st["diag_fail"])
t, x, y, z = tmag.all_read()
print("X {:+9.4f}  Y {:+9.4f}  Z {:+9.4f} mT   T {:+7.3f} C".format(x, y, z, t))
check("temperature plausible", 10.0 <= t <= 60.0, "{:.2f} C".format(t))
first = tmag.conv_status()["set_count"]
sleep_ms(20)
check("SET_COUNT advances", tmag.conv_status()["set_count"] != first)

seen = []
for _ in range(20):
    x, y, z = tmag.magnetic()
    seen.append((x * x + y * y + z * z) ** 0.5)
    print("  X {:+8.3f}  Y {:+8.3f}  Z {:+8.3f}  |B| {:7.3f} mT".format(x, y, z, seen[-1]))
    sleep_ms(50)
spread = max(seen) - min(seen)
if spread > 1.0:
    check("field moved during the sweep", True, "{:.3f} mT".format(spread))
else:
    info(
        "|B| spread {:.4f} mT: a static field. SET_COUNT above is the liveness check".format(spread)
    )

# %% [markdown]
# ## 5. CRC against silicon
#
# The part computes a checksum in hardware and the driver in software; they must agree.
# The negative check pins the scope: verifying *without* the command byte must fail.
# fast16 with CRC allows three channels, so temperature is off here.

# %%
saved = (tmag.read_mode, tmag.crc_enabled, tmag.channels, tmag.temperature_enabled)
tmag.temperature_enabled = False
tmag.channels = "XYZ"
tmag.crc_enabled = True
tmag.read_mode = "fast16"
sleep_ms(10)

length = tmag.fast_read_length()
raw = tmag.read_raw(length)
cmd = (TMAG_ADDR << 1) | 1
print("packet ({} bytes):".format(length), " ".join("{:02x}".format(b) for b in raw))
check("software CRC matches silicon", tmag.crc(bytes([cmd]) + raw[:-1]) == raw[-1])
check("omitting the command byte fails", tmag.crc(raw[:-1]) != raw[-1])
corrupt = bytearray(raw)
corrupt[1] ^= 1
check(
    "single-bit corruption caught", not tmag.crc_ok(bytes([cmd]) + bytes(corrupt[:-1]), corrupt[-1])
)

ok = 0
for _ in range(10):
    try:
        tmag.fast_read()
        ok += 1
    except ValueError as e:
        print("  checksum failure:", e)
    sleep_ms(20)
check("10 consecutive verified reads", ok == 10, "{}/10".format(ok))

tmag.crc_enabled = False
check("CRC off shortens the packet by one", tmag.fast_read_length() == length - 1)
tmag.read_mode, tmag.crc_enabled, tmag.channels, tmag.temperature_enabled = saved

# %% [markdown]
# ## 6. Fast-read modes against the register path
#
# Agreement is loose (separate conversions), but a mis-decoding mode is off by a factor
# or a sign. fast8 temperature is checked against its own truncation bound: dropping the
# LSB costs about 4.26 C and reads low, by design.

# %%
saved = (tmag.read_mode, tmag.crc_enabled, tmag.channels, tmag.temperature_enabled)
tmag.read_mode = "standard"
tmag.channels = "XYZ"
tmag.temperature_enabled = True
sleep_ms(10)
ref_t, ref_x, ref_y, ref_z = tmag.all_read()
ref = {"X": ref_x, "Y": ref_y, "Z": ref_z}

tmag.crc_enabled = False
tmag.read_mode = "fast16"
sleep_ms(10)
r16, _ = tmag.fast_read()
for ax in "XYZ":
    check("fast16 {} agrees".format(ax), abs(r16[ax] - ref[ax]) < 0.5)
check("fast16 T agrees", abs(r16["T"] - ref_t) < 2.0)

tmag.crc_enabled = True
tmag.read_mode = "fast8"
sleep_ms(10)
r8, _ = tmag.fast_read()
for ax in "XYZ":
    step = 2 * tmag.axis_scale(ax) / 256
    check("fast8 {} within a step".format(ax), abs(r8[ax] - ref[ax]) <= step + 0.5)
check("fast8 T within truncation", abs(r8["T"] - ref_t) <= tmag.TEMP_LSB_STEP_C + 1.0)

tmag.read_mode = "fast16"  # four channels with CRC: one too many
sleep_ms(5)
check_raises("fast16+CRC refuses 4 channels", ValueError, tmag.fast_read)

tmag.crc_enabled = False
tmag.temperature_enabled = False
tmag.channels = "XZX"
sleep_ms(10)
words, _, _ = tmag.fast_read_raw()
check("XZX streams X, Z, X", [n for n, _ in words] == ["X", "Z", "X"])
check("fast_read collapses repeats", set(tmag.fast_read()[0]) == {"X", "Z"})

tmag.read_mode, tmag.crc_enabled, tmag.channels, tmag.temperature_enabled = saved
tmag.read_mode = "standard"

# %% [markdown]
# ## 7. Angle and offset
#
# Without a rotating magnet there is no ground truth for the angle, so it is compared
# with software `atan2` of the same axes (gross errors only). The offset register is the
# stronger check: Equation 16 predicts the shift exactly.

# %%
saved_cfg2 = tmag.reg_read(tmag.REG_SENSOR_CONFIG_2)
saved_off = (
    tmag.reg_read(tmag.REG_MAG_OFFSET_CONFIG_1),
    tmag.reg_read(tmag.REG_MAG_OFFSET_CONFIG_2),
)
tmag.channels = "XYZ"
tmag.angle_pair = "XY"
sleep_ms(10)

x, y, _ = tmag.magnetic()
hw = tmag.angle()
sw = math.degrees(math.atan2(y, x)) % 360.0
diff = min(abs(hw - sw), 360.0 - abs(hw - sw))
if (x * x + y * y) ** 0.5 < 0.5:
    info("in-plane field too weak for an angle check; hold a magnet near and re-run")
else:
    check("hardware angle matches atan2", diff < 5.0, "hw {:.2f}, sw {:.2f}".format(hw, sw))
check("angle in sixteenths", abs(hw * 16 - round(hw * 16)) < 1e-9)

tmag.offsets_set(0.0, 0.0)
sleep_ms(10)
base_x, base_y, _ = tmag.magnetic()
tmag.reg_set(tmag.REG_MAG_OFFSET_CONFIG_1, 0b10000000)
sleep_ms(10)
off_x, off_y, _ = tmag.magnetic()
expected = tmag.offset_from_raw(0b10000000, tmag.xy_range)
check("offset shifts by Equation 16", abs((off_x - base_x) - expected) < 0.5,
      "observed {:+.4f}, predicted {:+.4f} mT".format(off_x - base_x, expected))  # fmt: skip
check("second axis unaffected", abs(off_y - base_y) < 0.5)

tmag.reg_set(tmag.REG_MAG_OFFSET_CONFIG_1, saved_off[0])
tmag.reg_set(tmag.REG_MAG_OFFSET_CONFIG_2, saved_off[1])
tmag.reg_set(tmag.REG_SENSOR_CONFIG_2, saved_cfg2)
tmag.range_refresh()

# %% [markdown]
# ## 8. Range switching, then restore
#
# The physical field does not change when the range does, so both ranges must report
# the same field. A driver that failed to resync its cached range would be off by
# exactly 2x. Then every config register goes back to its reset value.

# %%
saved_cfg2 = tmag.reg_read(tmag.REG_SENSOR_CONFIG_2)
tmag.ranges_set(40.0, 40.0)
sleep_ms(10)
at40 = tmag.magnetic()
tmag.ranges_set(80.0, 80.0)
sleep_ms(10)
at80 = tmag.magnetic()
check("driver tracked the range", (tmag.xy_range, tmag.z_range) == (80.0, 80.0))
for ax, a, b in zip("XYZ", at40, at80):
    check("{} agrees across ranges".format(ax), abs(a - b) < max(1.0, abs(a) * 0.1),
          "{:+.4f} vs {:+.4f} mT".format(a, b))  # fmt: skip

tmag.reg_set(tmag.REG_SENSOR_CONFIG_2, saved_cfg2)
tmag.reset_config()
tmag.status_clear()
for reg in tmag.CONFIG_REGS[:4]:
    check("reg {:#04x} at POR".format(reg), tmag.reg_read(reg) == tmag.CONFIG_POR)

# %% [markdown]
# ## 9. Summary
#
# Not covered here: angle accuracy (needs a graduated rotating mount), thresholds and
# interrupts (the Qwiic cable has no INT line), the provisional `T_THR_CONFIG` encoding,
# gain correction, and the A2 variant.

# %%
check_summary()

# %%
# %board disconnect
