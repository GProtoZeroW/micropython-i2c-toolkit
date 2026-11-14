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

# %%
# ruff: noqa
# NOTE: run this cell first. It connects the micropylab kernel to the board; with one USB board
# it finds the port by itself. With several, give the port: %board connect /dev/ttyACM0
# %board connect

# %% [markdown]
# # I2C Routing Harness — full run flow
#
# 1. lock helper + `I2CConnection` (routed, locked transport handle)
# 2. contract: `I2CNode` -> {`I2CDevice`, `I2CMux`}
# 3. drivers: `TMAG5273`, `SHTC3`, `TCA9548A`
# 4. `REGISTRY`
# 5. `RoutingTable` (ingest + structural + driver-aware validation)
# 6. `scan_check` (post-validate hardware probe, mux-aware)
# 7. `I2CManager` (build buses/locks/connections + instances)
# 8. run it: load -> validate -> scan -> build -> read

# %% [markdown]
# ## 1. Lock + I2CConnection

# %%
try:
    import _thread
    def _default_lock():
        return _thread.allocate_lock()
except ImportError:
    class _NullLock:
        def acquire(self, *a):
            return True
        def release(self):
            pass
        def __enter__(self):
            return self
        def __exit__(self, *a):
            return False
    def _default_lock():
        return _NullLock()


class I2CConnection:
    """One bound (bus, mux-path, address) transport handle — synchronous.

    Every transfer: acquire the per-bus lock, select the mux path, transfer
    at the baked-in address, release. The wrapper sees none of this; it just
    calls readfrom_mem / writeto / etc. The lock is injected (one per bus).
    """

    def __init__(self, bus, address, path, lock=None):
        self._bus  = bus                       # machine.I2C
        self._addr = address                   # baked in; wrapper never passes it
        self._path = path                      # [[mux_addr, channel], ...], resolved
        self._lock = lock or _default_lock()   # per-bus, injected by the manager

    def _select_path(self):
        for mux_addr, ch in self._path:
            self._bus.writeto(mux_addr, bytes([1 << ch]))

    # register-oriented transfers
    def readfrom_mem(self, reg, nbytes):
        with self._lock:
            self._select_path()
            return self._bus.readfrom_mem(self._addr, reg, nbytes)

    def readfrom_mem_into(self, reg, buf):
        with self._lock:
            self._select_path()
            self._bus.readfrom_mem_into(self._addr, reg, buf)

    def writeto_mem(self, reg, data):
        with self._lock:
            self._select_path()
            self._bus.writeto_mem(self._addr, reg, data)

    # raw transfers
    def writeto(self, data):
        with self._lock:
            self._select_path()
            return self._bus.writeto(self._addr, data)

    def readfrom(self, nbytes):
        with self._lock:
            self._select_path()
            return self._bus.readfrom(self._addr, nbytes)

    def readfrom_into(self, buf):
        with self._lock:
            self._select_path()
            self._bus.readfrom_into(self._addr, buf)


# %% [markdown]
# ## 2. Inheritance contract: I2CNode -> {I2CDevice, I2CMux}

# %%
class I2CNode:
    """Common ancestor for anything addressable on an I2C bus. Owns only the
    class-attribute 'datasheet facts' contract — no transfer model, since
    devices and muxes reach the bus differently."""
    ALLOWED_ADDRS = ()
    BAUD_NOMINAL  = 100_000
    BAUD_MAX      = 400_000
    BAUD_MIN      = 10_000

    @classmethod
    def addr_ok(cls, addr):
        return addr in cls.ALLOWED_ADDRS


class I2CDevice(I2CNode):
    """A routed device: talks THROUGH an I2CConnection, oblivious to
    addr/path/lock."""
    def __init__(self, conn, **kwargs):
        self.conn = conn
        self.kwargs = kwargs


class I2CMux(I2CNode):
    """A routing element: talks to the BARE bus at its own address, does its
    own upstream selection when chained. Defines the mux API contract the
    manager and connections rely on."""
    CHANNELS = 0                      # subclass declares its channel count

    def __init__(self, bus, address, lock=None, path=None, reset_pin=None):
        self._bus   = bus
        self._addr  = address
        self._lock  = lock or _default_lock()
        self._path  = path or []
        self._reset = None
        if reset_pin is not None:
            from machine import Pin
            self._reset = Pin(reset_pin, Pin.OUT, value=1)

    # contract: subclasses implement
    def select(self, channel):
        raise NotImplementedError
    def deselect(self):
        raise NotImplementedError
    def read_control(self):
        raise NotImplementedError
    def reset(self):
        raise NotImplementedError

    # shared helper
    def _select_upstream(self):
        for mux_addr, ch in self._path:
            self._bus.writeto(mux_addr, bytes([1 << ch]))


# %% [markdown]
# ## 3. Concrete drivers

# %%
import time


class TMAG5273(I2CDevice):
    """SparkFun Qwiic 3D Hall-effect sensor. Register-mapped, has identity."""
    ALLOWED_ADDRS = (0x35,)
    BAUD_NOMINAL  = 400_000
    BAUD_MAX      = 400_000

    REG_MANUFACTURER_ID_LSB = 0x0D
    REG_MANUFACTURER_ID_MSB = 0x0E
    REG_DEVICE_ID           = 0x0C

    def read_manufacturer_id(self):
        lsb = self.conn.readfrom_mem(self.REG_MANUFACTURER_ID_LSB, 1)[0]
        msb = self.conn.readfrom_mem(self.REG_MANUFACTURER_ID_MSB, 1)[0]
        return (msb << 8) | lsb

    def read_device_id(self):
        return self.conn.readfrom_mem(self.REG_DEVICE_ID, 1)[0]


class SHTC3(I2CDevice):
    """Sensirion humidity/temp. Command-based (not register-mapped)."""
    ALLOWED_ADDRS = (0x70,)
    BAUD_NOMINAL  = 400_000
    BAUD_MAX      = 400_000

    CMD_WAKEUP  = b'\x35\x17'
    CMD_SLEEP   = b'\xB0\x98'
    CMD_MEASURE = b'\x78\x66'     # T-first, normal mode, no clock stretch
    CMD_READ_ID = b'\xEF\xC8'

    def read_id(self):
        self.conn.writeto(self.CMD_WAKEUP)
        time.sleep_ms(1)
        self.conn.writeto(self.CMD_READ_ID)
        time.sleep_ms(1)
        d = self.conn.readfrom(3)          # 16-bit id + CRC
        self.conn.writeto(self.CMD_SLEEP)
        return (d[0] << 8) | d[1]

    def measure(self):
        self.conn.writeto(self.CMD_WAKEUP)
        time.sleep_ms(1)
        self.conn.writeto(self.CMD_MEASURE)
        time.sleep_ms(20)                  # conversion wait
        d = self.conn.readfrom(6)          # T(2)+CRC + RH(2)+CRC
        self.conn.writeto(self.CMD_SLEEP)
        t  = -45 + 175 * ((d[0] << 8 | d[1]) / 65536)
        rh = 100 * ((d[3] << 8 | d[4]) / 65536)
        return t, rh


class TCA9548A(I2CMux):
    """8-channel I2C mux. One 8-bit control register: each bit = one channel
    (1<<ch to select)."""
    ALLOWED_ADDRS = tuple(range(0x70, 0x78))   # 0x70..0x77, strap-selectable
    CHANNELS      = 8
    BAUD_NOMINAL  = 400_000
    BAUD_MAX      = 400_000

    def select(self, channel):
        if not (0 <= channel < self.CHANNELS):
            raise ValueError("channel {} out of range 0..{}".format(channel, self.CHANNELS - 1))
        with self._lock:
            self._select_upstream()
            self._bus.writeto(self._addr, bytes([1 << channel]))

    def select_mask(self, mask):
        with self._lock:
            self._select_upstream()
            self._bus.writeto(self._addr, bytes([mask & 0xFF]))

    def deselect(self):
        with self._lock:
            self._select_upstream()
            self._bus.writeto(self._addr, bytes([0x00]))

    def read_control(self):
        with self._lock:
            self._select_upstream()
            return self._bus.readfrom(self._addr, 1)[0]

    def reset(self):
        if self._reset is None:
            return False
        self._reset.value(0)
        time.sleep_us(10)
        self._reset.value(1)
        time.sleep_us(10)
        return True


# %% [markdown]
# ## 4. Registry: name -> class

# %%
REGISTRY = {
    "TMAG5273": TMAG5273,
    "SHTC3":    SHTC3,
    "TCA9548A": TCA9548A,
}


# %% [markdown]
# ## 5. RoutingTable — ingest + validation
#
# `validate(registry=None)` runs structural checks always; the driver-aware
# checks (driver exists, address legal per class `addr_ok`, channel within the
# mux class's `CHANNELS`) run only when a registry is passed. All checks
# accumulate into one list and raise once.

# %%
try:
    import json
except ImportError:
    import ujson as json


class ConfigError(Exception):
    def __init__(self, problems):
        self.problems = list(problems)
        super().__init__("routing table invalid:\n  " + "\n  ".join(self.problems))


class RoutingTable:
    def __init__(self, buses, muxes, devices):
        self.buses = buses
        self.muxes = muxes
        self.devices = devices

    @staticmethod
    def _parse_addr(v):
        if isinstance(v, str):
            return int(v, 16) if v.lower().startswith("0x") else int(v)
        return v

    @classmethod
    def loads(cls, raw):
        data = raw if isinstance(raw, dict) else json.loads(raw)
        return cls.from_dict(data)

    @classmethod
    def from_dict(cls, data):
        buses = list(data.get("buses", []))

        muxes = {}
        for name, m in data.get("muxes", {}).items():
            muxes[name] = {
                "bus": m["bus"],
                "address": cls._parse_addr(m["address"]),
                "path": [[hop[0], hop[1]] for hop in m.get("path", [])],
                "reset_pin": m.get("reset_pin"),
                "driver": m.get("driver"),       # optional: which mux class
            }

        devices = []
        for d in data.get("devices", []):
            devices.append({
                "name": d["name"],
                "bus": d["bus"],
                "address": cls._parse_addr(d["address"]),
                "path": [[hop[0], hop[1]] for hop in d.get("path", [])],
                "driver": d["driver"],
                "init_kwargs": d.get("init_kwargs", {}),
            })

        return cls(buses, muxes, devices)

    # ---- helpers ----

    def _segment_key(self, bus, path):
        hops = tuple((self.muxes[name]["address"], ch) for name, ch in path)
        return (bus, hops)

    def _refs_intact(self, problems):
        return not any("unknown mux" in p for p in problems)

    # ---- structural checks (no registry needed) ----

    def _check_mux_refs(self, problems):
        for d in self.devices:
            for name, ch in d["path"]:
                if name not in self.muxes:
                    problems.append(
                        "device {!r}: path references unknown mux {!r}".format(d["name"], name))
        for mname, m in self.muxes.items():
            for name, ch in m["path"]:
                if name not in self.muxes:
                    problems.append(
                        "mux {!r}: path references unknown mux {!r}".format(mname, name))

    def _check_bus_declared(self, problems):
        for d in self.devices:
            if d["bus"] not in self.buses:
                problems.append(
                    "device {!r}: bus {} not in declared buses {}".format(
                        d["name"], d["bus"], self.buses))
        for mname, m in self.muxes.items():
            if m["bus"] not in self.buses:
                problems.append(
                    "mux {!r}: bus {} not in declared buses {}".format(
                        mname, m["bus"], self.buses))

    def _check_dup_names(self, problems):
        seen = {}
        for d in self.devices:
            seen[d["name"]] = seen.get(d["name"], 0) + 1
        for name, n in seen.items():
            if n > 1:
                problems.append("duplicate device name {!r} ({} entries)".format(name, n))

    def _check_channel_range(self, problems):
        # structural range 0..7 (TCA9548A max); driver-aware check refines this
        for d in self.devices:
            for name, ch in d["path"]:
                if not (0 <= ch <= 7):
                    problems.append(
                        "device {!r}: channel {} out of range 0..7 on mux {!r}".format(
                            d["name"], ch, name))

    def _check_bus_path_consistency(self, problems):
        for d in self.devices:
            if d["path"]:
                first_mux = d["path"][0][0]
                if first_mux in self.muxes:
                    mbus = self.muxes[first_mux]["bus"]
                    if mbus != d["bus"]:
                        problems.append(
                            "device {!r}: bus {} but path enters mux {!r} on bus {}".format(
                                d["name"], d["bus"], first_mux, mbus))

    def _check_mux_cycles(self, problems):
        if not self._refs_intact(problems):
            return
        for start in self.muxes:
            seen_chain = set()
            cur = start
            while True:
                if cur in seen_chain:
                    problems.append("mux cycle detected involving {!r}".format(start))
                    break
                seen_chain.add(cur)
                p = self.muxes[cur]["path"]
                if not p:
                    break
                cur = p[0][0]
                if cur not in self.muxes:
                    break

    def _check_addr_collisions(self, problems):
        if not self._refs_intact(problems):
            return
        by_key = {}
        for d in self.devices:
            key = (self._segment_key(d["bus"], d["path"]), d["address"])
            by_key.setdefault(key, []).append(d["name"])
        for mname, m in self.muxes.items():
            key = (self._segment_key(m["bus"], m["path"]), m["address"])
            by_key.setdefault(key, []).append("mux:" + mname)
        for key, names in by_key.items():
            if len(names) > 1:
                (bus, hops), addr = key
                problems.append(
                    "address collision at bus {} addr {:#04x} path {}: {}".format(
                        bus, addr, list(hops), ", ".join(names)))

    # ---- driver-aware checks (need registry: name -> class) ----

    def _check_driver_exists(self, problems, registry):
        for d in self.devices:
            if d["driver"] not in registry:
                problems.append(
                    "device {!r}: driver {!r} not in registry".format(d["name"], d["driver"]))
        for mname, m in self.muxes.items():
            drv = m.get("driver")
            if drv is not None and drv not in registry:
                problems.append(
                    "mux {!r}: driver {!r} not in registry".format(mname, drv))

    def _check_addr_legal(self, problems, registry):
        for d in self.devices:
            cls = registry.get(d["driver"])
            if cls is None:
                continue  # already flagged by _check_driver_exists
            if not cls.addr_ok(d["address"]):
                problems.append(
                    "device {!r}: addr {:#04x} not valid for {} (allowed {})".format(
                        d["name"], d["address"], d["driver"],
                        [hex(a) for a in cls.ALLOWED_ADDRS]))
        for mname, m in self.muxes.items():
            drv = m.get("driver")
            cls = registry.get(drv) if drv else None
            if cls is None:
                continue
            if not cls.addr_ok(m["address"]):
                problems.append(
                    "mux {!r}: addr {:#04x} not valid for {} (allowed {})".format(
                        mname, m["address"], drv,
                        [hex(a) for a in cls.ALLOWED_ADDRS]))

    def _check_channel_vs_class(self, problems, registry):
        for d in self.devices:
            for name, ch in d["path"]:
                m = self.muxes.get(name)
                if m is None:
                    continue
                cls = registry.get(m.get("driver"))
                if cls is None:
                    continue
                if not (0 <= ch < cls.CHANNELS):
                    problems.append(
                        "device {!r}: channel {} out of range 0..{} for mux {!r} ({})".format(
                            d["name"], ch, cls.CHANNELS - 1, name, m.get("driver")))

    # ---- orchestrator ----

    def validate(self, registry=None):
        problems = []

        structural = (
            self._check_mux_refs,
            self._check_bus_declared,
            self._check_dup_names,
            self._check_channel_range,
            self._check_bus_path_consistency,
            self._check_mux_cycles,
            self._check_addr_collisions,
        )
        for check in structural:
            check(problems)

        if registry is not None:
            driver_aware = (
                self._check_driver_exists,
                self._check_addr_legal,
                self._check_channel_vs_class,
            )
            for check in driver_aware:
                check(problems, registry)

        if problems:
            raise ConfigError(problems)
        return self


# %% [markdown]
# ## 6. scan_check — post-validate hardware probe (mux-aware)
#
# Groups every declared target (devices + muxes) by segment (bus + resolved
# mux path), selects the path, scans once, confirms each declared address
# ACKs. NOT foolproof — a scan ACK means 'something is at that address', not
# 'the right chip'. Identity checks confirm the chip.

# %%
def scan_check(table, buses, verbose=True):
    targets = []
    for d in table.devices:
        targets.append((d["bus"], d["path"], d["address"], d["name"]))
    for mname, m in table.muxes.items():
        targets.append((m["bus"], m["path"], m["address"], "mux:" + mname))

    segments = {}
    for bus, path, addr, label in targets:
        hops = tuple((table.muxes[name]["address"], ch) for name, ch in path)
        segments.setdefault((bus, hops), []).append((addr, label))

    results = []
    all_ok = True

    for (bus_id, hops), members in segments.items():
        i2c = buses[bus_id]

        for mux_addr, ch in hops:
            i2c.writeto(mux_addr, bytes([1 << ch]))

        found = set(i2c.scan())

        for addr, label in members:
            present = addr in found
            results.append({
                "bus": bus_id, "hops": list(hops),
                "addr": addr, "label": label, "present": present,
            })
            if not present:
                all_ok = False

        for mux_addr, ch in hops:
            i2c.writeto(mux_addr, bytes([0x00]))

    if verbose:
        for r in results:
            mark = "OK " if r["present"] else "MISS"
            hs = "" if not r["hops"] else " via {}".format(
                ["{:#04x}:ch{}".format(a, c) for a, c in r["hops"]])
            print("[{}] bus{} {:#04x} {}{}".format(
                mark, r["bus"], r["addr"], r["label"], hs))
        print("scan_check:", "all present" if all_ok else "MISSING targets")

    return all_ok, results


# %% [markdown]
# ## 7. I2CManager — build buses/locks/connections + instances

# %%
from machine import I2C, Pin


BOARD_CATALOG = {
    "feathers3": {
        "buses": {0: {"scl": 9, "sda": 8, "baud_max": 400_000}},
    },
}


class I2CManager:
    """Consumes a validated RoutingTable, builds buses + per-bus locks +
    connections, constructs mux and device instances, owns them by name.

    Order: buses (clocked from class BAUD_MAX facts) -> mux instances ->
    device connections + instances. Frequency is resolved before any
    instance exists, so an init-time identity read runs on a safe bus.
    """

    def __init__(self, table, registry, board):
        self.table = table
        self.registry = registry
        self.board = BOARD_CATALOG[board]
        self.buses = {}      # bus_id -> machine.I2C
        self.locks = {}      # bus_id -> lock (one per bus, shared)
        self.muxes = {}      # name -> mux instance
        self.devices = {}    # name -> device instance

    def build(self):
        self._build_buses()
        self._build_muxes()
        self._build_devices()
        return self.devices

    def _build_buses(self):
        for bus_id in self.table.buses:
            cfg = self.board["buses"][bus_id]
            freq = self._bus_ceiling(bus_id, cfg)
            self.buses[bus_id] = I2C(bus_id,
                                     scl=Pin(cfg["scl"]),
                                     sda=Pin(cfg["sda"]),
                                     freq=freq)
            self.locks[bus_id] = _default_lock()

    def _bus_ceiling(self, bus_id, cfg):
        maxes = [cfg.get("baud_max", 400_000)]
        for d in self.table.devices:
            if d["bus"] == bus_id:
                maxes.append(self.registry[d["driver"]].BAUD_MAX)
        for m in self.table.muxes.values():
            if m["bus"] == bus_id and m.get("driver"):
                maxes.append(self.registry[m["driver"]].BAUD_MAX)
        return min(maxes)

    def _resolve_path(self, entry_path):
        return [[self.table.muxes[name]["address"], ch]
                for name, ch in entry_path]

    def _build_muxes(self):
        for name, m in self.table.muxes.items():
            cls = self.registry[m["driver"]]
            self.muxes[name] = cls(
                bus=self.buses[m["bus"]],
                address=m["address"],
                lock=self.locks[m["bus"]],
                path=self._resolve_path(m["path"]),
                reset_pin=m.get("reset_pin"),
            )

    def _build_devices(self):
        for d in self.table.devices:
            cls = self.registry[d["driver"]]
            conn = I2CConnection(
                bus=self.buses[d["bus"]],
                address=d["address"],
                path=self._resolve_path(d["path"]),
                lock=self.locks[d["bus"]],
            )
            self.devices[d["name"]] = cls(conn, **d["init_kwargs"])

    def get(self, name):
        return self.devices.get(name) or self.muxes.get(name)


# %% [markdown]
# ## 8. Run flow: load -> validate -> scan -> build -> read
#
# Mux strapped to 0x71 to dodge the SHTC3-at-0x70 collision.

# %%
config_json = '''
{
  "buses": [0],
  "muxes": {
    "mux_a": {"bus": 0, "address": "0x71", "path": [], "driver": "TCA9548A"}
  },
  "devices": [
    {"name": "humidity", "bus": 0, "address": "0x70", "path": [], "driver": "SHTC3"},
    {"name": "hall",     "bus": 0, "address": "0x35", "path": [], "driver": "TMAG5273"}
  ]
}
'''

# %%
# load + validate (structural + driver-aware)
table = RoutingTable.loads(config_json).validate(REGISTRY)
print("validation passed")

# %%
# build buses first so scan_check has live I2C to probe
mgr = I2CManager(table, REGISTRY, board="feathers3")
mgr._build_buses()   # buses/locks up; muxes/devices built below after scan

# %%
# hardware probe against the declared topology
ok, results = scan_check(table, mgr.buses)

# %%
# finish the build: mux + device instances
mgr._build_muxes()
mgr._build_devices()
devices = mgr.devices
print("built:", list(devices.keys()), "+ muxes", list(mgr.muxes.keys()))

# %%
# talk to real silicon through the wrappers
hall = devices["hall"]
humidity = devices["humidity"]

print("hall device id:    ", hex(hall.read_device_id()))
print("hall mfr id:       ", hex(hall.read_manufacturer_id()))
print("humidity sensor id:", hex(humidity.read_id()))
t, rh = humidity.measure()
print("temp {:.2f} C   rh {:.1f} %".format(t, rh))

# %%
# mux reachable by name too
print("mux_a control reg:", hex(mgr.get("mux_a").read_control()))
