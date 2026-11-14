"""``I2CManager`` builds live objects from a validated ``RoutingTable``; ``scan_check`` probes.

Build order: buses (one lock each, clocked to suit every part on them), then mux
instances, then device connections and instances. The clock is settled before any
driver exists, so a driver's init-time identity read runs on a safe bus.

Typical use::

    table = RoutingTable.loads(config_json).validate(registry())
    mgr = I2CManager(table, registry(), "feathers3")
    mgr.buses_build()
    ok, _ = mgr.scan()
    mgr.build()
    hall = mgr["hall"]
"""

from micropython_i2c.boards import board_get
from micropython_i2c.compat import default_lock
from micropython_i2c.connection import I2CConnection, path_release, path_select


def scan_check(table, buses, locks=None, verbose=True):
    """Confirm every declared address ACKs on its segment.

    Groups entries by segment (bus + path), selects each path, scans once, then
    releases the path innermost first. An ACK means *something* answers at that address,
    not that it is the right part; identity checks confirm the chip.

    Args:
        table: A ``RoutingTable``.
        buses: ``{bus_id: machine.I2C}``.
        locks: Optional ``{bus_id: lock}``; pass the manager's when other code may be
            using the bus concurrently.
        verbose: Print one line per target and a summary.

    Returns:
        ``(all_present, results)`` where each result is a dict with ``bus``, ``hops``,
        ``addr``, ``label`` and ``present``.
    """
    results = []
    for (bus_id, path), members in table.segments().items():
        hops = table.resolve_path(path)
        i2c = buses[bus_id]
        lock = locks[bus_id] if locks else default_lock()
        with lock:
            path_select(i2c, hops)
            try:
                found = set(i2c.scan())
            finally:
                path_release(i2c, hops)
        for addr, label in members:
            results.append(
                {
                    "bus": bus_id,
                    "hops": hops,
                    "addr": addr,
                    "label": label,
                    "present": addr in found,
                }  # fmt: skip
            )

    all_ok = all(r["present"] for r in results)
    if verbose:
        for r in results:
            via = "".join(" via {:#04x}:ch{}".format(a, c) for a, c in r["hops"])
            print(
                "[{}] bus{} {:#04x} {}{}".format(
                    "OK  " if r["present"] else "MISS", r["bus"], r["addr"], r["label"], via
                )
            )
        print("scan_check:", "all present" if all_ok else "MISSING targets")
    return all_ok, results


class I2CManager:
    """Owns the buses, per-bus locks, muxes and devices described by a routing table.

    Args:
        table: A validated ``RoutingTable``.
        registry: ``{driver name: class}``, e.g. ``drivers.registry()``.
        board: A ``boards.BOARDS`` key or a board dict.

    Attributes:
        buses: ``{bus_id: machine.I2C}``.
        locks: ``{bus_id: lock}``, one shared by everything on that bus.
        muxes: ``{name: I2CMux}``.
        devices: ``{name: I2CDevice}``.
    """

    def __init__(self, table, registry, board):
        self.table = table
        self.registry = registry
        self.board = board_get(board)
        self.buses = {}
        self.locks = {}
        self.muxes = {}
        self.devices = {}

    def __getitem__(self, name):
        return self.get(name)

    def get(self, name):
        """A device or mux by config name.

        Raises:
            KeyError: If no device or mux has that name.
        """
        if name in self.devices:
            return self.devices[name]
        return self.muxes[name]

    def bus_freq(self, bus_id):
        """Clock for ``bus_id``: the lowest ``BAUD_MAX`` of the board and every part on it.

        Raises:
            ValueError: If some part's ``BAUD_MIN`` is above that ceiling.
        """
        ceiling = self.board["buses"][bus_id].get("baud_max", 400_000)
        floor = 0
        for _, e in self.table.entries():
            if e["bus"] == bus_id and e.get("driver"):
                cls = self.registry[e["driver"]]
                ceiling = min(ceiling, cls.BAUD_MAX)
                floor = max(floor, cls.BAUD_MIN)
        if floor > ceiling:
            raise ValueError(
                "bus {}: no clock satisfies every part ({} Hz min > {} Hz max)".format(
                    bus_id, floor, ceiling
                )
            )
        return ceiling

    def buses_build(self):
        """Create each ``machine.I2C`` and its lock."""
        from machine import I2C, Pin

        for bus_id in self.table.buses:
            cfg = self.board["buses"][bus_id]
            self.buses[bus_id] = I2C(
                bus_id, scl=Pin(cfg["scl"]), sda=Pin(cfg["sda"]), freq=self.bus_freq(bus_id)
            )
            self.locks[bus_id] = default_lock()
        return self

    def scan(self, verbose=True):
        """``scan_check`` against the built buses. Call after ``buses_build()``."""
        return scan_check(self.table, self.buses, self.locks, verbose)

    def muxes_build(self):
        """Instantiate every mux on its bus, with the shared bus lock."""
        for name, m in self.table.muxes.items():
            self.muxes[name] = self.registry[m["driver"]](
                bus=self.buses[m["bus"]],
                address=m["address"],
                lock=self.locks[m["bus"]],
                path=self.table.resolve_path(m["path"]),
                reset_pin=m.get("reset_pin"),
            )
        return self

    def devices_build(self):
        """Build each device's connection, then the driver instance."""
        for d in self.table.devices:
            conn = I2CConnection(
                self.buses[d["bus"]],
                d["address"],
                self.table.resolve_path(d["path"]),
                self.locks[d["bus"]],
            )
            self.devices[d["name"]] = self.registry[d["driver"]](conn, **d["init_kwargs"])
        return self

    def build(self):
        """Build whatever is not built yet: buses, then muxes, then devices."""
        if not self.buses:
            self.buses_build()
        if not self.muxes:
            self.muxes_build()
        if not self.devices:
            self.devices_build()
        return self
