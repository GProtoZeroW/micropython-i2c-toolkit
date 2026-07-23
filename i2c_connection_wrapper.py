# -*- coding: utf-8 -*-
# ---
# jupyter:
#   jupytext:
#     formats: ipynb,py:percent
#     text_representation:
#       extension: .py
#       format_name: percent
#       format_version: '1.3'
#       jupytext_version: 1.19.1
#   kernelspec:
#     display_name: MicroPython - USB
#     language: micropython
#     name: micropython
# ---

# %% [markdown]
# # A PyVISA-Style I2C Connection System for MicroPython
#
# ## Purpose
#
# The `machine.I2C` API that MicroPython ships is a *bus-level* API: every call
# carries the target address, and nothing in the API knows about multiplexers,
# concurrency, or which device is which. That is the right primitive, but it is
# the wrong level of abstraction to write drivers against.
#
# This notebook builds a *connection-level* layer on top of it, modelled on the
# way PyVISA treats an instrument: you resolve a resource once, you get back a
# handle, and from then on the handle is the instrument. Addressing, routing and
# arbitration are the handle's problem, not the driver's.
#
# ## Design goals
#
# 1. **Nothing is hidden.** The underlying `machine.I2C` object is still there and
#    still usable. This layer wraps; it does not replace or shadow the existing
#    API.
# 2. **Bake in the boring parts.** A driver should never have to remember a device
#    address, nor which mux channels have to be selected to reach it.
# 3. **Thread/async safety.** A single I2C bus can only serve one transaction at a
#    time. Every transfer through this layer is serialised on a per-bus lock, so
#    drivers can be driven from `_thread` workers or `asyncio` tasks without
#    interleaving transfers and corrupting them.
# 4. **Mux paths are part of the address.** On a bus with TCA9548A-class muxes,
#    "address 0x40" is ambiguous — there may be eight of them. The full coordinate
#    is `(bus, mux path, address)`, and that whole triple is what gets baked into
#    a connection.
# 5. **Datasheet facts live on the class.** Allowed addresses and baud rate limits
#    are properties of the *part*, not of an instance, so they are class
#    attributes. This lets an auto-setup/config system reason about a device
#    before instantiating it.
#
# ## Layer map
#
# ```
#   machine.I2C                      MicroPython built-in, bus level
#        |
#   I2CConnection                    one bound (bus, path, address) handle
#        |                           locking + mux routing live here
#        |
#   I2CNode                          contract: datasheet facts + transport methods
#      /     \
#   I2CDevice  I2CMux                routed device  |  routing element
#        |          |
#        +----------+
#             |
#   BitTools / CRCMixin              stateless helpers | checksum contract
#             |
#   RegisterI2CBase / CommandI2CBase protocol mixins, transport-agnostic
#             |
#   TMP102Proto / TMP102             concrete drivers (sections 6 and 8)
# ```
#
# The key structural decision is that the protocol mixins call
# `self.read_mem()` / `self.write_raw()` / etc. — **never** `self.conn` — so the
# exact same protocol code runs on a device sitting behind three muxes and on a
# mux talking to the bare bus.
#
# ## Notebook layout
#
# | Section | Contents |
# |---|---|
# | 1–4 | The framework: lock, connection, node tree, helpers, protocol mixins |
# | 5 | Display helpers and bring-up of a real bus |
# | 6 | `TMP102Proto` — a datasheet-facts-only shell, used purely as a test vehicle |
# | 7 | Verification of every `RegisterI2CBase` feature, through that shell |
# | 8 | `TMP102` — the real driver |
# | 9 | Verification of the `TMP102` driver |
# | 10 | Offline verification of `BitTools` and `CRCMixin` |
# | 11 | Status and next steps |

# %%
# ruff: noqa
# NOTE: run this cell first — connects the Jupyter MicroPython kernel to the board.
%serialconnect --port=/dev/ttyACM1

# %% [markdown]
# # 1. Lock Mechanism
#
# ## Why a lock at all
#
# An I2C transaction is not atomic from the bus's point of view: a register read
# is a write of the register number followed by a read of the data, and a mux
# path selection is a separate transfer again. If a second thread issues its own
# transfer in the middle of that sequence, at best you get a wrong-register read,
# and at worst the mux is switched out from under an in-flight transaction and
# the data comes from an entirely different device.
#
# So the unit that has to be serialised is **select-path-plus-transfer**, not
# the individual `machine.I2C` call. Every transport method in this notebook
# takes the lock, selects, transfers, and releases.
#
# ## One lock per bus
#
# The contended resource is the *bus*, not the device and not the mux. Two
# devices on the same bus must exclude each other; two devices on different buses
# need not. The lock is therefore created once per bus by the manager and
# *injected* into every connection and mux on that bus — hence the `lock=None`
# parameters everywhere rather than each object minting its own.
#
# ## The fallback
#
# `_thread` is not present in every MicroPython port/build. Rather than making
# threading support a hard import requirement, we fall back to a null lock that
# satisfies the context-manager protocol and does nothing. On a single-threaded
# build this is correct and costs essentially nothing; on a threaded build the
# real `_thread.allocate_lock()` is used.

# %%
try:
    import _thread

    def default_lock():
        """Return a real mutex from the `_thread` module.

        Used on builds where `_thread` is available. `allocate_lock()` returns
        an object supporting acquire/release and the context-manager protocol,
        which is exactly the interface this module depends on.
        """
        return _thread.allocate_lock()

except ImportError:
    class NullLock:
        """No-op stand-in for a mutex on single-threaded builds.

        Implements just enough of the lock interface (`acquire`, `release`,
        `__enter__`, `__exit__`) that call sites need no conditional logic.
        Every method succeeds immediately and guards nothing — which is the
        correct behaviour when there is no concurrency to guard against.
        """

        def acquire(self, *a):
            """Always succeed; accepts and ignores blocking/timeout arguments."""
            return True

        def release(self):
            """No-op — nothing was ever held."""
            pass

        def __enter__(self):
            """Enter the context; returns self."""
            return self

        def __exit__(self, *a):
            """Exit the context, propagating any exception (returns False)."""
            return False

    def default_lock():
        """Return a no-op lock, for builds without `_thread`."""
        return NullLock()


# %% [markdown]
# # 2. The I2C Connection Object
#
# ## What a connection is
#
# `I2CConnection` is the PyVISA-resource analogue: an immutable binding of
# **one bus**, **one mux path**, and **one device address**, plus the lock that
# guards the bus.
#
# Once constructed, it presents the familiar `machine.I2C` method names —
# `readfrom_mem`, `writeto`, `readfrom_into`, and friends — but with the address
# argument removed, because the address is already baked in. A driver holding a
# connection cannot address the wrong device, because it has no way to express
# an address at all.
#
# ## The mux path
#
# `path` is an ordered list of `[mux_address, channel]` hops, outermost first:
#
# ```python
# []                              # device sits directly on the bus
# [[0x70, 3]]                     # reached via channel 3 of the mux at 0x70
# [[0x70, 3], [0x71, 0]]          # ...then channel 0 of the mux at 0x71
# ```
#
# The path is *resolved* before it reaches the connection — the manager has
# already turned human-readable topology into this list. `path_select()` simply
# replays the hops in order, writing a one-hot channel mask to each mux.
#
# ## Why every transfer re-selects
#
# `path_select()` runs on *every* transfer, not once at construction. This is
# deliberate. Mux state is global to the bus and any other connection may have
# changed it since we last looked. Re-selecting under the same lock that guards
# the transfer is what makes the pair atomic — it is the entire reason this is
# safe under concurrency. On an empty path it is a no-op loop and costs nothing.
#
# ## Method mirroring
#
# The methods deliberately keep MicroPython's names and argument order (minus
# `addr`). Existing driver code written against `machine.I2C` ports across with a
# near-mechanical edit, and there is no new vocabulary to learn.

# %%
class I2CConnection:
    """One bound (bus, mux-path, address) transport handle — synchronous.

    A connection is the PyVISA-style resource handle for this system: it pins
    down exactly which physical device is being talked to, and exposes transfer
    methods that need no address argument.

    Every transfer follows the same four steps:

      1. acquire the per-bus lock
      2. select the mux path (replay each [mux_addr, channel] hop)
      3. perform the transfer at the baked-in address
      4. release the lock

    Steps 2 and 3 are inside the same critical section on purpose: that is what
    prevents another thread from re-routing a mux between our selection and our
    transfer.

    The wrapper/driver layer above sees none of this. It just calls
    `readfrom_mem` / `writeto` / etc.

    Parameters
    ----------
    bus : machine.I2C
        The underlying MicroPython bus object. Not wrapped or hidden — callers
        who need the raw API can still reach it.
    address : int
        7-bit device address, baked in for the lifetime of the connection.
    path : list[list[int]]
        Resolved mux hops as ``[[mux_addr, channel], ...]``, outermost first.
        An empty list means the device sits directly on the bus.
    lock : lock-like, optional
        The per-bus mutex, injected by the manager. If omitted, a private lock
        is created — correct only when this connection is the sole user of the
        bus, so prefer to inject.

    Attributes
    ----------
    bus, addr, path, lock
        A connection's binding is not meant to be mutated after construction.
        Build a new connection instead.
    """

    def __init__(self, bus, address, path=None, lock=None):
        self.bus  = bus                       # machine.I2C
        self.addr = address                   # baked in; wrapper never passes it
        self.path = path or []                # [[mux_addr, channel], ...], resolved
        self.lock = lock or default_lock()    # per-bus, injected by the manager

    def path_select(self):
        """Replay the mux hops so this connection's device is reachable.

        Walks the path outermost-first, writing a one-hot channel mask
        (``1 << channel``) to each mux in turn. Each write must land before the
        next one is attempted, since the next mux is only visible once the
        upstream mux has switched to its channel.

        Called with the bus lock already held — it is never safe to call this
        on its own, because the selection is only meaningful up until another
        thread changes it.
        """
        for mux_addr, ch in self.path:
            self.bus.writeto(mux_addr, bytes([1 << ch]))

    # ------------------------------------------------------------------
    # Register-oriented transfers
    #
    # For devices with an internal register map. MicroPython handles the
    # write-register-then-read repeated-start sequence internally.
    # ------------------------------------------------------------------

    def readfrom_mem(self, reg, nbytes):
        """Read `nbytes` from register `reg`. Returns a bytes object."""
        with self.lock:
            self.path_select()
            return self.bus.readfrom_mem(self.addr, reg, nbytes)

    def readfrom_mem_into(self, reg, buf):
        """Read from register `reg` into a caller-supplied buffer of len(buf).

        Preferred over `readfrom_mem` in hot loops, since it reuses the caller's
        buffer instead of allocating — which matters on memory-constrained
        targets.
        """
        with self.lock:
            self.path_select()
            self.bus.readfrom_mem_into(self.addr, reg, buf)

    def writeto_mem(self, reg, data):
        """Write the bytes `data` to register `reg`."""
        with self.lock:
            self.path_select()
            self.bus.writeto_mem(self.addr, reg, data)

    # ------------------------------------------------------------------
    # Raw transfers
    #
    # For command-driven devices with no register map, where the payload is an
    # opcode or a bare data stream rather than a register number plus contents.
    # ------------------------------------------------------------------

    def writeto(self, data):
        """Write raw bytes to the device. Returns the ACK count from the bus."""
        with self.lock:
            self.path_select()
            return self.bus.writeto(self.addr, data)

    def readfrom(self, nbytes):
        """Read `nbytes` of raw data from the device. Returns a bytes object."""
        with self.lock:
            self.path_select()
            return self.bus.readfrom(self.addr, nbytes)

    def readfrom_into(self, buf):
        """Read raw data into a caller-supplied buffer of length `len(buf)`."""
        with self.lock:
            self.path_select()
            self.bus.readfrom_into(self.addr, buf)


# %% [markdown]
# # 3. The I2C Device Wrapper Framework
#
# ## Why a framework rather than ad-hoc drivers
#
# The connection layer above solves addressing and arbitration, but on its own
# it does not let an automatic setup/config system reason about a bus. To take a
# topology map and bring it up unattended, the setup code has to be able to ask,
# *before* it has a working connection:
#
# * Which addresses is this part allowed to answer on? (to validate a config, and
#   to disambiguate a scan)
# * What baud rate can this part tolerate? (to pick a bus speed that satisfies
#   every part sharing the bus)
#
# Those are datasheet facts about a *part*, not about an instance, so they live
# as class attributes on a common base. That is the whole reason `I2CNode`
# exists as a layer separate from `I2CDevice`.
#
# ## The class tree
#
# * **`I2CNode`** — common ancestor for anything addressable on a bus. Holds the
#   datasheet-facts contract (`ALLOWED_ADDRS`, `BAUD_*`) and *declares* four
#   transport methods without implementing them.
#
# * **`I2CDevice(I2CNode)`** — a device reached *through* an `I2CConnection`.
#   Its transport methods are thin delegations; the connection still does the
#   addressing, routing and locking.
#
# * **`I2CMux(I2CNode)`** — a routing element. It cannot route through itself, so
#   it talks to the bare bus at its own address, doing its own upstream selection
#   when muxes are chained. It implements the same four transport methods, plus
#   the mux API contract (`channel_select` / `channel_deselect` /
#   `control_read`) that the manager depends on.
#
# ## The four-method transport contract
#
# ```
# read_mem(reg, nbytes) -> bytes
# write_mem(reg, data)  -> None
# read_raw(nbytes)      -> bytes
# write_raw(data)       -> None
# ```
#
# This is the seam that makes the protocol mixins in section 4 reusable. Because
# a mixin only ever calls these four names, it neither knows nor cares whether it
# is bolted onto a routed device or onto a mux. A mux with a register map and a
# sensor with a register map get the identical `RegisterI2CBase` machinery.

# %%
class I2CNode:
    """Common ancestor for anything addressable on an I2C bus.

    Owns the class-attribute 'datasheet facts' contract, and declares the
    transport methods. Mixins call these — never self.conn — so the same
    protocol machinery works on routed devices and bare-bus muxes alike.

    Subclasses override the class attributes with values taken straight from the
    part's datasheet. They are class-level so that an auto-setup system can
    inspect them without needing a live instance or a working bus.

    Class attributes
    ----------------
    ALLOWED_ADDRS : tuple[int]
        Every 7-bit address the part can be strapped/configured to. Used to
        validate a topology map and to disambiguate bus scan results.
    BAUD_NOMINAL : int
        The part's normal operating speed — the sensible default.
    BAUD_MAX : int
        Ceiling from the datasheet. A shared bus must run at or below the lowest
        BAUD_MAX among its participants.
    BAUD_MIN : int
        Floor from the datasheet. Some parts time out or lose sync if clocked
        too slowly, so this is a real constraint, not a formality.
    """

    ALLOWED_ADDRS = ()
    BAUD_NOMINAL  = 100_000
    BAUD_MAX      = 400_000
    BAUD_MIN      = 10_000

    def __init__(self, **kwargs):
        # Unrecognised keyword arguments are retained rather than rejected, so
        # a config-map entry can carry extra per-node metadata (labels, units,
        # calibration hints) through to the driver without this base class
        # needing to know about any of it.
        self.kwargs = kwargs

    @classmethod
    def addr_ok(cls, addr):
        """Return True if `addr` is an address this part is allowed to use.

        A classmethod on purpose: config validation and scan disambiguation both
        need this answer before any instance exists.
        """
        return addr in cls.ALLOWED_ADDRS

    @classmethod
    def baud_ok(cls, freq):
        """Return True if `freq` falls within this part's datasheet baud limits."""
        return cls.BAUD_MIN <= freq <= cls.BAUD_MAX

    # ------------------------------------------------------------------
    # Transport contract
    #
    # Declared here, implemented by subclasses according to their bus-access
    # model (through a connection, or on the bare bus). Protocol mixins are
    # written against exactly these four names and nothing else.
    # ------------------------------------------------------------------

    def read_mem(self, reg, nbytes):
        """Read `nbytes` from register `reg`. Returns bytes."""
        raise NotImplementedError

    def write_mem(self, reg, data):
        """Write the bytes `data` to register `reg`."""
        raise NotImplementedError

    def read_raw(self, nbytes):
        """Read `nbytes` of raw data, no register phase. Returns bytes."""
        raise NotImplementedError

    def write_raw(self, data):
        """Write raw bytes, no register phase."""
        raise NotImplementedError


# %% [markdown]
# ## 3.1 `I2CDevice` — the routed leaf node
#
# The common case: a part somewhere on the bus, possibly behind muxes, that is
# reached through an `I2CConnection`.
#
# The transport methods here are deliberately trivial. All of the difficulty —
# which address, which mux path, which lock — was resolved when the connection
# was built, so there is nothing left for the device to do but forward the call.
# A device object holds no address and no path, and so cannot get either wrong.

# %%
class I2CDevice(I2CNode):
    """A routed device: talks THROUGH an I2CConnection, oblivious to
    addr/path/lock. Transport methods are thin delegations — the connection
    still does the addressing, mux selection, and locking.

    This is the base class for ordinary device drivers. A concrete driver
    subclasses it (usually alongside a protocol mixin from section 4), overrides
    the datasheet class attributes, and adds device-specific methods.

    Parameters
    ----------
    conn : I2CConnection
        A fully-bound connection to this device.
    **kwargs
        Passed up to `I2CNode` and retained on `self.kwargs`.
    """

    def __init__(self, conn, **kwargs):
        super().__init__(**kwargs)
        self.conn = conn

    def read_mem(self, reg, nbytes):
        """Read `nbytes` from register `reg` via the connection."""
        return self.conn.readfrom_mem(reg, nbytes)

    def write_mem(self, reg, data):
        """Write the bytes `data` to register `reg` via the connection."""
        self.conn.writeto_mem(reg, data)

    def read_raw(self, nbytes):
        """Read `nbytes` of raw data via the connection."""
        return self.conn.readfrom(nbytes)

    def write_raw(self, data):
        """Write raw bytes via the connection."""
        self.conn.writeto(data)


# %% [markdown]
# ## 3.2 `I2CMux` — the routing element
#
# ### Why a mux cannot use a connection
#
# An `I2CConnection` reaches its device by selecting a mux path. A mux *is* part
# of that path, so routing a mux through a connection would be circular: to talk
# to the mux you would first have to select a channel on the mux. Muxes therefore
# hold the bus, address, lock and their own upstream path directly, and implement
# the transport contract against the bare bus.
#
# ### Chained muxes and `upstream_select()`
#
# `self.path` on a mux is the list of hops needed to reach *this mux*, not to
# reach anything behind it. For a mux wired directly to the bus it is empty. For
# a mux hanging off channel 3 of an upstream mux at 0x70 it is `[[0x70, 3]]`.
# `upstream_select()` is the mux's equivalent of `I2CConnection.path_select()`,
# and is likewise called under the lock on every transfer.
#
# ### Same transport contract, so mixins still work
#
# Because a mux implements the same four transport methods as a device, the
# protocol mixins apply unchanged. A mux with a control register map can be given
# `RegisterI2CBase` and get shadow caching and verified writes for free.
#
# ### The mux API contract
#
# `channel_select`, `channel_deselect` and `control_read` are declared but not
# implemented — the encoding of a channel selection is part-specific. The manager
# calls these by name, so any concrete mux must provide them.
#
# ### Hardware reset
#
# A downstream device that hangs while holding SDA low can wedge the whole bus,
# and no amount of software talking can fix it, because you cannot get a start
# condition out. If a RESET pin is wired, `reset()` pulses it, which drops every
# channel and cuts the stuck device off the bus. This is the recovery path of
# last resort, and it only exists if the hardware provides it — hence the
# `False` return when no pin was configured.

# %%
class I2CMux(I2CNode):
    """A routing element: talks to the BARE bus at its own address, doing its
    own upstream selection when chained (it can't route through itself).
    Implements the same transport methods as I2CDevice, so protocol mixins
    work here too. Defines the mux API contract the manager relies on.

    Parameters
    ----------
    bus : machine.I2C
        The underlying bus object.
    address : int
        The mux's own 7-bit address.
    lock : lock-like, optional
        The per-bus mutex — the same object handed to every connection on this
        bus. Injecting the shared lock is essential here: a mux that locked
        independently could re-route the bus mid-transaction for someone else.
    path : list[list[int]], optional
        Hops required to reach THIS mux, for chained topologies. Empty (the
        default) means the mux sits directly on the bus.
    reset_pin : int | str, optional
        Pin identifier for the mux's active-low RESET line, if wired.
    **kwargs
        Passed up to `I2CNode`.

    Class attributes
    ----------------
    CHANNELS : int
        Number of downstream channels. Subclasses declare their real count; the
        manager uses it to bounds-check a topology map.
    """

    CHANNELS = 0                      # subclass declares its channel count

    def __init__(self, bus, address, lock=None, path=None, reset_pin=None, **kwargs):
        super().__init__(**kwargs)
        self.bus       = bus
        self.addr      = address
        self.lock      = lock or default_lock()
        self.path      = path or []   # hops to reach THIS mux (chaining)
        self.reset_pin = None
        if reset_pin is not None:
            # Imported lazily so this module stays importable off-target (host
            # test runs, linting) where `machine` does not exist.
            from machine import Pin
            # Idle high: RESET is active-low, so start de-asserted.
            self.reset_pin = Pin(reset_pin, Pin.OUT, value=1)

    def upstream_select(self):
        """Select the hops needed to make THIS mux reachable.

        The mux equivalent of `I2CConnection.path_select()`. A no-op for a mux
        wired directly to the bus. Called with the lock already held, for the
        same atomicity reason.
        """
        for mux_addr, ch in self.path:
            self.bus.writeto(mux_addr, bytes([1 << ch]))

    # ------------------------------------------------------------------
    # Transport: bare bus, under lock, upstream-selected.
    #
    # Same contract as I2CDevice, different implementation — these address the
    # bus directly rather than delegating to a connection.
    # ------------------------------------------------------------------

    def read_mem(self, reg, nbytes):
        """Read `nbytes` from the mux's own register `reg`."""
        with self.lock:
            self.upstream_select()
            return self.bus.readfrom_mem(self.addr, reg, nbytes)

    def write_mem(self, reg, data):
        """Write the bytes `data` to the mux's own register `reg`."""
        with self.lock:
            self.upstream_select()
            self.bus.writeto_mem(self.addr, reg, data)

    def read_raw(self, nbytes):
        """Read `nbytes` of raw data from the mux — e.g. its control byte."""
        with self.lock:
            self.upstream_select()
            return self.bus.readfrom(self.addr, nbytes)

    def write_raw(self, data):
        """Write raw bytes to the mux — e.g. a channel-select mask."""
        with self.lock:
            self.upstream_select()
            self.bus.writeto(self.addr, data)

    # ------------------------------------------------------------------
    # Mux API contract: subclasses implement.
    #
    # The channel encoding is part-specific (one-hot mask, binary index,
    # enable-bit-plus-index), so it cannot be implemented here. The manager
    # calls these methods by name on any mux it is given.
    # ------------------------------------------------------------------

    def channel_select(self, channel):
        """Route the mux to `channel`, deselecting any others."""
        raise NotImplementedError

    def channel_deselect(self):
        """Disconnect all channels, isolating everything downstream."""
        raise NotImplementedError

    def control_read(self):
        """Read back the mux's current channel selection state."""
        raise NotImplementedError

    def reset(self):
        """Hardware reset via RESET pin; returns False if no pin was wired.

        The recovery of last resort. A downstream device that hangs holding SDA
        low makes the segment unusable and cannot be talked out of it — no start
        condition can be issued. Pulsing RESET drops every channel, cutting the
        offending device off the bus and restoring the upstream segment.

        The 10 us dwells are a comfortable margin over the pulse-width and
        recovery-time minimums typical of TCA9548A-class parts.
        """
        if self.reset_pin is None:
            return False
        import time
        self.reset_pin.value(0)   # assert (active low)
        time.sleep_us(10)
        self.reset_pin.value(1)   # release
        time.sleep_us(10)         # let the part come back up before any transfer
        return True


# %% [markdown]
# # 4. Helpers and Protocol Mixins
#
# ## Two kinds of thing in this section
#
# Sections 4.1 and 4.2 are **helpers**: reusable computation with no bus traffic
# and no transport dependency. Sections 4.3 and 4.4 are the **protocol mixins**
# proper, which do talk to hardware through the four-method contract.
#
# The split matters because the two kinds compose differently. A helper can be
# mixed into anything, or called unbound as a classmethod against captured data.
# A protocol mixin only makes sense on something that implements the transport
# contract.
#
# ## The device split
#
# Devices divide fairly cleanly into two families by how you talk to them:
#
# * **Register-mapped** — the device exposes numbered internal registers. You
#   name a register and read or write its contents. Most sensors, ADCs, IO
#   expanders and PMICs work this way.
#
# * **Command-driven** — the device has no register map. You send a fixed opcode
#   and it streams back a result, typically with a checksum. Sensirion parts are
#   the canonical example.
#
# These need genuinely different machinery, so they are separate mixins. A driver
# picks the one matching its part, and adds `CRCMixin` if the part checksums but
# is not command-driven:
#
# ```python
# class MyRegisterSensor(RegisterI2CBase, I2CDevice):
#     ALLOWED_ADDRS = (0x48, 0x49)
#     BIG_ENDIAN    = True
#     ...
#
# class MyCommandSensor(CommandI2CBase, I2CDevice):
#     ALLOWED_ADDRS = (0x62,)
#     CMD_BYTES     = 2
#     CRC_POLY      = 0x31
#     ...
#
# class MyCheckedRegisterSensor(RegisterI2CBase, CRCMixin, I2CDevice):
#     CRC_POLY = 0x07
#     ...
# ```
#
# ## Why mixins and not base classes
#
# The protocol mixins call only the four transport methods from the `I2CNode`
# contract. They never touch `self.conn`, never touch `self.bus`, and never take
# a lock. That restraint is what lets the same mixin be composed with
# `I2CDevice` *or* with `I2CMux` — a register-mapped mux is a perfectly ordinary
# thing to want, and it gets the full caching and verification machinery at no
# extra cost.
#
# No mixin in this section defines `__init__`, which keeps the MRO trivial for
# the driver author: cooperative-`super()` bookkeeping is exactly the sort of
# thing that should not be required to write a temperature sensor driver.
# `RegisterI2CBase` achieves this with a lazily-created cache dict.
#
# ## Naming convention
#
# Every accessor is `noun_verb` — `reg_read`, `field_get`, `coded_set`,
# `crc_verify` — so related operations sort together and the subject of the call
# reads first. Nothing is name-mangled or underscore-private; a driver author
# poking at internals during bring-up should not have to fight the naming to do
# it.
#
# ## Class attribute vs. per-call argument
#
# The rule of thumb throughout: if the datasheet states it once for the whole
# chip, it is a class attribute; if it varies register by register, it is a
# per-call argument.
#
# | Fact | Where it lives | Why |
# |---|---|---|
# | Endianness | `BIG_ENDIAN`, class attr | one chip-wide convention |
# | Default register width | `REG_BYTES`, class attr | most chips are uniform |
# | Verify mask | `VERIFY_MASK`, class attr | the config register's quirks |
# | Opcode width | `CMD_BYTES`, class attr | fixed by the manufacturer |
# | CRC parameters | `CRC_*`, class attrs | fixed by the manufacturer |
# | Register width | `nbytes`, per-call | 8/16/24-bit registers coexist on one chip |
# | Tolerance | `tolerance`, per-call | only quantising registers need it |
# | Checksum scope | the caller's argument | varies by part, not inferable |

# %% [markdown]
# ## 4.1 `BitTools` — stateless bit and byte helpers
#
# ### Why these are collected rather than scattered
#
# Sign extension, mask construction and byte assembly get reimplemented in every
# driver, subtly differently each time. Sign extension in particular has a
# standard bug — extending before stripping padding bits, or after, depending on
# the part — and having exactly one implementation means fixing it once.
#
# ### Why staticmethods on a class rather than module functions
#
# Two reasons, both about the call site. `self.sign_extend(word, 12)` reads as
# something the driver *does*, consistent with `self.reg_read` beside it. And a
# part with a genuinely odd format — 20 bits packed into 24, say — can override
# the method for itself rather than working around a module function.
#
# ### Why this is separate from `CRCMixin`
#
# Nothing here is configurable. There are no class attributes, so there is no
# contract to satisfy and nothing to collide with in a multiple-inheritance
# chain. That is precisely what `CRCMixin` is not, and merging the two would
# force every driver wanting `sign_extend` to inherit five checksum parameters
# it never sets.
#
# ### `field_mask` moved here
#
# It previously lived on `RegisterI2CBase`, which was the wrong home: it is pure
# bit arithmetic with no register semantics, and `CRCMixin` has no reason to
# depend on the register mixin to reach it. `RegisterI2CBase` now inherits it,
# so every existing call site — including `RegisterI2CBase.field_mask(...)`
# unbound — continues to work unchanged.

# %%
class BitTools:
    """Stateless bit and byte manipulation helpers.

    Pure staticmethods with no class attributes and no contract to satisfy —
    safe to mix into anything, or to call unbound as `BitTools.sign_extend(...)`.

    Kept separate from `CRCMixin` precisely because nothing here is
    configurable: a subclass inherits behaviour, not obligations. Mixing this in
    adds no class attributes to the driver's namespace and cannot collide with
    another mixin's parameters.

    Mixed in anywhere it is useful:

        class MySensor(RegisterI2CBase, I2CDevice):   # already gets BitTools
            ...

    `RegisterI2CBase` and `CRCMixin` both inherit from this, so a driver using
    either gets these methods without naming `BitTools` explicitly.
    """

    @staticmethod
    def sign_extend(value, bits):
        """Interpret `value` as a `bits`-wide two's-complement number.

        The near-universal decode for sensor data. `value` is masked to width
        first, so a caller that passes a wider word — a raw register with
        padding already stripped, say — gets the same answer as one that
        pre-masked.

        Note the ordering trap this exists to centralise: on a left-justified
        part, the padding must be shifted out *before* calling this. Sign
        extending an unshifted word treats the padding zeros as magnitude.
        """
        value &= (1 << bits) - 1
        return value - (1 << bits) if value & (1 << (bits - 1)) else value

    @staticmethod
    def field_mask(field):
        """Return the aligned bitmask for a `(shift, width)` field tuple.

        A staticmethod so drivers can build composite masks — a VERIFY_MASK
        assembled from named fields, say — at class-definition time, without an
        instance.
        """
        shift, width = field
        return ((1 << width) - 1) << shift

    @staticmethod
    def field_extract(value, field):
        """Pull a `(shift, width)` field out of an already-read word.

        For decoding a word the caller already holds — a packet byte, a cached
        register — without the bus traffic `field_get` would incur.
        """
        shift, width = field
        return (value >> shift) & ((1 << width) - 1)

    @staticmethod
    def field_insert(value, field, new):
        """Return `value` with a `(shift, width)` field replaced by `new`.

        The read-modify-write arithmetic on its own, for callers assembling a
        word to write in one transfer rather than reading it back per field.
        `new` is masked to the field width, so an oversized value cannot spill
        into its neighbours.
        """
        shift, width = field
        mask = ((1 << width) - 1) << shift
        return (value & ~mask) | ((new << shift) & mask)

    @staticmethod
    def bit_reverse(value, bits=8):
        """Reverse the bit order of a `bits`-wide value.

        Needed by reflected CRC variants, and occasionally by parts that clock
        data out LSB-first into an MSB-first register.
        """
        out = 0
        for _ in range(bits):
            out = (out << 1) | (value & 1)
            value >>= 1
        return out

    @staticmethod
    def bytes_to_int(data, big_endian=True):
        """Assemble a byte sequence into an int, MSB-first unless told otherwise."""
        val = 0
        for b in (data if big_endian else reversed(data)):
            val = (val << 8) | b
        return val

    @staticmethod
    def int_to_bytes(value, nbytes, big_endian=True):
        """Split an int into `nbytes` bytes, masking to width first.

        Masking rather than raising means an oversized argument is truncated
        instead of producing a wrong-length transfer — the same policy
        `reg_write` has always had, now in one place.
        """
        value &= (1 << (8 * nbytes)) - 1
        seq = [(value >> (8 * i)) & 0xFF for i in range(nbytes)]   # little-endian
        if big_endian:
            seq.reverse()
        return bytes(seq)


# %% [markdown]
# ## 4.2 `CRCMixin` — checksums as a class-attribute contract
#
# ### Why a mixin rather than a function
#
# A CRC needs five or six parameters, and every one of them is a datasheet fact
# about the part. Putting them on the class makes them a **contract** in exactly
# the way `ALLOWED_ADDRS` and the baud triplet are: a driver that mixes this in
# must state its checksum parameters, and an auto-setup system can read them off
# the class without a live instance.
#
# A module-level `crc8(data, poly, init, ...)` would push those parameters to
# every call site, where they would be restated, drift, and eventually disagree.
#
# ### Reading the polynomial off a datasheet
#
# Datasheets print the polynomial as an equation or as a hex constant. They are
# the same thing: coefficients on x^(n-1) down to x^0, with the x^n term
# implicit and never stored, because it always shifts out.
#
# | Datasheet says | Bits set | `CRC_POLY` | `CRC_WIDTH` |
# |---|---|---|---|
# | x⁸+x²+x+1 | 2, 1, 0 | `0x07` | 8 |
# | x⁸+x⁵+x⁴+1 | 5, 4, 0 | `0x31` | 8 |
# | x¹⁶+x¹⁵+x²+1 | 15, 2, 0 | `0x8005` | 16 |
#
# ### Arbitrary width
#
# `CRC_WIDTH` parameterises the whole algorithm. The bitwise loop is identical
# regardless of width — only the top-bit test, the accumulator mask, and how the
# input byte is aligned into the accumulator change. Supporting 16-bit now costs
# a handful of shifts and saves retrofitting every driver later.
#
# ### `CRC_POLY = None` is deliberate
#
# There is no benign default. A wrong polynomial produces checksums that verify
# perfectly against themselves and against nothing else — the exact
# plausible-garbage failure this layer exists to prevent. So the default is
# `None` and every entry point raises `NotImplementedError` naming the missing
# attribute.
#
# ### Table acceleration
#
# `CRC_TABLE_ENABLE` swaps the bitwise loop for a 256-entry lookup, built lazily
# on first use and cached on the class that asked for it. Roughly eight times
# fewer operations per byte, at 256 entries of RAM per distinct parameter set.
# Worth it on a part being polled continuously; not worth it on one read once a
# second, or on a target where RAM is the binding constraint. Off by default.
#
# The cache is stored in `cls.__dict__` rather than by plain attribute
# assignment, so a subclass that narrows the parameters builds its own table
# instead of silently inheriting one computed for its parent's polynomial.
#
# ### Scope is the caller's problem
#
# What bytes a checksum covers cannot be inferred here, and differs sharply
# between parts:
#
# * **Sensirion (SHTC3, SHT4x, SPS30)** — one checksum per 16-bit word, so an
#   n-word read is 3n bytes and each word can be checked independently.
# * **TI TMAG5273** — one checksum over the *entire* packet, including the
#   command byte the host sent, appended once at the end.
#
# This class computes a checksum over whatever bytes it is handed. Deciding what
# to hand it belongs in the driver, which is the only layer that knows the wire
# format.

# %%
class CRCMixin(BitTools):
    """Cyclic redundancy check of arbitrary width, parameterized by class attributes.

    Exists as a mixin rather than a function so that the class attributes below
    form a *contract*: a driver that mixes this in must state its checksum
    parameters, the same way an `I2CNode` subclass must state its allowed
    addresses and baud limits. The parameters are datasheet facts, so they
    belong on the class where an auto-setup system can read them without a live
    instance.

    Implements the full parameterized CRC model — width, polynomial, initial
    value, input and output reflection, final XOR. Nearly every published CRC is
    one point in that space, so a part's datasheet row translates directly into
    these attributes with no algorithm changes.

    Inherits `BitTools` for `bit_reverse`, which the reflected variants need.

    ## Declaring the parameters

    Datasheets print the polynomial either as an equation or as a hex constant.
    They are the same thing: coefficients on x^(n-1)...x^0, with the x^n term
    implicit and never stored. So x^8+x^2+x+1 sets bits 2, 1, 0 -> 0x07, and
    x^8+x^5+x^4+1 sets bits 5, 4, 0 -> 0x31. `CRC_WIDTH` is the n that was
    dropped.

    `CRC_POLY` defaults to None and every entry point raises if it is still
    None. A benign default would let a driver silently compute checksums that
    verify against themselves and against nothing else.

    ## Table acceleration

    `CRC_TABLE_ENABLE` swaps the bitwise loop for a 256-entry lookup, building
    the table lazily on first use and caching it on the class that requested it.
    Roughly 8x fewer operations per byte, at 256 entries of RAM per distinct
    parameter set.

    ## Scope is the caller's problem

    What bytes a checksum covers varies by part and cannot be inferred here.
    Sensirion parts checksum each 16-bit word independently; the TMAG5273
    checksums an entire packet including the command byte the host sent. This
    class computes a checksum over whatever it is given.

    Class attributes
    ----------------
    CRC_WIDTH : int
        Checksum width in bits. Any width is supported; 8 and 16 are usual.
    CRC_POLY : int
        Generator polynomial, x^n term omitted. No default — must be set.
    CRC_INIT : int
        Starting value of the accumulator.
    CRC_REFLECT_IN : bool
        Reverse the bits of each input byte before processing.
    CRC_REFLECT_OUT : bool
        Reverse the bits of the final accumulator.
    CRC_XOR_OUT : int
        XORed into the result last, after any output reflection.
    CRC_TABLE_ENABLE : bool
        Use a precomputed lookup table instead of the bitwise loop.
    """

    CRC_WIDTH        = 8
    CRC_POLY         = None       # no default: an unset polynomial must fail loudly
    CRC_INIT         = 0xFF
    CRC_REFLECT_IN   = False
    CRC_REFLECT_OUT  = False
    CRC_XOR_OUT      = 0x00
    CRC_TABLE_ENABLE = False

    # ------------------------------------------------------------------
    # Derived parameters
    #
    # Classmethods rather than class attributes, because they depend on
    # CRC_WIDTH and would go stale if a subclass overrode the width after
    # these had been computed at class-definition time.
    # ------------------------------------------------------------------

    @classmethod
    def crc_mask(cls):
        """All-ones mask for the configured width."""
        return (1 << cls.CRC_WIDTH) - 1

    @classmethod
    def crc_topbit(cls):
        """The bit that shifts out of the accumulator and triggers the XOR."""
        return 1 << (cls.CRC_WIDTH - 1)

    @classmethod
    def crc_check_configured(cls):
        """Raise `NotImplementedError` if the polynomial contract is unsatisfied.

        Called at the head of every entry point. Naming the class in the message
        matters: in a multi-part project the failure is nearly always one driver
        that forgot the attribute, not the mixin.
        """
        if cls.CRC_POLY is None:
            raise NotImplementedError(
                "{}: CRC_POLY not set; declare the polynomial from the datasheet "
                "(x^8+x^2+x+1 -> 0x07, x^8+x^5+x^4+1 -> 0x31)".format(cls.__name__))

    # ------------------------------------------------------------------
    # Table
    # ------------------------------------------------------------------

    @classmethod
    def crc_table(cls):
        """Return the 256-entry lookup table, building it once on first use.

        Cached in `cls.__dict__` rather than by attribute assignment, so a
        subclass that narrows the parameters gets its own table instead of
        inheriting one computed for its parent's polynomial. `cls.__dict__.get`
        looks only at this class, not at its bases, which is exactly the
        distinction needed.

        Building the table is the same eight shifts the bitwise loop performs,
        run once per possible input byte instead of once per actual byte.
        """
        cached = cls.__dict__.get("_crc_tbl")
        if cached is not None:
            return cached

        cls.crc_check_configured()
        mask, topbit, shift = cls.crc_mask(), cls.crc_topbit(), cls.CRC_WIDTH - 8
        table = []
        for byte in range(256):
            # Align the byte with the top of the accumulator, then run the same
            # eight shifts the bitwise loop would.
            crc = (byte << shift) if shift > 0 else (byte >> -shift)
            for _ in range(8):
                crc = ((crc << 1) ^ cls.CRC_POLY) & mask if crc & topbit else (crc << 1) & mask
            table.append(crc)

        cls._crc_tbl = table
        return table

    @classmethod
    def crc_table_clear(cls):
        """Drop this class's cached table, if it has built one.

        Needed after changing any CRC parameter at runtime — which is a thing
        done during bring-up, when the datasheet's parameters and the part's
        actual behaviour have not yet been reconciled.
        """
        if "_crc_tbl" in cls.__dict__:
            del cls._crc_tbl

    # ------------------------------------------------------------------
    # Computation
    # ------------------------------------------------------------------

    @classmethod
    def crc(cls, data, init=None):
        """Checksum over `data`, per the class's parameters.

        Parameters
        ----------
        data : bytes
            Bytes to checksum.
        init : int, optional
            Override the starting accumulator value. For resuming a running
            checksum across several transfers — pass the previous result back
            in. Note that reflection and final XOR apply at the end of *every*
            call, so a resumed checksum must either undo them or be accumulated
            with those parameters left at their identity values.

        Returns
        -------
        int
            The checksum, `CRC_WIDTH` bits wide.

        Raises
        ------
        NotImplementedError
            If `CRC_POLY` has not been set on the subclass.
        """
        cls.crc_check_configured()

        mask, topbit = cls.crc_mask(), cls.crc_topbit()
        shift = cls.CRC_WIDTH - 8
        crc = cls.CRC_INIT if init is None else init
        table = cls.crc_table() if cls.CRC_TABLE_ENABLE else None

        for byte in data:
            if cls.CRC_REFLECT_IN:
                byte = cls.bit_reverse(byte, 8)

            if table is not None:
                # Index by the top byte of the accumulator XOR the input byte,
                # then shift the rest of the accumulator up under it.
                if shift > 0:
                    idx = ((crc >> shift) ^ byte) & 0xFF
                    crc = table[idx] ^ ((crc << 8) & mask)
                else:
                    idx = (crc ^ byte) & 0xFF
                    crc = table[idx]
            else:
                crc ^= (byte << shift) if shift > 0 else (byte >> -shift)
                for _ in range(8):
                    crc = ((crc << 1) ^ cls.CRC_POLY) & mask if crc & topbit else (crc << 1) & mask

        if cls.CRC_REFLECT_OUT:
            crc = cls.bit_reverse(crc, cls.CRC_WIDTH)
        return (crc ^ cls.CRC_XOR_OUT) & mask

    @classmethod
    def crc_ok(cls, data, expected, init=None):
        """True if the checksum over `data` matches `expected`.

        Convenience for the common verify-then-branch, kept alongside `crc` so a
        driver need not restate the comparison. Prefer `crc_verify` on a data
        path, where a caller forgetting to test the boolean is a live risk.
        """
        return cls.crc(data, init=init) == expected

    @classmethod
    def crc_verify(cls, data, expected, context="", init=None):
        """Raise `ValueError` unless the checksum over `data` matches `expected`.

        `context` is prepended to the message — a word index, a register name,
        whatever locates the failure for the caller.

        Raising rather than returning a flag is deliberate: a corrupted sensor
        reading is indistinguishable from a real one, so a caller that forgets
        to check a boolean gets plausible garbage rather than a loud failure.

        Returns
        -------
        int
            The computed checksum, on success.
        """
        actual = cls.crc(data, init=init)
        if actual != expected:
            raise ValueError("{}checksum fail: computed {:#04x}, received {:#04x}".format(
                context + ": " if context else "", actual, expected))
        return actual


# %% [markdown]
# ## 4.3 `RegisterI2CBase` — register-mapped devices
#
# Five things on top of raw transport.
#
# ### Shadow cache
#
# `reg_cache` is one dict per device instance, keyed by register number, holding
# the last known value of each register touched. Reads that do not need fresh
# data (a config register you wrote yourself and nothing else can change) are
# served from RAM at zero bus cost. `refresh=True` forces a real read.
#
# **Cache correctness is the driver's responsibility.** This layer cannot know
# that reading a status register clears it, or that a measurement register
# changes four times a second. Cache what is genuinely stable; use
# `refresh=True` for everything else. Note the cache stores no width, so mixing
# `nbytes` on the same register number across calls will return a stale-width
# value — a non-issue on uniform-width parts, a real trap otherwise.
#
# ### Raw vs. managed access
#
# Two pairs exist at the register level, and the distinction matters:
#
# * `reg_read` / `reg_write` — **raw**. No cache, no verification. For
#   write-only registers, registers whose read has side effects, and handshake
#   bits that read back inverted from what was written.
# * `reg_get` / `reg_set` — **managed**. `reg_get` serves from the shadow cache;
#   `reg_set` verifies the write landed and caches only the confirmed value.
#
# The managed pair is the default. Reach for the raw pair deliberately.
#
# ### Confirmed writes and the verify mask
#
# `reg_set` validates, writes, reads back **from the device** (not the cache),
# compares, and raises on mismatch. A write that silently did not take is a real
# failure mode on a marginal bus — a NAK the driver ignored, a brownout, an
# address collision — and it is far better to raise at the write than to spend
# an afternoon debugging a sensor that is quietly still in its default config.
#
# But an exact read-back comparison produces *false* failures on any register
# containing read-only bits, status bits, or handshake bits that read back
# inverted. Real configuration registers almost always contain at least one. The
# TMP102's config register contains three kinds at once: R1/R0 are read-only and
# always read `11`, AL is comparator-driven, and OS reads back inverted from
# what was written.
#
# `VERIFY_MASK` restricts the comparison to bits genuinely expected to read back
# as written. This is the strongest check the hardware permits, rather than
# either an exact match that fails spuriously or no check at all.
#
# ### Exact vs. tolerant comparison
#
# Masked comparison is bitwise and exact — right for bit-fields and flags.
# `tolerance` instead compares numerically, for devices that quantise a
# requested value to their own step size (a DAC, a current limit) where landing
# *near* the request is specified behaviour rather than a fault. The two are
# mutually exclusive: a tolerance implies the register holds a magnitude, in
# which case masking individual bits is meaningless.
#
# ### Bitfields and coded fields
#
# `field_get` / `field_set` take a `(shift, width)` tuple, declared as a
# class-level constant on the driver. This reads far better at the call site
# than open-coded shifts and keeps the bit layout in exactly one place.
#
# `coded_get` / `coded_set` go one step further, for fields whose bit pattern is
# an index into a table of physical values — conversion rates, gain steps, fault
# queue depths. The driver declares a `{code: value}` dict and the caller works
# in Hz or counts rather than in bit patterns.
#
# ### Inherited from `BitTools`
#
# This class now inherits `BitTools`, which supplies `field_mask`,
# `field_extract`, `field_insert`, `sign_extend`, `bytes_to_int` and
# `int_to_bytes`. `reg_read` and `reg_write` are written in terms of the last
# two rather than repeating the assembly arithmetic, and `field_mask` — which
# used to be defined here — is now inherited, so every existing call site
# including the unbound `RegisterI2CBase.field_mask(...)` continues to work.

# %%
class RegisterI2CBase(BitTools):
    """Protocol layer for register-mapped devices — those where you address an
    internal register by number and read or write its contents.

    Provides, on top of the four transport methods:

      * a shadow cache of last-known register values, so reads that don't need
        fresh data cost no bus traffic
      * width- and endian-aware assembly, so an 8-, 16-, or 24-bit register is
        one call
      * confirmed writes, with a per-register mask for bits that cannot read
        back as written
      * bitfield read-modify-write, so a driver names a field rather than
        recomputing masks
      * coded-field helpers, for fields whose bit pattern maps to a physical
        value

    Inherits `BitTools`, so `sign_extend`, `field_mask`, `field_extract`,
    `field_insert`, `bytes_to_int` and `int_to_bytes` are available to any
    driver using this mixin without naming `BitTools` explicitly.

    ## Naming

    Every accessor is `noun_verb`, so related operations sort together and the
    subject of the call reads first. Two pairs exist at the register level and
    the distinction matters:

      * `reg_read` / `reg_write` — raw. No cache, no verification. Use these
        when the read-back would be wrong or harmful: write-only registers,
        registers whose read has side effects, handshake bits that read back
        inverted.
      * `reg_get` / `reg_set` — managed. `reg_get` serves from the shadow cache;
        `reg_set` verifies the write landed and caches only the confirmed
        value. These are the defaults; reach for the raw pair deliberately.

    Field-level accessors (`field_get`, `field_set`, `coded_get`, `coded_set`)
    are all managed, and build on the pair above.

    ## Confirmed writes and the verify mask

    A plain read-back comparison catches writes that silently didn't take — a
    real failure mode on a marginal bus. But it produces false failures on any
    register containing read-only bits, status bits, or handshake bits that
    read back inverted. Real configuration registers almost always contain at
    least one.

    `VERIFY_MASK` (class attribute, default all-ones) and the per-call
    `verify_mask` argument restrict the comparison to bits genuinely expected
    to read back as written. This is the strongest check the hardware permits,
    rather than either an exact match that fails spuriously or no check at all.

    ## Exact vs. tolerant comparison

    Masked comparison is bitwise and exact — right for bit-fields and flags.
    `tolerance` instead compares numerically, for devices that quantize a
    requested value to their own step size, where landing near the request is
    specified behaviour. The two are mutually exclusive: a tolerance implies
    the register holds a magnitude, in which case masking individual bits is
    meaningless.

    ## Bitfields

    `field_get` / `field_set` take a `(shift, width)` pair. Drivers declare
    fields as class-level tuples, which reads far better at the call site than
    open-coded shifts and keeps the layout in one place.

    ## Class attributes

    Endianness, default register width, and the verify mask are chip facts, so
    they're class attributes. Per-register width is a per-call argument, since
    8-, 16- and 24-bit registers routinely coexist on one part.

    Mixed in ahead of `I2CDevice` or `I2CMux`:

        class MySensor(RegisterI2CBase, I2CDevice):
            ...

    Add `CRCMixin` for a register-mapped part that also checksums:

        class MyCheckedSensor(RegisterI2CBase, CRCMixin, I2CDevice):
            CRC_POLY = 0x07

    Defines no `__init__`, so it imposes no constructor bookkeeping on the
    subclass. Calls only the transport methods, so this works on routed devices
    and bare-bus muxes alike.
    """

    BIG_ENDIAN  = True            # chip fact; subclass overrides if little-endian
    REG_BYTES   = 2               # default register width, overridable per call
    VERIFY_MASK = None            # None means "all bits verify"; subclass narrows

    # ------------------------------------------------------------------
    # Cache
    # ------------------------------------------------------------------

    @property
    def reg_cache(self):
        """Dict of last-known register values, created lazily on first access.

        Lazy so this class needs no `__init__`, keeping multiple inheritance
        simple. One dict per instance, keyed by register number. Only values
        confirmed by a read-back are stored, so the cache never records
        something the device rejected.

        Trustworthy only insofar as the driver is disciplined: this layer
        cannot know which registers are volatile or self-clearing. Cache
        configuration and calibration; use `refresh=True` for everything else.
        """
        if not hasattr(self, "_rc"):
            self._rc = {}
        return self._rc

    def cache_clear(self, reg=None):
        """Drop `reg` from the cache, or the whole cache if `reg` is None.

        For use after anything that changes device state behind the driver's
        back — a general-call reset, a power cycle, a hardware reset line.
        """
        if reg is None:
            self.reg_cache.clear()
        else:
            self.reg_cache.pop(reg, None)

    # ------------------------------------------------------------------
    # Raw register access — no cache, no verification
    # ------------------------------------------------------------------

    def reg_read(self, reg, nbytes=None):
        """Read a register and assemble the bytes into an int. Bypasses the cache.

        Assembles in datasheet order — MSB-first when `BIG_ENDIAN`, else
        LSB-first — via `BitTools.bytes_to_int`. `nbytes` defaults to
        `REG_BYTES`.

        Always hits the bus. Use `reg_get` when a cached value will do.
        """
        nbytes = self.REG_BYTES if nbytes is None else nbytes
        return self.bytes_to_int(self.read_mem(reg, nbytes), self.BIG_ENDIAN)

    def reg_write(self, reg, value, nbytes=None):
        """Split an int into bytes and write it to a register. No verify, no cache.

        `value` is masked to `nbytes` by `BitTools.int_to_bytes`, so an
        oversized argument is truncated rather than producing a wrong-length
        transfer.

        Prefer `reg_set` unless the read-back is genuinely undesirable — a
        write-only register, or one whose read has side effects.
        """
        nbytes = self.REG_BYTES if nbytes is None else nbytes
        self.write_mem(reg, self.int_to_bytes(value, nbytes, self.BIG_ENDIAN))

    # ------------------------------------------------------------------
    # Managed register access — cached reads, verified writes
    # ------------------------------------------------------------------

    def reg_get(self, reg, nbytes=None, refresh=False):
        """Return a register's value, from the shadow cache where possible.

        Pass `refresh=True` to force a fresh read — necessary for measurement,
        status and self-clearing registers, which must never be cached.
        """
        if refresh or reg not in self.reg_cache:
            self.reg_cache[reg] = self.reg_read(reg, nbytes)
        return self.reg_cache[reg]

    def reg_set(self, reg, value, nbytes=None, tolerance=0,
                verify_mask=None, validator=None):
        """Write a register and confirm it took.

        Sequence: validate the requested value, write it, read the register
        back from the device, compare, cache the confirmed value.

        Parameters
        ----------
        reg : int
            Register number.
        value : int
            Value requested.
        nbytes : int, optional
            Register width in bytes. Defaults to `REG_BYTES`.
        tolerance : int, optional
            Maximum acceptable numeric difference between request and
            read-back. Default 0 (exact). Nonzero suits registers the device
            quantizes to its own step size. Mutually exclusive with
            `verify_mask`.
        verify_mask : int, optional
            Restrict comparison to these bits — for registers containing
            read-only, status, or inverted-handshake bits that cannot read back
            as written. Falls back to `VERIFY_MASK`, then to all bits.
        validator : callable, optional
            Called as `validator(value)` before any bus traffic. Should raise on
            an unacceptable value. Running it first means a bad request costs no
            transfers and cannot leave the device half-configured.

        Returns
        -------
        int
            The value actually read back — which may differ from the request
            within tolerance, or in masked-off bits. Callers that care about
            what landed should use this rather than assuming their request took
            verbatim.

        Raises
        ------
        ValueError
            If the read-back fails the comparison. The validator may raise
            anything it likes.
        """
        if validator is not None:
            validator(value)                      # reject bad input before bus traffic

        nbytes = self.REG_BYTES if nbytes is None else nbytes
        mask = verify_mask if verify_mask is not None else self.VERIFY_MASK

        self.reg_write(reg, value, nbytes)
        actual = self.reg_read(reg, nbytes)       # read the device, not the cache

        if tolerance:
            # Numeric comparison: the register holds a magnitude the device may
            # have quantized. Masking individual bits would be meaningless here.
            ok = abs(actual - value) <= tolerance
            detail = "tol {}".format(tolerance)
        elif mask is not None:
            ok = (actual & mask) == (value & mask)
            detail = "mask {:#x}".format(mask)
        else:
            ok = actual == value
            detail = "exact"

        if not ok:
            raise ValueError(
                "reg {:#04x}: wrote {:#x}, device holds {:#x} ({})".format(
                    reg, value, actual, detail))

        self.reg_cache[reg] = actual              # only cache confirmed values
        return actual

    # ------------------------------------------------------------------
    # Bitfields
    #
    # `field` is a (shift, width) tuple, declared as a class attribute on the
    # driver. Read-modify-write throughout, since a register normally holds
    # several unrelated settings and a bare write would clobber them.
    #
    # `field_mask`, `field_extract` and `field_insert` are inherited from
    # BitTools — the arithmetic has no register semantics, so it lives there.
    # ------------------------------------------------------------------

    def field_get(self, reg, field, nbytes=None, refresh=True):
        """Extract a `(shift, width)` field from a register.

        Defaults to `refresh=True`: fields commonly sit alongside live status
        bits, and a stale read is the more dangerous failure.
        """
        return self.field_extract(self.reg_get(reg, nbytes, refresh=refresh), field)

    def field_set(self, reg, field, value, nbytes=None, **kw):
        """Read-modify-write a `(shift, width)` field, then verify via `reg_set`.

        Extra keyword arguments pass through to `reg_set` — notably
        `verify_mask`, needed whenever the register holds bits that don't read
        back as written.
        """
        current = self.reg_read(reg, nbytes)      # fresh: never modify a stale word
        return self.reg_set(reg, self.field_insert(current, field, value), nbytes, **kw)

    # ------------------------------------------------------------------
    # Coded fields
    #
    # For fields where the bit pattern is an index into a table of physical
    # values — conversion rates, gain steps, fault-queue depths. The table is a
    # {code: value} dict class attribute on the driver.
    # ------------------------------------------------------------------

    def coded_get(self, reg, field, table, nbytes=None, refresh=True):
        """Read a coded field and map its bit pattern through `table` to a value."""
        return table[self.field_get(reg, field, nbytes, refresh=refresh)]

    def coded_set(self, reg, field, table, value, nbytes=None, **kw):
        """Map `value` back through `table` to a code and write it.

        Raises ValueError listing the accepted values if `value` isn't in the
        table — a far more useful error than writing a wrong code and having
        the device quietly do something else.
        """
        for code, v in table.items():
            if v == value:
                return self.field_set(reg, field, code, nbytes, **kw)
        raise ValueError("{} not in {}".format(value, sorted(table.values())))


# %% [markdown]
# ## 4.4 `CommandI2CBase` — command-driven devices
#
# ### The wire format
#
# No register map. You write an opcode — usually two bytes, MSB first — and then
# read a response consisting of 16-bit words, **each followed by a CRC byte**
# covering the two data bytes before it:
#
# ```
# [ MSB | LSB | CRC ] [ MSB | LSB | CRC ] ...
#    word 0              word 1
# ```
#
# Note the checksum is per-word, not per-message, so an n-word read is `3n`
# bytes on the wire. `words_read` handles the striding and the verification.
#
# ### Why the checksum matters here more than elsewhere
#
# A register-mapped device usually gives you some redundancy to sanity-check
# against — a known device-ID register, a fixed-value bit field. A command-driven
# sensor returns a bare number, and a corrupted transfer produces a number that
# looks entirely plausible. The CRC is the only thing standing between a bit flip
# and a silently wrong reading, so `words_read` verifies by default and raises
# rather than returning the value.
#
# `check=False` exists for diagnostics — reading a device whose CRC parameters
# you are still working out, or capturing a known-bad transfer for inspection.
# It is not for production use.
#
# ### Checksum parameters now come from `CRCMixin`
#
# This class inherits `CRCMixin` and sets `CRC_POLY = 0x31` / `CRC_INIT = 0xFF`
# as its defaults — the Sensirion convention, x⁸+x⁵+x⁴+1. Every parameter
# remains overridable per driver, and the full model (width, reflection, final
# XOR) is available for a manufacturer that differs.
#
# `crc8` is retained as an alias for `crc`, so existing driver code and captured
# test vectors keep working. New code should call `crc` or, better,
# `crc_verify`.

# %%
class CommandI2CBase(CRCMixin):
    """Protocol layer for command-driven devices — those with no register map,
    where you send a fixed opcode and read back a result.

    Data comes back as 16-bit words, each followed by a checksum byte covering
    the two bytes before it. This class sends opcodes and reads words with the
    checksum verified, so corrupted transfers raise instead of returning
    plausible-looking garbage.

    Inherits `CRCMixin` (and through it `BitTools`), defaulting to the Sensirion
    parameters — CRC-8, polynomial 0x31, init 0xFF, no reflection, no final XOR.
    A driver for a part from another manufacturer overrides whichever of those
    differ; the full parameterized model is available.

    Opcode width is a class attribute, since it varies by manufacturer. Calls
    only the transport methods, so this works on routed devices and bare-bus
    muxes alike.

    Mixed in ahead of `I2CDevice` or `I2CMux`:

        class MyGasSensor(CommandI2CBase, I2CDevice):
            ...

    Defines no `__init__`, so it imposes no constructor bookkeeping on the
    subclass.
    """

    CMD_BYTES = 2                 # opcode width in bytes

    # Sensirion convention: CRC-8, x^8+x^5+x^4+1, init 0xFF, no reflection.
    # Inherited CRC_WIDTH/REFLECT/XOR_OUT defaults are already correct for it.
    CRC_POLY  = 0x31
    CRC_INIT  = 0xFF

    def cmd_send(self, opcode):
        """Send a command opcode, split MSB-first into CMD_BYTES bytes.

        `opcode` is given as a single int exactly as the datasheet prints it
        (e.g. ``0x2C06``); the split into wire bytes happens here, via
        `BitTools.int_to_bytes`. Many parts need a settling delay between the
        command and the read — that timing is part-specific, so it belongs in
        the driver, not here.
        """
        self.write_raw(self.int_to_bytes(opcode, self.CMD_BYTES, True))

    @classmethod
    def crc8(cls, data):
        """Alias for `crc`, kept for compatibility with existing driver code.

        The checksum is no longer necessarily 8 bits wide — `CRC_WIDTH` is a
        parameter — so new code should call `crc` or `crc_verify` instead. This
        name survives because captured test vectors and older drivers use it.
        """
        return cls.crc(data)

    def words_read(self, n_words, check=True):
        """Read n 16-bit words, each followed by its checksum byte.

        The wire layout is ``[MSB, LSB, CRC]`` repeated, so the whole response
        is read in one `3 * n_words` transfer and then strided — one bus
        transaction rather than n, which also means the mux path is selected
        once.

        Parameters
        ----------
        n_words : int
            Number of 16-bit words expected.
        check : bool, optional
            Verify each word's checksum (default True). Set False only for
            diagnostics. A corrupted reading from this family of devices is
            indistinguishable from a real one, so leave verification on in
            production.

        Returns
        -------
        list[int]
            The decoded 16-bit words.

        Raises
        ------
        ValueError
            On the first checksum mismatch, naming the word index. Raising
            rather than returning is deliberate: a plausible-looking wrong
            number is worse than a loud failure.
        """
        raw = self.read_raw(n_words * 3)
        words = []
        for i in range(n_words):
            msb, lsb, crc = raw[i*3], raw[i*3+1], raw[i*3+2]
            if check:
                self.crc_verify(bytes([msb, lsb]), crc, "word {}".format(i))
            words.append((msb << 8) | lsb)
        return words


# %% [markdown]
# # 5. Bring-Up: Display Helpers and a Real Bus
#
# Everything above is framework. From here down the notebook talks to hardware.
#
# ## Display helpers
#
# Register debugging is mostly a matter of staring at bit patterns, and hex
# alone hides the thing you usually want to see. `show` prints a value three
# ways at once — hex, grouped binary, decimal — so a field boundary is visible
# without mental arithmetic.
#
# The width is passed explicitly rather than inferred. Inferring from magnitude
# gets it wrong whenever the high byte happens to be zero, and a 16-bit register
# reading `0x00A0` would print as one byte.
#
# ## A note on the Jupyter MicroPython kernel
#
# `%serialconnect` executes each cell on the board and pipes stdout back. A bare
# expression's repr is *not* stdout, so `dev.reg_read(0)` on its own line
# displays nothing. Everything below prints explicitly.

# %%
def bin_bytes(v, nbytes=2):
    """Format an int as space-separated 8-bit groups, MSB first."""
    return " ".join("{:08b}".format((v >> (8 * i)) & 0xFF)
                    for i in reversed(range(nbytes)))


def show(v, label="", nbytes=2):
    """Print a register value as hex, binary and decimal, zero-padded to nbytes.

    Raises ValueError if the value doesn't fit in nbytes, since a silently
    truncated binary column next to an untruncated hex column is worse than
    no output at all.
    """
    if v >= (1 << (8 * nbytes)) or v < 0:
        raise ValueError("{:#x} does not fit in {} byte(s)".format(v, nbytes))
    print("{:<14} {:#0{}x}  {}  {}".format(
        label, v, nbytes * 2 + 2, bin_bytes(v, nbytes), v))


def show_reg(dev, reg, label=""):
    """Read a register from `dev` and show it at the device's default width."""
    show(dev.reg_read(reg), label or "reg {:#04x}".format(reg), dev.REG_BYTES)


def check(label, passed, detail=""):
    """Print a PASS/FAIL line and return the boolean, for use in test cells."""
    print("  [{}] {:<34} {}".format("PASS" if passed else "FAIL", label, detail))
    return passed


# %% [markdown]
# ## 5.1 Bus construction
#
# Wired for an **Unexpected Maker FeatherS3** (ESP32-S3). The STEMMA QT / Qwiic
# connector and the breadboard `SDA`/`SCL` pads share GPIO 8 and GPIO 9.
#
# Two things worth knowing if the scan comes back empty:
#
# * **Pull-ups.** The QT connector has them on-board. Bare-wiring to the header
#   pads does not — the TMP102 datasheet calls for 5 kΩ.
# * **Peripheral choice.** The ESP32-S3's I2C routes through the GPIO matrix, so
#   either peripheral works on any pins. A wrong `BUS_ID` fails silently rather
#   than raising, so try the other one before suspecting wiring.

# %%
from machine import I2C, Pin

BUS_ID   = 1                 # ESP32-S3 has two hardware I2C peripherals: 0 and 1
SCL_PIN  = 9                 # FeatherS3 STEMMA QT / Qwiic SCL
SDA_PIN  = 8                 # FeatherS3 STEMMA QT / Qwiic SDA
TMP_ADDR = 0x48              # ADD0 -> GND

# One lock per bus — the manager's job later; done by hand here.
bus_lock = default_lock()

bus = I2C(BUS_ID, scl=Pin(SCL_PIN), sda=Pin(SDA_PIN), freq=100_000)

print("bus scan:", [hex(a) for a in bus.scan()])


# %% [markdown]
# # 6. `TMP102Proto` — a Test Vehicle for `RegisterI2CBase`
#
# ## Why a stripped-down class
#
# The real TMP102 driver in section 8 wraps every register access behind a named
# method — `temperature()`, `shutdown`, `high_limit_set()`. That is the right
# interface for using the part, and exactly the wrong one for *testing the mixin*,
# because every call goes through a layer of driver logic that could itself be
# the thing that's broken.
#
# `TMP102Proto` is the datasheet facts and nothing else: addresses, register
# numbers, field tuples, code tables, masks. No methods. It exists purely so the
# `RegisterI2CBase` verification cells in section 7 can call `reg_read`,
# `field_set`, `coded_get` and friends **directly**, against a real device, with
# no driver code in between.
#
# When section 7 passes, the mixin is known good, and any failure in section 9
# is unambiguously the TMP102 driver's fault. That separation is the whole point.
#
# ## The TMP102's pointer register
#
# The part has no conventional register file — it has an 8-bit *pointer
# register* whose two LSBs select one of four data registers (Table 6-7). But
# the wire sequence is identical to a register read: address byte, pointer byte,
# repeated start, two data bytes. That is exactly what `readfrom_mem` emits, so
# the pointer value passes straight through as `reg` and the mixin needs no
# modification.
#
# All four data registers are 16 bits, MSB first, so `BIG_ENDIAN = True` and
# `REG_BYTES = 2` cover the whole part.

# %%
class TMP102Proto(RegisterI2CBase, I2CDevice):
    """Datasheet facts for the TI TMP102, with no driver logic whatsoever.

    A deliberately method-free shell, used in section 7 to exercise
    `RegisterI2CBase` directly against real hardware. The real driver is
    `TMP102` in section 8.

    Everything here is a class attribute taken from TI SBOS397I. Because there
    are no methods, a failure in the section 7 cells can only be the mixin, the
    connection layer, or the wiring — never driver code.
    """

    # --- datasheet facts (I2CNode contract) ---
    ALLOWED_ADDRS = (0x48, 0x49, 0x4A, 0x4B)   # Table 6-4: ADD0 -> GND/V+/SDA/SCL
    BAUD_NOMINAL  = 100_000
    BAUD_MAX      = 400_000                    # fast mode; HS needs a master code we can't send
    BAUD_MIN      = 10_000                     # spec floor 1 kHz (SCL timeout); margin here

    BIG_ENDIAN    = True                       # register bytes are sent MSB first
    REG_BYTES     = 2                          # every register is 16-bit

    # --- pointer register values (Table 6-7) ---
    REG_TEMP   = 0x00                          # read-only
    REG_CONFIG = 0x01
    REG_TLOW   = 0x02
    REG_THIGH  = 0x03

    # --- config bitfields as (shift, width); byte 1 is the MSB (Tables 6-10/6-11) ---
    F_OS   = (15, 1)   # one-shot / conversion ready
    F_RES  = (13, 2)   # converter resolution, read-only, always 11
    F_FQ   = (11, 2)   # fault queue
    F_POL  = (10, 1)   # ALERT polarity
    F_TM   = (9, 1)    # thermostat mode: 0=comparator, 1=interrupt
    F_SD   = (8, 1)    # shutdown
    F_CR   = (6, 2)    # conversion rate
    F_AL   = (5, 1)    # alert status, read-only
    F_EM   = (4, 1)    # extended mode

    # OS reads back inverted from what's written, R1/R0 are read-only and always
    # read 11, AL is comparator-driven. Excluded from write verification.
    VERIFY_MASK = 0xFFFF & ~(0x8000 | 0x6000 | 0x0020)

    # Thresholds are magnitude registers with no read-only bits, so they verify
    # exactly. Passed per-call to override the config-oriented VERIFY_MASK,
    # which would otherwise skip real data bits in a temperature word.
    LIMIT_VERIFY_MASK = 0xFFFF

    CFG_POR   = 0x60A0                         # power-on reset value
    THIGH_POR = 0x5000                         # 80 C
    TLOW_POR  = 0x4B00                         # 75 C

    LSB_C = 0.0625                             # deg C per count, both modes

    CONV_RATES  = {0: 0.25, 1: 1.0, 2: 4.0, 3: 8.0}   # Table 6-5
    FAULT_QUEUE = {0: 1, 1: 2, 2: 4, 3: 6}            # Table 6-12


# %% [markdown]
# ## 6.1 Instantiate the test vehicle
#
# Note `addr_ok` and `baud_ok` are checked *before* the connection is built —
# they need no bus traffic, which is the point of putting datasheet facts on the
# class. A topology map can be validated entirely offline.

# %%
assert TMP102Proto.addr_ok(TMP_ADDR), \
    "{:#04x} not a valid TMP102 address; allowed: {}".format(
        TMP_ADDR, [hex(a) for a in TMP102Proto.ALLOWED_ADDRS])
assert TMP102Proto.baud_ok(100_000), "100 kHz outside the part's baud limits"
assert TMP_ADDR in bus.scan(), "no device ACKing at {:#04x}".format(TMP_ADDR)

proto_conn = I2CConnection(bus, TMP_ADDR, path=[], lock=bus_lock)
proto = TMP102Proto(proto_conn)

print("TMP102Proto bound at {:#04x}".format(TMP_ADDR))
show_reg(proto, proto.REG_CONFIG, "config")
show_reg(proto, proto.REG_TEMP, "temp")


# %% [markdown]
# # 7. Verifying `RegisterI2CBase`
#
# Every feature of the mixin, exercised directly against the hardware through
# `TMP102Proto`. No driver code is involved, so a failure here is the mixin, the
# connection layer, or the wiring.
#
# The cells are ordered by dependency: transport first, then caching, then
# writes, then the field helpers built on top of writes. Each one restores
# whatever it changed, so they are individually re-runnable and order-independent
# after the first pass.
#
# `RegisterI2CBase` now inherits `BitTools`, so several of these cells exercise
# the inherited helpers implicitly — `reg_read` goes through `bytes_to_int`,
# `field_set` through `field_insert`. Section 10 tests those directly and
# offline; the point here is that the composition still behaves as it did.
#
# **Expected values** assume a TMP102 at power-on defaults sitting at room
# temperature. If the part has been configured by an earlier run, the POR
# comparisons will differ — that is reported as information, not failure.

# %% [markdown]
# ## 7.1 Transport and the layer stack
#
# Before testing the mixin, confirm each layer beneath it. If this cell fails,
# the first failing line names the layer at fault and nothing below is worth
# reading.

# %%
print("layer stack, innermost first")
print()

raw_bus  = bus.readfrom_mem(TMP_ADDR, proto.REG_TEMP, 2)
print("  machine.I2C   ", raw_bus)

raw_conn = proto_conn.readfrom_mem(proto.REG_TEMP, 2)
print("  I2CConnection ", raw_conn)

raw_dev  = proto.read_mem(proto.REG_TEMP, 2)
print("  I2CDevice     ", raw_dev)

print()
check("all three agree in length", len(raw_bus) == len(raw_conn) == len(raw_dev) == 2)
check("assembled by mixin", proto.reg_read(proto.REG_TEMP) >> 8 == raw_dev[0],
      "MSB {:#04x}".format(raw_dev[0]))

# The mixin's assembly is now BitTools.bytes_to_int — confirm the two agree on
# the same bytes, which is the seam the refactor introduced.
check("matches bytes_to_int", proto.bytes_to_int(raw_dev, True) == (raw_dev[0] << 8) | raw_dev[1],
      "{:#06x}".format(proto.bytes_to_int(raw_dev, True)))


# %% [markdown]
# ## 7.2 `reg_read` — raw, uncached, always hits the bus
#
# Two reads of a volatile register. The values may differ by an LSB or two;
# that difference is proof both reads actually reached the device rather than
# one being served from somewhere else.

# %%
show(proto.reg_read(proto.REG_TEMP),   "temp #1")
show(proto.reg_read(proto.REG_TEMP),   "temp #2")
show(proto.reg_read(proto.REG_CONFIG), "config")
show(proto.reg_read(proto.REG_THIGH),  "T_HIGH")
show(proto.reg_read(proto.REG_TLOW),   "T_LOW")

print()
cfg = proto.reg_read(proto.REG_CONFIG)
if cfg == proto.CFG_POR:
    check("at POR default", True, "{:#06x}".format(proto.CFG_POR))
else:
    print("  [info] configured        {:#06x} (POR is {:#06x}) — "
          "already set up, not a fault".format(cfg, proto.CFG_POR))

# The temperature register's low nibble is zero-padded in normal mode, which is
# a cheap structural check that the read is aligned correctly.
raw = proto.reg_read(proto.REG_TEMP)
check("temp low nibble clear", raw & 0x000F == 0, "{:#06x}".format(raw))


# %% [markdown]
# ## 7.3 `reg_get` and the shadow cache
#
# One dict per instance, keyed by register number. Watch it fill on a miss,
# serve on a hit, and go stale against a volatile register — the stale read is
# the behaviour to internalise, not a bug.

# %%
proto.cache_clear()
print("cache before:", proto.reg_cache)
print()

show(proto.reg_get(proto.REG_TEMP), "get #1 (miss)")
print("cache after: ", proto.reg_cache)
print()

# Second call: no bus traffic, so the value is guaranteed identical.
first  = proto.reg_get(proto.REG_TEMP)
second = proto.reg_get(proto.REG_TEMP)
show(first,  "get #2 (hit)")
show(second, "get #3 (hit)")
check("cached reads identical", first == second)
print()

# refresh=True forces a real read — this is the one that can differ.
show(proto.reg_get(proto.REG_TEMP, refresh=True), "get #4 (refresh)")
print("cache now:   ", {hex(k): hex(v) for k, v in proto.reg_cache.items()})


# %% [markdown]
# ## 7.4 `cache_clear`
#
# Selective and wholesale invalidation. The selective form matters after
# anything that changes one register behind the driver's back; the wholesale
# form after a device reset or power cycle.

# %%
proto.reg_get(proto.REG_TEMP)
proto.reg_get(proto.REG_CONFIG)
proto.reg_get(proto.REG_THIGH)
print("populated:  ", sorted(hex(k) for k in proto.reg_cache))

proto.cache_clear(proto.REG_TEMP)
print("one dropped:", sorted(hex(k) for k in proto.reg_cache))
check("temp gone", proto.REG_TEMP not in proto.reg_cache)
check("others kept", proto.REG_CONFIG in proto.reg_cache)

proto.cache_clear()
print("all dropped:", proto.reg_cache)
check("cache empty", len(proto.reg_cache) == 0)


# %% [markdown]
# ## 7.5 `field_mask` and `field_get`
#
# Decode the config register field by field against the datasheet power-on
# values (Tables 6-10 / 6-11). `field_mask` is a staticmethod inherited from
# `BitTools`, so it works without an instance — useful for building composite
# masks at class-definition time — and remains reachable as
# `RegisterI2CBase.field_mask(...)` exactly as before the refactor.

# %%
show(proto.reg_read(proto.REG_CONFIG), "config")
print()

fields = (("OS  one-shot",    proto.F_OS,  0),
          ("RES resolution",  proto.F_RES, 0b11),   # read-only, always 11
          ("FQ  fault queue", proto.F_FQ,  0),
          ("POL polarity",    proto.F_POL, 0),
          ("TM  thermostat",  proto.F_TM,  0),
          ("SD  shutdown",    proto.F_SD,  0),
          ("CR  conv rate",   proto.F_CR,  0b10),   # 4 Hz default
          ("AL  alert",       proto.F_AL,  1),      # 1 = non-alert with POL=0
          ("EM  extended",    proto.F_EM,  0))

for name, field, expect_por in fields:
    got = proto.field_get(proto.REG_CONFIG, field)
    flag = "" if got == expect_por else "  <- differs from POR"
    print("  {:<18} = {:>2}  (mask {:#06x}){}".format(
        name, got, proto.field_mask(field), flag))

print()
# The resolution bits are hardwired: this one is a real assertion, not a
# POR comparison, and identifies the part as strongly as anything can.
check("R1:R0 == 11", proto.field_get(proto.REG_CONFIG, proto.F_RES) == 0b11,
      "read-only, fixed by the part")

print()
# Both the old unbound spelling and the BitTools spelling must agree, since the
# method moved classes in the refactor.
check("field_mask still unbound on mixin",
      RegisterI2CBase.field_mask(proto.F_RES) == BitTools.field_mask(proto.F_RES) == 0x6000)

print()
# Fields must not overlap, and must reconstruct the whole register.
covered = 0
for _, field, _ in fields:
    m = RegisterI2CBase.field_mask(field)
    check("no overlap {:#06x}".format(m), covered & m == 0)
    covered |= m
show(covered, "covered")
show(0xFFFF & ~covered, "reserved")


# %% [markdown]
# ## 7.6 `VERIFY_MASK` composition
#
# Confirm the class-level mask really excludes exactly the three problem
# fields, built from the field tuples rather than trusting the hex literal.

# %%
excluded = (RegisterI2CBase.field_mask(proto.F_OS)
            | RegisterI2CBase.field_mask(proto.F_RES)
            | RegisterI2CBase.field_mask(proto.F_AL))

show(proto.VERIFY_MASK, "VERIFY_MASK")
show(excluded,          "should exclude")
show(0xFFFF & ~proto.VERIFY_MASK, "does exclude")

print()
check("mask excludes OS/RES/AL", proto.VERIFY_MASK & excluded == 0)
check("mask excludes nothing else", (0xFFFF & ~proto.VERIFY_MASK) == excluded)


# %% [markdown]
# ## 7.7 `reg_set` — verified writes, and why the mask is load-bearing
#
# Writing the current config back is a no-op on the device but exercises the
# full verify path. Then the same write under an *exact* check, with the
# read-only resolution bits deliberately cleared in the request — those always
# read back as `11`, so exact verification is guaranteed to fail. That failure
# is the demonstration.

# %%
original = proto.reg_read(proto.REG_CONFIG)
show(original, "before")

show(proto.reg_set(proto.REG_CONFIG, original), "landed")
show(proto.reg_cache[proto.REG_CONFIG],         "cached")
check("confirmed value cached", proto.reg_cache[proto.REG_CONFIG] == original)

print()
print("now with R1:R0 cleared in the request:")
sabotaged = original & ~RegisterI2CBase.field_mask(proto.F_RES)
show(sabotaged, "requested")

try:
    proto.reg_set(proto.REG_CONFIG, sabotaged, verify_mask=0xFFFF)
    check("exact verify raises", False, "it did not")
except ValueError as e:
    check("exact verify raises", True)
    print("        ", e)

print()
# The identical write under the class mask passes, because R1/R0 aren't compared.
show(proto.reg_set(proto.REG_CONFIG, sabotaged), "masked verify")
show(proto.reg_set(proto.REG_CONFIG, original),  "restored")


# %% [markdown]
# ## 7.8 `field_set` — read-modify-write leaves neighbours alone
#
# The property that makes bitfield writes safe: flipping one field must not
# disturb the seven others sharing the register. `field_set` now builds the
# updated word with `BitTools.field_insert`, so this cell is also the
# hardware-side check on that helper.

# %%
before = proto.reg_read(proto.REG_CONFIG)
show(before, "before")

proto.field_set(proto.REG_CONFIG, proto.F_SD, 1)
after = proto.reg_read(proto.REG_CONFIG)
show(after, "SD = 1")

changed = before ^ after
show(changed, "bits changed")
check("only SD moved", changed & ~RegisterI2CBase.field_mask(proto.F_SD) == 0)
check("SD reads back 1", proto.field_get(proto.REG_CONFIG, proto.F_SD) == 1)

# field_insert computed offline must match what the device now holds, for the
# bits the part doesn't override.
predicted = proto.field_insert(before, proto.F_SD, 1)
check("field_insert predicted it", predicted & proto.VERIFY_MASK == after & proto.VERIFY_MASK,
      "predicted {:#06x}, device {:#06x}".format(predicted, after))

print()
proto.field_set(proto.REG_CONFIG, proto.F_SD, 0)
show(proto.reg_read(proto.REG_CONFIG), "restored")
check("back to start", proto.reg_read(proto.REG_CONFIG) == before)


# %% [markdown]
# ## 7.9 `coded_get` / `coded_set` — bit patterns as physical values
#
# The caller works in hertz and fault counts; the bit patterns stay inside the
# mixin. A value not in the table raises rather than writing a wrong code.

# %%
original = proto.reg_read(proto.REG_CONFIG)

print("conversion rate:", proto.coded_get(proto.REG_CONFIG, proto.F_CR, proto.CONV_RATES), "Hz")
print("fault queue:    ", proto.coded_get(proto.REG_CONFIG, proto.F_FQ, proto.FAULT_QUEUE), "faults")
print()

for hz in (0.25, 1.0, 8.0, 4.0):
    proto.coded_set(proto.REG_CONFIG, proto.F_CR, proto.CONV_RATES, hz)
    got = proto.coded_get(proto.REG_CONFIG, proto.F_CR, proto.CONV_RATES)
    check("rate {:>5} Hz".format(hz), got == hz, "read back {} Hz".format(got))

print()
for n in (1, 2, 4, 6):
    proto.coded_set(proto.REG_CONFIG, proto.F_FQ, proto.FAULT_QUEUE, n)
    got = proto.coded_get(proto.REG_CONFIG, proto.F_FQ, proto.FAULT_QUEUE)
    check("fault queue {}".format(n), got == n, "read back {}".format(got))

print()
try:
    proto.coded_set(proto.REG_CONFIG, proto.F_CR, proto.CONV_RATES, 2.0)
    check("2.0 Hz rejected", False, "it was accepted")
except ValueError as e:
    check("2.0 Hz rejected", True)
    print("        ", e)

print()
proto.reg_set(proto.REG_CONFIG, original)
show(proto.reg_read(proto.REG_CONFIG), "restored")


# %% [markdown]
# ## 7.10 `validator` and `tolerance`
#
# The threshold registers are writable magnitudes with no read-only bits, so
# they are the right place to exercise both comparison modes.
#
# The validator runs *before* any bus traffic, so a rejected value costs zero
# transfers and cannot leave the device half-configured. `tolerance` switches
# the comparison from bitwise to numeric; the TMP102 doesn't quantise, so a
# deliberate mismatch is used to show the failure path.

# %%
original = proto.reg_read(proto.REG_THIGH)
show(original, "T_HIGH")


def under_100c(v):
    """Reject a raw threshold word above 100 C in the 12-bit left-justified format."""
    if v > 0x6400:
        raise ValueError("{:#06x} exceeds 100 C".format(v))


print()
try:
    proto.reg_set(proto.REG_THIGH, 0x7000, validator=under_100c,
                  verify_mask=proto.LIMIT_VERIFY_MASK)
    check("validator fires", False, "it did not")
except ValueError as e:
    check("validator fires", True)
    print("        ", e)

check("no bus traffic occurred", proto.reg_read(proto.REG_THIGH) == original,
      "T_HIGH unchanged")

print()
landed = proto.reg_set(proto.REG_THIGH, 0x1E00, validator=under_100c,
                       verify_mask=proto.LIMIT_VERIFY_MASK)
show(landed, "T_HIGH = 30C")
check("exact verify passes", landed == 0x1E00)

print()
# `tolerance` switches the comparison from bitwise to numeric. Demonstrating it
# needs a register whose read-back genuinely differs from the request, and the
# threshold registers don't quantise — they store exactly what you write. The
# config register does differ, because R1:R0 always read back as 11, so it
# serves as the stand-in.
#
# Writing CFG_POR with R1:R0 cleared gives a read-back 0x6000 higher than the
# request. A tolerance below that must fail; one above it must pass.
saved_cfg = proto.reg_read(proto.REG_CONFIG)
sabotaged = proto.CFG_POR & ~RegisterI2CBase.field_mask(proto.F_RES)

# Establish the actual delta empirically rather than hardcoding it — the AL bit
# tracks temperature, so the exact read-back is not knowable in advance.
proto.reg_write(proto.REG_CONFIG, sabotaged)
readback = proto.reg_read(proto.REG_CONFIG)
delta = abs(readback - sabotaged)
show(sabotaged, "requested")
show(readback,  "reads back as")
print("               delta {:#x} ({})".format(delta, delta))
print()

try:
    proto.reg_set(proto.REG_CONFIG, sabotaged, tolerance=delta - 1)
    check("tolerance just under fires", False, "it did not")
except ValueError as e:
    check("tolerance just under fires", True)
    print("        ", e)

landed = proto.reg_set(proto.REG_CONFIG, sabotaged, tolerance=delta)
check("tolerance just over passes", True, "landed {:#06x}".format(landed))
check("returns the read-back, not the request", landed == readback)
check("caches the read-back", proto.reg_cache[proto.REG_CONFIG] == readback)

print()
proto.reg_set(proto.REG_CONFIG, saved_cfg)
proto.reg_set(proto.REG_THIGH, original, verify_mask=proto.LIMIT_VERIFY_MASK)
show(proto.reg_read(proto.REG_THIGH),  "T_HIGH restored")
show(proto.reg_read(proto.REG_CONFIG), "config restored")


# %% [markdown]
# ## 7.11 `reg_write` — the raw escape hatch
#
# No verification, no cache update. The cache consequently goes stale against
# the device, which is the exact trap `reg_set` exists to avoid — shown here
# deliberately so the failure mode is recognisable when it happens by accident.

# %%
original = proto.reg_read(proto.REG_THIGH)

proto.reg_get(proto.REG_THIGH, refresh=True)         # populate cache
show(proto.reg_cache[proto.REG_THIGH], "cached")

proto.reg_write(proto.REG_THIGH, 0x2800)             # 40 C, raw
show(proto.reg_read(proto.REG_THIGH),                "device")
show(proto.reg_get(proto.REG_THIGH),                 "reg_get STALE")
check("cache is stale", proto.reg_get(proto.REG_THIGH) != proto.reg_read(proto.REG_THIGH))

print()
show(proto.reg_get(proto.REG_THIGH, refresh=True),   "after refresh")
check("refresh fixes it", proto.reg_get(proto.REG_THIGH) == 0x2800)

print()
# int_to_bytes masks oversized values rather than raising, so a too-wide write
# truncates to the register width instead of producing a wrong-length transfer.
proto.reg_write(proto.REG_THIGH, 0xABCDEF)
check("oversized value truncated", proto.reg_read(proto.REG_THIGH) == 0xCDEF,
      "wrote 0xABCDEF, device holds {:#06x}".format(proto.reg_read(proto.REG_THIGH)))

print()
proto.reg_set(proto.REG_THIGH, original, verify_mask=proto.LIMIT_VERIFY_MASK)
show(proto.reg_read(proto.REG_THIGH), "restored")


# %% [markdown]
# ## 7.12 Endianness
#
# `BIG_ENDIAN` is a chip fact, so the only way to test it on a big-endian part
# is to flip the class attribute temporarily and confirm the bytes swap. Done on
# the instance so the class attribute is untouched.
#
# Both directions now route through `BitTools`, so this also confirms
# `bytes_to_int` and `int_to_bytes` agree on which end is which.

# %%
normal = proto.reg_read(proto.REG_CONFIG)
show(normal, "big-endian")

proto.BIG_ENDIAN = False                    # instance attribute shadows the class
swapped = proto.reg_read(proto.REG_CONFIG)
show(swapped, "little-endian")

del proto.BIG_ENDIAN                        # remove the shadow, class attr returns
show(proto.reg_read(proto.REG_CONFIG), "back to big")

print()
check("bytes swapped", swapped == ((normal & 0xFF) << 8) | (normal >> 8))
check("class attr intact", proto.BIG_ENDIAN is True)

# The same swap, computed offline through the helpers alone.
check("helpers agree offline",
      proto.bytes_to_int(proto.int_to_bytes(normal, 2, True), False) == swapped)


# %% [markdown]
# ## 7.13 Summary
#
# If every check above passed, `RegisterI2CBase` is verified against real
# hardware: transport, caching, invalidation, raw and managed access, masked
# verification, validators, tolerance, bitfields and coded fields — and the
# `BitTools` helpers it now delegates to behave identically to the arithmetic
# they replaced.
#
# Anything that fails from here on is driver code.

# %%
proto.cache_clear()
proto.reg_set(proto.REG_CONFIG, proto.CFG_POR)
proto.reg_set(proto.REG_THIGH, proto.THIGH_POR, verify_mask=proto.LIMIT_VERIFY_MASK)
proto.reg_set(proto.REG_TLOW,  proto.TLOW_POR,  verify_mask=proto.LIMIT_VERIFY_MASK)

print("device returned to power-on defaults:")
show_reg(proto, proto.REG_CONFIG, "config")
show_reg(proto, proto.REG_THIGH,  "T_HIGH")
show_reg(proto, proto.REG_TLOW,   "T_LOW")


# %% [markdown]
# # 8. `TMP102` — the Real Driver
#
# TI SBOS397I. A 12-bit (13-bit in extended mode) temperature sensor with a
# four-register pointer map, programmable ALERT thresholds, and one-shot mode.
#
# Everything the driver adds on top of `TMP102Proto` is *interpretation*: turning
# raw register words into degrees Celsius, and named settings into bitfield
# writes. The register plumbing is entirely inherited.
#
# ## Data format — the part that bites
#
# Temperature, T_HIGH and T_LOW share one format: a **left-justified**
# two's-complement value.
#
# | Mode | Value bits | Padding | Range |
# |---|---|---|---|
# | Normal (EM=0) | top 12 | low 4 read 0 | -55 to +128 °C |
# | Extended (EM=1) | top 13 | low 3 read 0 | -55 to +150 °C |
#
# One LSB is 0.0625 °C in both. Left-justification is the trap: the raw word
# must be shifted right **before** sign extension, never after. Sign-extending
# the unshifted word treats the padding zeros as magnitude and gives a value
# that is wrong by a factor of sixteen.
#
# `celsius_from_raw` now shifts and then hands off to `BitTools.sign_extend`,
# which is the one implementation of that arithmetic in the notebook. The
# shift-before-extend ordering stays here, in the driver, because it is a fact
# about the TMP102's wire format rather than about two's complement.
#
# From Table 6-2: 50 °C is `0x3200`, -25 °C is `0xE700`. Both are checked
# against the codecs in section 9.
#
# ## Why `em` is cached on the instance
#
# Every temperature decode depends on the EM bit. Reading it from the device on
# each conversion would double the bus traffic of the hottest path in the
# driver; caching it and resyncing after every config write costs one extra read
# per *write* instead. The risk is a stale flag if something outside the driver
# reconfigures the part — hence `mode_refresh()`, and the `__init__` call to it.
#
# Getting this wrong is silent: a 13-bit word decoded as 12-bit reads exactly
# half the true temperature, which at room temperature looks entirely plausible.
#
# ## Verification quirks, inherited from the datasheet
#
# * **Config register.** Three kinds of bit cannot read back as written — R1/R0
#   read-only, AL comparator-driven, OS inverted during conversion. Handled by
#   the class `VERIFY_MASK`.
# * **Threshold registers.** No read-only bits, so they verify exactly, which
#   means passing `LIMIT_VERIFY_MASK` to override the config-oriented default.
#   Without that override the verify would skip bits 15, 13-14 and 5 — all real
#   data bits in a temperature word.
# * **One-shot.** OS is a handshake, not a setting. Written with raw `reg_write`
#   because it is excluded from `VERIFY_MASK` precisely so a managed write
#   cannot check it.

# %%
class TMP102(RegisterI2CBase, I2CDevice):
    """TI TMP102 digital temperature sensor (SBOS397I).

    Register-mapped through the device's pointer register. Provides temperature
    reads in degrees Celsius, the full configuration bit-field API, ALERT
    threshold programming, and one-shot conversion.

    Extended mode is tracked in `self.em` and resynced from the device on
    construction and after every config write, because it changes how every
    temperature word must be decoded.

    Example
    -------
        conn = I2CConnection(bus, 0x48, path=[], lock=bus_lock)
        tmp  = TMP102(conn)
        print(tmp.temperature())          # -> 23.4375
    """

    # --- datasheet facts (I2CNode contract) ---
    ALLOWED_ADDRS = (0x48, 0x49, 0x4A, 0x4B)   # Table 6-4: ADD0 -> GND/V+/SDA/SCL
    BAUD_NOMINAL  = 100_000
    BAUD_MAX      = 400_000                    # fast mode; HS needs a master code we can't send
    BAUD_MIN      = 10_000                     # spec floor 1 kHz (SCL timeout); margin here

    BIG_ENDIAN    = True                       # register bytes are sent MSB first
    REG_BYTES     = 2                          # every register is 16-bit

    # --- pointer register values (Table 6-7) ---
    REG_TEMP   = 0x00                          # read-only
    REG_CONFIG = 0x01
    REG_TLOW   = 0x02
    REG_THIGH  = 0x03

    # --- config bitfields as (shift, width); byte 1 is the MSB (Tables 6-10/6-11) ---
    F_OS   = (15, 1)   # one-shot / conversion ready
    F_RES  = (13, 2)   # converter resolution, read-only, always 11
    F_FQ   = (11, 2)   # fault queue
    F_POL  = (10, 1)   # ALERT polarity
    F_TM   = (9, 1)    # thermostat mode: 0=comparator, 1=interrupt
    F_SD   = (8, 1)    # shutdown
    F_CR   = (6, 2)    # conversion rate
    F_AL   = (5, 1)    # alert status, read-only
    F_EM   = (4, 1)    # extended mode

    # OS reads back inverted from what's written, R1/R0 are read-only and always
    # read 11, AL is comparator-driven. Excluded from write verification.
    VERIFY_MASK = 0xFFFF & ~(0x8000 | 0x6000 | 0x0020)

    # Thresholds are magnitude registers with no read-only bits, so they verify
    # exactly. Passed per-call to override the config-oriented VERIFY_MASK,
    # which would otherwise skip real data bits in a temperature word.
    LIMIT_VERIFY_MASK = 0xFFFF

    CFG_POR   = 0x60A0                         # power-on reset value
    THIGH_POR = 0x5000                         # 80 C
    TLOW_POR  = 0x4B00                         # 75 C

    LSB_C     = 0.0625                         # deg C per count, both modes
    TEMP_MIN  = -55.0                          # operating range floor
    TEMP_MAX  = 150.0                          # extended-mode ceiling

    CONV_RATES  = {0: 0.25, 1: 1.0, 2: 4.0, 3: 8.0}   # Table 6-5
    FAULT_QUEUE = {0: 1, 1: 2, 2: 4, 3: 6}            # Table 6-12

    CONV_TIME_MS = 10                          # typical single-conversion time

    def __init__(self, conn, **kwargs):
        """Bind to a connection and read EM to establish the decode mode."""
        super().__init__(conn, **kwargs)
        self.em = False                        # provisional; corrected below
        self.mode_refresh()

    # ------------------------------------------------------------------
    # Format conversion
    #
    # Classmethods taking an explicit `em` flag, so they can be unit-tested
    # against the datasheet tables with no device present.
    # ------------------------------------------------------------------

    @classmethod
    def celsius_from_raw(cls, word, em):
        """Decode a raw left-justified two's-complement word into degrees C.

        Strips the padding first, then sign-extends via `BitTools.sign_extend`.
        The ordering is the whole trick: extending the unshifted word would
        treat the padding zeros as magnitude and give an answer wrong by a
        factor of sixteen.
        """
        shift, bits = (3, 13) if em else (4, 12)
        return cls.sign_extend(word >> shift, bits) * cls.LSB_C

    @classmethod
    def celsius_to_raw(cls, celsius, em):
        """Encode degrees C into a raw left-justified two's-complement word.

        Rounds half-away-from-zero rather than using round(), whose
        banker's-rounding would bias thresholds toward even LSBs.

        Two separate bounds apply and both are enforced. The register width
        caps what can be *represented* (12-bit reaches +127.9375 C, 13-bit
        +255.9375 C), while the part's operating range caps what is
        *meaningful* (-55 to +150 C). The second is tighter than the first in
        extended mode, so checking only the register width would silently
        accept a 200 C threshold the sensor can never reach.
        """
        shift, bits = (3, 13) if em else (4, 12)
        counts = int(celsius / cls.LSB_C + (0.5 if celsius >= 0 else -0.5))

        lo, hi = -(1 << (bits - 1)), (1 << (bits - 1)) - 1
        if not (lo <= counts <= hi):
            raise ValueError("{} C not representable in {}-bit mode ({:.4f} to {:.4f} C)".format(
                celsius, bits, lo * cls.LSB_C, hi * cls.LSB_C))

        if not (cls.TEMP_MIN <= celsius <= cls.TEMP_MAX):
            raise ValueError("{} C outside the part's operating range ({} to {} C)".format(
                celsius, cls.TEMP_MIN, cls.TEMP_MAX))

        return (counts & ((1 << bits) - 1)) << shift

    # ------------------------------------------------------------------
    # Temperature
    # ------------------------------------------------------------------

    def temperature(self):
        """Return the most recent conversion in degrees C. Always hits the bus.

        Uses raw `reg_read`, never `reg_get`: the temperature register is
        volatile by definition and must never be served from cache.
        """
        return self.celsius_from_raw(self.reg_read(self.REG_TEMP), self.em)

    def temperature_raw(self):
        """Return the raw 16-bit temperature word, undecoded.

        For diagnostics, and for callers doing fixed-point maths who do not
        want a float on a target without hardware FP.
        """
        return self.reg_read(self.REG_TEMP)

    # ------------------------------------------------------------------
    # Configuration
    #
    # Read-modify-write throughout, via the mixin. The config register holds
    # eight unrelated settings, so a bare write would clobber them.
    # ------------------------------------------------------------------

    def config_get(self, refresh=True):
        """Return the 16-bit configuration register.

        Defaults to refresh=True because OS and AL are live status bits.
        """
        return self.reg_get(self.REG_CONFIG, refresh=refresh)

    def config_set(self, value):
        """Write the whole configuration register, verified against VERIFY_MASK."""
        result = self.reg_set(self.REG_CONFIG, value)
        self.em = bool(self.field_get(self.REG_CONFIG, self.F_EM, refresh=False))
        return result

    def config_field_set(self, field, value):
        """Write one config bitfield, then resync EM in case the format changed."""
        result = self.field_set(self.REG_CONFIG, field, value)
        self.em = bool(self.field_get(self.REG_CONFIG, self.F_EM, refresh=False))
        return result

    def mode_refresh(self):
        """Re-read EM from the device and update the cached decode mode.

        Worth calling explicitly if anything outside this driver may have
        reconfigured the part — a general-call reset, or a power cycle the
        driver did not see.
        """
        self.em = bool(self.field_get(self.REG_CONFIG, self.F_EM))
        return self.em

    def config_reset(self):
        """Restore the configuration register to its power-on default."""
        return self.config_set(self.CFG_POR)

    # --- individual settings ---

    @property
    def extended_mode(self):
        """Extended 13-bit mode, allowing readings above 128 C."""
        return self.em

    @extended_mode.setter
    def extended_mode(self, enable):
        self.config_field_set(self.F_EM, 1 if enable else 0)

    @property
    def shutdown(self):
        """Shutdown mode; halts conversions. Required before `one_shot()`."""
        return bool(self.field_get(self.REG_CONFIG, self.F_SD))

    @shutdown.setter
    def shutdown(self, enable):
        self.config_field_set(self.F_SD, 1 if enable else 0)

    @property
    def conversion_rate(self):
        """Continuous-conversion rate in Hz: 0.25, 1, 4 (default) or 8."""
        return self.coded_get(self.REG_CONFIG, self.F_CR, self.CONV_RATES)

    @conversion_rate.setter
    def conversion_rate(self, hz):
        self.coded_set(self.REG_CONFIG, self.F_CR, self.CONV_RATES, hz)

    @property
    def fault_queue(self):
        """Consecutive out-of-limit conversions before ALERT asserts: 1, 2, 4 or 6."""
        return self.coded_get(self.REG_CONFIG, self.F_FQ, self.FAULT_QUEUE)

    @fault_queue.setter
    def fault_queue(self, faults):
        self.coded_set(self.REG_CONFIG, self.F_FQ, self.FAULT_QUEUE, faults)

    @property
    def thermostat_mode(self):
        """False = comparator mode, True = interrupt mode (ALERT latches)."""
        return bool(self.field_get(self.REG_CONFIG, self.F_TM))

    @thermostat_mode.setter
    def thermostat_mode(self, interrupt):
        self.config_field_set(self.F_TM, 1 if interrupt else 0)

    @property
    def alert_polarity_high(self):
        """False (default) = ALERT active low, True = ALERT active high."""
        return bool(self.field_get(self.REG_CONFIG, self.F_POL))

    @alert_polarity_high.setter
    def alert_polarity_high(self, active_high):
        self.config_field_set(self.F_POL, 1 if active_high else 0)

    @property
    def alert(self):
        """Raw AL status bit.

        Note the sense: with POL=0 the bit reads 1 in the NON-alert state and
        drops to 0 once T_HIGH is exceeded. POL inverts this. Returned raw, so
        interpret it against the current polarity setting.
        """
        return bool(self.field_get(self.REG_CONFIG, self.F_AL))

    @property
    def conversion_ready(self):
        """OS bit: True when idle, False mid-conversion.

        Only meaningful in shutdown mode, where OS is the one-shot handshake.
        In continuous mode it is not a useful conversion-complete signal.
        """
        return bool(self.field_get(self.REG_CONFIG, self.F_OS))

    # ------------------------------------------------------------------
    # Thresholds
    #
    # T_LOW and T_HIGH share the temperature register's format, so the same
    # codecs apply. Power-on defaults are T_HIGH = 80 C, T_LOW = 75 C.
    # ------------------------------------------------------------------

    def high_limit_get(self):
        """Return T_HIGH in degrees C (power-on default 80 C)."""
        return self.celsius_from_raw(self.reg_read(self.REG_THIGH), self.em)

    def low_limit_get(self):
        """Return T_LOW in degrees C (power-on default 75 C)."""
        return self.celsius_from_raw(self.reg_read(self.REG_TLOW), self.em)

    def high_limit_set(self, celsius):
        """Set T_HIGH, verified exactly; returns what landed, in degrees C.

        Exact verification is correct despite the rounding in `celsius_to_raw`:
        rounding happens before the write, so the raw word written and the raw
        word read back must match. The returned value may still differ from the
        request by up to half an LSB — that is the rounding, not a fault.
        """
        raw = self.celsius_to_raw(celsius, self.em)
        return self.celsius_from_raw(
            self.reg_set(self.REG_THIGH, raw, verify_mask=self.LIMIT_VERIFY_MASK),
            self.em)

    def low_limit_set(self, celsius):
        """Set T_LOW, verified exactly; returns what landed, in degrees C.

        T_LOW is the release point of the comparator hysteresis band, so it
        should sit below T_HIGH. The device does not enforce this and will
        behave oddly if the two are crossed.
        """
        raw = self.celsius_to_raw(celsius, self.em)
        return self.celsius_from_raw(
            self.reg_set(self.REG_TLOW, raw, verify_mask=self.LIMIT_VERIFY_MASK),
            self.em)

    def limits_set(self, low_c, high_c):
        """Set both thresholds, rejecting a crossed pair before any bus traffic."""
        if low_c >= high_c:
            raise ValueError("T_LOW ({}) must be below T_HIGH ({})".format(low_c, high_c))
        return self.low_limit_set(low_c), self.high_limit_set(high_c)

    def limits_reset(self):
        """Restore both thresholds to their power-on defaults (75 C / 80 C)."""
        self.reg_set(self.REG_TLOW,  self.TLOW_POR,  verify_mask=self.LIMIT_VERIFY_MASK)
        self.reg_set(self.REG_THIGH, self.THIGH_POR, verify_mask=self.LIMIT_VERIFY_MASK)

    # ------------------------------------------------------------------
    # One-shot
    # ------------------------------------------------------------------

    def one_shot(self, timeout_ms=100):
        """Trigger a single conversion from shutdown and return it in degrees C.

        Requires the device already in shutdown — writing OS in continuous mode
        does nothing useful, so this raises rather than silently returning a
        stale continuous-mode reading indistinguishable from a real one-shot.

        Handshake: write OS=1, poll until OS reads back 1 (it reads 0 while the
        conversion is in flight), then read the temperature. Typical conversion
        time is 10 ms; the default timeout gives an order of magnitude of margin.

        Raises
        ------
        RuntimeError
            If the device is not in shutdown mode, or the conversion does not
            complete within `timeout_ms`.
        """
        import time

        cfg = self.config_get()
        if not (cfg & self.field_mask(self.F_SD)):
            raise RuntimeError("one_shot() requires shutdown mode; set .shutdown = True")

        # Raw write: OS is excluded from VERIFY_MASK precisely because it will
        # not read back as written here, so a managed write cannot verify it.
        os_bit = self.field_mask(self.F_OS)
        self.reg_write(self.REG_CONFIG, cfg | os_bit)

        deadline = time.ticks_add(time.ticks_ms(), timeout_ms)
        while not (self.reg_read(self.REG_CONFIG) & os_bit):
            if time.ticks_diff(deadline, time.ticks_ms()) <= 0:
                raise RuntimeError("conversion timed out after {} ms".format(timeout_ms))
            time.sleep_ms(1)

        return self.temperature()


# %% [markdown]
# # 9. Verifying the `TMP102` Driver
#
# Section 7 established that `RegisterI2CBase` works. Everything here tests only
# what the driver adds on top: format conversion, the named settings, the
# threshold API, and the one-shot handshake.
#
# ## Identification, and its limits
#
# The TMP102 has **no device-ID register**, so identification is circumstantial.
# The strongest available evidence is the read-only R1/R0 resolution bits, fixed
# at `11` by the part. A device reading anything else is not a TMP102; a device
# reading `11` merely might be.
#
# A general-call reset (`0x00`, `0x06`) would confirm more, but it resets *every*
# device on the segment, so it is not something to do during routine bring-up.

# %%
tmp_conn = I2CConnection(bus, TMP_ADDR, path=[], lock=bus_lock)
tmp = TMP102(tmp_conn)

print("TMP102 bound at {:#04x}".format(TMP_ADDR))
print("extended mode:", tmp.extended_mode)
print("temperature:  {:.4f} C".format(tmp.temperature()))


# %% [markdown]
# ## 9.1 Format codecs — offline, against the datasheet
#
# No bus traffic at all. Both directions are checked for every value, so an
# error in one codec cannot be masked by a matching error in the other.
#
# This is also the strongest test of `BitTools.sign_extend` in the notebook:
# every negative case below exercises it, and a wrong implementation would show
# up immediately as a sign error rather than a rounding one.
#
# ### A trap in the datasheet's own tables
#
# Tables 6-2 and 6-3 list the temperature as a **right-justified count** in
# their HEX column — 50 °C appears as `320`, three hex digits, which is
# 800 counts. That is *not* the 16-bit word on the wire. The wire format comes
# from Tables 6-8 and 6-9, which show the value left-justified: `0x3200` in
# normal mode (count shifted left 4), `0x1900` for 25 °C, and so on.
#
# The expected words below are the **wire** words, derived by shifting the
# datasheet's count left by 4 (normal) or 3 (extended). Reading the HEX column
# as if it were the wire word is an easy mistake and produces values wrong by a
# factor of sixteen.

# %%
print("12-bit / normal mode — count << 4")
cases_12 = ((127.9375, 0x7FF0), (100.0, 0x6400), (75.0, 0x4B00),
            (50.0, 0x3200), (25.0, 0x1900), (0.25, 0x0040),
            (0.0, 0x0000), (-0.25, 0xFFC0), (-25.0, 0xE700), (-55.0, 0xC900))

for celsius, word in cases_12:
    enc = TMP102.celsius_to_raw(celsius, False)
    dec = TMP102.celsius_from_raw(word, False)
    check("{:>9} C".format(celsius), enc == word and dec == celsius,
          "enc {:#06x} (want {:#06x}), dec {}".format(enc, word, dec))

print()
print("13-bit / extended mode — count << 3")
cases_13 = ((150.0, 0x4B00), (128.0, 0x4000), (100.0, 0x3200),
            (25.0, 0x0C80), (0.0, 0x0000), (-25.0, 0xF380), (-55.0, 0xE480))

for celsius, word in cases_13:
    enc = TMP102.celsius_to_raw(celsius, True)
    dec = TMP102.celsius_from_raw(word, True)
    check("{:>9} C".format(celsius), enc == word and dec == celsius,
          "enc {:#06x} (want {:#06x}), dec {}".format(enc, word, dec))

print()
print("range rejection")
# Two separate bounds: register width, and the part's operating range. The
# second is the tighter one in extended mode, where 13 bits could represent
# 200 C but the sensor cannot measure it.
for celsius, em, why in ((200.0, False, "exceeds 12-bit width"),
                         (200.0, True,  "representable, but above 150 C"),
                         (-100.0, False, "representable, but below -55 C"),
                         (-100.0, True,  "representable, but below -55 C")):
    try:
        TMP102.celsius_to_raw(celsius, em)
        check("{:>7} C {}-bit".format(celsius, 13 if em else 12), False, "accepted")
    except ValueError:
        check("{:>7} C {}-bit".format(celsius, 13 if em else 12), True, why)

print()
# Round-trip every representable step at the extremes, to catch sign-extension
# errors that a handful of table values might miss.
ok = True
for em in (False, True):
    hi = 150.0 if em else 127.9375
    for celsius in (-55.0, -0.0625, 0.0, 0.0625, hi):
        w = TMP102.celsius_to_raw(celsius, em)
        if TMP102.celsius_from_raw(w, em) != celsius:
            ok = False
            print("      round-trip failed: {} C {}-bit -> {:#06x} -> {}".format(
                celsius, 13 if em else 12, w, TMP102.celsius_from_raw(w, em)))
check("round-trips at the extremes", ok)


# %% [markdown]
# ## 9.2 Identification and plausibility
#
# The structural checks that can be made against a live part without writing
# anything.

# %%
cfg = tmp.config_get()
show(cfg, "config")
print()

check("R1:R0 == 11", tmp.field_get(tmp.REG_CONFIG, tmp.F_RES) == 0b11,
      "read-only, fixed by the part")
check("reserved bits clear", cfg & 0x000F == 0,
      "low nibble {:#03x}".format(cfg & 0x000F))

raw = tmp.temperature_raw()
pad = 0b111 if tmp.extended_mode else 0b1111
check("temp zero-padded", raw & pad == 0,
      "raw {:#06x}, {}-bit mode".format(raw, 13 if tmp.extended_mode else 12))

t = tmp.temperature()
check("plausible reading", tmp.TEMP_MIN <= t <= tmp.TEMP_MAX, "{:.4f} C".format(t))
check("plausibly indoors", 5.0 <= t <= 45.0, "{:.4f} C".format(t))

print()
if cfg == tmp.CFG_POR:
    print("  [info] at POR default {:#06x}".format(tmp.CFG_POR))
else:
    print("  [info] configured {:#06x} (POR {:#06x}) — not a fault".format(
        cfg, tmp.CFG_POR))


# %% [markdown]
# ## 9.3 Thresholds
#
# The full round trip: degrees in, encode, write, verify, read back, decode,
# degrees out. A fault anywhere in that chain shows up as a wrong number rather
# than a silent success.
#
# The values chosen are exact multiples of the LSB, so no rounding is in play
# and the comparison can be exact.

# %%
saved_high = tmp.high_limit_get()
saved_low  = tmp.low_limit_get()
print("saved: T_HIGH {:.4f} C, T_LOW {:.4f} C".format(saved_high, saved_low))
print()

check("T_HIGH at POR", saved_high == 80.0, "{:.4f} C".format(saved_high))
check("T_LOW at POR",  saved_low  == 75.0, "{:.4f} C".format(saved_low))

print()
for target in (30.0, 0.0, -25.0, 100.0):
    landed = tmp.high_limit_set(target)
    check("T_HIGH = {:>6} C".format(target), landed == target,
          "landed {:.4f}, raw {:#06x}".format(landed, tmp.reg_read(tmp.REG_THIGH)))

print()
# Half-LSB rounding: 20.03 C is not representable, so it must land on a
# neighbouring step rather than being rejected or silently truncated.
landed = tmp.low_limit_set(20.03)
check("rounds to nearest LSB", abs(landed - 20.03) <= tmp.LSB_C / 2,
      "requested 20.03, landed {:.4f}".format(landed))

print()
# A crossed pair is caught before any bus traffic.
try:
    tmp.limits_set(80.0, 20.0)
    check("crossed limits rejected", False, "it was accepted")
except ValueError as e:
    check("crossed limits rejected", True)
    print("        ", e)

print()
tmp.limits_reset()
check("restored to POR", tmp.high_limit_get() == 80.0 and tmp.low_limit_get() == 75.0,
      "T_HIGH {:.4f}, T_LOW {:.4f}".format(tmp.high_limit_get(), tmp.low_limit_get()))


# %% [markdown]
# ## 9.4 Configuration properties
#
# Each named setting round-tripped through its property. The config register is
# saved and restored, so this cell is idempotent.

# %%
saved_cfg = tmp.config_get()
show(saved_cfg, "saved")
print()

print("current settings")
print("  conversion rate  ", tmp.conversion_rate, "Hz")
print("  fault queue      ", tmp.fault_queue, "faults")
print("  shutdown         ", tmp.shutdown)
print("  thermostat mode  ", "interrupt" if tmp.thermostat_mode else "comparator")
print("  ALERT polarity   ", "active high" if tmp.alert_polarity_high else "active low")
print("  extended mode    ", tmp.extended_mode)
print("  AL bit           ", tmp.alert, "(1 = non-alert when POL=0)")
print()

for hz in (0.25, 1.0, 8.0, 4.0):
    tmp.conversion_rate = hz
    check("rate {:>5} Hz".format(hz), tmp.conversion_rate == hz)

print()
for n in (1, 2, 4, 6):
    tmp.fault_queue = n
    check("fault queue {}".format(n), tmp.fault_queue == n)

print()
for name, attr in (("thermostat_mode", "thermostat_mode"),
                   ("alert_polarity_high", "alert_polarity_high"),
                   ("shutdown", "shutdown")):
    for state in (True, False):
        setattr(tmp, attr, state)
        check("{} = {}".format(name, state), getattr(tmp, attr) is state)

print()
try:
    tmp.conversion_rate = 2.0
    check("2.0 Hz rejected", False, "it was accepted")
except ValueError as e:
    check("2.0 Hz rejected", True)
    print("        ", e)

print()
tmp.config_set(saved_cfg)
check("config restored", tmp.config_get() & tmp.VERIFY_MASK == saved_cfg & tmp.VERIFY_MASK)


# %% [markdown]
# ## 9.5 Extended mode
#
# The one setting that changes how every subsequent read is *interpreted*, which
# makes it the one where a stale flag is silently wrong rather than loudly
# broken. A 13-bit word decoded as 12-bit reads half the true temperature.
#
# Reading the same physical temperature through both decode paths is the check:
# they must agree, because the part is the same temperature either way.

# %%
saved_cfg = tmp.config_get()

check("starts in normal mode", tmp.extended_mode is False)
t12 = tmp.temperature()
raw12 = tmp.temperature_raw()
show(raw12, "raw 12-bit")
print("       decoded {:.4f} C".format(t12))
print()

tmp.extended_mode = True
check("EM bit set", tmp.extended_mode is True)
check("driver tracked it", tmp.em is True)
t13 = tmp.temperature()
raw13 = tmp.temperature_raw()
show(raw13, "raw 13-bit")
print("       decoded {:.4f} C".format(t13))
print()

# The part keeps converting between reads, so exact equality is not expected —
# but a decode error would show up as a factor-of-two discrepancy, not 0.5 C.
check("both modes agree", abs(t13 - t12) < 0.5,
      "12-bit {:.4f}, 13-bit {:.4f}, delta {:.4f}".format(t13, t12, abs(t13 - t12)))

# Extended mode sets D0 of byte 2 as its indicator (datasheet Table 6-9).
check("EM indicator bit set", raw13 & 0x0001 == 1, "raw {:#06x}".format(raw13))

print()
tmp.extended_mode = False
check("EM cleared", tmp.extended_mode is False)

# mode_refresh must agree with what the driver already believes.
check("mode_refresh agrees", tmp.mode_refresh() == tmp.em)

print()
# A stale flag is the failure this guards against: force one and confirm
# mode_refresh corrects it.
tmp.em = True                                  # deliberately wrong
check("flag now stale", tmp.em is True)
tmp.mode_refresh()
check("mode_refresh corrects", tmp.em is False)

tmp.config_set(saved_cfg)


# %% [markdown]
# ## 9.6 One-shot conversion
#
# The OS handshake, which is the one place the driver deliberately bypasses the
# managed write path. Also confirms the guard against calling it outside
# shutdown mode.

# %%
saved_cfg = tmp.config_get()

# Guard: continuous mode must raise rather than return a stale reading.
tmp.shutdown = False
try:
    tmp.one_shot()
    check("guard fires in continuous mode", False, "it did not")
except RuntimeError as e:
    check("guard fires in continuous mode", True)
    print("        ", e)

print()
tmp.shutdown = True
check("in shutdown", tmp.shutdown is True)
check("conversion_ready", tmp.conversion_ready is True, "OS reads 1 when idle")

print()
import time
for i in range(3):
    start = time.ticks_ms()
    t = tmp.one_shot()
    elapsed = time.ticks_diff(time.ticks_ms(), start)
    check("one-shot #{}".format(i + 1), tmp.TEMP_MIN <= t <= tmp.TEMP_MAX,
          "{:.4f} C in {} ms".format(t, elapsed))

print()
# A timeout shorter than the conversion time must raise rather than return
# a value from the previous conversion.
try:
    tmp.one_shot(timeout_ms=0)
    print("  [info] timeout_ms=0 completed anyway — conversion was already done")
except RuntimeError as e:
    check("timeout fires", True)
    print("        ", e)

print()
tmp.config_set(saved_cfg)
tmp.shutdown = False
check("back in continuous mode", tmp.shutdown is False)


# %% [markdown]
# ## 9.7 Continuous read
#
# Confirms the readings track reality. Pinch the package and the number should
# climb within a second or two — the thermal path runs through the package
# leads, so it responds quickly.

# %%
import time

tmp.conversion_rate = 8.0
print("reading at 8 Hz for 5 s — pinch the sensor to see it move")
print()

for _ in range(20):
    print("  {:7.4f} C   raw {:#06x}".format(tmp.temperature(), tmp.temperature_raw()))
    time.sleep_ms(250)

tmp.config_reset()


# %% [markdown]
# ## 9.8 ALERT behaviour
#
# The comparator drives the AL bit off the thresholds, so setting T_HIGH below
# the current temperature must flip it. This exercises the threshold and status
# paths against each other, and needs no ALERT pin wired.
#
# Recall the polarity: with POL=0, AL reads **1** in the non-alert state.

# %%
saved_cfg = tmp.config_get()
tmp.limits_reset()
tmp.fault_queue = 1                            # respond to a single conversion
tmp.conversion_rate = 8.0

t_now = tmp.temperature()
print("current temperature: {:.4f} C".format(t_now))
print()

check("no alert at POR limits", tmp.alert is True, "AL = 1 means non-alert")

# Drop T_HIGH well below ambient: the comparator should trip.
tmp.limits_set(t_now - 20.0, t_now - 10.0)
time.sleep_ms(500)                             # several conversions at 8 Hz
check("alert asserts", tmp.alert is False,
      "T_HIGH {:.4f} C, temp {:.4f} C".format(tmp.high_limit_get(), tmp.temperature()))

# Raise T_LOW back above ambient: the comparator should release.
tmp.limits_set(t_now + 10.0, t_now + 20.0)
time.sleep_ms(500)
check("alert releases", tmp.alert is True,
      "T_LOW {:.4f} C, temp {:.4f} C".format(tmp.low_limit_get(), tmp.temperature()))

print()
tmp.limits_reset()
tmp.config_set(saved_cfg)


# %% [markdown]
# ## 9.9 Restore and summary
#
# Return the part to power-on defaults so the notebook can be re-run from the
# top without carrying state forward.

# %%
tmp.cache_clear()
tmp.config_reset()
tmp.limits_reset()

print("device at power-on defaults:")
show(tmp.config_get(),                "config")
show(tmp.reg_read(tmp.REG_THIGH),     "T_HIGH")
show(tmp.reg_read(tmp.REG_TLOW),      "T_LOW")
print()
print("final reading: {:.4f} C".format(tmp.temperature()))


# %% [markdown]
# # 10. Verifying `BitTools` and `CRCMixin`
#
# ## Why these are tested separately, and offline
#
# Neither class touches the bus. That is precisely what makes them testable in a
# way the rest of the notebook is not: there is no device state, no timing, no
# ambient temperature, and every expected value can come from a datasheet or a
# published check vector rather than from what the hardware happened to return.
#
# So these cells are **pure computation**, and they either pass or the
# implementation is wrong. No `[info]` escape hatches, no "differs from POR, not
# a fault". A failure here is unambiguous.
#
# They sit after section 9 rather than before section 7 for a practical reason:
# the board bring-up cells are the ones most likely to need re-running, and
# burying them behind three hundred lines of arithmetic tests makes that
# tedious. Nothing here depends on anything above except the `check` helper.
#
# ## What the CRC vectors are
#
# Three independent sources, which is the point — agreeing with one datasheet
# proves only that the datasheet was transcribed correctly.
#
# 1. **SHTC3 datasheet, Table 16.** `CRC(0x00) = 0xAC`, `CRC(0xBEEF) = 0x92`.
# 2. **TMAG5273 datasheet, section 7.4.1.** `CRC(0xE0) = 0x5D`,
#    `CRC(0x00000000) = 0xD1`, `CRC(0xFFFFFFFF) = 0x0F`. Note this part uses
#    polynomial 0x07, not Sensirion's 0x31, which makes it a genuinely
#    independent check of the parameterisation rather than of one polynomial.
# 3. **The standard `123456789` check value** published for every catalogued CRC
#    variant. Four are checked, spanning both widths and both reflection
#    settings, because reflection is the parameter most likely to be
#    implemented backwards and least likely to be caught by a part-specific
#    vector.
#
# ## A caution on the TMAG5273 vectors
#
# The OCR'd copy of that datasheet contains the vector list twice, and the
# second copy is garbled — it shows `0x0F`, `0x01`, `0x5A`, `0x31` against the
# same inputs. The values used below are from the clean page-24 render. If a
# future part disagrees, suspect the transcription before the implementation,
# and confirm against real silicon.

# %% [markdown]
# ## 10.1 `sign_extend`
#
# The single most-reimplemented piece of arithmetic in sensor drivers, and the
# one with a standard bug. Checked at every boundary that matters: the largest
# positive value, the most negative value, minus one, zero, and the wrap point
# either side of the sign bit.
#
# The last group is the ordering trap the TMP102 driver depends on — the value
# must already be shifted right before it gets here.

# %%
print("boundaries, per width")
print()

for bits in (8, 12, 13, 16, 24):
    hi   = (1 << (bits - 1)) - 1          # largest positive
    lo   = 1 << (bits - 1)                # most negative, as an unsigned pattern
    ones = (1 << bits) - 1                # all ones = -1

    check("{}-bit  max positive".format(bits),
          BitTools.sign_extend(hi, bits) == hi, "{:#x} -> {}".format(hi, hi))
    check("{}-bit  min negative".format(bits),
          BitTools.sign_extend(lo, bits) == -(1 << (bits - 1)),
          "{:#x} -> {}".format(lo, -(1 << (bits - 1))))
    check("{}-bit  all ones".format(bits),
          BitTools.sign_extend(ones, bits) == -1, "{:#x} -> -1".format(ones))
    check("{}-bit  zero".format(bits),
          BitTools.sign_extend(0, bits) == 0)
    print()

# Masking: a value wider than `bits` must give the same answer as one that was
# pre-masked, so a caller passing an unstripped word is not silently wrong in a
# different way.
check("masks oversized input",
      BitTools.sign_extend(0xFFFFF, 12) == BitTools.sign_extend(0xFFF, 12) == -1)

# Every 8-bit value against Python's own two's-complement interpretation.
ok = all(BitTools.sign_extend(v, 8) == (v - 256 if v & 0x80 else v) for v in range(256))
check("exhaustive over 8 bits", ok, "all 256 values")

print()
# The TMP102 wire format, as the driver uses it: shift the padding out first.
for word, em, want in ((0xE700, False, -400), (0x3200, False, 800),
                       (0xF380, True, -400),  (0x3200, True, 1600)):
    shift, bits = (3, 13) if em else (4, 12)
    got = BitTools.sign_extend(word >> shift, bits)
    check("TMP102 {:#06x} {}-bit".format(word, bits), got == want,
          "{} counts".format(got))

# And the error the ordering prevents: extending without shifting.
wrong = BitTools.sign_extend(0xE700, 12)
check("unshifted would be wrong", wrong != -400,
      "{} instead of -400 — shift first".format(wrong))


# %% [markdown]
# ## 10.2 Field helpers
#
# `field_mask`, `field_extract` and `field_insert` against the TMP102 config
# layout, which is convenient here because section 7 already validated those
# field tuples against real silicon.
#
# The properties that matter: extract and insert are inverses, insert never
# disturbs a neighbouring field, and an oversized value is clamped to the field
# rather than spilling.

# %%
FIELDS = (("OS",  (15, 1), 0x8000),
          ("RES", (13, 2), 0x6000),
          ("FQ",  (11, 2), 0x1800),
          ("POL", (10, 1), 0x0400),
          ("TM",  (9, 1),  0x0200),
          ("SD",  (8, 1),  0x0100),
          ("CR",  (6, 2),  0x00C0),
          ("AL",  (5, 1),  0x0020),
          ("EM",  (4, 1),  0x0010))

for name, field, want in FIELDS:
    check("field_mask {}".format(name), BitTools.field_mask(field) == want,
          "{:#06x}".format(BitTools.field_mask(field)))

print()
# Decode the POR word field by field, offline.
CFG_POR = 0x60A0
print("CFG_POR {:#06x} decoded offline".format(CFG_POR))
for name, field, _ in FIELDS:
    print("  {:<4} = {}".format(name, BitTools.field_extract(CFG_POR, field)))

print()
check("RES reads 11", BitTools.field_extract(CFG_POR, (13, 2)) == 0b11)
check("CR reads 4 Hz code", BitTools.field_extract(CFG_POR, (6, 2)) == 0b10)
check("AL reads 1", BitTools.field_extract(CFG_POR, (5, 1)) == 1)

print()
# extract and insert are inverses, for every field and every value it can hold.
ok = True
for name, field, _ in FIELDS:
    shift, width = field
    for v in range(1 << width):
        if BitTools.field_extract(BitTools.field_insert(CFG_POR, field, v), field) != v:
            ok = False
            print("      round-trip failed: {} = {}".format(name, v))
check("insert/extract round-trip", ok, "every field, every value")

print()
# Insert must leave every other field alone.
ok = True
for name, field, mask in FIELDS:
    shift, width = field
    for v in range(1 << width):
        out = BitTools.field_insert(CFG_POR, field, v)
        if (out & ~mask) != (CFG_POR & ~mask):
            ok = False
            print("      {} = {} disturbed neighbours: {:#06x}".format(name, v, out))
check("insert leaves neighbours alone", ok)

print()
# An oversized value is masked to the field width, not allowed to spill upward.
spilled = BitTools.field_insert(0x0000, (8, 1), 0xFF)
check("oversized value clamped", spilled == 0x0100, "{:#06x}".format(spilled))


# %% [markdown]
# ## 10.3 `bit_reverse`
#
# Needed only by reflected CRC variants, but wrong reflection is silent — it
# produces a checksum that is self-consistent and disagrees with the part. Worth
# checking on its own before it is buried inside a CRC.

# %%
for value, bits, want in ((0b10000000, 8, 0b00000001),
                          (0b00000001, 8, 0b10000000),
                          (0b11010010, 8, 0b01001011),
                          (0x0001, 16, 0x8000),
                          (0x8000, 16, 0x0001),
                          (0xFFFF, 16, 0xFFFF),
                          (0x0000, 8, 0x0000)):
    got = BitTools.bit_reverse(value, bits)
    check("{:#x} in {} bits".format(value, bits), got == want, "-> {:#x}".format(got))

print()
# Reversing twice is the identity, over every 8-bit value.
ok = all(BitTools.bit_reverse(BitTools.bit_reverse(v, 8), 8) == v for v in range(256))
check("involution over 8 bits", ok, "all 256 values")

# Palindromic patterns are their own reverse — a check that the loop isn't
# accidentally doing nothing, since it would pass those trivially. Paired with
# the asymmetric cases above, this pins the behaviour down.
check("palindrome fixed", BitTools.bit_reverse(0b10011001, 8) == 0b10011001)


# %% [markdown]
# ## 10.4 Byte assembly
#
# `bytes_to_int` and `int_to_bytes`, in both endiannesses, at each width the
# framework routinely sees. These now sit under every `reg_read` and
# `reg_write`, so an error here would be systematic rather than local.

# %%
for data, be, want in ((b"\x12\x34", True, 0x1234),
                       (b"\x12\x34", False, 0x3412),
                       (b"\xFF", True, 0xFF),
                       (b"\x12\x34\x56", True, 0x123456),
                       (b"\x12\x34\x56", False, 0x563412),
                       (b"\x00\x00", True, 0x0000)):
    got = BitTools.bytes_to_int(data, be)
    check("{} {}".format(data, "BE" if be else "LE"), got == want,
          "-> {:#x}".format(got))

print()
for value, n, be, want in ((0x1234, 2, True, b"\x12\x34"),
                           (0x1234, 2, False, b"\x34\x12"),
                           (0xFF, 1, True, b"\xFF"),
                           (0x123456, 3, True, b"\x12\x34\x56"),
                           (0x123456, 3, False, b"\x56\x34\x12")):
    got = BitTools.int_to_bytes(value, n, be)
    check("{:#x} in {} B {}".format(value, n, "BE" if be else "LE"), got == want,
          "-> {}".format(got))

print()
# Truncation rather than raising, matching what reg_write has always done.
check("oversized truncates", BitTools.int_to_bytes(0xABCDEF, 2, True) == b"\xCD\xEF",
      "0xABCDEF in 2 bytes -> 0xCDEF")

print()
# Round-trip in both directions, across widths.
ok = True
for n in (1, 2, 3, 4):
    for be in (True, False):
        for value in (0, 1, (1 << (8 * n)) - 1, (1 << (8 * n)) // 3):
            if BitTools.bytes_to_int(BitTools.int_to_bytes(value, n, be), be) != value:
                ok = False
                print("      round-trip failed: {:#x} in {} B {}".format(
                    value, n, "BE" if be else "LE"))
check("round-trips every width", ok)


# %% [markdown]
# ## 10.5 The `CRC_POLY` contract
#
# The whole reason `CRCMixin` is a mixin rather than a function: a driver that
# forgets its polynomial must fail loudly at the first call, not compute
# checksums against an arbitrary default.
#
# Every entry point is checked, because it would be easy to guard `crc` and
# forget `crc_table`.

# %%
class UnconfiguredCRC(CRCMixin):
    """A driver that forgot to declare its polynomial."""
    pass


for name, fn in (("crc", lambda: UnconfiguredCRC.crc(b"\x00")),
                 ("crc_ok", lambda: UnconfiguredCRC.crc_ok(b"\x00", 0x00)),
                 ("crc_verify", lambda: UnconfiguredCRC.crc_verify(b"\x00", 0x00)),
                 ("crc_table", lambda: UnconfiguredCRC.crc_table())):
    try:
        fn()
        check("{} raises".format(name), False, "it did not")
    except NotImplementedError as e:
        check("{} raises".format(name), True)
        print("        ", e)

print()
# The message must name the class, since in a multi-part project the culprit is
# nearly always one driver rather than the mixin.
try:
    UnconfiguredCRC.crc(b"\x00")
except NotImplementedError as e:
    check("message names the class", "UnconfiguredCRC" in str(e))


# %% [markdown]
# ## 10.6 CRC-8 against datasheet vectors
#
# The SHTC3 and TMAG5273 parameter sets, against the vectors their datasheets
# print. Two different polynomials on the same implementation, which is what
# makes this a test of the parameterisation rather than of one hard-coded
# algorithm.

# %%
class SHTC3_CRC(CRCMixin):
    """Sensirion convention: CRC-8, x^8+x^5+x^4+1, init 0xFF (SHTC3 Table 16)."""
    CRC_WIDTH = 8
    CRC_POLY  = 0x31
    CRC_INIT  = 0xFF


class TMAG_CRC(CRCMixin):
    """TI TMAG5273: CRC-8, x^8+x^2+x+1, init 0xFF (SLYS045 section 7.4.1)."""
    CRC_WIDTH = 8
    CRC_POLY  = 0x07
    CRC_INIT  = 0xFF


print("SHTC3 — poly 0x31, init 0xFF")
for data, want in ((b"\x00", 0xAC), (b"\xBE\xEF", 0x92)):
    got = SHTC3_CRC.crc(data)
    check("CRC({})".format(data.hex().upper()), got == want,
          "{:#04x}, want {:#04x}".format(got, want))

print()
print("TMAG5273 — poly 0x07, init 0xFF")
for data, want in ((b"\xE0", 0x5D),
                   (b"\x00\x00\x00\x00", 0xD1),
                   (b"\xFF\xFF\xFF\xFF", 0x0F)):
    got = TMAG_CRC.crc(data)
    check("CRC({})".format(data.hex().upper()), got == want,
          "{:#04x}, want {:#04x}".format(got, want))

print()
# The two must disagree on the same input, or the polynomial isn't being used.
check("polynomials actually differ",
      SHTC3_CRC.crc(b"\x00") != TMAG_CRC.crc(b"\x00"),
      "0x31 -> {:#04x}, 0x07 -> {:#04x}".format(
          SHTC3_CRC.crc(b"\x00"), TMAG_CRC.crc(b"\x00")))

print()
# CommandI2CBase inherits the Sensirion defaults, so it must agree with the
# standalone SHTC3 class — this is the compatibility check on the reparenting.
check("CommandI2CBase defaults match SHTC3",
      CommandI2CBase.crc(b"\x00") == 0xAC and CommandI2CBase.crc(b"\xBE\xEF") == 0x92,
      "poly {:#04x}, init {:#04x}".format(CommandI2CBase.CRC_POLY, CommandI2CBase.CRC_INIT))

check("crc8 alias still works",
      CommandI2CBase.crc8(b"\xBE\xEF") == CommandI2CBase.crc(b"\xBE\xEF") == 0x92)


# %% [markdown]
# ## 10.7 The standard check value
#
# Every catalogued CRC variant publishes its output for the ASCII string
# `123456789`. Four are checked here, chosen to span the parameter space rather
# than to cover any particular part:
#
# | Variant | Width | Poly | Init | Reflect | XOR out | Check |
# |---|---|---|---|---|---|---|
# | CRC-8/SMBUS | 8 | 0x07 | 0x00 | no | 0x00 | 0xF4 |
# | CRC-8/MAXIM | 8 | 0x31 | 0x00 | yes | 0x00 | 0xA1 |
# | CRC-16/CCITT-FALSE | 16 | 0x1021 | 0xFFFF | no | 0x0000 | 0x29B1 |
# | CRC-16/ARC | 16 | 0x8005 | 0x0000 | yes | 0x0000 | 0xBB3D |
#
# The reflected pairs are the ones that matter most. Reflection implemented
# backwards still produces stable, self-consistent checksums, so nothing
# short of an external vector will catch it.

# %%
CHECK_INPUT = b"123456789"


class CRC8_SMBUS(CRCMixin):
    CRC_WIDTH = 8
    CRC_POLY  = 0x07
    CRC_INIT  = 0x00


class CRC8_MAXIM(CRCMixin):
    CRC_WIDTH       = 8
    CRC_POLY        = 0x31
    CRC_INIT        = 0x00
    CRC_REFLECT_IN  = True
    CRC_REFLECT_OUT = True


class CRC16_CCITT_FALSE(CRCMixin):
    CRC_WIDTH = 16
    CRC_POLY  = 0x1021
    CRC_INIT  = 0xFFFF


class CRC16_ARC(CRCMixin):
    CRC_WIDTH       = 16
    CRC_POLY        = 0x8005
    CRC_INIT        = 0x0000
    CRC_REFLECT_IN  = True
    CRC_REFLECT_OUT = True


for name, cls, want in (("CRC-8/SMBUS", CRC8_SMBUS, 0xF4),
                        ("CRC-8/MAXIM", CRC8_MAXIM, 0xA1),
                        ("CRC-16/CCITT-FALSE", CRC16_CCITT_FALSE, 0x29B1),
                        ("CRC-16/ARC", CRC16_ARC, 0xBB3D)):
    got = cls.crc(CHECK_INPUT)
    check(name, got == want, "{:#06x}, want {:#06x}".format(got, want))

print()
# Width must actually constrain the result.
check("8-bit result fits 8 bits", CRC8_SMBUS.crc(CHECK_INPUT) <= 0xFF)
check("16-bit result fits 16 bits", CRC16_ARC.crc(CHECK_INPUT) <= 0xFFFF)
check("crc_mask tracks width",
      CRC8_SMBUS.crc_mask() == 0xFF and CRC16_ARC.crc_mask() == 0xFFFF)
check("crc_topbit tracks width",
      CRC8_SMBUS.crc_topbit() == 0x80 and CRC16_ARC.crc_topbit() == 0x8000)


# %% [markdown]
# ## 10.8 `CRC_XOR_OUT` and reflection, in isolation
#
# Each parameter changed on its own, against a fixed baseline, so a parameter
# that is silently ignored shows up as an unchanged result rather than being
# masked by another difference.

# %%
class Base(CRCMixin):
    CRC_WIDTH = 8
    CRC_POLY  = 0x07
    CRC_INIT  = 0xFF


class XorOut(Base):
    CRC_XOR_OUT = 0xFF


class ReflIn(Base):
    CRC_REFLECT_IN = True


class ReflOut(Base):
    CRC_REFLECT_OUT = True


data = b"\x12\x34\x56"
base = Base.crc(data)
show(base, "baseline", 1)
print()

check("XOR_OUT applied", XorOut.crc(data) == base ^ 0xFF,
      "{:#04x} -> {:#04x}".format(base, XorOut.crc(data)))
check("REFLECT_OUT applied", ReflOut.crc(data) == BitTools.bit_reverse(base, 8),
      "{:#04x} -> {:#04x}".format(base, ReflOut.crc(data)))
check("REFLECT_IN changes result", ReflIn.crc(data) != base,
      "{:#04x} -> {:#04x}".format(base, ReflIn.crc(data)))

print()
# Identity settings must be no-ops, so a driver leaving them at their defaults
# is definitely unaffected.
class Identity(Base):
    CRC_XOR_OUT     = 0x00
    CRC_REFLECT_IN  = False
    CRC_REFLECT_OUT = False


check("identity parameters are no-ops", Identity.crc(data) == base)

print()
# `init` overrides the class attribute, for resuming across transfers.
check("init argument overrides", Base.crc(data, init=0x00) != base,
      "init 0xFF -> {:#04x}, init 0x00 -> {:#04x}".format(base, Base.crc(data, init=0x00)))

# A resumed checksum, split across two calls, must equal the one-shot result.
# Only valid with identity reflection and XOR, which is why it is checked on
# Base rather than on a reflected variant.
split = Base.crc(data[2:], init=Base.crc(data[:2]))
check("resumable across calls", split == base,
      "split {:#04x}, whole {:#04x}".format(split, base))


# %% [markdown]
# ## 10.9 Table acceleration
#
# The table path is an optimisation, so the only thing that matters is that it
# is indistinguishable from the bitwise path. Checked across every parameter
# combination the notebook cares about, over a spread of input lengths.
#
# Also checked: the per-class table cache. A subclass that narrows the
# polynomial must build its own table rather than inheriting one computed for
# its parent — which is why `crc_table` looks in `cls.__dict__` rather than
# using ordinary attribute lookup.

# %%
PARAM_SETS = (("CRC-8/SMBUS", 8, 0x07, 0x00, False, False, 0x00),
              ("SHTC3", 8, 0x31, 0xFF, False, False, 0x00),
              ("TMAG5273", 8, 0x07, 0xFF, False, False, 0x00),
              ("CRC-8/MAXIM", 8, 0x31, 0x00, True, True, 0x00),
              ("CRC-16/CCITT", 16, 0x1021, 0xFFFF, False, False, 0x0000),
              ("CRC-16/ARC", 16, 0x8005, 0x0000, True, True, 0x0000))

# A fixed corpus rather than random data, so a failure is reproducible.
CORPUS = (b"\x00", b"\xFF", b"\x12\x34", b"\xBE\xEF", b"123456789",
          b"\x00\x00\x00\x00", b"\xFF\xFF\xFF\xFF",
          bytes(range(32)), bytes(range(255, 223, -1)))

for name, width, poly, init, ri, ro, xo in PARAM_SETS:
    ns = dict(CRC_WIDTH=width, CRC_POLY=poly, CRC_INIT=init,
              CRC_REFLECT_IN=ri, CRC_REFLECT_OUT=ro, CRC_XOR_OUT=xo)

    Bitwise = type("Bitwise_" + name.replace("-", "_").replace("/", "_"),
                   (CRCMixin,), dict(ns, CRC_TABLE_ENABLE=False))
    Tabled  = type("Tabled_" + name.replace("-", "_").replace("/", "_"),
                   (CRCMixin,), dict(ns, CRC_TABLE_ENABLE=True))

    mismatch = None
    for data in CORPUS:
        if Bitwise.crc(data) != Tabled.crc(data):
            mismatch = data
            break

    check(name, mismatch is None,
          "{} inputs".format(len(CORPUS)) if mismatch is None
          else "differs on {}".format(mismatch.hex()))

print()
# Table shape and laziness.
class Tbl(CRCMixin):
    CRC_POLY = 0x07
    CRC_TABLE_ENABLE = True


check("table not built until used", "_crc_tbl" not in Tbl.__dict__)
Tbl.crc(b"\x00")
check("table built on first use", "_crc_tbl" in Tbl.__dict__)
check("256 entries", len(Tbl.crc_table()) == 256)
check("cached, not rebuilt", Tbl.crc_table() is Tbl.crc_table())

Tbl.crc_table_clear()
check("crc_table_clear drops it", "_crc_tbl" not in Tbl.__dict__)
check("rebuilds on demand", len(Tbl.crc_table()) == 256)

print()
# The subclass trap: a child narrowing the polynomial must not inherit the
# parent's table.
class Parent(CRCMixin):
    CRC_POLY = 0x07
    CRC_INIT = 0xFF
    CRC_TABLE_ENABLE = True


class Child(Parent):
    CRC_POLY = 0x31            # Sensirion, not TI


Parent.crc(b"\x00")            # force the parent's table into existence
Child.crc(b"\x00")             # child must build its own

check("child has its own table", Parent.crc_table() is not Child.crc_table())
check("child result is correct", Child.crc(b"\x00") == 0xAC,
      "{:#04x}, want 0xAC (SHTC3 vector)".format(Child.crc(b"\x00")))
check("parent unaffected", Parent.crc(b"\xE0") == 0x5D,
      "{:#04x}, want 0x5D (TMAG vector)".format(Parent.crc(b"\xE0")))


# %% [markdown]
# ## 10.10 `crc_ok` and `crc_verify`
#
# The two comparison wrappers. `crc_verify` is the one that belongs on a data
# path: it raises, so a caller who forgets to test the result gets a loud
# failure rather than a plausible number.

# %%
good = SHTC3_CRC.crc(b"\xBE\xEF")

check("crc_ok true on match", SHTC3_CRC.crc_ok(b"\xBE\xEF", good) is True)
check("crc_ok false on mismatch", SHTC3_CRC.crc_ok(b"\xBE\xEF", good ^ 0xFF) is False)

print()
check("crc_verify returns on match", SHTC3_CRC.crc_verify(b"\xBE\xEF", good) == good)

try:
    SHTC3_CRC.crc_verify(b"\xBE\xEF", good ^ 0xFF)
    check("crc_verify raises on mismatch", False, "it did not")
except ValueError as e:
    check("crc_verify raises on mismatch", True)
    print("        ", e)

print()
# The context string locates the failure for the caller.
try:
    SHTC3_CRC.crc_verify(b"\xBE\xEF", 0x00, context="word 3")
except ValueError as e:
    check("context appears in message", "word 3" in str(e))
    print("        ", e)

print()
# Single-bit corruption in the data must be caught — the entire point.
ok = True
for bit in range(16):
    corrupt = bytes([0xBE ^ ((1 << bit) >> 8 & 0xFF), 0xEF ^ ((1 << bit) & 0xFF)])
    if corrupt != b"\xBE\xEF" and SHTC3_CRC.crc_ok(corrupt, good):
        ok = False
        print("      missed corruption: {}".format(corrupt.hex()))
check("catches every single-bit flip", ok, "16 positions")


# %% [markdown]
# ## 10.11 Simulated packet decode
#
# The two wire formats from section 4.2, decoded end to end with no hardware.
# This is the closest the notebook currently gets to exercising
# `CommandI2CBase.words_read`, and it is the pattern a TMAG5273 driver will
# follow for its fast-read mode.
#
# The point is the **scope** difference: Sensirion checksums each word, TI
# checksums the whole packet including the command byte the host sent. Both use
# the same `crc` primitive with different inputs, which is exactly the division
# of labour section 4.2 argues for.

# %%
print("Sensirion layout — [MSB, LSB, CRC] per word")
print()

words = (0xBEEF, 0x1234, 0x0000)
packet = b""
for w in words:
    pair = BitTools.int_to_bytes(w, 2, True)
    packet += pair + bytes([SHTC3_CRC.crc(pair)])

print("  packet:", packet.hex())
check("3n bytes", len(packet) == 3 * len(words), "{} bytes".format(len(packet)))

decoded = []
for i in range(len(words)):
    msb, lsb, crc = packet[i*3], packet[i*3+1], packet[i*3+2]
    SHTC3_CRC.crc_verify(bytes([msb, lsb]), crc, "word {}".format(i))
    decoded.append((msb << 8) | lsb)

check("all words decode", tuple(decoded) == words,
      " ".join("{:#06x}".format(w) for w in decoded))

print()
# Corrupt one word and confirm the failure names it.
bad = bytearray(packet)
bad[4] ^= 0x01                                  # flip a bit in word 1's LSB
try:
    for i in range(len(words)):
        msb, lsb, crc = bad[i*3], bad[i*3+1], bad[i*3+2]
        SHTC3_CRC.crc_verify(bytes([msb, lsb]), crc, "word {}".format(i))
    check("corruption detected", False, "it was not")
except ValueError as e:
    check("corruption detected", True)
    print("        ", e)

print()
print("TI layout — whole packet, command byte included, one trailing CRC")
print()

# TMAG5273 fast-read: the command byte is (addr << 1) | 1, then the enabled
# channels, then CONV_STATUS, then one CRC over all of it.
cmd_byte = (0x35 << 1) | 1                      # default address, read
payload  = bytes([0x12, 0x34, 0x56, 0x78, 0x10])   # X_MSB..Z_LSB, CONV_STATUS
covered  = bytes([cmd_byte]) + payload
tmag_packet = payload + bytes([TMAG_CRC.crc(covered)])

print("  command:", hex(cmd_byte))
print("  packet: ", tmag_packet.hex())

check("whole packet verifies",
      TMAG_CRC.crc_ok(bytes([cmd_byte]) + tmag_packet[:-1], tmag_packet[-1]))

# Omitting the command byte must fail — the trap this layout sets.
check("omitting command byte fails",
      not TMAG_CRC.crc_ok(tmag_packet[:-1], tmag_packet[-1]),
      "the command byte is part of the covered data")

print()
# And a corrupted payload is caught.
bad = bytearray(tmag_packet)
bad[2] ^= 0x01
check("payload corruption detected",
      not TMAG_CRC.crc_ok(bytes([cmd_byte]) + bytes(bad[:-1]), bad[-1]))


# %% [markdown]
# ## 10.12 Summary
#
# If every check above passed, both helper classes are verified against
# published vectors from three independent sources, across both widths, both
# reflection settings, and both the bitwise and table code paths.
#
# What that establishes:
#
# * `sign_extend` is correct at every boundary, and the shift-before-extend
#   ordering the TMP102 driver relies on is confirmed, along with the failure it
#   prevents.
# * The field helpers are inverses, never disturb neighbours, and clamp
#   oversized input — the properties `field_set` depends on.
# * Byte assembly round-trips in both endiannesses at every width, and truncates
#   rather than raising, matching the behaviour `reg_write` has always had.
# * `CRCMixin` reproduces the SHTC3 and TMAG5273 datasheet vectors and four
#   standard check values, and refuses to compute anything at all without a
#   declared polynomial.
# * The table path is indistinguishable from the bitwise path, and a subclass
#   that narrows the parameters gets its own table.
# * `CommandI2CBase` still produces the Sensirion checksums it always did, and
#   the `crc8` alias still resolves — so the reparenting is backward compatible.
#
# What it does **not** establish: that any real part agrees. The TMAG5273
# vectors in particular come from a datasheet whose OCR is visibly damaged, and
# nothing here has touched silicon. Confirm against a live part before trusting
# a CRC on a data path.

# %%
print("BitTools     — sign extension, fields, reflection, byte assembly")
print("CRCMixin     — arbitrary width, full parameter model, table acceleration")
print()
print("vector sources")
print("  SHTC3 datasheet Table 16          poly 0x31 init 0xFF")
print("  TMAG5273 SLYS045 section 7.4.1    poly 0x07 init 0xFF")
print("  standard '123456789' check value  4 catalogued variants")
print()
print("still unverified against hardware: every CRC parameter set above")


# %% [markdown]
# # 11. Status and Next Steps
#
# ## What this notebook provides
#
# * **Locking** — a per-bus mutex with a null-lock fallback for non-threaded
#   builds, so the same code runs on either.
# * **`I2CConnection`** — an address-and-path-bound handle that makes mux
#   selection and bus arbitration atomic and invisible.
# * **`I2CNode` / `I2CDevice` / `I2CMux`** — the datasheet-facts contract and the
#   four-method transport contract, in both routed and bare-bus flavours.
# * **`BitTools`** — stateless bit and byte helpers with no class attributes and
#   no contract, verified offline in section 10.
# * **`CRCMixin`** — arbitrary-width CRC parameterised entirely by class
#   attributes, with an unset polynomial as a hard error, verified in section 10
#   against three independent vector sources.
# * **`RegisterI2CBase` / `CommandI2CBase`** — protocol machinery that composes
#   with either node type, verified against hardware in section 7.
# * **`TMP102`** — a complete driver, verified in section 9, serving as the
#   worked example for everything above.
#
# ## What is not here yet
#
# * **A part using `CRCMixin` against hardware.** Section 10 verifies the
#   arithmetic offline, but no real device has confirmed a checksum yet. The
#   SHTC3 (per-word) and TMAG5273 (whole-packet, command byte included) are the
#   two obvious first targets, and between them they exercise both scopes.
# * **A stream-read mixin.** `CommandI2CBase.words_read` hardcodes the
#   `[MSB, LSB, CRC]` layout. The TMAG5273's 8-bit fast-read returns one byte
#   per channel plus a status byte — not words, not a multiple of three — so it
#   needs a sibling class rather than a wider `words_read`. Section 10.11
#   sketches the decode by hand; that is what wants generalising.
# * **Command timing helpers.** Every Sensirion driver will write the same
#   `cmd_send` → `sleep_ms` → `words_read` sequence. A `measure(opcode,
#   delay_ms, n_words)` helper would collapse it, but the signature should wait
#   until a second command-driven part exists to generalise from.
# * **Repeated-start reads.** `read_raw` after `write_raw` issues a STOP between
#   them. Most command parts tolerate it; some require repeated-start, which
#   would need a combined write-then-read on `I2CConnection`.
# * **The bus manager.** The piece that owns a `machine.I2C`, mints the one lock
#   per bus, walks the topology map, instantiates muxes, resolves human-readable
#   locations into `[[mux_addr, channel], ...]` paths, and hands out built
#   `I2CConnection` objects. The `lock=None` injection points throughout exist
#   for it.
# * **The setup/config map format.** The declarative description of the topology
#   that the manager consumes.
# * **Auto baud negotiation.** Reconciling `BAUD_MIN` / `BAUD_MAX` /
#   `BAUD_NOMINAL` across everything sharing a bus into one workable clock rate.
#   `baud_ok()` is the per-part half of this.
# * **Bus scanning and identification.** Using `addr_ok()` to match discovered
#   addresses against candidate part classes. Note the TMP102 shows why this is
#   hard: no device-ID register means identification is circumstantial, and
#   0x48 is a crowded address.
# * **Concrete mux drivers.** A TCA9548A implementing the `channel_select` /
#   `channel_deselect` / `control_read` contract is the obvious first one, and
#   would exercise the mux path that section 7 could not.
#
# ## Known rough edges
#
# * **The cache stores no width.** Keyed by register number alone, so mixing
#   `nbytes` across calls to the same register returns a stale-width value. A
#   non-issue on uniform-width parts like the TMP102; a real trap on a part with
#   mixed 8- and 16-bit registers. Storing `(value, nbytes)` and invalidating on
#   a width mismatch would fix it.
# * **Sub-byte CRC widths are untested.** `CRC_WIDTH` below 8 — CRC-5, some
#   SMBus PEC variants — takes the negative-shift branches in `crc` and
#   `crc_table`, which are written but exercised by nothing in section 10. Leave
#   `CRC_TABLE_ENABLE` off there until a real vector confirms it.
# * **`celsius_to_raw` uses floats.** Fine on ESP32/RP2040, but a target without
#   float support would need an integer-only path. The codecs are isolated as
#   classmethods partly for that reason.
# * **`VERIFY_MASK` is class-wide.** It suits the config register, so every
#   other register on the part has to opt back out — hence `LIMIT_VERIFY_MASK`
#   passed explicitly at four call sites. A per-register mask table on the
#   driver would be tidier if a part had more than two distinct patterns.
#
# ## Notes for driver authors
#
# * Subclass `I2CDevice` and mix in the protocol layer matching the part.
# * Override the datasheet class attributes — `ALLOWED_ADDRS`, `BAUD_*`, and
#   `BIG_ENDIAN` / `REG_BYTES` / `VERIFY_MASK` or the `CMD_*` / `CRC_*` set.
# * A register-mapped part that also checksums mixes in both:
#   `class Part(RegisterI2CBase, CRCMixin, I2CDevice)`. `BitTools` comes free
#   with either, so never import it explicitly.
# * Declare `CRC_POLY` from the datasheet's polynomial, dropping the x^n term:
#   x⁸+x²+x+1 is `0x07`, x⁸+x⁵+x⁴+1 is `0x31`. There is no default, and there
#   should not be.
# * Decide the checksum *scope* in the driver, not the mixin. Feed `crc_verify`
#   exactly the bytes the datasheet says are covered — which for TI parts
#   includes the command byte the host sent.
# * Declare bitfields as `(shift, width)` class tuples and coded fields as
#   `{code: value}` dicts. Never open-code a shift in a method body; use
#   `field_extract` / `field_insert` if you need the arithmetic without a
#   transfer.
# * Use `sign_extend` rather than reimplementing it, and remember to strip any
#   padding *before* calling it.
# * Never reach for `self.conn` or `self.bus` from driver code. Use the four
#   transport methods and the driver stays portable between devices and muxes.
# * Do not take the lock yourself. It is not reentrant on most ports, and every
#   transport method already takes it — locking around a transfer will deadlock.
# * Use `reg_set` over `reg_write` unless the read-back is genuinely unwanted,
#   and be conservative about what you allow to be cached.
# * Build a `*Proto` shell first. A datasheet-facts-only class lets you verify
#   the mixin against real hardware before any driver logic exists to confuse
#   the diagnosis.


# %% [markdown]
# # TMAG5273 — a Part That Needs Both Mixins
#
# **Append this after the TMP102 section.** It assumes `I2CConnection`,
# `I2CDevice`, `BitTools`, `CRCMixin`, `RegisterI2CBase`, the `show` / `check`
# display helpers, and the live `bus` / `bus_lock` objects are already defined.
#
# ## Why this part, and why now
#
# The TMP102 exercised `RegisterI2CBase` and nothing else. The TMAG5273 is the
# first part in this notebook that needs **both** protocol layers at once, and
# it is the first thing to put `CRCMixin` in front of real silicon rather than
# offline vectors.
#
# It earns its place for three reasons:
#
# 1. **It is genuinely dual-mode.** `I2C_RD` switches the part between an
#    ordinary register map and a contentless fast-read that streams a
#    config-determined packet. The same driver has to speak both.
# 2. **Its checksum scope is the awkward one.** The CRC covers the entire
#    packet *including the command byte the host sent* — not per-word like
#    Sensirion, and not just the returned bytes. That is exactly the case
#    section 4.2 argued the mixin should not try to infer.
# 3. **It has a device ID.** Unlike the TMP102, identification is not
#    circumstantial: `MANUFACTURER_ID` is a fixed 16-bit constant and
#    `DEVICE_ID` names the variant. Bring-up can actually confirm what it is
#    talking to.
#
# ## The part
#
# TI TMAG5273 (SLYS045, June 2021 — marked ADVANCE INFORMATION). A three-axis
# Hall-effect magnetometer with an on-die temperature sensor, hardware angle and
# magnitude computation between any two axes, per-axis thresholds with
# interrupt, and several duty-cycled power modes.
#
# This driver targets the **A1** variant, which is what SparkFun's Qwiic board
# carries. A2 differs only in magnetic range (±133/±266 mT rather than ±40/±80),
# so supporting it later is a table entry and a `VER` check, not a new class.
#
# ## Wire formats
#
# `I2C_RD` in `DEVICE_CONFIG_1` selects one of three read behaviours:
#
# | Code | Mode | Wire |
# |---|---|---|
# | `0h` | Standard register read | pointer byte, repeated start, data — auto-increments |
# | `1h` | 16-bit fast read | address byte only; streams enabled channels + `CONV_STATUS` |
# | `2h` | 8-bit fast read | as above, MSB byte per channel |
#
# **Writes are always register-based**, whatever `I2C_RD` says. Only the read
# path changes. So the driver configures the part through `RegisterI2CBase` in
# every mode, and fast-read is purely an optimisation on the data path.
#
# In the fast-read modes the command byte carries no register number — what
# comes back is determined entirely by `MAG_CH_EN`, `T_CH_EN` and `CRC_EN`,
# which the host set earlier. The packet length is therefore *implicit*, and the
# driver has to compute it from configuration it wrote. Getting that wrong
# clocks out garbage or NACKs early.
#
# One restriction from the datasheet: in 16-bit fast read with CRC enabled, at
# most three channels can be sent. The 8-bit mode has no such limit.
#
# ## Checksum
#
# CRC-8, polynomial x⁸+x²+x+1 (`0x07`), init `0xFF`, no reflection, no final
# XOR — the CRC-8/SMBUS parameters but with a non-zero seed. Enabled by
# `CRC_EN`. One byte, appended after the data, computed over the command byte
# *and* every returned data byte.
#
# Section 10.6 of the main notebook already checks this parameter set against
# the three vectors the datasheet publishes. What has not happened yet is a
# live part agreeing, which is what section T.7 below is for.
#
# ## Data formats
#
# * **Magnetic** — 16-bit two's complement, `B = raw / 2¹⁶ × 2|B_R|` mT, where
#   `B_R` is the per-axis range. At ±40 mT that is 819.2 LSB/mT, which rounds to
#   the 820 the datasheet's Electrical Characteristics table states.
# * **Temperature** — 16-bit, `T = 25 + (raw − 17508) / 60.1` °C. Not two's
#   complement: it is an unsigned ADC code with a reference point, so negative
#   temperatures come out of the subtraction rather than the sign bit.
# * **Angle** — 13 bits: `D12:D4` integer degrees 0–360, `D3:D0` sixteenths of a
#   degree. Top three bits always read zero.
# * **Offset** — 8-bit two's complement, `Δ = raw / 2¹² × 2|B_R|` mT.
#
# ## Structure of this section
#
# | Cell | Contents |
# |---|---|
# | T.1 | `TMAG5273Proto` — datasheet facts only, the section 6 pattern |
# | T.2 | Bind and identify |
# | T.3 | `RegisterI2CBase` against a second, differently-shaped part |
# | T.4 | `TMAG5273` — the driver |
# | T.5 | Offline codec verification |
# | T.6 | Live conversion and configuration |
# | T.7 | CRC against hardware — the first live test of `CRCMixin` |
# | T.8 | Fast-read modes |
# | T.9 | Angle and magnitude |
# | T.10 | Restore and summary |

# %% [markdown]
# ## T.1 `TMAG5273Proto` — datasheet facts only
#
# Same pattern as `TMP102Proto`: no methods, so a failure in T.3 is the mixin,
# the connection layer or the wiring, never driver logic.
#
# Worth noting what is different from the TMP102 and why it exercises the mixin
# harder:
#
# * **8-bit registers.** `REG_BYTES = 1` here, against the TMP102's 2. The
#   16-bit results are two adjacent 8-bit registers, read as one 2-byte
#   transfer by passing `nbytes=2` — which is the per-call width override the
#   mixin has always had but that no part has used until now.
# * **A crowded reset map.** Most registers reset to `0x00`, but `I2C_ADDRESS`
#   resets to `0x6A` and `MANUFACTURER_ID` is a fixed constant. Both are useful
#   bring-up anchors.
# * **Write-1-to-clear status bits.** `POR`, `OSC_ER`, `INT_ER`, `OTP_CRC_ER`
#   and `VCC_UV_ER` are `R/W1CP`. Writing them back is how you clear them,
#   which means they cannot participate in a verified write — the first part in
#   this notebook to need `VERIFY_MASK` for a reason other than read-only bits.

# %%
class TMAG5273Proto(RegisterI2CBase, CRCMixin, I2CDevice):
    """Datasheet facts for the TI TMAG5273A1, with no driver logic whatsoever.

    A deliberately method-free shell, used in T.3 to exercise
    `RegisterI2CBase` against a second part with a different register shape
    from the TMP102 -- 8-bit registers, a 16-bit ID constant, and write-1-to-
    clear status bits.

    Everything here is a class attribute taken from TI SLYS045. Because there
    are no methods, a failure in T.3 can only be the mixin, the connection
    layer, or the wiring.

    Mixes in `CRCMixin` as well as `RegisterI2CBase`, which is what makes this
    the first composition of both protocol layers in the notebook. The CRC
    parameters are declared here so that T.7 can check them against the part
    without any driver code in between.
    """

    # --- datasheet facts (I2CNode contract) ---
    #
    # The factory address is loaded from OTP and varies by orderable part
    # number; the register reset value 0x6A corresponds to 0x35, which is what
    # SparkFun's Qwiic board ships. Any 7-bit address can be programmed at
    # runtime through I2C_ADDRESS, so the allowed set is deliberately wide --
    # narrow it if a topology map needs stricter validation.
    ALLOWED_ADDRS = tuple(range(0x08, 0x78))   # 7-bit programmable via I2C_ADDRESS
    ADDR_DEFAULT  = 0x35                       # I2C_ADDRESS reset 0x6A >> 1

    BAUD_NOMINAL  = 400_000                    # Fast mode, section 6.10
    BAUD_MAX      = 1_000_000                  # Fast Mode Plus, needs VCC >= 2.3 V
    BAUD_MIN      = 10_000                     # no stated floor; margin here

    BIG_ENDIAN    = True                       # multi-byte results are MSB first
    REG_BYTES     = 1                          # every register is 8-bit

    # --- register map (Table 7-6) ---
    REG_DEVICE_CONFIG_1      = 0x00
    REG_DEVICE_CONFIG_2      = 0x01
    REG_SENSOR_CONFIG_1      = 0x02
    REG_SENSOR_CONFIG_2      = 0x03
    REG_X_THR_CONFIG         = 0x04
    REG_Y_THR_CONFIG         = 0x05
    REG_Z_THR_CONFIG         = 0x06
    REG_T_CONFIG             = 0x07
    REG_INT_CONFIG_1         = 0x08
    REG_MAG_GAIN_CONFIG      = 0x09
    REG_MAG_OFFSET_CONFIG_1  = 0x0A
    REG_MAG_OFFSET_CONFIG_2  = 0x0B
    REG_I2C_ADDRESS          = 0x0C
    REG_DEVICE_ID            = 0x0D
    REG_MANUFACTURER_ID_LSB  = 0x0E
    REG_MANUFACTURER_ID_MSB  = 0x0F
    REG_T_MSB_RESULT         = 0x10
    REG_T_LSB_RESULT         = 0x11
    REG_X_MSB_RESULT         = 0x12
    REG_X_LSB_RESULT         = 0x13
    REG_Y_MSB_RESULT         = 0x14
    REG_Y_LSB_RESULT         = 0x15
    REG_Z_MSB_RESULT         = 0x16
    REG_Z_LSB_RESULT         = 0x17
    REG_CONV_STATUS          = 0x18
    REG_ANGLE_RESULT_MSB     = 0x19
    REG_ANGLE_RESULT_LSB     = 0x1A
    REG_MAGNITUDE_RESULT     = 0x1B
    REG_DEVICE_STATUS        = 0x1C

    # --- DEVICE_CONFIG_1 fields (Table 7-8) ---
    F_CRC_EN      = (7, 1)   # 0 = CRC disabled, 1 = enabled
    F_MAG_TEMPCO  = (5, 2)   # magnet temperature coefficient
    F_CONV_AVG    = (2, 3)   # oversampling / averaging
    F_I2C_RD      = (0, 2)   # read mode: standard / 16-bit fast / 8-bit fast

    # --- DEVICE_CONFIG_2 fields (Table 7-9) ---
    F_THR_HYST      = (5, 3)
    F_LP_LN         = (4, 1)   # 0 = low active current, 1 = low noise
    F_GLITCH_FILTER = (3, 1)   # 0 = filter on, 1 = filter off
    F_TRIGGER_MODE  = (2, 1)   # 0 = I2C command, 1 = INT pin
    F_OPERATING_MODE = (0, 2)  # standby / sleep / continuous / duty-cycled

    # --- SENSOR_CONFIG_1 fields (Table 7-10) ---
    F_MAG_CH_EN  = (4, 4)      # which magnetic channels convert
    F_SLEEPTIME  = (0, 4)      # duty-cycled sleep interval

    # --- SENSOR_CONFIG_2 fields (Table 7-11) ---
    F_THRX_COUNT  = (6, 1)
    F_MAG_THR_DIR = (5, 1)
    F_MAG_GAIN_CH = (4, 1)
    F_ANGLE_EN    = (2, 2)     # which axis pair feeds the angle engine
    F_X_Y_RANGE   = (1, 1)     # 0 = +-40 mT, 1 = +-80 mT (A1)
    F_Z_RANGE     = (0, 1)     # 0 = +-40 mT, 1 = +-80 mT (A1)

    # --- T_CONFIG fields (Table 7-15) ---
    F_T_THR_CONFIG = (1, 7)
    F_T_CH_EN      = (0, 1)    # temperature channel enable

    # --- INT_CONFIG_1 fields (Table 7-16) ---
    F_RSLT_INT   = (7, 1)
    F_THRSLD_INT = (6, 1)
    F_INT_STATE  = (5, 1)      # 0 = latched, 1 = 10 us pulse
    F_INT_MODE   = (2, 3)
    F_MASK_INTB  = (0, 1)

    # --- DEVICE_ID fields (Table 7-21) ---
    F_VER = (0, 2)             # 1 = A1 (+-40/+-80 mT), 2 = A2 (+-133/+-266 mT)

    # --- CONV_STATUS fields (Table 7-32) ---
    F_SET_COUNT     = (5, 3)   # rolling count of completed conversion sets
    F_POR           = (4, 1)   # W1C: power-on reset occurred
    F_DIAG_STATUS   = (1, 1)   # any internal diagnostic failure
    F_RESULT_STATUS = (0, 1)   # 1 = conversion data complete

    # --- DEVICE_STATUS fields (Table 7-36) ---
    F_INTB_RB     = (4, 1)     # level read back from the INT pin
    F_OSC_ER      = (3, 1)     # W1C
    F_INT_ER      = (2, 1)     # W1C
    F_OTP_CRC_ER  = (1, 1)     # W1C
    F_VCC_UV_ER   = (0, 1)     # W1C

    # --- code tables ---
    CONV_AVG = {0: 1, 1: 2, 2: 4, 3: 8, 4: 16, 5: 32}      # samples averaged
    MAG_CH   = {0x0: "",    0x1: "X",   0x2: "Y",   0x3: "XY",
                0x4: "Z",   0x5: "ZX",  0x6: "YZ",  0x7: "XYZ",
                0x8: "XYX", 0x9: "YXY", 0xA: "YZY", 0xB: "XZX"}
    OP_MODE  = {0: "standby", 1: "sleep", 2: "continuous", 3: "wake_sleep"}
    I2C_RD   = {0: "standard", 1: "fast16", 2: "fast8"}
    TEMPCO   = {0: 0.0, 1: 0.12, 3: 0.2}                    # %/degC, 2 reserved
    ANGLE_EN = {0: "off", 1: "XY", 2: "YZ", 3: "XZ"}
    SLEEPTIME = {0x0: 1, 0x1: 5, 0x2: 10, 0x3: 15, 0x4: 20, 0x5: 30,
                 0x6: 50, 0x7: 100, 0x8: 500, 0x9: 1000, 0xA: 2000,
                 0xB: 5000, 0xC: 20000}                      # ms

    # --- identification ---
    MANUFACTURER_ID = 0x5449                 # 'TI' in ASCII, fixed
    VER_A1 = 1
    VER_A2 = 2

    # --- reset values worth asserting during bring-up ---
    I2C_ADDRESS_POR = 0x6A                   # 0x35 << 1
    CONFIG_POR      = 0x00                   # every config register

    # --- CRC (section 7.4.1) ---
    # x^8 + x^2 + x + 1, init 0xFF, no reflection, no final XOR. Same
    # polynomial as CRC-8/SMBUS but seeded 0xFF rather than 0x00.
    CRC_WIDTH = 8
    CRC_POLY  = 0x07
    CRC_INIT  = 0xFF

    # Datasheet vectors, section 7.4.1. Checked offline in the main notebook's
    # section 10.6 and against the live part in T.7.
    CRC_VECTORS = ((b"\xE0", 0x5D),
                   (b"\x00\x00\x00\x00", 0xD1),
                   (b"\xFF\xFF\xFF\xFF", 0x0F))

    # --- magnetic ranges, A1 (section 6.7) ---
    # {range bit: full-scale mT}. A2 would be {0: 133.0, 1: 266.0}.
    RANGE_A1 = {0: 40.0, 1: 80.0}
    RANGE_A2 = {0: 133.0, 1: 266.0}

    # --- temperature (section 6.6) ---
    T_ADC_T0   = 17508                       # ADC code at the reference point
    T_SENS_T0  = 25.0                        # degC at that code
    T_ADC_RES  = 60.1                        # LSB per degC
    TEMP_MIN   = -40.0
    TEMP_MAX   = 170.0

    # --- timing (section 6.11) ---
    T_STARTUP_US   = 270                     # power-up to standby
    T_WAKE_US      = 50                      # sleep to standby
    T_MEASURE_US   = 50                      # CONV_AVG=000b, one channel
    T_PER_CHAN_US  = 25                      # each extra channel at CONV_AVG=000b
    T_MEASURE_MAX_US = 825                   # CONV_AVG=101b, one channel


# %% [markdown]
# ## T.2 Bind and identify
#
# Unlike the TMP102, this part can prove what it is. `MANUFACTURER_ID` is a
# fixed 16-bit constant spanning two adjacent 8-bit registers — which is the
# first use in this notebook of the mixin's per-call `nbytes` override, since
# the class default is 1 here.
#
# Note the LSB register sits at the *lower* address (0x0E) with the MSB at
# 0x0F, so a 2-byte auto-incrementing read returns them in little-endian order
# even though the part is otherwise big-endian. That is handled explicitly
# below rather than by flipping `BIG_ENDIAN`, because it is a quirk of this one
# register pair rather than a chip-wide convention.

# %%
TMAG_ADDR = TMAG5273Proto.ADDR_DEFAULT          # 0x35 on SparkFun's Qwiic board

TMAG_ADDR=0x22

assert TMAG5273Proto.addr_ok(TMAG_ADDR), \
    "{:#04x} outside the programmable 7-bit range".format(TMAG_ADDR)
assert TMAG5273Proto.baud_ok(100_000), "100 kHz outside the part's baud limits"

scan = bus.scan()
print("bus scan:", [hex(a) for a in scan])
assert TMAG_ADDR in scan, \
    "no device ACKing at {:#04x} -- check the Qwiic cable and the board's " \
    "address jumper".format(TMAG_ADDR)

tmag_proto_conn = I2CConnection(bus, TMAG_ADDR, path=[], lock=bus_lock)
tmag_proto = TMAG5273Proto(tmag_proto_conn)

print()
print("TMAG5273Proto bound at {:#04x}".format(TMAG_ADDR))
print()

# MANUFACTURER_ID: LSB at 0x0E, MSB at 0x0F, so read both and assemble by hand.
mid_lsb = tmag_proto.reg_read(tmag_proto.REG_MANUFACTURER_ID_LSB)
mid_msb = tmag_proto.reg_read(tmag_proto.REG_MANUFACTURER_ID_MSB)
mid = (mid_msb << 8) | mid_lsb
show(mid, "MANUFACTURER_ID")
check("manufacturer ID", mid == tmag_proto.MANUFACTURER_ID,
      "{:#06x}, want {:#06x} ('TI')".format(mid, tmag_proto.MANUFACTURER_ID))

print()
dev_id = tmag_proto.reg_read(tmag_proto.REG_DEVICE_ID)
ver = tmag_proto.field_extract(dev_id, tmag_proto.F_VER)
show(dev_id, "DEVICE_ID", 1)
check("VER is A1", ver == tmag_proto.VER_A1,
      "VER = {} ({})".format(ver, {1: "A1, +-40/+-80 mT",
                                   2: "A2, +-133/+-266 mT"}.get(ver, "unknown")))

print()
addr_reg = tmag_proto.reg_read(tmag_proto.REG_I2C_ADDRESS)
show(addr_reg, "I2C_ADDRESS", 1)
check("address register agrees", (addr_reg >> 1) == TMAG_ADDR,
      "reg {:#04x} -> address {:#04x}".format(addr_reg, addr_reg >> 1))

print()
status = tmag_proto.reg_read(tmag_proto.REG_DEVICE_STATUS)
show(status, "DEVICE_STATUS", 1)
for name, field in (("VCC undervoltage", tmag_proto.F_VCC_UV_ER),
                    ("OTP CRC error", tmag_proto.F_OTP_CRC_ER),
                    ("INT pin error", tmag_proto.F_INT_ER),
                    ("oscillator error", tmag_proto.F_OSC_ER)):
    flagged = tmag_proto.field_extract(status, field)
    check(name, flagged == 0, "clear" if flagged == 0 else "FLAGGED -- see section 7.5")


# %% [markdown]
# ## T.3 `RegisterI2CBase` against a differently-shaped part
#
# Section 7 verified the mixin against the TMP102: uniform 16-bit registers,
# big-endian, one config register. This part is the opposite shape — 8-bit
# registers with 16-bit results spanning adjacent pairs — so it exercises paths
# section 7 could not.
#
# Three things are new here:
#
# * **`nbytes` per call.** `REG_BYTES = 1`, but the result registers are read
#   two at a time. The main notebook's rough-edges list warns that the cache
#   stores no width; this cell demonstrates that trap deliberately, because it
#   is a live risk on this part rather than a theoretical one.
# * **Auto-increment across registers.** A single 2-byte read starting at
#   `X_MSB_RESULT` returns X_MSB then X_LSB, which is what makes the pair
#   readable in one transfer.
# * **Write-1-to-clear bits.** `POR` in `CONV_STATUS` reads 1 after power-up
#   and is cleared by writing 1 back — so it cannot be verified by read-back,
#   and needs the raw `reg_write` path.

# %%
print("8-bit register reads")
print()

for name, reg in (("DEVICE_CONFIG_1", tmag_proto.REG_DEVICE_CONFIG_1),
                  ("DEVICE_CONFIG_2", tmag_proto.REG_DEVICE_CONFIG_2),
                  ("SENSOR_CONFIG_1", tmag_proto.REG_SENSOR_CONFIG_1),
                  ("SENSOR_CONFIG_2", tmag_proto.REG_SENSOR_CONFIG_2),
                  ("T_CONFIG", tmag_proto.REG_T_CONFIG),
                  ("CONV_STATUS", tmag_proto.REG_CONV_STATUS)):
    show(tmag_proto.reg_read(reg), name, 1)

print()
# A part fresh from power-up has every config register at 0x00. If an earlier
# run configured it, that is information rather than failure.
cfg1 = tmag_proto.reg_read(tmag_proto.REG_DEVICE_CONFIG_1)
if cfg1 == tmag_proto.CONFIG_POR:
    check("DEVICE_CONFIG_1 at POR", True, "{:#04x}".format(tmag_proto.CONFIG_POR))
else:
    print("  [info] DEVICE_CONFIG_1 = {:#04x} (POR {:#04x}) -- already "
          "configured, not a fault".format(cfg1, tmag_proto.CONFIG_POR))

print()
print("16-bit reads via the nbytes override")
print()

# REG_BYTES is 1, so a 16-bit result needs nbytes=2 per call. The MSB register
# sits at the lower address and the part auto-increments, so a single 2-byte
# transfer assembles correctly under BIG_ENDIAN.
for name, reg in (("X", tmag_proto.REG_X_MSB_RESULT),
                  ("Y", tmag_proto.REG_Y_MSB_RESULT),
                  ("Z", tmag_proto.REG_Z_MSB_RESULT),
                  ("T", tmag_proto.REG_T_MSB_RESULT)):
    word = tmag_proto.reg_read(reg, nbytes=2)
    show(word, name + " word")

print()
# The 2-byte read must equal the two 1-byte reads concatenated. Checked on the
# MANUFACTURER_ID pair rather than a result register: the results are volatile
# and change between transfers, so comparing across three separate reads would
# fail intermittently for reasons that have nothing to do with auto-increment.
# The ID is a constant, so any difference is the mechanism under test.
id_lsb = tmag_proto.reg_read(tmag_proto.REG_MANUFACTURER_ID_LSB)
id_msb = tmag_proto.reg_read(tmag_proto.REG_MANUFACTURER_ID_MSB)
id_pair = tmag_proto.reg_read(tmag_proto.REG_MANUFACTURER_ID_LSB, nbytes=2)
check("auto-increment assembles", id_pair == (id_lsb << 8) | id_msb,
      "{:#04x},{:#04x} -> {:#06x}".format(id_lsb, id_msb, id_pair))
print("         note this pair is LSB-first on the wire, so the assembled")
print("         word is byte-swapped relative to MANUFACTURER_ID itself")

print()
print("the width trap the cache cannot see")
print()

# Documented rough edge, made concrete: the cache keys on register number
# alone, so mixing widths on one register returns a stale-width value.
#
# Demonstrated on MANUFACTURER_ID rather than a result register, because the
# ID is constant -- with a volatile register a difference between the two calls
# could be the field moving rather than the cache, and the point would be lost.
tmag_proto.cache_clear()
narrow = tmag_proto.reg_get(tmag_proto.REG_MANUFACTURER_ID_LSB)             # 1 byte
wide   = tmag_proto.reg_get(tmag_proto.REG_MANUFACTURER_ID_LSB, nbytes=2)   # cached!
check("cache returns the stale width", narrow == wide,
      "1-byte {:#04x}, 2-byte request returned {:#04x}".format(narrow, wide))
print("         this is the documented limitation, not a fault -- pass")
print("         refresh=True, or don't mix widths on one register number")

tmag_proto.cache_clear()
fresh = tmag_proto.reg_get(tmag_proto.REG_MANUFACTURER_ID_LSB, nbytes=2,
                           refresh=True)
check("clearing the cache fixes it", fresh != narrow and fresh > 0xFF,
      "{:#06x} -- the full 16-bit value".format(fresh))

print()
print("verified writes on an 8-bit register")
print()

original = tmag_proto.reg_read(tmag_proto.REG_SENSOR_CONFIG_2)
show(original, "before", 1)

# SENSOR_CONFIG_2 has no read-only or self-clearing bits, so it verifies
# exactly with no mask -- the simplest possible case, worth confirming before
# anything harder.
landed = tmag_proto.reg_set(tmag_proto.REG_SENSOR_CONFIG_2, 0x03)
show(landed, "wrote 0x03", 1)
check("exact verify passes", landed == 0x03)

# Field-level write must leave neighbours alone, same property as section 7.8.
before = tmag_proto.reg_read(tmag_proto.REG_SENSOR_CONFIG_2)
tmag_proto.field_set(tmag_proto.REG_SENSOR_CONFIG_2, tmag_proto.F_ANGLE_EN, 1)
after = tmag_proto.reg_read(tmag_proto.REG_SENSOR_CONFIG_2)
changed = before ^ after
check("only ANGLE_EN moved",
      changed & ~tmag_proto.field_mask(tmag_proto.F_ANGLE_EN) == 0,
      "changed {:#04x}".format(changed))

tmag_proto.reg_set(tmag_proto.REG_SENSOR_CONFIG_2, original)
show(tmag_proto.reg_read(tmag_proto.REG_SENSOR_CONFIG_2), "restored", 1)

print()
print("write-1-to-clear, which no managed write can verify")
print()

conv = tmag_proto.reg_read(tmag_proto.REG_CONV_STATUS)
por = tmag_proto.field_extract(conv, tmag_proto.F_POR)
show(conv, "CONV_STATUS", 1)
print("       POR = {} ({})".format(por, "set since power-up" if por else "already cleared"))

if por:
    # Writing 1 back clears it, so the read-back is guaranteed to differ from
    # what was written. reg_set would raise; reg_write is the correct tool.
    tmag_proto.reg_write(tmag_proto.REG_CONV_STATUS,
                         tmag_proto.field_mask(tmag_proto.F_POR))
    after = tmag_proto.field_get(tmag_proto.REG_CONV_STATUS, tmag_proto.F_POR)
    check("W1C cleared POR", after == 0, "raw reg_write, no verification possible")
else:
    print("  [info] POR already clear -- nothing to demonstrate")


# %% [markdown]
# ## T.4 `TMAG5273` — the driver
#
# Everything the driver adds is interpretation: raw words into millitesla and
# degrees, named settings into bitfield writes, and the two fast-read modes
# into a packet decoder.
#
# ### Range is cached, for the same reason `em` was on the TMP102
#
# Every magnetic decode needs `B_R`, which depends on `X_Y_RANGE` and `Z_RANGE`.
# Reading those on every conversion would double the traffic on the hottest
# path, so they are cached in `self.xy_range` / `self.z_range` and resynced
# after every write that could touch them. `range_refresh()` exists for when
# something outside the driver reconfigures the part.
#
# The failure mode is the TMP102's exactly: a wrong cached range gives readings
# off by a factor of two, which at a typical field strength looks entirely
# plausible.
#
# ### Fast-read packet length is computed, not requested
#
# In modes `1h` and `2h` the caller cannot say how many bytes it wants — the
# part decides, from `MAG_CH_EN`, `T_CH_EN` and `CRC_EN`. `fast_read_length()`
# reproduces that arithmetic from the driver's own cached configuration, and
# `fast_read()` uses it. If the two disagree the transfer NACKs or returns
# garbage, so the length calculation is as load-bearing as the decode.
#
# ### Checksum scope lives here
#
# `CRCMixin` computes a checksum over whatever bytes it is handed; deciding
# what to hand it is this class's job. For the TMAG5273 that is the command
# byte the host sent, followed by every returned byte except the checksum
# itself. Omitting the command byte is the standard mistake, and it produces a
# checksum that is stable and wrong.

# %%
class TMAG5273(RegisterI2CBase, CRCMixin, I2CDevice):
    """TI TMAG5273A1 three-axis Hall-effect magnetometer (SLYS045).

    Register-mapped for configuration in every mode; optionally fast-read for
    data. Provides magnetic field in millitesla, temperature in degrees
    Celsius, hardware angle and magnitude, per-axis thresholds, and the full
    configuration bit-field API.

    The selected magnetic ranges are tracked in `self.xy_range` / `self.z_range`
    and resynced from the device on construction and after every write that
    could change them, because they scale every magnetic decode.

    Targets the A1 variant (+-40/+-80 mT). For A2, override `RANGES` with
    `RANGE_A2` -- nothing else differs.

    Example
    -------
        conn = I2CConnection(bus, 0x35, path=[], lock=bus_lock)
        mag  = TMAG5273(conn)
        mag.configure(channels="XYZ", temperature=True)
        print(mag.magnetic())         # -> (0.317, -1.245, 12.038)
    """

    # --- datasheet facts (I2CNode contract) ---
    ALLOWED_ADDRS = tuple(range(0x08, 0x78))
    ADDR_DEFAULT  = 0x35
    BAUD_NOMINAL  = 400_000
    BAUD_MAX      = 1_000_000
    BAUD_MIN      = 10_000

    BIG_ENDIAN    = True
    REG_BYTES     = 1

    # --- register map (Table 7-6) ---
    REG_DEVICE_CONFIG_1      = 0x00
    REG_DEVICE_CONFIG_2      = 0x01
    REG_SENSOR_CONFIG_1      = 0x02
    REG_SENSOR_CONFIG_2      = 0x03
    REG_X_THR_CONFIG         = 0x04
    REG_Y_THR_CONFIG         = 0x05
    REG_Z_THR_CONFIG         = 0x06
    REG_T_CONFIG             = 0x07
    REG_INT_CONFIG_1         = 0x08
    REG_MAG_GAIN_CONFIG      = 0x09
    REG_MAG_OFFSET_CONFIG_1  = 0x0A
    REG_MAG_OFFSET_CONFIG_2  = 0x0B
    REG_I2C_ADDRESS          = 0x0C
    REG_DEVICE_ID            = 0x0D
    REG_MANUFACTURER_ID_LSB  = 0x0E
    REG_MANUFACTURER_ID_MSB  = 0x0F
    REG_T_MSB_RESULT         = 0x10
    REG_X_MSB_RESULT         = 0x12
    REG_Y_MSB_RESULT         = 0x14
    REG_Z_MSB_RESULT         = 0x16
    REG_CONV_STATUS          = 0x18
    REG_ANGLE_RESULT_MSB     = 0x19
    REG_MAGNITUDE_RESULT     = 0x1B
    REG_DEVICE_STATUS        = 0x1C

    # --- bitfields ---
    F_CRC_EN         = (7, 1)
    F_MAG_TEMPCO     = (5, 2)
    F_CONV_AVG       = (2, 3)
    F_I2C_RD         = (0, 2)

    F_THR_HYST       = (5, 3)
    F_LP_LN          = (4, 1)
    F_GLITCH_FILTER  = (3, 1)
    F_TRIGGER_MODE   = (2, 1)
    F_OPERATING_MODE = (0, 2)

    F_MAG_CH_EN      = (4, 4)
    F_SLEEPTIME      = (0, 4)

    F_THRX_COUNT     = (6, 1)
    F_MAG_THR_DIR    = (5, 1)
    F_MAG_GAIN_CH    = (4, 1)
    F_ANGLE_EN       = (2, 2)
    F_X_Y_RANGE      = (1, 1)
    F_Z_RANGE        = (0, 1)

    F_T_THR_CONFIG   = (1, 7)
    F_T_CH_EN        = (0, 1)

    F_RSLT_INT       = (7, 1)
    F_THRSLD_INT     = (6, 1)
    F_INT_STATE      = (5, 1)
    F_INT_MODE       = (2, 3)
    F_MASK_INTB      = (0, 1)

    F_VER            = (0, 2)

    F_SET_COUNT      = (5, 3)
    F_POR            = (4, 1)
    F_DIAG_STATUS    = (1, 1)
    F_RESULT_STATUS  = (0, 1)

    F_INTB_RB        = (4, 1)
    F_OSC_ER         = (3, 1)
    F_INT_ER         = (2, 1)
    F_OTP_CRC_ER     = (1, 1)
    F_VCC_UV_ER      = (0, 1)

    # CONV_STATUS and DEVICE_STATUS hold write-1-to-clear bits, so they must
    # never take the managed write path. Config registers have no read-only
    # bits at all, so the class-wide mask is all-ones and nothing needs to opt
    # out -- the opposite of the TMP102's situation.
    VERIFY_MASK = 0xFF

    # --- code tables ---
    CONV_AVG   = {0: 1, 1: 2, 2: 4, 3: 8, 4: 16, 5: 32}
    MAG_CH     = {0x0: "",    0x1: "X",   0x2: "Y",   0x3: "XY",
                  0x4: "Z",   0x5: "ZX",  0x6: "YZ",  0x7: "XYZ",
                  0x8: "XYX", 0x9: "YXY", 0xA: "YZY", 0xB: "XZX"}
    OP_MODE    = {0: "standby", 1: "sleep", 2: "continuous", 3: "wake_sleep"}
    READ_MODE  = {0: "standard", 1: "fast16", 2: "fast8"}
    TEMPCO     = {0: 0.0, 1: 0.12, 3: 0.2}
    ANGLE_PAIR = {0: "off", 1: "XY", 2: "YZ", 3: "XZ"}
    SLEEPTIME  = {0x0: 1, 0x1: 5, 0x2: 10, 0x3: 15, 0x4: 20, 0x5: 30,
                  0x6: 50, 0x7: 100, 0x8: 500, 0x9: 1000, 0xA: 2000,
                  0xB: 5000, 0xC: 20000}

    # Which axes each MAG_CH_EN code streams, in the order the part sends them.
    # The repeated-axis codes (0x8-0xB) are the pseudo-simultaneous sampling
    # modes from section 8.1.3.3, where one axis is sampled twice around
    # another so the two share a time stamp.
    CH_SEQUENCE = {0x0: (), 0x1: ("X",), 0x2: ("Y",), 0x3: ("X", "Y"),
                   0x4: ("Z",), 0x5: ("Z", "X"), 0x6: ("Y", "Z"),
                   0x7: ("X", "Y", "Z"),
                   0x8: ("X", "Y", "X"), 0x9: ("Y", "X", "Y"),
                   0xA: ("Y", "Z", "Y"), 0xB: ("X", "Z", "X")}

    # --- identification ---
    MANUFACTURER_ID = 0x5449
    VER_A1 = 1
    VER_A2 = 2

    I2C_ADDRESS_POR = 0x6A
    CONFIG_POR      = 0x00

    # --- CRC (section 7.4.1) ---
    # x^8 + x^2 + x + 1, init 0xFF, no reflection, no final XOR. The same
    # polynomial as CRC-8/SMBUS but seeded 0xFF rather than 0x00.
    CRC_WIDTH = 8
    CRC_POLY  = 0x07
    CRC_INIT  = 0xFF

    # Vectors the datasheet publishes, kept on the class so a driver can
    # self-check without a device. Note the OCR of section 7.4.1 contains a
    # second, garbled copy of this list showing 0x0F/0x01/0x5A/0x31 -- these are
    # from the clean render and are the ones that verify.
    CRC_VECTORS = ((b"\xE0", 0x5D),
                   (b"\x00\x00\x00\x00", 0xD1),
                   (b"\xFF\xFF\xFF\xFF", 0x0F))

    # --- magnetic ranges, A1 (section 6.7) ---
    RANGE_A1 = {0: 40.0, 1: 80.0}
    RANGE_A2 = {0: 133.0, 1: 266.0}
    RANGES   = RANGE_A1                      # override for an A2 part

    MAG_BITS    = 16                         # result word width
    OFFSET_BITS = 8                          # MAG_OFFSET_CONFIG_x width
    OFFSET_DIV  = 4096                       # 2^12, per Equation 16

    # --- temperature (section 6.6) ---
    T_ADC_T0  = 17508
    T_SENS_T0 = 25.0
    T_ADC_RES = 60.1
    TEMP_MIN  = -40.0
    TEMP_MAX  = 170.0

    # One MSB step is 256 ADC counts, so an 8-bit fast read quantises
    # temperature to ~4.26 degC. Exposed because callers deciding between read
    # modes need the number, and because truncation is toward zero rather than
    # to nearest, so the error is one-sided.
    TEMP_LSB_STEP_C = 256 / T_ADC_RES

    # T_THR_CONFIG encodes temperature as code*8 - 249 degC, derived from the
    # datasheet's two stated points (-41 C at 0x1A, 167 C at 0x34). Only two
    # points, so treat as provisional until confirmed against silicon.
    T_THR_STEP   = 8
    T_THR_OFFSET = 249

    # --- angle (section 7.4.2.3) ---
    ANGLE_INT  = (4, 9)                      # D12:D4, integer degrees
    ANGLE_FRAC = (0, 4)                      # D3:D0, sixteenths

    # --- timing (section 6.11) ---
    T_STARTUP_US     = 270
    T_WAKE_US        = 50
    T_MEASURE_US     = 50
    T_PER_CHAN_US    = 25
    T_MEASURE_MAX_US = 825

    def __init__(self, conn, **kwargs):
        """Bind to a connection and read the ranges to establish the decode scale."""
        super().__init__(conn, **kwargs)
        self.xy_range = self.RANGES[0]       # provisional; corrected below
        self.z_range  = self.RANGES[0]
        self.range_refresh()

    # ------------------------------------------------------------------
    # Format conversion
    #
    # Classmethods taking explicit scale arguments, so they can be unit-tested
    # against the datasheet with no device present -- the TMP102 pattern.
    # ------------------------------------------------------------------

    @classmethod
    def mag_from_raw(cls, word, full_scale):
        """Decode a 16-bit two's-complement magnetic word into millitesla.

        Equation 10: B = raw / 2^16 * 2|B_R|. At +-40 mT that gives 819.2
        LSB/mT, which is the 820 the Electrical Characteristics table states.
        """
        return cls.sign_extend(word, cls.MAG_BITS) / (1 << cls.MAG_BITS) * 2 * full_scale

    @classmethod
    def mag_to_raw(cls, mT, full_scale):
        """Encode millitesla into a 16-bit two's-complement word.

        For building expected values in tests and for threshold arithmetic.
        Rounds half-away-from-zero, matching `celsius_to_raw` in the TMP102.
        """
        counts = mT * (1 << cls.MAG_BITS) / (2 * full_scale)
        counts = int(counts + (0.5 if counts >= 0 else -0.5))
        lo, hi = -(1 << (cls.MAG_BITS - 1)), (1 << (cls.MAG_BITS - 1)) - 1
        if not (lo <= counts <= hi):
            raise ValueError("{} mT outside the +-{} mT range".format(mT, full_scale))
        return counts & ((1 << cls.MAG_BITS) - 1)

    @classmethod
    def celsius_from_raw(cls, word):
        """Decode a 16-bit temperature word into degrees Celsius.

        Equation 12: T = T_SENS_T0 + (raw - T_ADC_T0) / T_ADC_RES.

        Note this is *not* two's complement -- the word is an unsigned ADC code
        and negative temperatures fall out of the subtraction. Sign-extending
        it, which is the reflex after the magnetic channels, gives nonsense.
        """
        return cls.T_SENS_T0 + (word - cls.T_ADC_T0) / cls.T_ADC_RES

    @classmethod
    def celsius_to_raw(cls, celsius):
        """Encode degrees Celsius into a 16-bit ADC code."""
        code = int(cls.T_ADC_T0 + (celsius - cls.T_SENS_T0) * cls.T_ADC_RES + 0.5)
        if not (0 <= code <= 0xFFFF):
            raise ValueError("{} C not representable".format(celsius))
        return code

    @classmethod
    def angle_from_raw(cls, word):
        """Decode a 13-bit angle word into degrees.

        D12:D4 hold the integer part, D3:D0 sixteenths. The top three bits are
        always zero. Datasheet examples: 354.50 deg is 0x1628, 17.25 is 0x0114.
        """
        return (cls.field_extract(word, cls.ANGLE_INT)
                + cls.field_extract(word, cls.ANGLE_FRAC) / 16.0)

    @classmethod
    def angle_to_raw(cls, degrees):
        """Encode degrees into a 13-bit angle word, to the nearest sixteenth."""
        if not (0.0 <= degrees < 360.0):
            raise ValueError("{} outside 0-360 degrees".format(degrees))
        whole = int(degrees)
        frac = int((degrees - whole) * 16 + 0.5)
        if frac == 16:                       # rounding carried into the integer
            whole, frac = whole + 1, 0
        return (whole << 4) | frac

    @classmethod
    def offset_from_raw(cls, byte, full_scale):
        """Decode an 8-bit two's-complement offset code into millitesla.

        Equation 16: delta = raw / 2^12 * 2|B_R|. The datasheet's worked
        example -- 0b10000000 at +-40 mT giving -2.5 mT -- is checked in T.5.
        """
        return (cls.sign_extend(byte, cls.OFFSET_BITS)
                / cls.OFFSET_DIV * 2 * full_scale)

    @classmethod
    def offset_limit(cls, full_scale):
        """Largest magnitude offset representable at a given range, in mT.

        The offset field is 8 bits over a 2^12 divisor, so it spans only 1/16
        of full scale -- +-1.25 mT at +-40 mT, +-2.5 mT at +-80. Easy to
        overshoot, hence a named accessor rather than a number buried in an
        error message.
        """
        return 128 / cls.OFFSET_DIV * 2 * full_scale

    @classmethod
    def offset_to_raw(cls, mT, full_scale):
        """Encode a millitesla offset into an 8-bit two's-complement code.

        Raises if the request exceeds what 8 bits can express at this range,
        naming the actual limit -- the field covers only 1/16 of full scale, so
        a plausible-looking offset is easy to overshoot.
        """
        counts = mT * cls.OFFSET_DIV / (2 * full_scale)
        counts = int(counts + (0.5 if counts >= 0 else -0.5))
        if not (-128 <= counts <= 127):
            limit = cls.offset_limit(full_scale)
            raise ValueError(
                "{} mT offset outside +-{:g} mT, the representable range at "
                "+-{:g} mT full scale".format(mT, limit, full_scale))
        return counts & 0xFF

    # ------------------------------------------------------------------
    # Range tracking
    # ------------------------------------------------------------------

    def range_refresh(self):
        """Re-read both range bits and update the cached decode scales.

        Worth calling explicitly if anything outside this driver may have
        reconfigured the part. A wrong cached range scales every magnetic
        reading by two, which looks plausible rather than broken.
        """
        cfg = self.reg_read(self.REG_SENSOR_CONFIG_2)
        self.xy_range = self.RANGES[self.field_extract(cfg, self.F_X_Y_RANGE)]
        self.z_range  = self.RANGES[self.field_extract(cfg, self.F_Z_RANGE)]
        return self.xy_range, self.z_range

    def axis_scale(self, axis):
        """Return the full-scale range in mT for a named axis."""
        return self.z_range if axis.upper() == "Z" else self.xy_range

    # ------------------------------------------------------------------
    # Measurement -- standard register reads
    # ------------------------------------------------------------------

    def magnetic(self):
        """Return (X, Y, Z) in millitesla from the result registers.

        One 6-byte auto-incrementing read covering 0x12 through 0x17, so all
        three axes come from the same transfer. Always hits the bus; result
        registers are volatile and must never be cached.
        """
        raw = self.read_mem(self.REG_X_MSB_RESULT, 6)
        return tuple(
            self.mag_from_raw(self.bytes_to_int(raw[i*2:i*2+2], True),
                              self.axis_scale(ax))
            for i, ax in enumerate("XYZ"))

    def magnetic_raw(self):
        """Return the three raw 16-bit axis words, undecoded."""
        raw = self.read_mem(self.REG_X_MSB_RESULT, 6)
        return tuple(self.bytes_to_int(raw[i*2:i*2+2], True) for i in range(3))

    def temperature(self):
        """Return the die temperature in degrees Celsius."""
        return self.celsius_from_raw(self.reg_read(self.REG_T_MSB_RESULT, nbytes=2))

    def temperature_raw(self):
        """Return the raw 16-bit temperature ADC code."""
        return self.reg_read(self.REG_T_MSB_RESULT, nbytes=2)

    def angle(self):
        """Return the hardware-computed angle in degrees.

        Meaningful only when `ANGLE_EN` selects an axis pair; returns 0.0 with
        the angle engine off rather than raising, since that is what the
        register holds.
        """
        return self.angle_from_raw(self.reg_read(self.REG_ANGLE_RESULT_MSB, nbytes=2))

    def magnitude(self):
        """Return the resultant vector magnitude as a raw 8-bit code.

        Equation 15 computes sqrt(ch1^2 + ch2^2) across the angle pair. The
        datasheet does not give a scaling to millitesla for this register, so
        it is returned raw -- it should stay constant through a full rotation,
        which is what makes it useful as a sanity check on angle measurements.
        """
        return self.reg_read(self.REG_MAGNITUDE_RESULT)

    def all_read(self):
        """Return (T, X, Y, Z) decoded, from one 8-byte transfer.

        Covers 0x10 through 0x17 in a single auto-incrementing read, so every
        channel comes from the same bus transaction. Cheaper than four separate
        calls and closer to atomic, though the part may still complete a
        conversion mid-transfer.
        """
        raw = self.read_mem(self.REG_T_MSB_RESULT, 8)
        t = self.celsius_from_raw(self.bytes_to_int(raw[0:2], True))
        xyz = tuple(
            self.mag_from_raw(self.bytes_to_int(raw[2+i*2:4+i*2], True),
                              self.axis_scale(ax))
            for i, ax in enumerate("XYZ"))
        return (t,) + xyz

    # ------------------------------------------------------------------
    # Fast read -- modes 1h and 2h
    #
    # No register pointer on the wire. The packet is whatever the current
    # configuration says it is, so the driver computes the length rather than
    # requesting one.
    # ------------------------------------------------------------------

    def fast_read_channels(self):
        """Return the axis sequence the part will stream, per MAG_CH_EN."""
        code = self.field_get(self.REG_SENSOR_CONFIG_1, self.F_MAG_CH_EN)
        return self.CH_SEQUENCE.get(code, ())

    def fast_read_length(self, mode=None, channels=None, temp=None, crc=None):
        """Compute the expected fast-read packet length in bytes.

        The length is implicit on the wire -- the part sends what the
        configuration dictates and the host must know how much to clock out.
        Getting it wrong NACKs early or returns garbage, so this is as
        load-bearing as the decode.

        Layout: enabled channels (temperature first, then the MAG_CH_EN
        sequence), then CONV_STATUS, then the CRC byte if enabled. Each channel
        is two bytes in mode 1h, one byte in mode 2h.

        Parameters default to whatever the device is currently configured for;
        pass them explicitly to compute a hypothetical length without bus
        traffic.
        """
        if mode is None:
            mode = self.field_get(self.REG_DEVICE_CONFIG_1, self.F_I2C_RD)
        if channels is None:
            channels = self.fast_read_channels()
        if temp is None:
            temp = bool(self.field_get(self.REG_T_CONFIG, self.F_T_CH_EN))
        if crc is None:
            crc = bool(self.field_get(self.REG_DEVICE_CONFIG_1, self.F_CRC_EN))

        if mode not in (1, 2):
            raise ValueError("fast read needs I2C_RD 1h or 2h, not {:#x}".format(mode))

        per_channel = 2 if mode == 1 else 1
        n = len(channels) + (1 if temp else 0)
        return n * per_channel + 1 + (1 if crc else 0)   # + CONV_STATUS + CRC

    def fast_read(self, verify=True):
        """Perform a fast read and return (readings, conv_status).

        `readings` is a dict keyed by channel name -- "T", "X", "Y", "Z" --
        holding decoded values. Repeated-axis modes (XYX and friends) return
        the last sample for that axis; use `fast_read_raw` if both are wanted.

        **Mode 2h loses resolution**, and the temperature channel loses a lot
        of it. Only each channel's MSB is sent, so the missing LSB is treated
        as zero: magnetic readings quantise to 1/256 of full scale (0.3125 mT
        at +-40 mT), which is usually tolerable, but one temperature MSB step
        is 256 ADC counts -- about 4.26 degC. A temperature read in 8-bit mode
        therefore carries roughly +-2 degC of truncation error on top of the
        sensor's own noise, and reads low rather than centred, since truncation
        is toward zero rather than to nearest. Use mode 1h or a standard
        register read when temperature accuracy matters. `TEMP_LSB_STEP_C`
        holds the step size for callers that want to size the error.

        The checksum, when enabled, covers the command byte the host sent
        followed by every returned byte except the checksum itself. Omitting
        the command byte is the standard mistake and produces a stable, wrong
        result -- hence the explicit reconstruction below.

        Raises
        ------
        ValueError
            On a checksum mismatch, or if the device is not in a fast-read mode.
        """
        raw, status, mode = self.fast_read_raw(verify=verify)

        readings = {}
        for name, word in raw:
            if name == "T":
                # In 8-bit mode only the MSB is sent, so the LSB reads as zero.
                readings["T"] = self.celsius_from_raw(word if mode == 1 else word << 8)
            else:
                readings[name] = self.mag_from_raw(word if mode == 1 else word << 8,
                                                   self.axis_scale(name))
        return readings, status

    def fast_read_raw(self, verify=True):
        """Perform a fast read and return ([(name, word), ...], status, mode).

        The undecoded form, preserving every sample including repeated axes and
        their order on the wire. `fast_read` builds on this.
        """
        mode = self.field_get(self.REG_DEVICE_CONFIG_1, self.F_I2C_RD)
        if mode not in (1, 2):
            raise ValueError(
                "device is in {} read mode; set read_mode to 'fast16' or 'fast8'"
                .format(self.READ_MODE.get(mode, mode)))

        crc_on   = bool(self.field_get(self.REG_DEVICE_CONFIG_1, self.F_CRC_EN))
        temp_on  = bool(self.field_get(self.REG_T_CONFIG, self.F_T_CH_EN))
        channels = self.fast_read_channels()

        if mode == 1 and crc_on and len(channels) + (1 if temp_on else 0) > 3:
            raise ValueError(
                "16-bit fast read with CRC supports at most 3 channels; "
                "{} are enabled".format(len(channels) + (1 if temp_on else 0)))

        length = self.fast_read_length(mode, channels, temp_on, crc_on)
        packet = self.read_raw(length)

        if crc_on:
            # The command byte is (address << 1) | 1 -- a read. It never appears
            # in the returned bytes, but it is covered by the checksum, so it
            # has to be reconstructed here.
            cmd = (self.conn.addr << 1) | 1
            if verify:
                self.crc_verify(bytes([cmd]) + packet[:-1], packet[-1],
                                "fast read")
            packet = packet[:-1]

        status = packet[-1]
        body = packet[:-1]

        names = (("T",) if temp_on else ()) + channels
        step = 2 if mode == 1 else 1
        words = []
        for i, name in enumerate(names):
            chunk = body[i*step:(i+1)*step]
            words.append((name, self.bytes_to_int(chunk, True)))
        return words, status, mode

    # ------------------------------------------------------------------
    # Configuration
    # ------------------------------------------------------------------

    def configure(self, channels="XYZ", temperature=False, averaging=1,
                  mode="continuous", xy_range=None, z_range=None):
        """Set up the common case in one call, in dependency order.

        Ranges first (so the cached scales are right before any conversion),
        then channels and temperature, then averaging, and finally the
        operating mode -- which is last because it starts conversions and
        everything else should be settled by then.

        Parameters
        ----------
        channels : str
            One of the `CH_SEQUENCE` combinations, e.g. "XYZ", "XY", "Z",
            or a pseudo-simultaneous mode like "XZX".
        temperature : bool
            Enable the temperature channel.
        averaging : int
            Samples averaged per conversion: 1, 2, 4, 8, 16 or 32.
        mode : str
            "standby", "sleep", "continuous" or "wake_sleep".
        xy_range, z_range : float, optional
            Full-scale range in mT. Defaults to leaving them alone.
        """
        if xy_range is not None or z_range is not None:
            self.ranges_set(xy_range, z_range)

        code = next((k for k, v in self.CH_SEQUENCE.items()
                     if "".join(v) == channels.upper()), None)
        if code is None:
            raise ValueError("{} not a valid channel combination; try {}".format(
                channels, sorted("".join(v) for v in self.CH_SEQUENCE.values() if v)))
        self.field_set(self.REG_SENSOR_CONFIG_1, self.F_MAG_CH_EN, code)

        self.field_set(self.REG_T_CONFIG, self.F_T_CH_EN, 1 if temperature else 0)
        self.coded_set(self.REG_DEVICE_CONFIG_1, self.F_CONV_AVG,
                       self.CONV_AVG, averaging)
        self.operating_mode = mode
        return self

    def ranges_set(self, xy_mT=None, z_mT=None):
        """Set the full-scale magnetic ranges, then resync the cached scales."""
        cfg = self.reg_read(self.REG_SENSOR_CONFIG_2)
        for value, field in ((xy_mT, self.F_X_Y_RANGE), (z_mT, self.F_Z_RANGE)):
            if value is None:
                continue
            code = next((k for k, v in self.RANGES.items() if v == value), None)
            if code is None:
                raise ValueError("{} mT not in {}".format(
                    value, sorted(self.RANGES.values())))
            cfg = self.field_insert(cfg, field, code)
        self.reg_set(self.REG_SENSOR_CONFIG_2, cfg)
        return self.range_refresh()

    def reset_config(self):
        """Return every configuration register to its power-on value.

        The part has no software reset command, so this writes the reset values
        back one register at a time. `I2C_ADDRESS` is deliberately excluded --
        rewriting it mid-session would change the address out from under the
        connection.
        """
        for reg in (self.REG_DEVICE_CONFIG_1, self.REG_DEVICE_CONFIG_2,
                    self.REG_SENSOR_CONFIG_1, self.REG_SENSOR_CONFIG_2,
                    self.REG_X_THR_CONFIG, self.REG_Y_THR_CONFIG,
                    self.REG_Z_THR_CONFIG, self.REG_T_CONFIG,
                    self.REG_INT_CONFIG_1, self.REG_MAG_GAIN_CONFIG,
                    self.REG_MAG_OFFSET_CONFIG_1, self.REG_MAG_OFFSET_CONFIG_2):
            self.reg_set(reg, self.CONFIG_POR)
        self.cache_clear()
        return self.range_refresh()

    # --- individual settings ---

    @property
    def operating_mode(self):
        """Operating mode: standby, sleep, continuous or wake_sleep."""
        return self.coded_get(self.REG_DEVICE_CONFIG_2, self.F_OPERATING_MODE,
                              self.OP_MODE)

    @operating_mode.setter
    def operating_mode(self, name):
        self.coded_set(self.REG_DEVICE_CONFIG_2, self.F_OPERATING_MODE,
                       self.OP_MODE, name)

    @property
    def read_mode(self):
        """I2C read mode: standard, fast16 or fast8."""
        return self.coded_get(self.REG_DEVICE_CONFIG_1, self.F_I2C_RD,
                              self.READ_MODE)

    @read_mode.setter
    def read_mode(self, name):
        self.coded_set(self.REG_DEVICE_CONFIG_1, self.F_I2C_RD,
                       self.READ_MODE, name)

    @property
    def crc_enabled(self):
        """Whether the part appends a CRC byte to fast reads."""
        return bool(self.field_get(self.REG_DEVICE_CONFIG_1, self.F_CRC_EN))

    @crc_enabled.setter
    def crc_enabled(self, enable):
        self.field_set(self.REG_DEVICE_CONFIG_1, self.F_CRC_EN, 1 if enable else 0)

    @property
    def averaging(self):
        """Samples averaged per conversion: 1, 2, 4, 8, 16 or 32."""
        return self.coded_get(self.REG_DEVICE_CONFIG_1, self.F_CONV_AVG,
                              self.CONV_AVG)

    @averaging.setter
    def averaging(self, samples):
        self.coded_set(self.REG_DEVICE_CONFIG_1, self.F_CONV_AVG,
                       self.CONV_AVG, samples)

    @property
    def channels(self):
        """The enabled magnetic channel combination, as a string."""
        return self.coded_get(self.REG_SENSOR_CONFIG_1, self.F_MAG_CH_EN,
                              self.MAG_CH)

    @channels.setter
    def channels(self, combination):
        self.coded_set(self.REG_SENSOR_CONFIG_1, self.F_MAG_CH_EN,
                       self.MAG_CH, combination.upper())

    @property
    def temperature_enabled(self):
        """Whether the temperature channel converts."""
        return bool(self.field_get(self.REG_T_CONFIG, self.F_T_CH_EN))

    @temperature_enabled.setter
    def temperature_enabled(self, enable):
        self.field_set(self.REG_T_CONFIG, self.F_T_CH_EN, 1 if enable else 0)

    @property
    def angle_pair(self):
        """Which axis pair feeds the angle engine: off, XY, YZ or XZ."""
        return self.coded_get(self.REG_SENSOR_CONFIG_2, self.F_ANGLE_EN,
                              self.ANGLE_PAIR)

    @angle_pair.setter
    def angle_pair(self, pair):
        self.coded_set(self.REG_SENSOR_CONFIG_2, self.F_ANGLE_EN,
                       self.ANGLE_PAIR, pair)
        self.range_refresh()          # same register holds the range bits

    @property
    def low_noise(self):
        """False = low active current mode, True = low noise mode."""
        return bool(self.field_get(self.REG_DEVICE_CONFIG_2, self.F_LP_LN))

    @low_noise.setter
    def low_noise(self, enable):
        self.field_set(self.REG_DEVICE_CONFIG_2, self.F_LP_LN, 1 if enable else 0)

    @property
    def magnet_tempco(self):
        """Magnet temperature coefficient compensation, in %/degC."""
        return self.coded_get(self.REG_DEVICE_CONFIG_1, self.F_MAG_TEMPCO,
                              self.TEMPCO)

    @magnet_tempco.setter
    def magnet_tempco(self, percent_per_c):
        self.coded_set(self.REG_DEVICE_CONFIG_1, self.F_MAG_TEMPCO,
                       self.TEMPCO, percent_per_c)

    @property
    def sleep_time(self):
        """Sleep interval in ms between conversions in wake_sleep mode."""
        return self.coded_get(self.REG_SENSOR_CONFIG_1, self.F_SLEEPTIME,
                              self.SLEEPTIME)

    @sleep_time.setter
    def sleep_time(self, ms):
        self.coded_set(self.REG_SENSOR_CONFIG_1, self.F_SLEEPTIME,
                       self.SLEEPTIME, ms)

    # ------------------------------------------------------------------
    # Offset and gain correction
    # ------------------------------------------------------------------

    def offsets_get(self):
        """Return the two offset corrections in mT, as (first, second).

        Which physical axes these apply to is set by `ANGLE_EN` -- the first
        offset corrects the first axis of the selected pair.
        """
        pair = self.angle_pair
        scale1 = self.axis_scale(pair[0]) if pair != "off" else self.xy_range
        scale2 = self.axis_scale(pair[1]) if pair != "off" else self.xy_range
        return (self.offset_from_raw(self.reg_read(self.REG_MAG_OFFSET_CONFIG_1), scale1),
                self.offset_from_raw(self.reg_read(self.REG_MAG_OFFSET_CONFIG_2), scale2))

    def offsets_set(self, first_mT=None, second_mT=None):
        """Set either or both offset corrections, in millitesla."""
        pair = self.angle_pair
        scale1 = self.axis_scale(pair[0]) if pair != "off" else self.xy_range
        scale2 = self.axis_scale(pair[1]) if pair != "off" else self.xy_range
        if first_mT is not None:
            self.reg_set(self.REG_MAG_OFFSET_CONFIG_1,
                         self.offset_to_raw(first_mT, scale1))
        if second_mT is not None:
            self.reg_set(self.REG_MAG_OFFSET_CONFIG_2,
                         self.offset_to_raw(second_mT, scale2))
        return self.offsets_get()

    # ------------------------------------------------------------------
    # Status
    # ------------------------------------------------------------------

    def conv_status(self):
        """Return the CONV_STATUS register decoded into a dict.

        Always a fresh read: `SET_COUNT` rolls with every completed conversion
        set and `RESULT_STATUS` is the data-ready flag, so neither may be
        cached.
        """
        raw = self.reg_read(self.REG_CONV_STATUS)
        return {"raw": raw,
                "set_count": self.field_extract(raw, self.F_SET_COUNT),
                "por": bool(self.field_extract(raw, self.F_POR)),
                "diag_fail": bool(self.field_extract(raw, self.F_DIAG_STATUS)),
                "ready": bool(self.field_extract(raw, self.F_RESULT_STATUS))}

    def device_status(self):
        """Return the DEVICE_STATUS register decoded into a dict."""
        raw = self.reg_read(self.REG_DEVICE_STATUS)
        return {"raw": raw,
                "int_pin": bool(self.field_extract(raw, self.F_INTB_RB)),
                "osc_error": bool(self.field_extract(raw, self.F_OSC_ER)),
                "int_error": bool(self.field_extract(raw, self.F_INT_ER)),
                "otp_crc_error": bool(self.field_extract(raw, self.F_OTP_CRC_ER)),
                "vcc_uv_error": bool(self.field_extract(raw, self.F_VCC_UV_ER))}

    def status_clear(self):
        """Clear every write-1-to-clear status bit in both status registers.

        Uses raw `reg_write` throughout: these bits clear when written back, so
        the read-back never matches what was written and no managed write can
        verify them.
        """
        self.reg_write(self.REG_CONV_STATUS, self.field_mask(self.F_POR))
        self.reg_write(self.REG_DEVICE_STATUS,
                       self.field_mask(self.F_OSC_ER)
                       | self.field_mask(self.F_INT_ER)
                       | self.field_mask(self.F_OTP_CRC_ER)
                       | self.field_mask(self.F_VCC_UV_ER))
        self.cache_clear(self.REG_CONV_STATUS)
        self.cache_clear(self.REG_DEVICE_STATUS)

    def identify(self):
        """Return (manufacturer_id, version) read from the device."""
        lsb = self.reg_read(self.REG_MANUFACTURER_ID_LSB)
        msb = self.reg_read(self.REG_MANUFACTURER_ID_MSB)
        ver = self.field_get(self.REG_DEVICE_ID, self.F_VER)
        return (msb << 8) | lsb, ver

    def conversion_time_us(self, channels=None, averaging=None):
        """Estimate one conversion's duration in microseconds.

        Section 6.11: 50 us for one channel at CONV_AVG=000b, plus 25 us per
        additional magnetic channel, rising to 825 us at CONV_AVG=101b. The
        datasheet gives only those two anchor points, so intermediate averaging
        settings are interpolated linearly -- good enough for sizing a timeout,
        not a specification.
        """
        if channels is None:
            channels = len(self.fast_read_channels())
        if averaging is None:
            averaging = self.averaging
        base = self.T_MEASURE_US + max(0, channels - 1) * self.T_PER_CHAN_US
        if averaging <= 1:
            return base
        span = self.T_MEASURE_MAX_US - self.T_MEASURE_US
        return int(base + span * (averaging - 1) / 31)


# %% [markdown]
# ## T.5 Offline codec verification
#
# No bus traffic. Every expected value comes from the datasheet's own text or
# from a stated Electrical Characteristics figure, so a failure here is
# unambiguous arithmetic rather than anything about the hardware.
#
# The magnetic decode is checkable against a published number in a way the
# TMP102's was not: section 6.7 states 820 LSB/mT at ±40 mT and 410 at ±80.
# Equation 10 gives 819.2 and 409.6, so agreement to the datasheet's rounding
# is the check.
#
# The offset decode has an even better anchor — section 7.4.3 works a full
# example, and reproducing −2.5 mT from `0b10000000` at ±40 mT exercises
# `sign_extend`, the 2¹² divisor and the `2|B_R|` scaling all at once.

# %%
print("magnetic -- Equation 10, B = raw / 2^16 * 2|B_R|")
print()

for word, br, want in ((0x0000, 40.0, 0.0),
                       (0x1000, 40.0, 5.0),
                       (0xF000, 40.0, -5.0),
                       (0x0800, 40.0, 2.5),
                       (0x7FFF, 40.0, 39.99878),
                       (0x8000, 40.0, -40.0),
                       (0x1000, 80.0, 10.0),
                       (0x8000, 80.0, -80.0)):
    got = TMAG5273.mag_from_raw(word, br)
    check("{:#06x} at +-{:g} mT".format(word, br), abs(got - want) < 1e-4,
          "{:.5f} mT, want {:g}".format(got, want))

print()
# The datasheet states sensitivity in LSB/mT, so derive it from the codec and
# compare -- a cross-check on the whole equation rather than one value.
for br, stated in ((40.0, 820), (80.0, 410)):
    lsb_per_mT = 1 / TMAG5273.mag_from_raw(1, br)
    check("sensitivity at +-{:g} mT".format(br), abs(lsb_per_mT - stated) < 1.0,
          "{:.1f} LSB/mT, datasheet states {}".format(lsb_per_mT, stated))

print()
ok = True
for br in (40.0, 80.0):
    for mT in (0.0, 5.0, -5.0, 0.001221, br * 0.99, -br * 0.99):
        w = TMAG5273.mag_to_raw(mT, br)
        if abs(TMAG5273.mag_from_raw(w, br) - mT) > 2 * br / 65536:
            ok = False
            print("      round-trip failed: {} mT at +-{}".format(mT, br))
check("magnetic round-trips", ok, "within one LSB")

for mT, br in ((50.0, 40.0), (-50.0, 40.0), (100.0, 80.0)):
    try:
        TMAG5273.mag_to_raw(mT, br)
        check("{:g} mT at +-{:g} rejected".format(mT, br), False, "accepted")
    except ValueError:
        check("{:g} mT at +-{:g} rejected".format(mT, br), True, "out of range")

print()
print("temperature -- Equation 12, T = 25 + (raw - 17508) / 60.1")
print()

check("reference point", TMAG5273.celsius_from_raw(17508) == 25.0,
      "ADC 17508 -> 25.0 C exactly")

for celsius in (-40.0, 0.0, 25.0, 100.0, 170.0):
    code = TMAG5273.celsius_to_raw(celsius)
    back = TMAG5273.celsius_from_raw(code)
    check("{:>6.1f} C".format(celsius), abs(back - celsius) < 0.02,
          "-> ADC {} -> {:.3f} C".format(code, back))

print()
# The trap this codec sets: the word is an unsigned ADC code, not two's
# complement. Sign-extending it -- the reflex after three magnetic channels --
# is silently harmless below 0x8000 and catastrophic above, which is the worst
# possible failure shape. Every in-range temperature sits below 0x8000
# (170 C is ADC 26223), so the bug would survive every plausible bench test
# and only surface on a corrupt or out-of-range word.
check("in-range codes stay below 0x8000",
      TMAG5273.celsius_to_raw(TMAG5273.TEMP_MAX) < 0x8000,
      "170 C is ADC {}".format(TMAG5273.celsius_to_raw(TMAG5273.TEMP_MAX)))

hazard = 0x8000
right = TMAG5273.celsius_from_raw(hazard)
wrong = (TMAG5273.T_SENS_T0
         + (BitTools.sign_extend(hazard, 16) - TMAG5273.T_ADC_T0)
         / TMAG5273.T_ADC_RES)
check("sign-extending diverges above 0x8000", abs(right - wrong) > 1000,
      "{:.1f} C correct vs {:.1f} C sign-extended".format(right, wrong))

print()
print("       8-bit fast read truncates temperature to {:.2f} C steps".format(
    TMAG5273.TEMP_LSB_STEP_C))
coarse = TMAG5273.celsius_from_raw((17508 >> 8) << 8)
check("one MSB step of error", abs(coarse - 25.0) <= TMAG5273.TEMP_LSB_STEP_C,
      "25.0 C reads {:.3f} C with the LSB dropped".format(coarse))

print()
print("angle -- 13-bit, D12:D4 degrees, D3:D0 sixteenths")
print()

# Both examples the datasheet spells out in section 7.4.2.3.
for word, want in ((0x1628, 354.50), (0x0114, 17.25)):
    got = TMAG5273.angle_from_raw(word)
    check("{:#06x}".format(word), got == want,
          "{} degrees, datasheet says {}".format(got, want))

ok = True
for deg in (0.0, 17.25, 90.0, 180.0, 354.5, 359.9375):
    if TMAG5273.angle_from_raw(TMAG5273.angle_to_raw(deg)) != deg:
        ok = False
        print("      round-trip failed: {}".format(deg))
check("angle round-trips", ok, "every sixteenth is representable")

check("top three bits unused", TMAG5273.angle_to_raw(359.9375) <= 0x1FFF,
      "{:#06x}".format(TMAG5273.angle_to_raw(359.9375)))

print()
print("offset -- Equation 16, delta = raw / 2^12 * 2|B_R|")
print()

# Section 7.4.3's worked example, verbatim: MAG_OFFSET_CONFIG_1 = 10000000b at
# +-40 mT gives -2.5 mT. The datasheet writes the second value as "0001000b",
# seven digits for an 8-bit field; read as 00010000b it gives 0.3125, which
# rounds to the 0.312 printed.
check("0b10000000 at +-40 mT", TMAG5273.offset_from_raw(0b10000000, 40.0) == -2.5,
      "{} mT, datasheet says -2.5".format(TMAG5273.offset_from_raw(0b10000000, 40.0)))
check("0b00010000 at +-40 mT",
      abs(TMAG5273.offset_from_raw(0b00010000, 40.0) - 0.3125) < 1e-9,
      "{} mT, datasheet prints 0.312".format(
          TMAG5273.offset_from_raw(0b00010000, 40.0)))

print()
# The offset field is 8 bits over a 2^12 divisor, so it reaches only 1/16 of
# full scale. Worth stating explicitly: -2.5 mT is the datasheet's own example
# and is representable at +-40 mT only because it is exactly the endpoint.
for br in (40.0, 80.0):
    print("  at +-{:g} mT full scale, offsets span +-{:g} mT".format(
        br, TMAG5273.offset_limit(br)))

ok = True
for br in (40.0, 80.0):
    limit = TMAG5273.offset_limit(br)
    for mT in (0.0, limit / 8, -limit / 8, -limit, limit * 127 / 128):
        if abs(TMAG5273.offset_from_raw(
                TMAG5273.offset_to_raw(mT, br), br) - mT) > 1e-9:
            ok = False
            print("      offset round-trip failed: {} mT at +-{}".format(mT, br))
check("offset round-trips", ok, "both ranges, endpoints included")

# Overshooting must raise rather than wrap, and say what the limit is.
try:
    TMAG5273.offset_to_raw(5.0, 40.0)
    check("oversized offset rejected", False, "accepted")
except ValueError as e:
    check("oversized offset rejected", True)
    print("        ", e)

print()
print("packet length arithmetic")
print()

# The length is implicit on the wire, so this arithmetic is as load-bearing as
# the decode. Checked without a device by passing every parameter explicitly.
for mode, chans, temp, crc, want in ((1, ("X", "Y", "Z"), False, False, 7),
                                     (1, ("X", "Y", "Z"), True,  False, 9),
                                     (1, ("X",),          False, True,  4),
                                     (1, ("X", "Y"),      True,  True,  8),
                                     (2, ("X", "Y", "Z"), True,  False, 5),
                                     (2, ("X", "Y", "Z"), True,  True,  6),
                                     (2, ("X", "Y", "X"), False, True,  5)):
    got = TMAG5273.fast_read_length(TMAG5273, mode, chans, temp, crc)
    check("mode {}h {} T={} CRC={}".format(mode, "".join(chans), int(temp), int(crc)),
          got == want, "{} bytes, want {}".format(got, want))

print()
for mode in (0, 3):
    try:
        TMAG5273.fast_read_length(TMAG5273, mode, ("X",), False, False)
        check("mode {}h rejected".format(mode), False, "accepted")
    except ValueError:
        check("mode {}h rejected".format(mode), True, "not a fast-read mode")


# %% [markdown]
# ## T.6 Bind the driver and take live readings
#
# From here the cells talk to the part. The first thing to establish is that a
# conversion actually completes and the numbers are physically plausible — a
# magnetometer sitting on a bench reads the Earth's field, roughly 25–65 µT
# depending on latitude, plus whatever ferrous clutter is nearby.
#
# That is a weak check by itself, so the stronger one follows: bring a magnet
# near and confirm the readings move by orders of magnitude more than the
# noise floor. A part returning a stuck value passes the plausibility test and
# fails that one.

# %%
tmag_conn = I2CConnection(bus, TMAG_ADDR, path=[], lock=bus_lock)
tmag = TMAG5273(tmag_conn)

mid, ver = tmag.identify()
print("TMAG5273 bound at {:#04x}".format(TMAG_ADDR))
print("  manufacturer  {:#06x}".format(mid))
print("  version       {} ({})".format(ver, "A1" if ver == 1 else "A2"))
print("  ranges        X/Y +-{:g} mT, Z +-{:g} mT".format(tmag.xy_range, tmag.z_range))
print()

# Continuous conversion, all three axes plus temperature, modest averaging.
tmag.configure(channels="XYZ", temperature=True, averaging=4, mode="continuous")

import time
time.sleep_ms(10)                              # let a conversion or two land

print("configuration read back")
print("  channels      ", tmag.channels)
print("  temperature   ", tmag.temperature_enabled)
print("  averaging     ", tmag.averaging, "samples")
print("  operating mode", tmag.operating_mode)
print("  read mode     ", tmag.read_mode)
print("  conversion    ~{} us".format(tmag.conversion_time_us()))
print()

st = tmag.conv_status()
print("CONV_STATUS: set_count={} ready={} por={} diag_fail={}".format(
    st["set_count"], st["ready"], st["por"], st["diag_fail"]))
check("conversion data ready", st["ready"])
check("no diagnostic failure", not st["diag_fail"])

print()
t, x, y, z = tmag.all_read()
print("  X {:+9.4f} mT".format(x))
print("  Y {:+9.4f} mT".format(y))
print("  Z {:+9.4f} mT".format(z))
print("  T {:+9.4f} C".format(t))
print()

check("temperature plausible", 10.0 <= t <= 60.0,
      "{:.2f} C -- die runs a little above ambient".format(t))
for name, value in (("X", x), ("Y", y), ("Z", z)):
    check("{} within range".format(name), abs(value) <= tmag.axis_scale(name),
          "{:+.4f} mT".format(value))

print()
# SET_COUNT rolls with each completed conversion set, so it must advance in
# continuous mode. A stalled count means conversions are not running.
first = tmag.conv_status()["set_count"]
time.sleep_ms(20)
second = tmag.conv_status()["set_count"]
check("SET_COUNT advances", first != second,
      "{} -> {} (rolls 0-7)".format(first, second))

print()
print("bring a magnet near the package -- 20 readings at ~50 ms")
print()
seen = []
for _ in range(20):
    x, y, z = tmag.magnetic()
    mag = (x * x + y * y + z * z) ** 0.5
    seen.append(mag)
    print("  X {:+8.3f}  Y {:+8.3f}  Z {:+8.3f}  |B| {:7.3f} mT".format(x, y, z, mag))
    time.sleep_ms(50)

print()
spread = max(seen) - min(seen)
print("  |B| spread {:.4f} mT over 20 samples".format(spread))

# What this can and cannot prove. A spread far above the noise floor means a
# magnet moved and the signal path certainly works. A spread at the noise floor
# is consistent with a working part in a static field -- and also with a part
# returning a frozen value, which is why SET_COUNT was checked above and is the
# real liveness test. Only a completely identical set of samples is suspicious,
# and even that is possible on a very quiet bench.
if spread > 1.0:
    check("field changed during the sweep", True,
          "{:.3f} mT excursion -- a magnet moved, signal path confirmed".format(spread))
elif spread > 0.0:
    check("readings dither at the noise floor", True,
          "{:.4f} mT -- consistent with a static field".format(spread))
else:
    print("  [info] every sample identical. Not necessarily a fault in a very")
    print("         quiet environment, but SET_COUNT advancing above is the")
    print("         liveness evidence. Wave a magnet and re-run to be sure.")


# %% [markdown]
# ## T.7 CRC against hardware
#
# The main notebook's section 10 verified `CRCMixin` against published vectors
# and offline packets. This is the first time a real device confirms the
# parameter set — which matters because nothing offline can catch a datasheet
# transcription error, and this datasheet's OCR is visibly damaged in places.
#
# Two independent confirmations:
#
# 1. **The datasheet vectors, recomputed here.** Not a hardware test, but worth
#    repeating in context so a failure is attributable without cross-referencing
#    another section.
# 2. **A live fast-read packet.** The part computes a checksum in silicon; the
#    driver computes one in software; they must agree. This is what actually
#    validates the polynomial, the seed, and the scope decision.
#
# The scope decision is the part most likely to be wrong, so it gets an explicit
# negative check: verifying without the command byte must *fail*. If it passed,
# the scope would be under-determined and the agreement above would be luck.

# %%
print("datasheet vectors, recomputed")
print()
for data, want in tmag.CRC_VECTORS:
    got = tmag.crc(data)
    check("CRC({})".format(data.hex().upper()), got == want,
          "{:#04x}, want {:#04x}".format(got, want))

print()
print("parameters in use")
print("  width      {} bits".format(tmag.CRC_WIDTH))
print("  polynomial {:#04x}  (x^8 + x^2 + x + 1)".format(tmag.CRC_POLY))
print("  init       {:#04x}".format(tmag.CRC_INIT))
print("  reflect    in={} out={}".format(tmag.CRC_REFLECT_IN, tmag.CRC_REFLECT_OUT))
print("  final XOR  {:#04x}".format(tmag.CRC_XOR_OUT))

print()
print("live packet -- the part computes a checksum, the driver checks it")
print()

saved_read_mode = tmag.read_mode
saved_crc = tmag.crc_enabled
saved_channels = tmag.channels
saved_temp = tmag.temperature_enabled

# 16-bit fast read with CRC caps out at three channels, so drop temperature.
tmag.temperature_enabled = False
tmag.channels = "XYZ"
tmag.crc_enabled = True
tmag.read_mode = "fast16"
time.sleep_ms(10)

length = tmag.fast_read_length()
print("expecting {} bytes: 3 axes x 2 + CONV_STATUS + CRC".format(length))

raw = tmag.read_raw(length)
print("packet:", " ".join("{:02x}".format(b) for b in raw))

cmd = (TMAG_ADDR << 1) | 1
print("command byte: {:#04x}".format(cmd))
print()

computed = tmag.crc(bytes([cmd]) + raw[:-1])
check("software CRC matches silicon", computed == raw[-1],
      "computed {:#04x}, device sent {:#04x}".format(computed, raw[-1]))

# The negative check that pins the scope down.
without_cmd = tmag.crc(raw[:-1])
check("omitting the command byte fails", without_cmd != raw[-1],
      "gives {:#04x} instead -- the command byte is covered".format(without_cmd))

print()
# Corrupting a byte must be caught. Done on a copy, so no bad data reaches the
# decode path.
corrupt = bytearray(raw)
corrupt[1] ^= 0x01
check("single-bit corruption caught",
      not tmag.crc_ok(bytes([cmd]) + bytes(corrupt[:-1]), corrupt[-1]),
      "flipped one bit in the X LSB")

print()
# Ten consecutive packets, all verified through the driver's own path. One
# agreement could be coincidence; ten across changing data cannot.
ok = 0
for _ in range(10):
    try:
        readings, status = tmag.fast_read()
        ok += 1
    except ValueError as e:
        print("  checksum failure:", e)
    time.sleep_ms(20)
check("10 consecutive verified reads", ok == 10, "{}/10 passed".format(ok))

print()
# And with CRC disabled the packet must be one byte shorter -- confirming the
# length arithmetic tracks CRC_EN rather than assuming it.
tmag.crc_enabled = False
time.sleep_ms(5)
shorter = tmag.fast_read_length()
check("CRC off shortens the packet", shorter == length - 1,
      "{} bytes without CRC, {} with".format(shorter, length))
check("still reads correctly", len(tmag.read_raw(shorter)) == shorter)


# %% [markdown]
# ## T.8 Fast-read modes
#
# Both fast modes against the standard register path. The two should agree to
# within the quantisation of the narrower mode and whatever the field did
# between reads — so the comparison is loose, but a mode decoding wrongly is
# off by a factor or a sign rather than by a millitesla.
#
# The 8-bit mode's temperature is deliberately checked against its own
# truncation bound rather than against the 16-bit value, because dropping the
# LSB costs about 4.26 °C of resolution and reads low. That is not an error to
# fix; it is the mode working as specified, and the check documents it.

# %%
saved = (tmag.read_mode, tmag.crc_enabled, tmag.channels, tmag.temperature_enabled)

print("standard register read, as reference")
tmag.read_mode = "standard"
tmag.channels = "XYZ"
tmag.temperature_enabled = True
time.sleep_ms(10)
ref_t, ref_x, ref_y, ref_z = tmag.all_read()
print("  X {:+8.4f}  Y {:+8.4f}  Z {:+8.4f}  T {:+7.3f}".format(
    ref_x, ref_y, ref_z, ref_t))

print()
print("mode 1h -- 16-bit fast read, XYZ + T, CRC off")
tmag.crc_enabled = False
tmag.read_mode = "fast16"
time.sleep_ms(10)

print("  expecting {} bytes".format(tmag.fast_read_length()))
r16, status16 = tmag.fast_read()
print("  " + "  ".join("{} {:+8.4f}".format(k, v) for k, v in sorted(r16.items())))
print("  CONV_STATUS {:#04x}".format(status16))

for name, ref in (("X", ref_x), ("Y", ref_y), ("Z", ref_z)):
    delta = abs(r16[name] - ref)
    check("16-bit {} agrees".format(name), delta < 0.5,
          "fast {:+.4f} vs register {:+.4f}, delta {:.4f} mT".format(
              r16[name], ref, delta))
check("16-bit T agrees", abs(r16["T"] - ref_t) < 2.0,
      "fast {:.3f} vs register {:.3f} C".format(r16["T"], ref_t))

print()
print("mode 2h -- 8-bit fast read, XYZ + T, CRC on")
tmag.crc_enabled = True
tmag.read_mode = "fast8"
time.sleep_ms(10)

print("  expecting {} bytes (no 3-channel limit in 8-bit mode)".format(
    tmag.fast_read_length()))
r8, status8 = tmag.fast_read()
print("  " + "  ".join("{} {:+8.4f}".format(k, v) for k, v in sorted(r8.items())))

# One MSB step is 1/256 of full scale, and truncation is toward zero.
step = 2 * tmag.xy_range / 256
for name, ref in (("X", ref_x), ("Y", ref_y), ("Z", ref_z)):
    axis_step = 2 * tmag.axis_scale(name) / 256
    delta = abs(r8[name] - ref)
    check("8-bit {} within a step".format(name), delta <= axis_step + 0.5,
          "delta {:.4f} mT, step {:.4f}".format(delta, axis_step))

t_delta = abs(r8["T"] - ref_t)
check("8-bit T within its truncation bound", t_delta <= tmag.TEMP_LSB_STEP_C + 1.0,
      "delta {:.3f} C, one MSB step is {:.2f} C".format(
          t_delta, tmag.TEMP_LSB_STEP_C))
print("         8-bit temperature is coarse by design -- use mode 1h or a")
print("         register read when temperature accuracy matters")

print()
print("the 3-channel limit in 16-bit mode with CRC")
print()

tmag.read_mode = "fast16"
tmag.crc_enabled = True
tmag.channels = "XYZ"
tmag.temperature_enabled = True         # four channels, one too many
time.sleep_ms(5)
try:
    tmag.fast_read()
    check("4 channels refused", False, "it was accepted")
except ValueError as e:
    check("4 channels refused", True)
    print("        ", e)

# Three is fine.
tmag.temperature_enabled = False
time.sleep_ms(5)
r3, _ = tmag.fast_read()
check("3 channels accepted", len(r3) == 3, "XYZ with CRC")

print()
print("pseudo-simultaneous sampling -- XZX")
print()

# Section 8.1.3.3: X is sampled either side of Z so the two share a timestamp.
tmag.crc_enabled = False
tmag.channels = "XZX"
time.sleep_ms(10)
words, status, mode = tmag.fast_read_raw()
print("  sequence:", " ".join(n for n, _ in words))
check("three samples returned", len(words) == 3)
check("order is X, Z, X", [n for n, _ in words] == ["X", "Z", "X"])

x1 = tmag.mag_from_raw(words[0][1], tmag.xy_range)
x2 = tmag.mag_from_raw(words[2][1], tmag.xy_range)
print("  X1 {:+.4f} mT, X2 {:+.4f} mT, delta {:.4f}".format(x1, x2, abs(x2 - x1)))
check("the two X samples are close", abs(x2 - x1) < 1.0,
      "same axis, adjacent conversions")

print()
# fast_read collapses repeated axes; fast_read_raw preserves them. Both are
# useful, and confusing them would silently lose a sample.
collapsed, _ = tmag.fast_read()
check("fast_read collapses repeats", set(collapsed) == {"X", "Z"},
      "keys {}".format(sorted(collapsed)))

tmag.read_mode, tmag.crc_enabled, tmag.channels, tmag.temperature_enabled = saved
tmag.read_mode = "standard"


# %% [markdown]
# ## T.9 Angle, magnitude and offset
#
# The angle engine computes `atan2` across a selected axis pair in hardware,
# which is the part's headline feature and the reason the offset and gain
# registers exist at all.
#
# Without a rotating magnet there is no ground truth to check against, so these
# cells verify what can be verified without one: that the angle tracks the
# software `atan2` of the same two axes, that magnitude stays roughly constant
# while it does, and that the offset registers move readings by the amount
# Equation 16 predicts.
#
# The last of those is the strongest check here, because the expected shift is
# computable rather than merely plausible.

# %%
saved_cfg2 = tmag.reg_read(tmag.REG_SENSOR_CONFIG_2)

import math

tmag.channels = "XYZ"
tmag.angle_pair = "XY"
time.sleep_ms(10)

print("angle engine on the X-Y pair")
print()
check("angle_pair reads back", tmag.angle_pair == "XY")

for _ in range(8):
    x, y, z = tmag.magnetic()
    hw = tmag.angle()
    sw = math.degrees(math.atan2(y, x)) % 360.0
    mag = tmag.magnitude()
    print("  X {:+7.3f}  Y {:+7.3f}  hw {:7.3f} deg  sw {:7.3f} deg  |M| {:3d}".format(
        x, y, hw, sw, mag))
    time.sleep_ms(100)

print()
x, y, z = tmag.magnetic()
hw = tmag.angle()
sw = math.degrees(math.atan2(y, x)) % 360.0
# Circular difference, so 359 and 1 degree count as 2 apart rather than 358.
diff = min(abs(hw - sw), 360.0 - abs(hw - sw))
field = (x * x + y * y) ** 0.5

if field < 0.5:
    print("  [info] in-plane field is only {:.3f} mT -- too weak for a".format(field))
    print("         meaningful angle. Hold a magnet near the package and")
    print("         re-run this cell for a real comparison.")
else:
    check("hardware angle matches atan2", diff < 5.0,
          "hw {:.2f}, sw {:.2f}, delta {:.2f} deg at {:.2f} mT".format(
              hw, sw, diff, field))

print()
check("angle within 0-360", 0.0 <= hw < 360.0, "{:.4f} deg".format(hw))
check("angle quantised to sixteenths", abs(hw * 16 - round(hw * 16)) < 1e-9,
      "{:.4f} = {}/16".format(hw, round(hw * 16)))

print()
print("offset correction -- Equation 16, checked against a computed shift")
print()

saved_off = (tmag.reg_read(tmag.REG_MAG_OFFSET_CONFIG_1),
             tmag.reg_read(tmag.REG_MAG_OFFSET_CONFIG_2))

tmag.reg_set(tmag.REG_MAG_OFFSET_CONFIG_1, 0x00)
tmag.reg_set(tmag.REG_MAG_OFFSET_CONFIG_2, 0x00)
time.sleep_ms(10)
base_x, base_y, _ = tmag.magnetic()
print("  with no offset:  X {:+.4f}  Y {:+.4f}".format(base_x, base_y))

# The datasheet's own example value, so the expected shift is the one printed
# in section 7.4.3.
tmag.reg_set(tmag.REG_MAG_OFFSET_CONFIG_1, 0b10000000)
time.sleep_ms(10)
off_x, off_y, _ = tmag.magnetic()
expected = tmag.offset_from_raw(0b10000000, tmag.xy_range)
print("  with 0b10000000: X {:+.4f}  Y {:+.4f}".format(off_x, off_y))
print("  Equation 16 predicts a {:+.4f} mT shift on the first axis".format(expected))

shift = off_x - base_x
check("offset shifts by the predicted amount", abs(shift - expected) < 0.5,
      "observed {:+.4f} mT, predicted {:+.4f}".format(shift, expected))
check("second axis unaffected", abs(off_y - base_y) < 0.5,
      "Y moved {:+.4f} mT".format(off_y - base_y))

print()
tmag.reg_set(tmag.REG_MAG_OFFSET_CONFIG_1, saved_off[0])
tmag.reg_set(tmag.REG_MAG_OFFSET_CONFIG_2, saved_off[1])
tmag.reg_set(tmag.REG_SENSOR_CONFIG_2, saved_cfg2)
tmag.range_refresh()
show(tmag.reg_read(tmag.REG_SENSOR_CONFIG_2), "SENSOR_CONFIG_2", 1)


# %% [markdown]
# ## T.10 Range switching, then restore
#
# The last live check is the one whose failure is quietest. Switching the range
# doubles or halves the scale factor, so a driver that fails to resync
# `xy_range` reads every subsequent value off by exactly two — a number that
# looks entirely reasonable.
#
# The physical field does not change when the register does, so the two ranges
# must report the same field. That is the check.

# %%
saved_cfg2 = tmag.reg_read(tmag.REG_SENSOR_CONFIG_2)

tmag.channels = "XYZ"
tmag.ranges_set(40.0, 40.0)
time.sleep_ms(10)
x40, y40, z40 = tmag.magnetic()
raw40 = tmag.magnetic_raw()
print("  +-40 mT:  X {:+8.4f}  Y {:+8.4f}  Z {:+8.4f}".format(x40, y40, z40))
print("            raw {}".format(" ".join("{:#06x}".format(w) for w in raw40)))

tmag.ranges_set(80.0, 80.0)
time.sleep_ms(10)
x80, y80, z80 = tmag.magnetic()
raw80 = tmag.magnetic_raw()
print("  +-80 mT:  X {:+8.4f}  Y {:+8.4f}  Z {:+8.4f}".format(x80, y80, z80))
print("            raw {}".format(" ".join("{:#06x}".format(w) for w in raw80)))

print()
check("driver tracked the range", (tmag.xy_range, tmag.z_range) == (80.0, 80.0),
      "X/Y +-{:g}, Z +-{:g}".format(tmag.xy_range, tmag.z_range))

# Same field, different scale -- the decoded values must still agree. The raw
# counts roughly halve, which is the mechanism, and the decode multiplies the
# scale back in.
#
# The tolerance is deliberately loose. These are two separate conversions with
# a register write between them, so the field genuinely may have moved, and the
# 80 mT range has half the resolution. What this catches is a missing resync,
# which shows up as a factor of two -- not as a fraction of a millitesla.
for name, a, b in (("X", x40, x80), ("Y", y40, y80), ("Z", z40, z80)):
    step80 = 2 * 80.0 / 65536
    check("{} agrees across ranges".format(name),
          abs(a - b) < max(1.0, abs(a) * 0.1) + step80,
          "{:+.4f} at 40 mT, {:+.4f} at 80 mT".format(a, b))

print()
# The failure this guards against, made explicit. Decoded from the same raw
# word both ways, so no second bus read can perturb the comparison -- the whole
# point is that the arithmetic differs by exactly two, not that two live
# readings happen to.
word = raw80[0]
correct = TMAG5273.mag_from_raw(word, 80.0)
stale   = TMAG5273.mag_from_raw(word, 40.0)
check("a stale range halves the reading", abs(stale * 2 - correct) < 1e-9,
      "{:#06x} decodes {:+.4f} at 80 mT, {:+.4f} at 40 -- plausible, and wrong"
      .format(word, correct, stale))

print()
tmag.ranges_set(40.0, 40.0)
tmag.reg_set(tmag.REG_SENSOR_CONFIG_2, saved_cfg2)
tmag.range_refresh()

print("restoring power-on configuration")
tmag.reset_config()
tmag.status_clear()

print()
for name, reg in (("DEVICE_CONFIG_1", tmag.REG_DEVICE_CONFIG_1),
                  ("DEVICE_CONFIG_2", tmag.REG_DEVICE_CONFIG_2),
                  ("SENSOR_CONFIG_1", tmag.REG_SENSOR_CONFIG_1),
                  ("SENSOR_CONFIG_2", tmag.REG_SENSOR_CONFIG_2),
                  ("T_CONFIG", tmag.REG_T_CONFIG),
                  ("INT_CONFIG_1", tmag.REG_INT_CONFIG_1)):
    value = tmag.reg_read(reg)
    show(value, name, 1)
    check("{} at POR".format(name), value == tmag.CONFIG_POR)

print()
print("device left in standby at power-on defaults")


# %% [markdown]
# ## T.11 Summary
#
# What this section established, and what it did not.
#
# ### Confirmed against hardware
#
# * **`CRCMixin` works on silicon.** The part's hardware checksum and the
#   driver's software checksum agree across ten consecutive packets, using
#   polynomial `0x07` seeded `0xFF`. This is the first live confirmation of
#   that mixin anywhere in the notebook.
# * **The scope decision is right.** Verifying without the command byte fails,
#   so the agreement above is not coincidence — the covered range really does
#   include the byte the host sent.
# * **`RegisterI2CBase` composes with a second part shape.** 8-bit registers,
#   16-bit results spanning adjacent pairs, per-call `nbytes`, auto-increment
#   across a six-byte read, and write-1-to-clear status bits that need the raw
#   write path.
# * **Both mixins on one class.** `TMAG5273(RegisterI2CBase, CRCMixin,
#   I2CDevice)` is the first composition of both protocol layers, and neither
#   interferes with the other.
#
# ### Confirmed offline
#
# * Magnetic decode reproduces the datasheet's stated 820 and 410 LSB/mT.
# * Temperature decode is exact at the 17508 reference point, and the cell
#   demonstrates that sign-extending it — the reflex after three magnetic
#   channels — is wrong.
# * Angle decode reproduces both worked examples, 354.50° and 17.25°.
# * Offset decode reproduces the datasheet's worked example, −2.5 mT.
# * Packet-length arithmetic across seven mode/channel/CRC combinations.
#
# ### Not established
#
# * **Angle accuracy.** Without a rotating magnet on a graduated mount there is
#   no ground truth. The comparison against software `atan2` catches gross
#   errors and nothing finer.
# * **Threshold and interrupt behaviour.** `X_THR_CONFIG` and friends, the INT
#   pin, and the wake-and-sleep duty-cycled mode are all unexercised. The
#   interrupt paths need a wired INT pin, which the Qwiic connector does not
#   carry.
# * **`T_THR_CONFIG` encoding.** The `code * 8 − 249` relation is derived from
#   two points in a datasheet whose OCR is damaged. Treat it as provisional.
# * **Gain correction.** `MAG_GAIN_CONFIG` is declared but untouched.
# * **A2 variant.** Only the range table differs, but nothing has confirmed it.
#
# ### For the main notebook's status section
#
# This closes the "no part using `CRCMixin` against hardware" gap. Still open:
# the stream-read mixin that would generalise `fast_read_raw` out of this
# driver, command timing helpers, repeated-start reads, and everything on the
# bus-manager side.

# %%
print("TMAG5273A1 at {:#04x} -- verification complete".format(TMAG_ADDR))
print()
print("confirmed against hardware")
print("  CRC-8 poly 0x07 init 0xFF, whole-packet scope incl. command byte")
print("  8-bit register map, 16-bit results, per-call nbytes, auto-increment")
print("  write-1-to-clear status bits via the raw write path")
print("  both protocol mixins composed on one driver class")
print()
print("confirmed offline against the datasheet")
print("  820 / 410 LSB/mT       section 6.7")
print("  17508 ADC at 25 C      section 6.6")
print("  354.50 and 17.25 deg   section 7.4.2.3")
print("  -2.5 mT offset         section 7.4.3")
print()
print("still unverified: angle accuracy, thresholds, interrupts,")
print("                  T_THR_CONFIG encoding, gain correction, A2 ranges")
