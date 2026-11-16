# Design notes

The reasoning behind each layer of `micropython_i2c`. The code's docstrings say *what*;
this says *why*. (Condensed from the original `i2c_connection_wrapper` notebook.)

## Goals

1. **Nothing is hidden.** `machine.I2C` stays reachable (`conn.bus`). This layer wraps;
   it never shadows.
2. **Bake in the boring parts.** A driver never remembers an address or which mux
   channels reach its part.
3. **Thread/async safety.** One bus serves one transaction at a time. Every transfer is
   serialised on a per-bus lock, so drivers can run from `_thread` workers or `asyncio`
   tasks without corrupting each other's transfers.
4. **Mux paths are part of the address.** Behind TCA9548A-class muxes "0x40" is
   ambiguous; the real coordinate is `(bus, mux path, address)`.
5. **Datasheet facts live on the class.** Allowed addresses, baud limits and CRC
   parameters are properties of the *part*, so config validation and scan matching can
   use them before any instance exists.

## Layers

```
machine.I2C                        MicroPython built-in, bus level
I2CConnection                      bound (bus, path, address); lock + mux routing
I2CNode -> I2CDevice | I2CMux      datasheet facts + 4-method transport contract
BitTools / CRCMixin                stateless helpers | checksum contract
RegisterI2CBase / CommandI2CBase   protocol mixins, transport-agnostic
devices.{sensors,drivers,muxes}.*  concrete parts, by type
RoutingTable / I2CManager          config -> validated topology -> live objects
```

### Locking

An I2C "operation" is several transfers: a register read is a pointer write then a
read, and a mux selection is another transfer before both. If another thread slips in
between, you get a wrong-register read at best, or a mux switched mid-transaction and
data from a different device at worst. So the unit serialised is
**select-path-plus-transfer**.

The contended resource is the *bus*, so there is one lock per bus, created by the
manager and injected into every connection and mux on it (hence the `lock=` parameters).
Builds without `_thread` get a `NullLock`, which is correct when there is no concurrency.

Driver code must never take the lock itself: it is not reentrant on most ports and every
transport method already holds it, so wrapping a transfer in it deadlocks.

### Connections and why every transfer re-selects

`I2CConnection` is the PyVISA resource: it presents `machine.I2C`'s method names minus
the address. A driver holding one *cannot* address the wrong device. Existing
`machine.I2C` driver code ports with a near-mechanical edit.

The path is replayed on every transfer, not once at construction, because mux state is
global to the bus and anyone may have changed it. Doing it under the same lock as the
transfer is what makes the pair atomic. On an empty path it costs nothing.

### Node tree and the transport contract

```
read_mem(reg, nbytes) -> bytes      write_mem(reg, data)
read_raw(nbytes)      -> bytes      write_raw(data)
```

Protocol mixins call only these four. That restraint is what lets a mixin serve a sensor
behind three muxes and a register-mapped mux on the bare bus alike. Both node types
transfer through an `I2CConnection`; a mux builds its own from its *upstream* path (the
hops that reach the mux itself), which avoids the circularity of routing a mux through
itself.

`I2CMux` declares the API the manager uses (`channel_select`, `channel_deselect`,
`control_read`) because channel encoding is part-specific, and `reset()` pulses a wired
RESET line: the recovery of last resort when a downstream part holds SDA low.

### Helpers vs. protocol mixins

`BitTools` (sign extension, fields, byte assembly) has no class attributes and no
contract, so mixing it in costs nothing. `CRCMixin` *is* a contract: a driver mixing it
in must declare its polynomial. Keeping them apart means a driver that wants
`sign_extend` does not inherit five checksum parameters.

Register-mapped and command-driven parts need different machinery, so they are separate
mixins:

```python
class MyRegisterSensor(RegisterI2CBase, I2CDevice): ...


class MyCommandSensor(CommandI2CBase, I2CDevice): ...


class MyCheckedSensor(RegisterI2CBase, CRCMixin, I2CDevice): ...
```

No mixin defines `__init__`, so there is no cooperative-`super()` bookkeeping for
driver authors. `RegisterI2CBase` creates its cache lazily for that reason.

| Fact | Where it lives | Why |
|---|---|---|
| Endianness | `BIG_ENDIAN`, class | one chip-wide convention |
| Default register width | `REG_BYTES`, class | most chips are uniform |
| Verify masks | `VERIFY_MASK` / `VERIFY_MASKS`, class | config registers' quirks |
| Opcode width, CRC params | `CMD_BYTES`, `CRC_*`, class | fixed by the vendor |
| Register-number width | `REG_ADDR_BITS`, class | 16 on parts like the VL53L4CD |
| Register width | `nbytes`, per call | 8/16/24-bit registers coexist |
| Tolerance | `tolerance`, per call | only quantising registers need it |
| Checksum scope | the driver | varies by part, not inferable |

### Confirmed writes

`reg_set` writes, reads back *from the device*, compares and raises on mismatch. A write
that silently did not take (a NAK, a brownout, an address collision) is far cheaper to
catch at the write than to debug later. Real config registers almost always contain bits
that cannot read back as written (read-only, status, inverted handshakes; the TMP102
config has all three), so the comparison is restricted by a mask: the strongest check
the hardware permits. `tolerance` instead compares numerically, for registers a part
quantises (DACs, current limits).

### Checksums

There is no default polynomial because a wrong one produces checksums that verify
against themselves and nothing else. What a checksum *covers* is the driver's decision:
Sensirion parts checksum each 16-bit word; the TMAG5273 checksums the whole packet
including the command byte the host sent.

## Routing table and manager

The config names muxes and gives each entry a full path of `[mux_name, channel]` hops.
Validation checks that every hop's mux is declared at exactly the preceding hops (which
rules out cycles), and finds address collisions across everything *visible* at once:
selecting a path leaves every upstream segment connected, so a part behind a mux
collides with a part of the same address upstream of it.

The manager clocks each bus at the lowest `BAUD_MAX` of the board and every part on it,
and refuses if some part's `BAUD_MIN` is higher. It builds buses before any driver, so
init-time identity reads run on a safe clock.

## Writing a driver

* Subclass `I2CDevice` with the matching protocol mixin, and override the datasheet
  attributes (`ALLOWED_ADDRS`, `BAUD_*`, plus `BIG_ENDIAN`/`REG_BYTES`/`VERIFY_MASKS` or
  `CMD_BYTES`/`CRC_*`).
* Put datasheet facts in a `<Part>Regs` class with no methods and derive the driver from
  it. Bring-up can then drive the mixin directly against the part before driver logic
  exists to confuse the diagnosis (see `device_notebooks/sensors/environmental/tmp102_devNB`).
* Declare fields as `(shift, width)` tuples and coded fields as `{code: value}` dicts.
  Never open-code a shift; use `field_extract` / `field_insert`.
* Use `sign_extend`, and strip padding *before* calling it.
* Use the four transport methods, never `self.conn` or a bus, and never take the lock.
* Prefer `reg_set` over `reg_write`; cache only what is genuinely stable.
* Keep codecs as classmethods with explicit scale/mode arguments so they test offline.
* Put the module in `devices/<type>/`: `sensors/<kind>/` (`environmental`, `magnetic`,
  `distance`), `drivers/` for outputs such as relays and I/O expanders, or `muxes/`. Register
  it in `devices/__init__.py` `DRIVERS` by its path under `devices`
  (`"sensors.magnetic.tmag5273"`), and add offline tests.

## Known limitations and next steps

* **Sibling muxes are never deselected.** A connection selects its own path but does not
  turn off channels on *other* muxes on the bus. With two root-level muxes, parts behind
  a channel left on by one can appear alongside parts behind the other. Fix: a per-bus
  router that tracks the current selection and deselects what the next path does not
  use. Until then, avoid duplicate addresses across different muxes' channels.
* **No repeated-start combined transfer.** `write_raw` then `read_raw` issues a STOP
  between them. Some command parts need a repeated start (`writeto(..., stop=False)`).
* **Stream reads.** `CommandI2CBase.words_read` hardcodes `[MSB, LSB, CRC]`. The
  TMAG5273's fast read lives in its driver; generalise it once a second part needs it.
* **Command timing helper.** Sensirion drivers repeat send -> sleep -> read; wait for a
  second command part before fixing the signature.
* **Sub-byte CRC widths** (CRC-5, some SMBus PEC) take untested branches.
* **Integer-only codecs** for targets without floats.
* **Auto-identification** from a bus scan using `addr_ok()` plus ID registers. The
  TMP102 shows the limit: no ID register, and 0x48 is a crowded address.
