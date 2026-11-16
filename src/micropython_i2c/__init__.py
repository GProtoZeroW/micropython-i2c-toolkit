"""PyVISA-style connection layer over MicroPython's ``machine.I2C``.

``machine.I2C`` is bus-level: every call carries an address, and nothing knows about
muxes, concurrency or which device is which. This package adds a connection level: a
device is resolved once to a ``(bus, mux path, address)`` handle, and from then on the
handle is the device. Addressing, mux routing and bus locking are the handle's job.

Layers, bottom up::

    machine.I2C                        bus level, never hidden
    I2CConnection                      bound (bus, path, address); lock + routing
    I2CNode -> I2CDevice | I2CMux      datasheet facts + 4-method transport
    BitTools / CRCMixin                stateless helpers | checksum contract
    RegisterI2CBase / CommandI2CBase   protocol mixins, transport-agnostic
    devices.{sensors,drivers,muxes}.*  concrete parts, by type
    RoutingTable / I2CManager          config -> validated topology -> live objects

Devices live in ``micropython_i2c.devices`` (``sensors``, ``drivers``, ``muxes``) and are
imported on demand. See ``docs/design.md`` in the repo for the reasoning behind each layer.
"""

from .bits import BitTools
from .compat import NullLock, default_lock
from .connection import I2CConnection
from .crc import CRCMixin
from .node import I2CDevice, I2CMux, I2CNode
from .protocol import CommandI2CBase, RegisterI2CBase

__version__ = "0.1.0"

__all__ = [
    "BitTools",
    "CRCMixin",
    "CommandI2CBase",
    "I2CConnection",
    "I2CDevice",
    "I2CMux",
    "I2CNode",
    "NullLock",
    "RegisterI2CBase",
    "default_lock",
]
