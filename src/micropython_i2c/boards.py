"""Board pin maps, and ``bus_open`` for building a ``machine.I2C`` from one.

Add a board by adding an entry to ``BOARDS``. ``baud_max`` caps the bus clock below
what the parts allow, e.g. for long wires or weak pull-ups.
"""

BOARDS = {
    # Unexpected Maker FeatherS3 (ESP32-S3). The STEMMA QT / Qwiic connector and the
    # SDA/SCL pads share GPIO 8/9 and have on-board pull-ups; bare header wiring does
    # not. The S3 routes I2C through the GPIO matrix, so peripheral 0 or 1 both work.
    "feathers3": {
        "buses": {
            0: {"scl": 9, "sda": 8, "baud_max": 400_000},
            # Second Qwiic chain. Confirmed 2026-11-13: the SparkFun mux answers here
            # (drivers_tca9548a_devNB, "Find the mux").
            1: {"scl": 15, "sda": 16, "baud_max": 400_000},
        },
    },
}


def board_get(board):
    """Return a board dict, given either a ``BOARDS`` key or the dict itself."""
    if isinstance(board, dict):
        return board
    try:
        return BOARDS[board]
    except KeyError:
        raise ValueError("unknown board {!r}; known: {}".format(board, sorted(BOARDS))) from None


def bus_open(board, bus_id=0, freq=100_000):
    """Create the ``machine.I2C`` for ``bus_id`` on ``board``."""
    from machine import I2C, Pin

    cfg = board_get(board)["buses"][bus_id]
    return I2C(bus_id, scl=Pin(cfg["scl"]), sda=Pin(cfg["sda"]), freq=freq)
