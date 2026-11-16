import pytest

from micropython_i2c.compat import (
    NullLock,
    RealClock,
    clock_get,
    clock_set,
    default_lock,
    sleep_ms,
    sleep_us,
    ticks_add,
    ticks_diff,
    ticks_ms,
    ticks_us,
)
# Source: compat_devNB.ipynb | Cell In[12] | 2026-11-16 13:19:06
def test_real_clock_starts_near_zero_and_advances():
    rc = RealClock()
    a = rc.ticks_us()
    rc.sleep_us(1000)
    b = rc.ticks_us()
    assert 0 <= a < 1_000_000
    assert b - a >= 1000

# Source: compat_devNB.ipynb | Cell In[15] | 2026-11-16 13:19:07
class StepClock:
    """Simulated time for the tests: moves only on sleep."""

    def __init__(self):
        self.now = 0

    def ticks_us(self):
        return self.now

    def sleep_us(self, us):
        self.now += max(0, int(us))

# Source: compat_devNB.ipynb | Cell In[18] | 2026-11-16 13:19:07
def test_swapped_clock_drives_every_timing_function():
    sc = StepClock()
    before = clock_set(sc)
    try:
        sleep_ms(5)
        sleep_us(250)
        assert ticks_us() == 5250
        assert ticks_ms() == 5
    finally:
        clock_set(before)

# Source: compat_devNB.ipynb | Cell In[19] | 2026-11-16 13:19:07
def test_clock_set_returns_the_previous_clock():
    sc = StepClock()
    before = clock_set(sc)
    try:
        assert clock_get() is sc
        assert before is not sc
    finally:
        assert clock_set(before) is sc
    assert clock_get() is before

# Source: compat_devNB.ipynb | Cell In[20] | 2026-11-16 13:19:07
def test_clock_set_refuses_an_object_that_is_not_a_clock():
    before = clock_get()
    with pytest.raises(TypeError, match="ticks_us"):
        clock_set(object())
    assert clock_get() is before

# Source: compat_devNB.ipynb | Cell In[21] | 2026-11-16 13:19:08
def test_tick_arithmetic():
    assert ticks_add(100, 25) == 125
    assert ticks_diff(100, 125) == -25
    assert ticks_diff(ticks_add(7, 3), 7) == 3

# Source: compat_devNB.ipynb | Cell In[25] | 2026-11-16 13:19:08
def test_default_lock_is_exclusive_on_cpython():
    lk = default_lock()
    with lk:
        assert lk.acquire(False) is False
    assert lk.acquire(False) is True
    lk.release()

# Source: compat_devNB.ipynb | Cell In[26] | 2026-11-16 13:19:09
def test_null_lock_never_blocks():
    nl = NullLock()
    with nl as same:
        assert same is nl
        assert nl.acquire() is True
    nl.release()

