import time

import pytest

from micropython_i2c.compat import clock_get, sleep_ms, sleep_us, ticks_ms, ticks_us
from micropython_i2c.sim.clock import SimClock
# Source: sim_clock_devNB.ipynb | Cell In[13] | 2026-11-16 13:23:28
def test_sleeps_take_no_real_time_and_add_up_exactly():
    wall = time.monotonic()
    with SimClock() as clock:
        for _ in range(1000):
            sleep_us(1)
        sleep_ms(60_000)
    assert clock.now_us == 60_001_000
    assert time.monotonic() - wall < 1.0

# Source: sim_clock_devNB.ipynb | Cell In[14] | 2026-11-16 13:23:28
def test_ticks_follow_the_simulated_time():
    with SimClock(start_s=2.5) as clock:
        assert ticks_ms() == 2500
        clock.advance(0.0015)
        assert ticks_us() == 2_501_500

# Source: sim_clock_devNB.ipynb | Cell In[15] | 2026-11-16 13:23:28
def test_the_previous_clock_comes_back_and_blocks_nest():
    before = clock_get()
    outer, inner = SimClock(), SimClock()
    with outer:
        with inner:
            assert clock_get() is inner
        assert clock_get() is outer
    assert clock_get() is before

# Source: sim_clock_devNB.ipynb | Cell In[16] | 2026-11-16 13:23:29
def test_the_clock_is_restored_after_an_error():
    before = clock_get()
    with pytest.raises(KeyError):
        with SimClock():
            raise KeyError("boom")
    assert clock_get() is before

# Source: sim_clock_devNB.ipynb | Cell In[17] | 2026-11-16 13:23:29
def test_time_never_goes_back():
    c = SimClock()
    with pytest.raises(ValueError, match="never goes back"):
        c.advance(-0.001)
    c.sleep_us(-5)
    assert c.now_us == 0

# Source: sim_clock_devNB.ipynb | Cell In[18] | 2026-11-16 13:23:29
def test_uninstall_without_install_is_refused():
    with pytest.raises(RuntimeError, match="not installed"):
        SimClock().uninstall()

