import pytest

from micropython_i2c.sim.clock import SimClock
from micropython_i2c.sim.signals import lag, noisy, now_s, ramp, sequence, sine, step, value
# Source: sim_signals_devNB.ipynb | Cell In[10] | 2026-11-16 13:23:38
def test_value_resolves_plain_values_signals_and_tuples():
    with SimClock(start_s=4):
        assert now_s() == 4.0
        assert value(7) == 7
        assert value(lambda t: t * 10) == 40.0
        assert value((1, lambda t: t)) == (1, 4.0)
        assert value([lambda t: -t]) == [-4.0]
        assert value(lambda t: t, t_s=1.5) == 1.5

# Source: sim_signals_devNB.ipynb | Cell In[16] | 2026-11-16 13:23:38
def test_ramp_is_flat_then_linear_then_flat():
    r = ramp(20.0, 26.0, 10, 70)
    assert value(r, 0) == 20.0
    assert value(r, 40) == pytest.approx(23.0)
    assert value(r, 100) == 26.0

# Source: sim_signals_devNB.ipynb | Cell In[17] | 2026-11-16 13:23:39
def test_step_switches_at_its_time():
    s = step("off", "on", 5)
    assert [value(s, t) for t in (4.999, 5, 6)] == ["off", "on", "on"]

# Source: sim_signals_devNB.ipynb | Cell In[18] | 2026-11-16 13:23:39
def test_sine_period_and_amplitude():
    s = sine(10.0, 2.0, 8)
    assert value(s, 0) == pytest.approx(10.0)
    assert value(s, 2) == pytest.approx(12.0)
    assert value(s, 6) == pytest.approx(8.0)
    assert value(s, 8) == pytest.approx(10.0, abs=1e-9)

# Source: sim_signals_devNB.ipynb | Cell In[19] | 2026-11-16 13:23:39
def test_sequence_interpolates_and_holds_its_ends():
    s = sequence([(1, 0.0), (3, 10.0), (4, 4.0)])
    assert [value(s, t) for t in (0, 1, 2, 3, 3.5, 9)] == [0.0, 0.0, 5.0, 10.0, 7.0, 4.0]

# Source: sim_signals_devNB.ipynb | Cell In[20] | 2026-11-16 13:23:39
def test_builders_accept_signals_as_arguments():
    r = ramp(lambda t: t, 100.0, 10, 20)  # the start moves with time
    assert value(r, 5) == 5.0
    assert value(r, 15) == pytest.approx(57.5)

# Source: sim_signals_devNB.ipynb | Cell In[21] | 2026-11-16 13:23:40
@pytest.mark.parametrize(
    "build, match",
    [
        (lambda: ramp(0, 1, 2, 1), "after"),
        (lambda: sine(0, 1, -1), "positive"),
        (lambda: sequence([]), "at least one"),
        (lambda: sequence([(2, 0), (1, 0)]), "increase"),
    ],
)
def test_bad_shapes_are_refused(build, match):
    with pytest.raises(ValueError, match=match):
        build()

# Source: sim_signals_devNB.ipynb | Cell In[25] | 2026-11-16 13:23:40
def test_lag_covers_63_percent_in_one_time_constant():
    heat = lag(step(20.0, 30.0, 0), 60, initial=20.0)
    assert heat(0) == 20.0
    assert heat(60) == pytest.approx(20.0 + 10.0 * (1 - 2.718281828 ** -1), abs=1e-6)

# Source: sim_signals_devNB.ipynb | Cell In[26] | 2026-11-16 13:23:40
def test_lag_result_does_not_depend_on_how_often_it_is_read():
    rare = lag(30.0, 10, initial=20.0)
    often = lag(30.0, 10, initial=20.0)
    rare(0)
    often(0)
    for t in range(1, 21):
        often(t)
    assert rare(20) == pytest.approx(often(20))

# Source: sim_signals_devNB.ipynb | Cell In[27] | 2026-11-16 13:23:40
def test_lag_starts_over_when_time_goes_back():
    f = lag(30.0, 10, initial=20.0)
    f(100)
    f(200)
    assert f(0) == 20.0

# Source: sim_signals_devNB.ipynb | Cell In[28] | 2026-11-16 13:23:40
def test_noise_has_the_right_spread_and_repeats_by_seed():
    a = noisy(25.0, 0.5, seed=3)
    samples = [a(0) for _ in range(4000)]
    mean = sum(samples) / len(samples)
    sd = (sum((s - mean) ** 2 for s in samples) / len(samples)) ** 0.5
    assert mean == pytest.approx(25.0, abs=0.05)
    assert sd == pytest.approx(0.5, abs=0.05)
    b = noisy(25.0, 0.5, seed=3)
    assert [b(0) for _ in range(5)] == samples[:5]

# Source: sim_signals_devNB.ipynb | Cell In[29] | 2026-11-16 13:23:41
def test_signals_follow_the_simulated_clock():
    r = ramp(0.0, 100.0, 0, 100)
    with SimClock() as clock:
        assert value(r) == 0.0
        clock.advance(25)
        assert value(r) == 25.0

