import pytest

from micropython_i2c.devices import DRIVERS, FAKES, driver_class, fake_class, registry
from micropython_i2c.node import I2CDevice, I2CMux
# Source: devices_devNB.ipynb | Cell In[12] | 2026-11-16 13:44:04
def test_registry_imports_every_driver():
    reg = registry()
    assert set(reg) == set(DRIVERS)
    for name, cls in reg.items():
        assert cls.__name__ == DRIVERS[name][1]
        assert issubclass(cls, (I2CDevice, I2CMux))

# Source: devices_devNB.ipynb | Cell In[13] | 2026-11-16 13:44:05
def test_registry_of_some_names_and_one_class():
    assert set(registry(["TMP102", "TCA9548A"])) == {"TMP102", "TCA9548A"}
    assert driver_class("SHTC3").__name__ == "SHTC3"

# Source: devices_devNB.ipynb | Cell In[14] | 2026-11-16 13:44:05
def test_every_driver_has_a_fake_that_builds_with_no_arguments():
    assert set(FAKES) == set(DRIVERS)
    for name in FAKES:
        cls = fake_class(name)
        assert cls.__name__ == FAKES[name][1]
        cls()  # every fake has defaults for everything

# Source: devices_devNB.ipynb | Cell In[15] | 2026-11-16 13:44:05
def test_unknown_names_raise_key_error():
    with pytest.raises(KeyError):
        driver_class("BME280")
    with pytest.raises(KeyError, match="no fake for driver 'BME280'"):
        fake_class("BME280")

