# micropython-i2c-toolkit

PyVISA-style I2C connection layer, drivers and mux-aware routing for MicroPython boards
(package `micropython_i2c`, board-side code that also imports on CPython). Design notes:
`docs/design.md`. Bench wiring: README "The bench".

## Tooling
- Python 3.13 via uv and the project `.venv`; the project is installed editable (`uv_build`,
  `src/micropython_i2c`). Run tools with `uv run`.
- Kernel for every notebook: `micropylab` ("MicroPython (micropylab)"), from the sibling
  `../micropylab` checkout (editable). Before `%board connect` cells run on the host (CPython);
  after it they run on the board.
- micropylab has `%%extract` built in (same flags as jupyter_lab_extractor). Dev notebooks have
  no `%load_ext jupyter_lab_extractor` cell.
- Notebook pairing command: `uv run jlx pair <notebook>.py`
- Sync after editing a paired `.py` (only while the notebook is not open with unsaved changes):
  `uv run jupytext --sync <notebook>.py`
- Run a notebook: `uv run jlx run <notebook>.ipynb`. A dev notebook's last section needs the
  board; without it, the run stops at `# Hardware validation` after the files are written.
- Tests and lint: `uv run pytest`, `uv run ruff check`.

## Layout and workflow
- Library code is written notebook-first: jupytext percent `<name>_devNB.py` paired with
  `<name>_devNB.ipynb`.
  - `dev_notebooks/` is for the system the devices run on (connection, routing, manager, ...):
    `<module>_devNB` writes `src/micropython_i2c/<module>.py`.
  - `device_notebooks/` is for specific parts, in the same tree as
    `src/micropython_i2c/devices/`: `drivers/` (outputs: SparkFun relay, Adafruit MCP23017),
    `muxes/`, and `sensors/<kind>/` with kinds `environmental/` (temperature, humidity),
    `magnetic/`, `distance/` (optical ToF). `device_notebooks/sensors/environmental/tmp102_devNB`
    writes `src/micropython_i2c/devices/sensors/environmental/tmp102.py`. New type or kind: add
    the folder in both trees, with an `__init__.py` in the package one. Register each part in
    `devices/__init__.py` `DRIVERS` by its path under `devices` (`"sensors.magnetic.tmag5273"`).
- Run a notebook from its own folder. `%%extract` paths are relative to it (`../src/...` from
  `dev_notebooks/`, `../../src/...` from `device_notebooks/muxes/`, `../../../src/...` from
  `device_notebooks/sensors/<kind>/`); test cells append to the flat `tests/test_<module>.py`.
- Never hand-edit a file a dev notebook generates (it says so in its docstring). Edit the
  notebook and re-run it. Converted so far: `devices/sensors/environmental/{tmp102,shtc3}.py`,
  `devices/sensors/magnetic/tmag5273.py`, `devices/sensors/distance/vl53l1x.py`,
  `devices/muxes/tca9548a.py`. Every other module is still hand-written until its notebook
  exists.
- When a module is converted, add its module and test file to the generated-files lists in
  `pyproject.toml` (`I001` per-file ignore and `[tool.ruff.format] exclude`): the extractor's
  header spacing is not ruff's, and the files are rewritten on every run.
- Never run `ruff check --fix` with import sorting on `dev_notebooks/*.py` or
  `device_notebooks/**/*.py`: it moves imports across cells (I001 is off there for that reason).
- Run model (decided 2026-11-13): a dev notebook runs on the host first (code, validation and
  unit tests against `micropython_i2c.fakes`, no board needed), then a final
  `# Hardware validation` section connects, uploads `src/micropython_i2c` with
  `%files copy -r`, runs the hardware checks on the board and ends with `%board disconnect`.
- Plain notebooks (`notebooks/`) are whole-bench hardware checks, not module sources.
- `archive/` holds the original single-file notebooks and the old bring-up notebooks (replaced by
  the dev notebooks), for reference only.

## Working rules
- Board code runs on MicroPython: no type hints, no f-strings in `src/` (use `str.format`),
  absolute imports (`from micropython_i2c.x import y`) except in `__init__.py`.
- In a dev notebook's tests, use names imported in the test-file header, not imports inside a
  test function.
- `jlx run` rewrites the `# Source:` timestamps in every extracted file. Revert timestamp-only
  changes.
- Ask before running anything that talks to the board (a `jlx run` reaches the hardware
  section) or before flashing it.
