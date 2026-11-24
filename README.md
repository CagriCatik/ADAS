# ADAS Simulation Platform

A reproducible **autonomous parking research baseline** with deterministic low-speed Ackermann dynamics, grid-quantized Hybrid A* motion planning, oriented vehicle-body collision checks, command replay, validation metrics, and dependency-free SVG visualization. Python 3.10+.

> **Simulation only.** This is not automotive safety software and has not been validated against real vehicle measurements. It is not yet a full ADAS or production autonomous parking system.

## Quick start

```bash
python -m pip install -e ".[test]"
pytest -q
adas-parking --scenario open_bay
adas-parking --scenario all
```

Results are saved in `output/parking/` as SVG and JSON trajectory/controls/summary reports. Nonzero CLI exit status denotes an unsuccessful scenario.

## Architecture

```text
src/adas_sim/
  geometry.py     Oriented vehicle footprint and SAT collision checks
  plant.py        Low-speed no-slip Ackermann kinematics
  planner.py      Deterministic Hybrid A* baseline
  scenarios.py    Parallel, perpendicular, open-bay fixtures
  validation.py   Independent replay and acceptance metrics
  svg.py          Top-down SVG renderer
  cli.py          Run scenarios and export reports
```

`Vehicle-Dynamics` remains the source of truth for detailed vehicle dynamics: [CagriCatik/Vehicle-Dynamics](https://github.com/CagriCatik/Vehicle-Dynamics). A future optional adapter will connect the two without copying the physics or changing the existing plant.

## Scope / limitations

- Park spaces and obstacles are ground truth, not perceived from sensors.
- Motion primitives enforce the steering-angle limit but not actuator-rate dynamics.
- Finite collision sampling is not a continuous-time swept-volume safety guarantee.
- No parking perception, sensor fusion, dynamic obstacle avoidance, localization or real vehicle validation.
- Next milestones: continuous swept clearance, speed/steering tracking and gear-switch dwell, Reeds-Shepp and trajectory smoothing, sensor simulation, fault injection, and browser UI.
- Add AEB, ACC and lane keeping only through isolated ADAS modules and shared interfaces.

## Reproducible development

```bash
python -m pytest -q
python -m adas_sim --scenario all
```

## Licensing

No license has been granted by the initial scaffold; confirm a license before accepting contributions or redistributing code.
