# ADAS Simulation Platform

A Python research baseline for autonomous parking: deterministic low-speed Ackermann simulation, oriented vehicle collision checking, Hybrid A* planning, reproducible parking scenarios, validation and SVG outputs.

**Simulation only:** not automotive safety software; no real-vehicle validation.

## Quick start

```bash
python -m pip install -e '.[test]'
pytest -q
adas-parking --scenario all
```

The project is designed to integrate the external [Vehicle-Dynamics](https://github.com/CagriCatik/Vehicle-Dynamics) physics plant through an adapter in a later milestone. It does not copy that project's physics.
