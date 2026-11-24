# Contributing

This is a research simulation, not certified automotive safety software.

## Development

```bash
python -m pip install -e ".[test]"
python -m pytest -q
adas-parking --scenario all
```

Keep physical models, geometry, planning, validation and rendering separated. Each new
feature must include deterministic tests and distinguish ground-truth simulation
state from estimated sensor values. Avoid adding optional dependencies to the core.

## Required engineering checks

- Geometry and vehicle limits (dimensions, orientation, steering, gear reversal).
- Collision checks at all available integration samples; never claim continuous-time
  safety from finite samples.
- Failed/infeasible plans must be reported, never replaced by visual animations.
- All numerical results must be labelled simulated unless externally validated.
- Preserve the Vehicle-Dynamics integration boundary.
