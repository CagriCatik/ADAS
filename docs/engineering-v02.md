# ADAS V0.2 Engineering status

The project is a **simulation research baseline**, not production automotive
software or an automotive safety system. Screens and GIFs do not prove safe
operation; failures remain visible.

## Run the 2D workbench

```bash
python -m pip install -e ".[test]"
adas-workbench --host 127.0.0.1 --port 8766
```

Open http://127.0.0.1:8766/ and select the scenario. Playback shows a
scaled body and four wheel polygons with live speed, steering and time
readouts. It is a local read-only visualization of a planned/replayed
trajectory. It is **not** a real-time interactive replanning user interface,
and does not control a real vehicle.

## Architecture / capabilities

| Area | Implemented | Still required |
| --- | --- | --- |
| Actuators | Steering slew, acceleration and deceleration bounds, simple gear dwell with separate execution status | Robust stop/gear state logic and closed-loop controller |
| Planning | Hybrid A*, motion-primitive RRT variants, independent replay, gear-aware speed envelope | Full analytic Reeds-Shepp connector, curvature-continuous smoothing, correct time parameterization |
| Perception | Seeded ray-cast ultrasound and noisy pose generator | Scanned free-space reconstruction, data association, fusion and covariance |
| Safety | Discrete collision lookahead, missing-localization inhibit, STOP/REPLAN_REQUIRED transitions | Stopping-distance envelope, dynamic obstacles, perception uncertainty, continuous swept volume checks |
| Live viewer | Browser-based 2D time slider and playback | Streaming sensors and closed-loop live simulation |
| Vehicle-Dynamics | Optional kinematic model adapter with explicit CoG-to-rear-axle transformation and parameter consistency checks | 7-DOF input/output adapter, tire-model validation, stationary/gear realism |

### Vehicle-Dynamics adapter

The optional `VehicleDynamicsKinematicAdapter` requires the external package
`vehicle_model` (from the user's separate Vehicle-Dynamics repository)
installed in the same Python environment. Its kinematic model uses CoG position
and acceleration/steering-rate controls. The bridge converts to ADAS's
rear-axle midpoint representation, enforces matched wheelbase and CoG offset
and rate-limits setpoints.

The 7-DOF model is **not** integrated: speed/gear transitions, torque/brake
actuator mapping and low-speed tire physics require separate engineering work.
This capability is intentionally not reported as complete.

### Replanning

`ParkingSupervisor` holds an explicit state machine. Updating obstacles
moves ACTIVE to REPLAN_REQUIRED, where all subsequent tick commands are forced
to zero. `replan(pose,goal)` calls Hybrid A* with the updated obstacle map
and resumes ACTIVE only when a geometrically valid route is found. This is
a research-level stop-and-replan behavior, not asynchronous replanning or
dynamic-obstacle prediction.

### GIFs / plots

CI generates and checks in images under `docs/assets/parking/` and
`docs/assets/steering/`. All share the centrally configured world scale,
layout and frame timing within their scenario family. The repository README
embeds these assets directly. Avoid adding unvalidated scenes to the gallery.

### Known limitations

- Discrete collision checks cannot guarantee continuous-time clearance.
- Open-loop actuator replay can diverge from planned path and must report this.
- No measured tire data or real vehicle validation datasets are used.
- No safety integrity level, homologation, or functional safety claim.
