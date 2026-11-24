"""Independent command replay and acceptance metrics."""
from dataclasses import dataclass
import math
from .geometry import footprint_collision, wrap_angle
from .plant import step
from .planner import Plan
from .scenarios import Scenario


@dataclass(frozen=True)
class Report:
    scenario: str
    result: str
    final_position_error_m: float
    final_heading_error_deg: float
    duration_s: float
    reversals: int
    path_length_m: float
    collision: bool
    expanded: int


def validate(s: Scenario, p: Plan) -> Report:
    state = s.start
    collision = footprint_collision(state, s.vehicle, s.obstacles)
    gear = 0
    reversals = 0
    distance = 0.0
    for i, command in enumerate(p.commands):
        curr = 1 if command.velocity > 0 else -1
        if gear and gear != curr:
            reversals += 1
        gear = curr
        state = step(state, command, s.vehicle, p.step_dt)
        distance += abs(command.velocity) * p.step_dt
        if footprint_collision(state, s.vehicle, s.obstacles):
            collision = True
        if math.hypot(state.x - p.poses[i + 1].x, state.y - p.poses[i + 1].y) > 1e-6:
            raise ValueError("plan/replay divergence")
    pos = math.hypot(state.x - s.goal.x, state.y - s.goal.y)
    heading = abs(wrap_angle(state.yaw - s.goal.yaw))
    passed = p.success and not collision and pos <= s.position_tolerance and heading <= s.yaw_tolerance
    return Report(s.name, "PASS" if passed else "FAIL", pos, math.degrees(heading), len(p.commands) * p.step_dt, reversals, distance, collision, p.expanded)
