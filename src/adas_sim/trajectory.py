"""Collision-preserving, gear-aware geometric trajectory refinement.

This is *not* an exact Reeds-Shepp solver. It provides local shortcut attempts
with the same kinematic plant, plus a gear-aware speed envelope.
"""
from __future__ import annotations
from dataclasses import dataclass
import math
from .planner import Plan
from .plant import Command, step
from .geometry import footprint_collision
from .scenarios import Scenario

@dataclass(frozen=True)
class TrajectoryProfile:
    max_speed: float = 1.0
    max_acceleration: float = 1.0
    max_deceleration: float = 1.5
    stop_at_gear_change: bool = True

def speed_profile(plan: Plan, profile: TrajectoryProfile = TrajectoryProfile()) -> tuple[float, ...]:
    """A forward/backward speed envelope that stops at gear changes."""
    if min(profile.max_speed, profile.max_acceleration, profile.max_deceleration) <= 0:
        raise ValueError("speed limits must be positive")
    n = len(plan.commands)
    if not n:
        return ()
    dt = plan.step_dt
    if dt <= 0:
        raise ValueError("invalid step_dt")
    max_v = [min(profile.max_speed, abs(cmd.velocity)) for cmd in plan.commands]
    for i in range(n):
        if i == 0 or (i and plan.commands[i-1].velocity * plan.commands[i].velocity < 0):
            if profile.stop_at_gear_change:
                max_v[i] = 0.0
        if i == n - 1 or (i + 1 < n and plan.commands[i].velocity * plan.commands[i+1].velocity < 0):
            if profile.stop_at_gear_change:
                max_v[i] = 0.0
    for i in range(1, n):
        max_v[i] = min(max_v[i], max_v[i-1] + profile.max_acceleration * dt)
    for i in range(n - 2, -1, -1):
        max_v[i] = min(max_v[i], max_v[i+1] + profile.max_deceleration * dt)
    return tuple(math.copysign(v, plan.commands[i].velocity) for i, v in enumerate(max_v))

def refine(plan: Plan, scene: Scenario) -> Plan:
    """Reject inconsistent paths; retain validated nonholonomic geometry.

    A placeholder interface for a later exact Reeds-Shepp and curvature-
    continuous smoother. Never interpolate pose coordinates geometrically:
    that would invalidate the no-slip constraints and collision guarantees.
    """
    if not plan.success or not plan.commands:
        return plan
    pose = scene.start
    rebuilt = [pose]
    for command in plan.commands:
        if abs(command.steer) > scene.vehicle.max_steer:
            raise ValueError("invalid steering")
        pose = step(pose, command, scene.vehicle, plan.step_dt)
        if footprint_collision(pose, scene.vehicle, scene.obstacles):
            raise ValueError("trajectory intersects obstacle")
        rebuilt.append(pose)
    return Plan(tuple(rebuilt), plan.commands, plan.step_dt, plan.expanded,
                plan.success, "validated_geometric_path")
