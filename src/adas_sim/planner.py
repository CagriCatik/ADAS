"""Deterministic grid-quantized Hybrid A* baseline with nonholonomic motion primitives."""
from __future__ import annotations
from dataclasses import dataclass
import heapq
import itertools
import math
from .geometry import Pose, Vehicle, wrap_angle, footprint_collision
from .plant import Command, step
from .scenarios import Scenario


@dataclass(frozen=True)
class Plan:
    poses: tuple[Pose, ...]
    commands: tuple[Command, ...]
    step_dt: float
    expanded: int
    success: bool
    reason: str


@dataclass(frozen=True)
class Settings:
    cell: float = 0.45
    angle_bins: int = 48
    primitive_length: float = 0.8
    primitive_steps: int = 8
    speed: float = 1.0
    max_expansions: int = 60000
    reverse_penalty: float = 1.18
    switch_penalty: float = 1.2
    steering_penalty: float = 0.05


def _key(p: Pose, gear: int, c: Settings):
    return (round(p.x / c.cell), round(p.y / c.cell), round((wrap_angle(p.yaw) + math.pi) / (2 * math.pi) * c.angle_bins) % c.angle_bins, gear)


def _heuristic(p: Pose, goal: Pose, vehicle: Vehicle):
    distance = math.hypot(p.x - goal.x, p.y - goal.y)
    turn = abs(wrap_angle(p.yaw - goal.yaw)) * vehicle.wheelbase / max(math.tan(vehicle.max_steer), .01)
    return max(distance, turn * 0.65)


def _within_goal(p: Pose, s: Scenario):
    return math.hypot(p.x - s.goal.x, p.y - s.goal.y) <= s.position_tolerance and abs(wrap_angle(p.yaw - s.goal.yaw)) <= s.yaw_tolerance


def _valid_motion(p: Pose, command: Command, dt: float, count: int, s: Scenario):
    points = []
    for _ in range(count):
        p = step(p, command, s.vehicle, dt)
        if footprint_collision(p, s.vehicle, s.obstacles):
            return None
        points.append(p)
    return tuple(points)


def plan(s: Scenario, c: Settings = Settings()) -> Plan:
    if c.primitive_steps < 1 or c.speed <= 0 or c.primitive_length <= 0 or c.cell <= 0 or c.angle_bins < 1:
        raise ValueError("invalid planner settings")
    if footprint_collision(s.start, s.vehicle, s.obstacles):
        return Plan((s.start,), (), 0, 0, False, "start_in_collision")
    if footprint_collision(s.goal, s.vehicle, s.obstacles):
        return Plan((s.start,), (), 0, 0, False, "goal_in_collision")
    if _within_goal(s.start, s):
        return Plan((s.start,), (), 0, 0, True, "already_at_goal")
    dt = c.primitive_length / c.primitive_steps / c.speed
    counter = itertools.count()
    nodes = [(s.start, 0, -1, None, (), None)]
    best = {_key(s.start, 0, c): 0.0}
    queue = [(_heuristic(s.start, s.goal, s.vehicle), next(counter), 0)]
    expansions = 0
    while queue and expansions < c.max_expansions:
        _, _, idx = heapq.heappop(queue)
        p, g, parent, _, _, lastgear = nodes[idx]
        if g > best.get(_key(p, lastgear or 0, c), math.inf) + 1e-9:
            continue
        if _within_goal(p, s):
            segments = []
            while idx > 0:
                p, g, parent, command, pts, lastgear = nodes[idx]
                segments.append((command, pts))
                idx = parent
            poses = [s.start]
            commands = []
            for command, pts in reversed(segments):
                poses.extend(pts)
                commands.extend([command] * len(pts))
            return Plan(tuple(poses), tuple(commands), dt, expansions, True, "goal_reached")
        expansions += 1
        for gear in (1, -1):
            for steer in (-s.vehicle.max_steer, -s.vehicle.max_steer / 2, 0, s.vehicle.max_steer / 2, s.vehicle.max_steer):
                command = Command(steer, gear * c.speed)
                sampled = _valid_motion(p, command, dt, c.primitive_steps, s)
                if sampled is None:
                    continue
                nxt = sampled[-1]
                cost = c.primitive_length * (c.reverse_penalty if gear < 0 else 1) + c.steering_penalty * abs(steer) / s.vehicle.max_steer
                if lastgear is not None and lastgear != gear:
                    cost += c.switch_penalty
                ng = g + cost
                k = _key(nxt, gear, c)
                if ng >= best.get(k, math.inf) - 1e-9:
                    continue
                best[k] = ng
                nid = len(nodes)
                nodes.append((nxt, ng, idx, command, sampled, gear))
                heapq.heappush(queue, (ng + 1.35 * _heuristic(nxt, s.goal, s.vehicle), next(counter), nid))
    return Plan((s.start,), (), dt, expansions, False, "search_limit" if expansions >= c.max_expansions else "no_path")
