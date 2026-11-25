"""Four-wheel-steering Hybrid A* using the same physical footprints as 2WS.

Unlike equal-input comparisons, this planner searches *independently* for the
same destination pose. Steering is kinematic, counter-phase and constant
within each motion primitive; actuator-rate feasibility is checked separately.
"""
from __future__ import annotations
import heapq
import itertools
import math
from dataclasses import dataclass
from .geometry import Pose, footprint_collision
from .four_wheel_steering import RearSteerConfig, step_4ws
from .planner import Plan, Settings, _heuristic, _key, _within_goal
from .plant import Command
from .scenarios import Scenario


def _sweep(start: Pose, front: float, speed: float, scene: Scenario,
           dt: float, samples: int, rear_config: RearSteerConfig):
    rear = rear_config.rear_target(front)
    poses = []
    state = start
    for _ in range(samples):
        state = step_4ws(state, front, rear, speed, scene.vehicle, dt)
        if footprint_collision(state, scene.vehicle, scene.obstacles):
            return None
        poses.append(state)
    return tuple(poses)


def plan_4ws(scene: Scenario, config: Settings = Settings(),
             rear_config: RearSteerConfig = RearSteerConfig()) -> Plan:
    """Search steering-bounded motion primitives; no implicit 2WS fallback."""
    if config.primitive_steps < 1 or config.speed <= 0 or config.primitive_length <= 0 or config.cell <= 0:
        raise ValueError("invalid planner configuration")
    if footprint_collision(scene.start, scene.vehicle, scene.obstacles):
        return Plan((scene.start,), (), 0.0, 0, False, "start_in_collision")
    if footprint_collision(scene.goal, scene.vehicle, scene.obstacles):
        return Plan((scene.start,), (), 0.0, 0, False, "goal_in_collision")
    if _within_goal(scene.start, scene):
        return Plan((scene.start,), (), 0.0, 0, True, "already_at_goal")
    dt = config.primitive_length / (config.primitive_steps * config.speed)
    nodes = [(scene.start, 0.0, -1, None, (), 0)]
    best = {_key(scene.start, 0, config): 0.0}
    tie = itertools.count()
    frontier = [(_heuristic(scene.start, scene.goal, scene.vehicle), next(tie), 0)]
    expansions = 0
    while frontier and expansions < config.max_expansions:
        _, _, index = heapq.heappop(frontier)
        state, cost, parent, _, _, old_gear = nodes[index]
        if cost > best.get(_key(state, old_gear, config), math.inf) + 1e-9:
            continue
        if _within_goal(state, scene):
            segments = []
            while index:
                _, _, parent, command, poses, _ = nodes[index]
                segments.append((command, poses))
                index = parent
            commands = []
            positions = [scene.start]
            for command, poses in reversed(segments):
                positions.extend(poses)
                commands.extend([command] * len(poses))
            return Plan(tuple(positions), tuple(commands), dt, expansions, True, "goal_reached")
        expansions += 1
        for gear in (1, -1):
            for steer in (-scene.vehicle.max_steer, -scene.vehicle.max_steer / 2,
                          0.0, scene.vehicle.max_steer / 2, scene.vehicle.max_steer):
                velocity = gear * config.speed
                poses = _sweep(state, steer, velocity, scene, dt,
                               config.primitive_steps, rear_config)
                if poses is None:
                    continue
                end = poses[-1]
                penalty = (config.reverse_penalty if gear < 0 else 1.0)
                new_cost = cost + config.primitive_length * penalty
                new_cost += config.steering_penalty * abs(steer) / scene.vehicle.max_steer
                if old_gear and old_gear != gear:
                    new_cost += config.switch_penalty
                key = _key(end, gear, config)
                if new_cost >= best.get(key, math.inf) - 1e-9:
                    continue
                best[key] = new_cost
                node_id = len(nodes)
                nodes.append((end, new_cost, index, Command(steer, velocity), poses, gear))
                heapq.heappush(frontier,
                               (new_cost + 1.35 * _heuristic(end, scene.goal, scene.vehicle),
                                next(tie), node_id))
    return Plan((scene.start,), (), dt, expansions, False,
                "search_limit" if expansions >= config.max_expansions else "no_path")


def replay_4ws(scene: Scenario, route: Plan,
               rear_config: RearSteerConfig = RearSteerConfig()):
    """Independently replay all commands, including every reverse segment."""
    if not route.commands:
        return ((scene.start,), not route.success or _within_goal(scene.start, scene))
    current = scene.start
    poses = [current]
    for command in route.commands:
        current = step_4ws(current, command.steer,
                           rear_config.rear_target(command.steer),
                           command.velocity, scene.vehicle, route.step_dt)
        if footprint_collision(current, scene.vehicle, scene.obstacles):
            return (tuple(poses) + (current,), False)
        poses.append(current)
    matching = all(math.hypot(a.x - b.x, a.y - b.y) < 1e-6
                   for a, b in zip(poses, route.poses))
    return tuple(poses), bool(matching and len(poses) == len(route.poses)
                               and _within_goal(poses[-1], scene))
