"""Explicit park-assist safety state machine and obstacle-change supervision."""
from __future__ import annotations
from dataclasses import dataclass
from enum import Enum
from .geometry import footprint_collision
from .plant import Command
from .safety import evaluate

class Status(str, Enum):
    IDLE = "IDLE"
    ACTIVE = "ACTIVE"
    STOPPED = "STOPPED"
    REPLAN_REQUIRED = "REPLAN_REQUIRED"
    COMPLETE = "COMPLETE"
    FAILED = "FAILED"

@dataclass(frozen=True)
class Supervision:
    status: Status
    command: Command
    reason: str

class ParkingSupervisor:
    def __init__(self, vehicle, obstacles=()):
        self.vehicle = vehicle
        self.obstacles = tuple(obstacles)
        self.status = Status.IDLE

    def begin(self):
        if self.status not in (Status.IDLE, Status.STOPPED, Status.REPLAN_REQUIRED):
            raise RuntimeError("cannot start from " + self.status.value)
        self.status = Status.ACTIVE

    def update_obstacles(self, obstacles):
        changed = tuple(obstacles) != self.obstacles
        self.obstacles = tuple(obstacles)
        if changed and self.status == Status.ACTIVE:
            self.status = Status.REPLAN_REQUIRED

    def tick(self, pose, command, *, localization_ok=True):
        stop = Command(0.0, 0.0)
        if self.status != Status.ACTIVE:
            return Supervision(self.status, stop, "inactive")
        decision = evaluate(pose, command, self.vehicle, self.obstacles,
                            localization_ok=localization_ok)
        if not decision.allowed:
            self.status = Status.STOPPED
            return Supervision(self.status, stop, decision.reason)
        return Supervision(self.status, decision.output, "clear")

    def replan(self, pose, goal, *, name="replanned"):
        """Recompute a route after an obstacle update. No automatic resume on failure."""
        from .scenarios import Scenario
        from .planner import plan
        if self.status not in (Status.REPLAN_REQUIRED, Status.STOPPED):
            raise RuntimeError("replan requires stopped or changed-obstacle state")
        scene = Scenario(name, "Replanned parking", pose, goal, self.obstacles, self.vehicle)
        result = plan(scene)
        self.status = Status.ACTIVE if result.success else Status.STOPPED
        return result

    def finish(self):
        if self.status != Status.ACTIVE:
            raise RuntimeError("cannot finish")
        self.status = Status.COMPLETE
