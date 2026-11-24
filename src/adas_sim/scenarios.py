"""Deterministic benchmark worlds; the reference pose is the rear axle."""
from dataclasses import dataclass
import math
from .geometry import Pose, Rectangle, Vehicle


@dataclass(frozen=True)
class Scenario:
    name: str
    description: str
    start: Pose
    goal: Pose
    obstacles: tuple[Rectangle, ...]
    vehicle: Vehicle = Vehicle()
    position_tolerance: float = 0.65
    yaw_tolerance: float = math.radians(16)


SCENARIOS = {
    "parallel_left": Scenario("parallel_left", "Parallel parking into an open curb-side bay", Pose(0, 0, 0), Pose(7.0, 2.45, 0),
                              (Rectangle(1.0, 2.45, 4.5, 1.8), Rectangle(14.0, 2.45, 4.5, 1.8))),
    "parallel_right": Scenario("parallel_right", "Mirrored curb-side parallel parking", Pose(0, 0, 0), Pose(7.0, -2.45, 0),
                               (Rectangle(1.0, -2.45, 4.5, 1.8), Rectangle(14.0, -2.45, 4.5, 1.8))),
    "perpendicular_reverse": Scenario("perpendicular_reverse", "Back into a perpendicular space", Pose(0, 0, 0), Pose(6.0, -3.7, math.pi / 2),
                                      (Rectangle(3.4, -5.5, 4.5, 1.8, math.pi / 2), Rectangle(8.6, -5.5, 4.5, 1.8, math.pi / 2))),
    "open_bay": Scenario("open_bay", "Basic open-space pose acquisition benchmark", Pose(0, 0, 0), Pose(5, 2.5, 0), ()),
}
