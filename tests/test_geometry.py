import math
from adas_sim.geometry import Pose, Rectangle, Vehicle, footprint_collision, polygons_overlap, polygon
from adas_sim.plant import Command, step


def test_collision_at_overlap_and_separation():
    vehicle = Vehicle()
    assert footprint_collision(Pose(0, 0, 0), vehicle, (Rectangle(0, 0, 2, 2),))
    assert not footprint_collision(Pose(0, 0, 0), vehicle, (Rectangle(20, 20, 2, 2),))


def test_rotated_rectangle_sat():
    a = polygon(0, 0, 0, 4, 2)
    b = polygon(1, 0, math.pi / 4, 2, 2)
    assert polygons_overlap(a, b)


def test_reverse_motion_and_steer_limit():
    vehicle = Vehicle()
    rearward = step(Pose(0, 0, 0), Command(0, -1), vehicle, 0.1)
    assert rearward.x < 0
    try:
        step(Pose(0, 0, 0), Command(2, 1), vehicle, 0.1)
    except ValueError:
        pass
    else:
        assert False, "invalid steering was accepted"
