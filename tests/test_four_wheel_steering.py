"""Planar 2WS/4WS kinematic equivalence, symmetry and visual smoke tests."""
import math
from adas_sim.geometry import Pose,Vehicle
from adas_sim.plant import Command,step
from adas_sim.four_wheel_steering import step_4ws
from adas_sim.steering_benchmark import compare,maneuvers
from adas_sim.planar_vehicle import planar_parts

def test_two_wheel_model_matches_existing_ackermann():
    p=Pose(1.,2.,.2);v=Vehicle()
    a=step(p,Command(.2,1.),v,.01)
    b=step_4ws(p,.2,0.,1.,v,.01)
    assert math.isclose(a.x,b.x,abs_tol=1e-12)
    assert math.isclose(a.y,b.y,abs_tol=1e-12)
    assert math.isclose(a.yaw,b.yaw,abs_tol=1e-12)

def test_counterphase_changes_turning_radius():
    result=compare(maneuvers()["left_u_turn"])
    a=result["2WS"][3];b=result["4WS"][3]
    assert b["effective_radius_m"]<a["effective_radius_m"]
    assert a["rear_steer_max_deg"]==0
    assert 0<b["rear_steer_max_deg"]<=10

def test_left_right_mirror():
    left=compare(maneuvers()["left_u_turn"])
    right=compare(maneuvers()["right_u_turn"])
    for mode in ("2WS","4WS"):
        l=left[mode][0][-1];r=right[mode][0][-1]
        assert math.isclose(l.x,r.x,abs_tol=1e-8)
        assert math.isclose(l.y,-r.y,abs_tol=1e-8)
        assert math.isclose(l.yaw,-r.yaw,abs_tol=1e-8)

def test_four_independent_wheel_polygons():
    shapes=planar_parts(Pose(0.,0.,0.),Vehicle(),.2,-.1)
    assert len(shapes)==5 and all(len(poly)==4 for poly in shapes)

def test_comparison_gif(tmp_path):
    from PIL import Image
    from adas_sim.steering_animation import render_comparison
    target=tmp_path/"comparison.gif"
    render_comparison("left_u_turn",compare(maneuvers()["left_u_turn"]),target,max_frames=12)
    with Image.open(target) as im:
        assert im.n_frames>2
        assert im.size==(1600,800)
