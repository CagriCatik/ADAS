"""Sampling planner regression and GIF smoke tests."""
from adas_sim.geometry import Pose, Rectangle
from adas_sim.scenarios import Scenario, SCENARIOS
from adas_sim.sampling import choose, SamplingSettings
from adas_sim.validation import validate
from adas_sim.animation import render_gif
from PIL import Image

def test_seeded_paths_are_reproducible():
    scene=SCENARIOS["open_bay"]
    settings=SamplingSettings(iterations=80,max_vertices=65,seed=11)
    for method in ("rrt","rrt_star","fmt_star"):
        a=choose(scene,method,settings=settings)
        b=choose(scene,method,settings=settings)
        assert a.poses==b.poses
        assert a.commands==b.commands
        validate(scene,a)

def test_blocked_goal_rejected():
    original=SCENARIOS["open_bay"]
    blocked=Scenario("blocked","blocked goal",original.start,original.goal,
                     (Rectangle(original.goal.x,original.goal.y,4.5,1.8),))
    for method in ("hybrid_astar","rrt","rrt_star","fmt_star"):
        solution=choose(blocked,method)
        assert solution.reason=="goal_in_collision"
        assert not solution.success

def test_gif_contains_multiple_planar_vehicle_frames(tmp_path):
    scene=Scenario("straight","simple",Pose(0,0,0),Pose(3,0,0),())
    route=choose(scene,"hybrid_astar")
    assert validate(scene,route).result=="PASS"
    target=tmp_path/"demo.gif"
    render_gif(scene,route,target,fps=10)
    with Image.open(target) as im:
        assert im.n_frames > 1
        assert im.size == (1280,720)
