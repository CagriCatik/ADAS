"""Regression invariants for the shared, fixed-scale visualization layer."""
from dataclasses import replace
import pytest
from PIL import Image

from adas_sim.geometry import Pose
from adas_sim.scenarios import SCENARIOS
from adas_sim.planner import plan
from adas_sim.animation import render_gif
from adas_sim.svg import render as render_svg
from adas_sim.steering_benchmark import compare, maneuvers
from adas_sim.steering_animation import render_comparison
from adas_sim.visualization.common import CAMERA, Camera, image_indices, PALETTE
from adas_sim.visualization.plots import bar_chart

def test_world_scale_is_shared_across_all_parking_scenarios():
    cam=Camera(CAMERA["parking"])
    assert cam.xy((0.,0.))[0]-cam.xy((1.,0.))[0] == -29
    for scene in SCENARIOS.values():
        cam.assert_visible(pt for pose in (scene.start,scene.goal)
                           for pt in ((pose.x,pose.y),))

def test_stereo_panels_have_identical_world_scale():
    left=Camera(CAMERA["steering"],0,2)
    right=Camera(CAMERA["steering"],1,2)
    for camera in (left,right):
        assert camera.xy((1,0))[0]-camera.xy((0,0))[0]==22

def test_family_specs_are_distinct_but_stable():
    assert CAMERA["parking"].width == 1280
    assert CAMERA["parking"].height == 720
    assert CAMERA["steering"].width == 1600
    assert CAMERA["steering"].height == 800
    assert PALETTE["2WS"] != PALETTE["4WS"]

def test_camera_rejects_out_of_bounds_geometry():
    with pytest.raises(ValueError,match="fixed camera bounds"):
        Camera(CAMERA["parking"]).assert_visible(((500.,500.),))

def test_playback_includes_last_frame_and_respects_budget():
    spec=replace(CAMERA["parking"],max_frames=12)
    result=image_indices(501,.05,spec)
    assert result[0]==0 and result[-1]==500
    assert len(result)<=12

def test_gif_and_svg_use_identical_parking_canvas(tmp_path):
    scene=SCENARIOS["open_bay"]
    trajectory=plan(scene)
    gif=tmp_path/"parking.gif";svg=tmp_path/"parking.svg"
    render_gif(scene,trajectory,gif)
    render_svg(scene,trajectory,svg)
    with Image.open(gif) as image:
        assert image.size == (1280,720)
    assert 'viewBox="0 0' in svg.read_text()\n    assert "matplotlib" in svg.read_text().lower()

def test_two_wheel_comparison_has_shared_panels(tmp_path):
    pair=compare(maneuvers()["left_u_turn"])
    gif=tmp_path/"steering.gif"
    render_comparison("left_u_turn",pair,gif,max_frames=10)
    with Image.open(gif) as image:
        assert image.size==(1600,800)
        assert image.n_frames>=2

def test_bar_chart_fixed_scale_and_safety(tmp_path):
    svg=tmp_path/"metric.svg"
    bar_chart([{"name":"2WS","architecture":"2WS","value":4.9},
               {"name":"4WS","architecture":"4WS","value":3.9}],
              label_key="name",value_key="value",group_key="architecture",
              out=svg,title="Comparison",y_max=8.0)
    assert 'viewBox="0 0 1280 720"' in svg.read_text()
    with pytest.raises(ValueError,match="shared plot scale"):
        bar_chart([{"name":"too high","value":9.}],
                  label_key="name",value_key="value",out=svg,title="Limit",y_max=8.0)
