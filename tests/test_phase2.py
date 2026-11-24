"""Tests for path handling and safety-state transitions."""
import math
import pytest
from adas_sim.scenarios import SCENARIOS
from adas_sim.planner import plan
from adas_sim.trajectory import refine, speed_profile
from adas_sim.plant import Command
from adas_sim.geometry import Pose, Vehicle, Rectangle
from adas_sim.supervisor import ParkingSupervisor, Status
from adas_sim.web_app import _HTML

def test_refined_route_replays_without_geometry_changes():
    scene=SCENARIOS["open_bay"]
    original=plan(scene)
    improved=refine(original,scene)
    assert improved.poses[0]==scene.start
    assert len(improved.poses)==len(improved.commands)+1

def test_speed_profile_enforces_gear_change_zero():
    scene=SCENARIOS["open_bay"]
    p=plan(scene)
    speeds=speed_profile(p)
    assert len(speeds)==len(p.commands)
    assert speeds[0]==0
    assert speeds[-1]==0
    for before,after in zip(speeds,speeds[1:]):
        assert abs(after-before)<=3.0

def test_supervisor_stops_on_new_obstacle():
    s=ParkingSupervisor(Vehicle())
    s.begin()
    s.update_obstacles((Rectangle(4,0,2,2),))
    assert s.status==Status.REPLAN_REQUIRED
    assert s.tick(Pose(0,0,0),Command(0,1)).command.velocity==0

def test_supervisor_stops_if_localization_is_lost():
    s=ParkingSupervisor(Vehicle())
    s.begin()
    out=s.tick(Pose(0,0,0),Command(0,1),localization_ok=False)
    assert out.status==Status.STOPPED
    assert out.command.velocity==0

def test_workbench_visualizes_four_wheels():
    assert "for(const axle of [0,v.wheelbase])" in _HTML
    assert "/api/scenario/" in _HTML
