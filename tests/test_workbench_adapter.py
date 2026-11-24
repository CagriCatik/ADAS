"""Adapter / replanning smoke tests for a simulated plant boundary."""
from dataclasses import dataclass
import math
from types import SimpleNamespace
import pytest
from adas_sim.geometry import Pose, Rectangle, Vehicle
from adas_sim.plant import Command
from adas_sim.supervisor import ParkingSupervisor, Status
from adas_sim.vehicle_adapter import VehicleDynamicsKinematicAdapter

def test_replan_after_obstacle_change():
    vehicle=Vehicle()
    sup=ParkingSupervisor(vehicle)
    sup.begin()
    sup.update_obstacles((Rectangle(12, 10, 2, 2),))
    assert sup.status is Status.REPLAN_REQUIRED
    plan=sup.replan(Pose(0,0,0),Pose(3,0,0))
    assert plan.success
    assert sup.status is Status.ACTIVE

def test_replan_rejects_blocked_goal():
    sup=ParkingSupervisor(Vehicle())
    sup.begin()
    sup.update_obstacles((Rectangle(3,0,4.5,1.8),))
    route=sup.replan(Pose(-5,0,0),Pose(3,0,0))
    assert not route.success
    assert route.reason == "goal_in_collision"
    assert sup.status is Status.STOPPED

def test_external_plant_geometry_must_match():
    vehicle=Vehicle()
    model=SimpleNamespace(params=SimpleNamespace(wheelbase=999,lr=1.35))
    with pytest.raises(ValueError,match="wheelbase"):
        VehicleDynamicsKinematicAdapter(vehicle,external_model=model)

def test_external_plant_reference_frame_conversion():
    np=pytest.importorskip("numpy")
    class FakeExternal:
        def __init__(self):
            self.params=SimpleNamespace(wheelbase=2.75,lr=1.35)
        def odeintRK4(self,state,times,u):
            p=state.copy()
            p[3]+=u[0]*(times[-1]-times[0])
            p[4]+=u[1]*(times[-1]-times[0])
            p[0]+=p[3]*(times[-1]-times[0])
            return p
    adapter=VehicleDynamicsKinematicAdapter(external_model=FakeExternal())
    adapter.reset(Pose(2.,3.,0.))
    output=adapter.step(Command(0.,.2),.1)
    assert math.isclose(output.x,2.01,abs_tol=1e-9)
    assert math.isclose(output.y,3.,abs_tol=1e-9)
