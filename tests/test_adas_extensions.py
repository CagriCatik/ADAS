"""Regression checks for bounded execution, ultrasonic sensing and safety."""
from adas_sim.geometry import Pose,Rectangle,Vehicle
from adas_sim.plant import Command
from adas_sim.control import execute
from adas_sim.sensors import ultrasound,noisy_pose
from adas_sim.safety import evaluate
from adas_sim.planner import plan
from adas_sim.scenarios import SCENARIOS

def test_ultrasound_hit():
    measurements=ultrasound(Pose(0,0,0),(Rectangle(4,0,2,2),),bearings=(0,),noise_std_m=0)
    assert measurements[0].valid and abs(measurements[0].range_m-3)<1e-8

def test_emergency_stop():
    decision=evaluate(Pose(0,0,0),Command(0,1),Vehicle(),
                      (Rectangle(4,0,2,2),),horizon_s=3)
    assert not decision.allowed and decision.output.velocity==0

def test_seeded_sensor_noise():
    a=noisy_pose(Pose(0,0,0),seed=7)
    assert a==noisy_pose(Pose(0,0,0),seed=7)

def test_bounded_execution_is_independently_labelled():
    scene=SCENARIOS["open_bay"]
    result=execute(scene,plan(scene))
    assert len(result.poses)==len(result.commands)+1
    assert result.status in ("PASS","TRACKING_ERROR","COLLISION")
    assert all(abs(cmd.steer)<=scene.vehicle.max_steer for cmd in result.commands)
