"""PLA geometry and functional regression tests for 4+4+4 configuration."""
from dataclasses import replace
from math import isclose,pi
from adas_sim.geometry import Pose,Rectangle,Vehicle
from adas_sim.ultrasound_12 import default_channels,mount_world,scan_12
from adas_sim.sensor_driven_parking import evaluate_parking
from adas_sim.scenarios import SCENARIOS
from adas_sim.parking_perception import SurveyFrame,detect_parallel_gap

def test_pla_transducers_have_four_front_four_rear_four_side():
    c=default_channels()
    assert len(c)==12
    assert [p.id for p in c]==[
        "F01","F02","F03","F04","R01","R02","R03","R04",
        "SLF","SRF","SLR","SRR"
    ]
    assert all(p.role=="slot_survey" for p in c[8:])
    assert all(p.role=="obstacle_monitor" for p in c[:8])
    assert all(p.x_body>0 for p in c[:4])
    assert all(p.x_body<0 for p in c[4:8])
    assert all(abs(abs(p.yaw_body)-pi/2)<1e-8 for p in c[8:])
    assert c[8].y_body>0 and c[10].y_body>0
    assert c[9].y_body<0 and c[11].y_body<0

def test_left_right_side_layout_is_mirrored():
    by_id={p.id:p for p in default_channels()}
    for a,b in (("SLF","SRF"),("SLR","SRR")):
        x,y=by_id[a],by_id[b]
        assert isclose(x.x_body,y.x_body,abs_tol=1e-9)
        assert isclose(x.y_body,-y.y_body,abs_tol=1e-9)
        assert isclose(x.yaw_body,-y.yaw_body,abs_tol=1e-9)

def test_all_side_mounts_stay_on_actual_vehicle_outline():
    v=Vehicle()
    lo=-v.rear_axle_from_center
    hi=v.length-v.rear_axle_from_center
    for sensor in default_channels(v):
        assert lo<=sensor.x_body<=hi
        assert abs(sensor.y_body)<=v.width/2

def test_two_side_sensors_independently_confirm_bay_and_goal():
    for side,name in (("left","parallel_left"),("right","parallel_right")):
        decision=evaluate_parking(SCENARIOS[name].obstacles,side=side)
        assert decision.status=="PARK_ALLOWED",decision.reason
        assert decision.validation and decision.validation.result=="PASS"
        assert decision.perception.slot and decision.perception.slot.certified_length_m>=5.1
        assert decision.plan and len(decision.plan.commands)>0

def test_front_rear_bumper_measurements_cannot_fake_side_gap():
    d=evaluate_parking(SCENARIOS["parallel_left"].obstacles)
    frames=tuple(SurveyFrame(x.pose,tuple(replace(e,status="DROPOUT",range_m=None)
                if e.id in ("SLF","SLR") else e for e in x.echoes)) for x in d.survey)
    result=detect_parallel_gap(frames,"left")
    assert result.slot is None

def test_insufficient_space_and_interference_prevent_planning():
    original=SCENARIOS["parallel_left"].obstacles
    tiny=(original[0],Rectangle(10,2.45,4.5,1.8))
    denied=evaluate_parking(tiny)
    assert denied.status=="NO_PARK" and denied.plan is None
    denied=evaluate_parking(original,transmit_mode="simultaneous")
    assert denied.status=="NO_PARK" and denied.plan is None
