"""Twelve-channel ultrasonic mounting, beam geometry and web payload regression."""
from dataclasses import asdict
from math import isclose,pi
from adas_sim.geometry import Pose,Rectangle,Vehicle
from adas_sim.ultrasound_12 import default_channels,mount_world,scan_12
from adas_sim.web_app import _HTML

def test_twelve_unique_bumper_channels():
    channels=default_channels()
    assert len(channels)==12
    assert len({c.id for c in channels})==12
    assert [c.id for c in channels[:6]]==[f"F{i:02}" for i in range(1,7)]
    assert [c.id for c in channels[6:]]==[f"R{i:02}" for i in range(1,7)]
    assert all(c.min_range_m<c.max_range_m for c in channels)

def test_mounts_follow_vehicle_rotation():
    sensor=default_channels()[0]
    origin=mount_world(Pose(0,0,0),sensor)
    rotated=mount_world(Pose(2,3,pi/2),sensor)
    assert isclose(rotated[0],2-origin[1],abs_tol=1e-9)
    assert isclose(rotated[1],3+origin[0],abs_tol=1e-9)

def test_front_sensor_detects_nearby_obstacle():
    vehicle=Vehicle()
    rect=(Rectangle(4.5,0,1.0,3.0),)
    echoes=scan_12(Pose(0,0,0),rect,vehicle=vehicle)
    assert len(echoes)==12
    assert any(e.status=="DETECTED" for e in echoes[:6])
    assert any(e.range_m is not None and e.range_m>0 for e in echoes)
    assert all(e.range_m is None or e.min_range_m<=e.range_m<=e.max_range_m for e in echoes)

def test_no_echo_and_dropout_are_distinct():
    no_echo=scan_12(Pose(0,0,0),())
    assert all(e.status=="NO_ECHO" and e.range_m is None for e in no_echo)
    dropout=scan_12(Pose(0,0,0),(),dropout_probability=1)
    assert all(e.status=="DROPOUT" and e.range_m is None for e in dropout)

def test_seeded_scan_and_json_compatible_record():
    scene=(Rectangle(4.5,0,1.,3.),)
    a=scan_12(Pose(0,0,0),scene,seed=45,noise_std_m=.03)
    b=scan_12(Pose(0,0,0),scene,seed=45,noise_std_m=.03)
    assert a==b
    assert "hit_x" in asdict(a[0])

def test_viewer_has_sensor_visibility_toggle_and_12_readouts():
    assert 'id="sensorGrid"' in _HTML
    assert 'id="beams"' in _HTML
    assert "function sensors" in _HTML
    assert "hit_x" in _HTML
