"""Regression tests: sensor-authorized parking, no-slot inhibition, and TDMA.

This suite deliberately verifies that ground truth hit coordinates are never
needed to generate a target and that interference never masquerades as free space.
"""
from dataclasses import replace
from adas_sim.geometry import Rectangle,Pose
from adas_sim.scenarios import SCENARIOS
from adas_sim.sensor_driven_parking import evaluate_parking
from adas_sim.parking_perception import detect_parallel_gap,SurveyFrame
from adas_sim.acoustic_schedule import AcousticSchedule,invalidate_interference
from adas_sim.ultrasound_12 import scan_12

LEFT=SCENARIOS["parallel_left"].obstacles
RIGHT=SCENARIOS["parallel_right"].obstacles

def test_measured_bounded_bay_plans_without_ground_truth_map():
    for side,obstacles in (("left",LEFT),("right",RIGHT)):
        result=evaluate_parking(obstacles,side=side)
        assert result.status=="PARK_ALLOWED",result.reason
        slot=result.perception.slot
        assert slot is not None
        assert slot.certified_length_m>=5.1
        assert slot.supporting_echoes>=24
        assert slot.free_space_checks>=5
        assert result.plan and result.plan.commands
        assert result.validation and result.validation.result=="PASS"
        assert slot.perceived_obstacles!=obstacles

def test_slot_position_is_measured_not_hardcoded():
    base=evaluate_parking(LEFT)
    moved=(LEFT[0],Rectangle(16,2.45,4.5,1.8))
    result=evaluate_parking(moved,survey_end=21)
    assert result.status=="PARK_ALLOWED",result.reason
    assert result.perception.slot.goal.x>base.perception.slot.goal.x+.7

def test_oracle_hit_coordinates_not_used():
    result=evaluate_parking(LEFT)
    frames=tuple(SurveyFrame(f.pose,tuple(replace(e,hit_x=None,hit_y=None) for e in f.echoes))
                 for f in result.survey)
    assert detect_parallel_gap(frames,"left")==result.perception

def test_short_and_unbounded_bays_block_before_planning():
    cases=((LEFT[0],Rectangle(8.5,2.45,4.5,1.8)),
           (LEFT[0],Rectangle(10,2.45,4.5,1.8)),(LEFT[0],))
    for obstacles in cases:
        result=evaluate_parking(obstacles)
        assert result.status=="NO_PARK"
        assert result.plan is None and result.validation is None

def test_dropouts_reject_freedom_assumption():
    result=evaluate_parking(LEFT,dropout_probability=1.)
    assert result.status=="NO_PARK" and result.plan is None

def test_acoustic_interference_requires_timing_conflict():
    raw=scan_12(Pose(0,0,0),LEFT)
    schedule=AcousticSchedule()
    ids=[e.id for e in raw]
    assert all(e.status!="INTERFERENCE" for e in invalidate_interference(raw,schedule.tdma(ids)))
    simultaneous=invalidate_interference(raw,schedule.simultaneous(ids))
    assert all(e.status=="INTERFERENCE" for e in simultaneous)
    blocked=evaluate_parking(LEFT,transmit_mode="simultaneous")
    assert blocked.status=="NO_PARK"
    assert blocked.reason=="unreliable_acoustic_measurements"
    assert blocked.plan is None

def test_truncated_survey_cannot_fabricate_second_boundary():
    result=evaluate_parking(LEFT,survey_end=6)
    assert result.status=="NO_PARK" and result.plan is None
