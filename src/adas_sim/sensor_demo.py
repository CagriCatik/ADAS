"""Transport-neutral, deterministic sensor-first parking demonstration.

No preset scenario goal is used by the detector or planner. Survey data is
acquired before the decision; a parking route exists only if authorized.
"""
from __future__ import annotations
from dataclasses import asdict
from .geometry import Rectangle
from .scenarios import SCENARIOS
from .sensor_driven_parking import evaluate_parking
from .ultrasound_12 import scan_12
from .acoustic_schedule import AcousticSchedule

CASES=("measured_left_gap","measured_right_gap","too_short_gap","acoustic_interference")

def case_setup(name):
    if name not in CASES:raise ValueError("Unknown functional parking scenario")
    if name=="measured_right_gap":
        return SCENARIOS["parallel_right"].obstacles,"right","tdma"
    obstacles=SCENARIOS["parallel_left"].obstacles
    if name=="too_short_gap":
        obstacles=(obstacles[0],Rectangle(10.,2.45,4.5,1.8))
    mode="simultaneous" if name=="acoustic_interference" else "tdma"
    return obstacles,"left",mode

def run_case(name):
    obstacles,side,mode=case_setup(name)
    return evaluate_parking(obstacles,side=side,transmit_mode=mode)

def build_case_payload(name):
    """Survey first, parking only if a measured gap and valid plan exist."""
    obstacles,side,mode=case_setup(name)
    decision=run_case(name)
    survey_period=AcousticSchedule().flight_window_s()*12
    frames=[];elapsed=0.
    for survey in decision.survey:
        previous=frames[-1]["pose"]["x"] if frames else survey.pose.x
        frames.append({
            "pose":asdict(survey.pose),
            "echoes":[asdict(e) for e in survey.echoes],
            "stage":"SEARCH",
            "time_s":round(elapsed,4),
            "front_steer_rad":0.,
            "velocity_m_s":(survey.pose.x-previous)/survey_period,
        })
        elapsed+=survey_period
    if decision.status=="PARK_ALLOWED" and decision.plan is not None:
        for i,pose in enumerate(decision.plan.poses[1:]):
            cmd=decision.plan.commands[i]
            elapsed+=decision.plan.step_dt
            frames.append({
                "pose":asdict(pose),
                "echoes":[asdict(e) for e in scan_12(pose,obstacles,seed=i)],
                "stage":"PARK",
                "time_s":round(elapsed,4),
                "front_steer_rad":cmd.steer,
                "velocity_m_s":cmd.velocity,
            })
    return {
        "name":name,"side":side,"tx_mode":mode,
        "status":decision.status,"reason":decision.reason,
        "obstacles":[asdict(o) for o in obstacles],
        "vehicle":asdict(SCENARIOS["parallel_left"].vehicle),
        "slot":asdict(decision.perception.slot) if decision.perception.slot else None,
        "survey_samples":len(decision.survey),
        "frames":frames,
        "path_steps":len(decision.plan.commands) if decision.plan else 0,
        "validation":asdict(decision.validation) if decision.validation else None,
        "limitations":"Sensor survey samples use frozen pose within each 12-pulse cycle; after authorization the planned trajectory is open-loop with static perceived obstacles.",
    }
