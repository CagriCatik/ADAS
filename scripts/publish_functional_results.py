"""Publish evidence-based sensor-first parking demos, including rejected cases.

Only publish when both positive and negative decision tests pass. No planner
fallback, preselected target or speculative 'successful parking' animation.
"""
from __future__ import annotations
from pathlib import Path
import csv,json
from adas_sim.sensor_demo import CASES,run_case,build_case_payload
from adas_sim.functional_visualization import render_case

EXPECTED={
    "measured_left_gap":"PARK_ALLOWED",
    "measured_right_gap":"PARK_ALLOWED",
    "too_short_gap":"NO_PARK",
    "acoustic_interference":"NO_PARK",
}

def publish(folder="docs/assets/functional_parking"):
    directory=Path(folder)
    directory.mkdir(parents=True,exist_ok=True)
    summaries=[]
    for name in CASES:
        result=run_case(name)
        expected=EXPECTED[name]
        if result.status!=expected:
            raise AssertionError(f"{name}: expected {expected}, got {result.status}: {result.reason}")
        if result.status=="PARK_ALLOWED":
            if result.plan is None or result.validation is None or result.validation.result!="PASS":
                raise AssertionError("Cannot publish unsuccessful parking as PASS")
        elif result.plan is not None:
            raise AssertionError("NO_PARK must have no parking plan")
        payload=render_case(name,directory,max_frames=20)
        slot=result.perception.slot
        summary={
            "case":name,
            "decision":result.status,
            "reason":result.reason,
            "survey_frames":len(result.survey),
            "transmit_mode":payload["tx_mode"],
            "detected_gap_m":round(slot.certified_length_m,3) if slot else None,
            "slot_goal_x_m":round(slot.goal.x,3) if slot else None,
            "slot_goal_y_m":round(slot.goal.y,3) if slot else None,
            "parking_commands":len(result.plan.commands) if result.plan else 0,
            "collision_free_replay":bool(result.validation and result.validation.result=="PASS"),
            "acoustic_model":"idealized_12ch_TDMA_with_interference_quality_flags",
            "planning_source":"only_reconstructed_ultrasonic_map",
        }
        summaries.append(summary)
        sensor_file=directory/f"{name}_survey.csv"
        with sensor_file.open("w",newline="",encoding="utf-8") as f:
            writer=csv.writer(f)
            writer.writerow(["survey_index","x_m","y_m",*[f"{e.id}_range_m" for e in result.survey[0].echoes],
                             *[f"{e.id}_status" for e in result.survey[0].echoes]])
            for i,frame in enumerate(result.survey):
                writer.writerow([i,frame.pose.x,frame.pose.y,
                                 *[e.range_m if e.range_m is not None else "" for e in frame.echoes],
                                 *[e.status for e in frame.echoes]])
        print(f"{name}: {result.status} ({result.reason}), gap={summary['detected_gap_m']}",flush=True)
    with (directory/"summary.csv").open("w",newline="",encoding="utf-8") as f:
        writer=csv.DictWriter(f,fieldnames=list(summaries[0]))
        writer.writeheader()
        writer.writerows(summaries)
    (directory/"summary.json").write_text(json.dumps(summaries,indent=2)+"\n",encoding="utf-8")
    return summaries

if __name__=="__main__":
    publish()
