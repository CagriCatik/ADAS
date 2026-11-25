"""Functional viewer must not construct a parking path before ultrasonic authorization."""
from adas_sim.sensor_demo import CASES,build_case_payload
from adas_sim.web_app import _HTML

def test_functional_scenarios_enforce_decisions():
    expected={"measured_left_gap":"PARK_ALLOWED",
              "measured_right_gap":"PARK_ALLOWED",
              "too_short_gap":"NO_PARK",
              "acoustic_interference":"NO_PARK"}
    assert set(CASES)==set(expected)
    for case,status in expected.items():
        data=build_case_payload(case)
        assert data["status"]==status,(case,data["reason"])
        assert data["survey_samples"]>=10
        assert all(len(frame["echoes"])==12 for frame in data["frames"])
        assert all(frame["stage"]=="SEARCH" for frame in data["frames"][:data["survey_samples"]])
        if status=="NO_PARK":
            assert data["slot"] is None
            assert data["path_steps"]==0
            assert len(data["frames"])==data["survey_samples"]
        else:
            assert data["slot"] is not None
            assert data["path_steps"]>0
            assert all(frame["stage"]=="PARK" for frame in data["frames"][data["survey_samples"]:])

def test_survey_data_is_not_a_preset_goal():
    from adas_sim.scenarios import SCENARIOS
    data=build_case_payload("measured_left_gap")
    assert abs(data["slot"]["goal"]["x"]-SCENARIOS["parallel_left"].goal.x)>.2

def test_browser_gates_goal_and_beam_rendering():
    assert "frame>=data.survey_samples-1" in _HTML
    assert 'data.tx_mode==="tdma"' in _HTML
    assert "INTERFERENCE" in _HTML
    assert "data.phases[frame]" in _HTML
