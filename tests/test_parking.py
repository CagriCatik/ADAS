from adas_sim.scenarios import SCENARIOS, Scenario
from adas_sim.geometry import Pose, Rectangle
from adas_sim.planner import plan, Settings
from adas_sim.validation import validate


def test_reference_scenarios():
    for s in SCENARIOS.values():
        route = plan(s)
        report = validate(s, route)
        assert report.result == "PASS", (s.name, route.reason, report)


def test_colliding_goal_is_rejected():
    base = SCENARIOS["open_bay"]
    s = Scenario("blocked", "Goal occupied", base.start, base.goal,
                 (Rectangle(base.goal.x, base.goal.y, 4.5, 1.8),), base.vehicle)
    route = plan(s)
    assert not route.success
    assert route.reason == "goal_in_collision"


def test_search_budget_is_reported():
    route = plan(SCENARIOS["parallel_left"], Settings(max_expansions=1))
    assert not route.success
    assert route.reason in {"search_limit", "no_path"}


def test_start_already_at_goal():
    s = Scenario("done", "Already parked", Pose(0, 0, 0), Pose(0, 0, 0), ())
    route = plan(s)
    assert route.success
    assert validate(s, route).result == "PASS"
