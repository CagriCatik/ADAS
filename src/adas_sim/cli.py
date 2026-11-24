"""CLI entry point for reproducible parking scenarios."""
import argparse
import json
from pathlib import Path
from dataclasses import asdict
from .scenarios import SCENARIOS
from .planner import plan, Settings
from .validation import validate
from .svg import render


def main():
    parser = argparse.ArgumentParser(description="Autonomous parking research baseline")
    parser.add_argument("--scenario", choices=["all", *SCENARIOS], default="open_bay")
    parser.add_argument("--out", default="output/parking")
    parser.add_argument("--max-expansions", type=int, default=60000)
    args = parser.parse_args()
    output = Path(args.out)
    output.mkdir(parents=True, exist_ok=True)
    summary = []
    names = SCENARIOS if args.scenario == "all" else (args.scenario,)
    for name in names:
        scene = SCENARIOS[name]
        route = plan(scene, Settings(max_expansions=args.max_expansions))
        report = validate(scene, route)
        render(scene, route, output / f"{name}.svg")
        (output / f"{name}.json").write_text(json.dumps({
            "report": asdict(report), "reason": route.reason, "step_dt": route.step_dt,
            "poses": [asdict(p) for p in route.poses],
            "controls": [asdict(c) for c in route.commands],
        }, indent=2) + "\n")
        summary.append(asdict(report))
        print(f"{name}: {report.result} ({route.reason}); expansions={route.expanded}, position error={report.final_position_error_m:.3f} m")
    (output / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")
    return 0 if all(x["result"] == "PASS" for x in summary) else 1


if __name__ == "__main__":
    raise SystemExit(main())
