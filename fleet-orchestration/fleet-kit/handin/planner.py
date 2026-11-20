"""Hand-in entry point:  python3 planner.py scenario.json plan.json

Pipeline (see fleetplan/): preprocessing -> ALNS over scout tours with effector matching ->
3D realisation with separation deconfliction -> official validator and repair -> best of
(our plan, greedy baseline). A plan is always written, and it is always valid.

Environment: FLEET_BUDGET (seconds, default 45) caps the run time; FLEET_NO_BASELINE=1 skips the
baseline comparison (used by the benchmark);
FLEET_VERIFY=1 adds re-observations for the optional verify board (lowers the main score).
"""
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
if HERE not in sys.path:
    sys.path.insert(0, HERE)
try:  # the evaluation puts the official fleetkit on the path; locally, fall back to the kit folder
    import fleetkit  # noqa: F401
except ImportError:
    sys.path.append(os.path.dirname(HERE))

from fleetkit.model import Scenario, load_json, save_json  # noqa: E402
from fleetkit.scorer import evaluate  # noqa: E402

HARD_LIMIT = float(os.environ.get("FLEET_HARD_LIMIT", "55"))  # the evaluation kills runs at 60 s


def plan_for(data: dict, budget: float, use_baseline: bool = True, log=None, verify: bool = False):
    """Returns (plan, score, source).

    The greedy baseline runs first (a few seconds, even on L4), so a valid fallback exists before
    the optimiser starts; the optimiser then gets the rest of the budget.
    """
    t0 = time.monotonic()
    sc = Scenario.from_dict(data)
    best, best_score, source = {"seed": sc.seed, "routes": []}, 0.0, "empty"
    if use_baseline:
        try:
            from fleetkit.baseline import greedy_plan
            plan = greedy_plan(sc)
            res = evaluate(sc, plan, verify=verify)
            if res.valid and res.score > best_score:
                best, best_score, source = plan, res.score, "baseline"
        except Exception as exc:
            if log is not None:
                log["baseline_error"] = repr(exc)
    remaining = min(budget, HARD_LIMIT) - (time.monotonic() - t0)
    if remaining > 2.0:
        try:
            from fleetplan.pipeline import solve
            plan, score = solve(sc, time.monotonic(), remaining, log, verify=verify)
            if plan is not None and score > best_score:
                best, best_score, source = plan, score, "alns"
        except Exception as exc:  # never crash: keep the fallback
            if log is not None:
                log["error"] = repr(exc)
    return best, best_score, source


def main() -> None:
    budget = float(os.environ.get("FLEET_BUDGET", "45"))
    use_baseline = os.environ.get("FLEET_NO_BASELINE", "") != "1"
    verify = os.environ.get("FLEET_VERIFY", "") == "1"
    data = load_json(sys.argv[1])
    try:
        plan, score, source = plan_for(data, budget, use_baseline, verify=verify)
    except Exception:
        plan, score, source = {"seed": data.get("seed"), "routes": []}, 0.0, "empty"
    save_json(plan, sys.argv[2])
    print(f"{source} {score:.2f}")


if __name__ == "__main__":
    main()
