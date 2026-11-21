"""Hand-in entry point:  python3 planner.py scenario.json plan.json

1. the official greedy baseline runs first, so a valid plan exists within seconds;
2. the optimiser (fleetplan: ALNS + 3D realisation + validator repair) gets the rest of the budget;
3. the better valid plan is written. The planner never crashes and never writes an invalid plan.

Run-time switches (FLEET_BUDGET, FLEET_VERIFY, ...) are documented in fleetplan/config.py.
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

MIN_OPTIMISER_S = 2.0      # below this, the optimiser is not worth starting
DEFAULT_HARD_LIMIT_S = 55.0


def _now() -> float:
    """The planner's clock (virtual in deterministic test runs)."""
    try:
        from fleetplan import clock
        return clock.now()
    except Exception:
        return time.monotonic()


def _hard_limit() -> float:
    try:
        from fleetplan.config import SETTINGS
        return SETTINGS.hard_limit_s
    except Exception:
        return DEFAULT_HARD_LIMIT_S


def _baseline(sc: Scenario, verify: bool, log):
    """Official greedy plan, or None if it fails or is invalid."""
    try:
        from fleetkit.baseline import greedy_plan
        plan = greedy_plan(sc)
        res = evaluate(sc, plan, verify=verify)
        return (plan, res.score) if res.valid else None
    except Exception as exc:
        if log is not None:
            log["baseline_error"] = repr(exc)
        return None


def _optimise(sc: Scenario, budget_s: float, verify: bool, log):
    """Our pipeline, or None if it fails or produces no valid plan."""
    try:
        from fleetplan.pipeline import solve
        plan, score = solve(sc, _now(), budget_s, log, verify=verify)
        return (plan, score) if plan is not None else None
    except Exception as exc:  # never crash: the fallback stays
        if log is not None:
            log["error"] = repr(exc)
        return None


def plan_for(data: dict, budget: float, use_baseline: bool = True, log=None, verify: bool = False):
    """Returns (plan, score, source) where source is 'alns', 'baseline' or 'empty'."""
    t0 = _now()
    sc = Scenario.from_dict(data)
    best, best_score, source = {"seed": sc.seed, "routes": []}, 0.0, "empty"
    if use_baseline:
        found = _baseline(sc, verify, log)
        if found is not None and found[1] > best_score:
            (best, best_score), source = found, "baseline"
    remaining = min(budget, _hard_limit()) - (_now() - t0)
    if remaining > MIN_OPTIMISER_S:
        found = _optimise(sc, remaining, verify, log)
        if found is not None and found[1] > best_score:
            (best, best_score), source = found, "alns"
    return best, best_score, source


def main() -> None:
    try:
        from fleetplan.config import SETTINGS
        budget, use_baseline, verify = SETTINGS.budget_s, SETTINGS.use_baseline, SETTINGS.verify
    except Exception:
        budget, use_baseline, verify = 45.0, True, False
    data = load_json(sys.argv[1])
    try:
        plan, score, source = plan_for(data, budget, use_baseline, verify=verify)
    except Exception:
        plan, score, source = {"seed": data.get("seed"), "routes": []}, 0.0, "empty"
    save_json(plan, sys.argv[2])
    print(f"{source} {score:.2f}")


if __name__ == "__main__":
    main()
