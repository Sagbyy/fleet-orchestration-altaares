"""The solver: preprocessing, ALNS, realisation of the elite solutions, repair."""
from __future__ import annotations

from typing import Optional, Tuple

from fleetkit.model import Scenario
from fleetkit.scorer import evaluate

from .. import clock
from ..config import SETTINGS
from ..flight import realise
from ..search import alns
from ..search.params import level_params
from ..world import World
from .pruning import prune_scouts
from .repair import build_plan, repair_until_valid
from .verify import verify_pass


def _realise_and_repair(world: World, sc: Scenario, sol, t_start: float, budget: float, t_real: float,
                        verify: bool, log: dict):
    """Flies one ALNS solution in 3D and repairs it. Returns (score, routes, validator result)."""
    routes, groups, air = realise(world, sol, t_real)
    prune_scouts(world, sol, air, routes, groups, t_real)
    if verify:
        log["verify_visits"] = verify_pass(world, air, routes, groups, t_start + budget * 0.97)
    routes, res = repair_until_valid(sc, routes, groups, log=log)
    score = res.score if res.valid else -1.0
    if verify and res.valid:
        score = evaluate(sc, build_plan(sc, routes), verify=True).score
    return score, routes, res


def solve(sc: Scenario, t_start: float, budget: float, log: Optional[dict] = None,
          verify: bool = False) -> Tuple[Optional[dict], float]:
    """Runs the full pipeline. Returns (plan or None, score; the verify score if `verify`).

    1. preprocessing (World); 2. ALNS until ~85 % of the budget, leaving the realisation >= 3 s;
    3. realise the elite solutions (each well under a second) and keep the best flown plan,
    since the model and the 3D plan do not always rank them the same way."""
    if log is None:
        log = {}
    params, shares = level_params(sc.level)
    world = World(sc)
    log["candidates"] = len(world.cands)
    t_alns = t_start + min(budget * shares["alns"], budget - 3.0)
    best = alns.run(world, t_alns, params, seed=sc.seed + SETTINGS.seed_offset, log=log)
    t_real = t_start + max(budget * shares["realise"], budget - 1.0)
    candidates = getattr(best, "elite", None) or [best]
    best_out = None
    for i, sol in enumerate(candidates):
        if i > 0 and clock.now() > t_real - 1.5:
            break
        score, routes, res = _realise_and_repair(world, sc, sol, t_start, budget, t_real, verify, log)
        if best_out is None or score > best_out[0]:
            best_out = (score, routes, res, i)
    score, routes, res, pick = best_out
    log["elite_tried"], log["elite_pick"] = len(candidates), pick
    log["served"] = len(res.served) if res.valid else 0
    if not res.valid:
        return None, 0.0
    return build_plan(sc, routes), score
