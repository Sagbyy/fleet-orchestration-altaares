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


def solve(sc: Scenario, t_start: float, budget: float, log: Optional[dict] = None,
          verify: bool = False) -> Tuple[Optional[dict], float]:
    """Runs the full pipeline. Returns (plan or None, score; the verify score if `verify`)."""
    if log is None:
        log = {}
    P, shares = level_params(sc.level)
    W = World(sc)
    log["candidates"] = len(W.cands)
    # the 3D realisation normally takes < 0.5 s; always leave it at least 3 s
    t_alns = t_start + min(budget * shares["alns"], budget - 3.0)
    best = alns.run(W, t_alns, P, seed=sc.seed + SETTINGS.seed_offset, log=log)
    t_real = t_start + max(budget * shares["realise"], budget - 1.0)
    # realise several distinct good solutions (each takes well under a second) and keep the
    # one that scores best once flown: the model and the 3D plan do not rank them identically
    cands = getattr(best, "elite", None) or [best]
    best_out = None
    for i, sol in enumerate(cands):
        if i > 0 and clock.now() > t_real - 1.5:
            break
        routes, groups, air = realise(W, sol, t_real)
        prune_scouts(W, sol, air, routes, groups, t_real)
        if verify:
            log["verify_visits"] = verify_pass(W, air, routes, groups, t_start + budget * 0.97)
        routes, res = repair_until_valid(sc, routes, groups, log=log)
        score = res.score if res.valid else -1.0
        if verify and res.valid:
            score = evaluate(sc, build_plan(sc, routes), verify=True).score
        if best_out is None or score > best_out[0]:
            best_out = (score, routes, res, i)
    score, routes, res, pick = best_out
    log["elite_tried"], log["elite_pick"] = len(cands), pick
    log["served"] = len(res.served) if res.valid else 0
    if not res.valid:
        return None, 0.0
    return build_plan(sc, routes), score
