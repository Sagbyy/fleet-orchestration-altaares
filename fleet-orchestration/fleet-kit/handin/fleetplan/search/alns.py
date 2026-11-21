"""The ALNS loop: destroy, repair, reassign, accept (simulated annealing), adapt operator weights."""
from __future__ import annotations

import math
import random
from typing import Dict, List

from .. import clock
from ..world import World
from .destroy import DESTROY
from .insertion import reoptimise_points, repair
from .local_search import local_search
from .params import Params
from .reassign import reassign_tours
from .solution import Sol


REPAIRS = [("regret", True, 0.0), ("regret_noise", True, 1.0), ("greedy_noise", False, 1.0)]


def _roulette(w: List[float], rng: random.Random) -> int:
    s = sum(w)
    x = rng.random() * s
    for i, v in enumerate(w):
        x -= v
        if x <= 0:
            return i
    return len(w) - 1


def initial(W: World, P: Params, rng: random.Random) -> Sol:
    sol = Sol(W)
    repair(W, sol, list(range(len(W.cands))), rng, P, regret=True)
    for k in range(len(sol.routes)):
        reoptimise_points(W, sol, k)
    sol.evaluate()
    return sol


def run(W: World, deadline: float, P: Params, seed: int = 0, log=None) -> Sol:
    rng = random.Random(seed)
    cur = initial(W, P, rng)
    best = cur.copy()
    if not W.cands:
        return best
    t_start = clock.now()
    span = max(1e-3, deadline - t_start)
    wd = [1.0] * len(DESTROY)
    wr = [1.0] * len(REPAIRS)
    sd = [0.0] * len(DESTROY)
    sr = [0.0] * len(REPAIRS)
    nd = [0] * len(DESTROY)
    nr = [0] * len(REPAIRS)
    t0 = max(1.0, P.t0_frac * abs(cur.obj))
    it = 0
    last_best = 0
    last_ls = 0.0
    elite: Dict[frozenset, "Sol"] = {}   # best distinct solutions (by set of routed contacts)

    def keep(sol_):
        key = frozenset(sol_.where)
        if key in elite and elite[key].obj >= sol_.obj:
            return
        if len(elite) >= P.elite and key not in elite:
            worst = min(elite, key=lambda kk: elite[kk].obj)
            if elite[worst].obj >= sol_.obj:
                return
            del elite[worst]
        elite[key] = sol_.copy()
    while clock.now() < deadline and it - last_best < P.stall:
        clock.CLOCK.tick()
        it += 1
        di, ri = _roulette(wd, rng), _roulette(wr, rng)
        new = cur.copy()
        n_routed = len(new.where)
        q = max(P.q_min, int(round(rng.uniform(0.1, P.q_frac) * n_routed)))
        removed = DESTROY[di](W, new, min(q, n_routed), rng)
        touched = set()
        for j in removed:
            if j in new.where:
                touched.add(new.where[j])
                new.remove(j)
        pool = [c.idx for c in W.cands if c.idx not in new.where]
        _, regret, noise = REPAIRS[ri]
        before = {k: len(r.cands) for k, r in enumerate(new.routes)}
        repair(W, new, pool, rng, P, regret=regret, noise=noise * P.noise)
        if P.reassign and reassign_tours(W, new):
            touched = set(range(len(new.routes)))
            pool = [c.idx for c in W.cands if c.idx not in new.where]
            repair(W, new, pool, rng, P, regret=regret, noise=noise * P.noise)
        for k, r in enumerate(new.routes):
            if k in touched or len(r.cands) != before[k]:
                reoptimise_points(W, new, k)
        new.evaluate()
        if new.obj > best.obj - P.elite_gap * abs(best.obj):
            keep(new)
        frac = (clock.now() - t_start) / span
        temp = t0 * max(0.0, 1.0 - frac) + 1e-9
        score = 0.0
        if new.obj > best.obj + 1e-6:
            if P.local_search and clock.now() - last_ls > 0.5:
                new = local_search(W, new, P, rng, min(deadline, clock.now() + 0.4))
                last_ls = clock.now()
            best = new.copy()
            cur = new
            score = 33.0
            last_best = it
        elif new.obj > cur.obj + 1e-6:
            cur = new
            score = 9.0
        elif rng.random() < math.exp(min(0.0, (new.obj - cur.obj) / temp)):
            cur = new
            score = 3.0
        sd[di] += score
        sr[ri] += score
        nd[di] += 1
        nr[ri] += 1
        if it % P.segment == 0:
            for i in range(len(wd)):
                if nd[i]:
                    wd[i] = (1 - P.reaction) * wd[i] + P.reaction * sd[i] / nd[i]
                wd[i] = max(wd[i], 0.05)
                sd[i], nd[i] = 0.0, 0
            for i in range(len(wr)):
                if nr[i]:
                    wr[i] = (1 - P.reaction) * wr[i] + P.reaction * sr[i] / nr[i]
                wr[i] = max(wr[i], 0.05)
                sr[i], nr[i] = 0.0, 0
    if P.local_search:   # final polish (bounded: the 3D realisation keeps its reserve)
        best = local_search(W, best, P, rng, clock.now() + 0.8)
    keep(best)
    best.elite = sorted((e for e in elite.values()), key=lambda e: -e.obj)
    if log is not None:
        log["iterations"] = it
        log["model_obj"] = best.obj
    return best
