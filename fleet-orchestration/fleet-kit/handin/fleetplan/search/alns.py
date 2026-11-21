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


def initial(world: World, params: Params, rng: random.Random) -> Sol:
    sol = Sol(world)
    repair(world, sol, list(range(len(world.cands))), rng, params, regret=True)
    for k in range(len(sol.routes)):
        reoptimise_points(world, sol, k)
    sol.evaluate()
    return sol


class OperatorSelector:
    """Adaptive roulette over a set of operators (the "adaptive" in ALNS).

    Operators earn a score each time they are used (new best, improvement, accepted); every
    `segment` iterations each weight moves towards its average score, with a floor."""

    REWARD_BEST, REWARD_IMPROVED, REWARD_ACCEPTED = 33.0, 9.0, 3.0

    def __init__(self, n: int, reaction: float, segment: int):
        self.weights = [1.0] * n
        self.scores = [0.0] * n
        self.uses = [0] * n
        self.reaction, self.segment = reaction, segment

    def pick(self, rng: random.Random) -> int:
        return _roulette(self.weights, rng)

    def reward(self, i: int, score: float) -> None:
        self.scores[i] += score
        self.uses[i] += 1

    def end_of_iteration(self, it: int) -> None:
        if it % self.segment:
            return
        for i in range(len(self.weights)):
            if self.uses[i]:
                self.weights[i] = (1 - self.reaction) * self.weights[i] + self.reaction * self.scores[i] / self.uses[i]
            self.weights[i] = max(self.weights[i], 0.05)
            self.scores[i], self.uses[i] = 0.0, 0


class ElitePool:
    """The best distinct solutions seen (distinct = different set of routed contacts)."""

    def __init__(self, size: int):
        self.size = size
        self._pool: Dict[frozenset, Sol] = {}

    def offer(self, sol: Sol) -> None:
        key = frozenset(sol.where)
        pool = self._pool
        if key in pool and pool[key].obj >= sol.obj:
            return
        if len(pool) >= self.size and key not in pool:
            worst = min(pool, key=lambda kk: pool[kk].obj)
            if pool[worst].obj >= sol.obj:
                return
            del pool[worst]
        pool[key] = sol.copy()

    def ranked(self) -> List[Sol]:
        return sorted(self._pool.values(), key=lambda e: -e.obj)


class SimulatedAnnealing:
    """Accepts a worse solution with probability exp(delta / T), T falling linearly to zero."""

    def __init__(self, t0: float, t_start: float, deadline: float):
        self.t0, self.t_start = t0, t_start
        self.span = max(1e-3, deadline - t_start)

    def temperature(self) -> float:
        frac = (clock.now() - self.t_start) / self.span
        return self.t0 * max(0.0, 1.0 - frac) + 1e-9

    @staticmethod
    def accepts(delta: float, temp: float, rng: random.Random) -> bool:
        return rng.random() < math.exp(min(0.0, delta / temp))


def _destroy_and_repair(world: World, cur: Sol, di: int, ri: int, params: Params,
                        rng: random.Random) -> Sol:
    """One large-neighbourhood move: destroy part of `cur`, rebuild, reassign tours, polish points."""
    new = cur.copy()
    n_routed = len(new.where)
    q = max(params.q_min, int(round(rng.uniform(0.1, params.q_frac) * n_routed)))
    removed = DESTROY[di](world, new, min(q, n_routed), rng)
    touched = set()
    for j in removed:
        if j in new.where:
            touched.add(new.where[j])
            new.remove(j)
    pool = [c.idx for c in world.cands if c.idx not in new.where]
    _, regret, noise = REPAIRS[ri]
    before = {k: len(r.cands) for k, r in enumerate(new.routes)}
    repair(world, new, pool, rng, params, regret=regret, noise=noise * params.noise)
    if params.reassign and reassign_tours(world, new):
        touched = set(range(len(new.routes)))
        pool = [c.idx for c in world.cands if c.idx not in new.where]
        repair(world, new, pool, rng, params, regret=regret, noise=noise * params.noise)
    for k, r in enumerate(new.routes):
        if k in touched or len(r.cands) != before[k]:
            reoptimise_points(world, new, k)
    new.evaluate()
    return new


def run(world: World, deadline: float, params: Params, seed: int = 0, log=None) -> Sol:
    """Runs the ALNS until `deadline` (or `params.stall` iterations without a new best).

    Returns the best solution; `best.elite` lists the best distinct solutions for the realiser."""
    rng = random.Random(seed)
    cur = initial(world, params, rng)
    best = cur.copy()
    if not world.cands:
        return best
    destroy_ops = OperatorSelector(len(DESTROY), params.reaction, params.segment)
    repair_ops = OperatorSelector(len(REPAIRS), params.reaction, params.segment)
    annealing = SimulatedAnnealing(max(1.0, params.t0_frac * abs(cur.obj)), clock.now(), deadline)
    elite = ElitePool(params.elite)
    it = last_best = 0
    last_ls = 0.0
    while clock.now() < deadline and it - last_best < params.stall:
        clock.CLOCK.tick()
        it += 1
        di, ri = destroy_ops.pick(rng), repair_ops.pick(rng)
        new = _destroy_and_repair(world, cur, di, ri, params, rng)
        if new.obj > best.obj - params.elite_gap * abs(best.obj):
            elite.offer(new)
        temp = annealing.temperature()
        score = 0.0
        if new.obj > best.obj + 1e-6:
            if params.local_search and clock.now() - last_ls > 0.5:
                new = local_search(world, new, params, rng, min(deadline, clock.now() + 0.4))
                last_ls = clock.now()
            best, cur, score, last_best = new.copy(), new, OperatorSelector.REWARD_BEST, it
        elif new.obj > cur.obj + 1e-6:
            cur, score = new, OperatorSelector.REWARD_IMPROVED
        elif annealing.accepts(new.obj - cur.obj, temp, rng):
            cur, score = new, OperatorSelector.REWARD_ACCEPTED
        destroy_ops.reward(di, score)
        repair_ops.reward(ri, score)
        destroy_ops.end_of_iteration(it)
        repair_ops.end_of_iteration(it)
    if params.local_search:   # final polish (bounded: the 3D realisation keeps its reserve)
        best = local_search(world, best, params, rng, clock.now() + 0.8)
    elite.offer(best)
    best.elite = elite.ranked()
    if log is not None:
        log["iterations"] = it
        log["model_obj"] = best.obj
    return best
