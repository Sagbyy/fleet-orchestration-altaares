"""Regret insertion of contacts into scout tours, and observation-point polishing."""
from __future__ import annotations

import math
import random
from typing import Dict, List

from ..world import Cand, World
from .params import Params
from .solution import Sol
from .tour import Route, forward

INF = math.inf


def _gain(W: World, j: int, d_energy: float, d_end: float, P: Params) -> float:
    return W.cands[j].value - W.rules.lambda_e * d_energy - P.time_price * d_end


def eval_route(W: World, sol: Sol, k: int, j: int, P: Params):
    """Best insertion of contact j in route k: (gain, pos, point, l1, l2) or None."""
    r = sol.routes[k]
    c = W.cands[j]
    n = len(r.cands)
    home = r.s.home
    cx, cy = c.x, c.y
    est = []
    prev = home
    for pos in range(n + 1):
        nxt = home if pos == n else r.pts[pos]
        e = math.hypot(prev[0] - cx, prev[1] - cy) + math.hypot(cx - nxt[0], cy - nxt[1]) \
            - math.hypot(prev[0] - nxt[0], prev[1] - nxt[1])
        est.append((e, pos))
        prev = nxt
    est.sort()
    best = None
    for _, pos in est[:P.k_pos]:
        a = home if pos == 0 else r.pts[pos - 1]
        b = home if pos == n else r.pts[pos]
        p = W.obs_point(c, a, b)
        l1, l2 = W.leg(a, p), W.leg(p, b)
        legs = r.legs[:pos] + [l1, l2] + r.legs[pos + 1:]
        cidx = r.cands[:pos] + [j] + r.cands[pos:]
        ok, end, energy, _ = forward(W, r.s, cidx, legs)
        if not ok:
            continue
        g = _gain(W, j, energy - r.energy, end - r.end, P)
        if best is None or g > best[0]:
            best = (g, pos, p, l1, l2)
    return best


def _cheap(W: World, r: Route, c: Cand) -> float:
    n = len(r.cands)
    home = r.s.home
    prev = home
    best = INF
    for pos in range(n + 1):
        nxt = home if pos == n else r.pts[pos]
        e = math.hypot(prev[0] - c.x, prev[1] - c.y) + math.hypot(c.x - nxt[0], c.y - nxt[1]) \
            - math.hypot(prev[0] - nxt[0], prev[1] - nxt[1])
        if e < best:
            best = e
        prev = nxt
    return best


def _need_ok(W: World, sol: Sol, j: int) -> bool:
    for sub, n in W.cands[j].needs.items():
        if sol.need.get(sub, 0) + n > W.n_sub.get(sub, 0):
            return False
    return True


def repair(W: World, sol: Sol, pool: List[int], rng: random.Random, P: Params,
           regret: bool = True, noise: float = 0.0) -> None:
    """Regret (or greedy) insertion of the pool into the solution."""
    cache: Dict[int, Dict[int, tuple]] = {}
    pool = [j for j in pool if j not in sol.where]
    for j in pool:
        c = W.cands[j]
        busy = [k for k, r in enumerate(sol.routes) if r.cands]
        busy.sort(key=lambda k: _cheap(W, sol.routes[k], c))
        chosen = busy[:P.k_routes] + [k for k, r in enumerate(sol.routes) if not r.cands]
        cache[j] = {}
        for k in chosen:
            res = eval_route(W, sol, k, j, P)
            if res is not None:
                cache[j][k] = res
    while pool:
        best_j, best_score, best_ins = None, -INF, None
        for j in pool:
            if not _need_ok(W, sol, j):
                continue
            entries = sorted((v[0], k, v) for k, v in cache[j].items())
            if not entries:
                continue
            g1, k1, v1 = entries[-1]
            if g1 <= 0.0:
                continue
            if noise:
                g1n = g1 * (1.0 + noise * (2.0 * rng.random() - 1.0))
            else:
                g1n = g1
            if regret:
                g2 = entries[-2][0] if len(entries) > 1 else 0.0
                score = g1n + P.regret_w * (g1 - max(g2, 0.0))
            else:
                score = g1n
            if score > best_score:
                best_j, best_score, best_ins = j, score, (k1, v1)
        if best_j is None:
            break
        k, (g, pos, p, l1, l2) = best_ins
        sol.insert(k, pos, best_j, p, l1, l2)
        pool.remove(best_j)
        for j in pool:
            res = eval_route(W, sol, k, j, P)
            if res is None:
                cache[j].pop(k, None)
            else:
                cache[j][k] = res


def reoptimise_points(W: World, sol: Sol, k: int) -> None:
    """Moves each observation point to the best spot given its neighbours, when that helps."""
    r = sol.routes[k]
    n = len(r.cands)
    for pos in range(n):
        a = r.s.home if pos == 0 else r.pts[pos - 1]
        b = r.s.home if pos == n - 1 else r.pts[pos + 1]
        p = W.obs_point(W.cands[r.cands[pos]], a, b)
        if p == r.pts[pos]:
            continue
        l1, l2 = W.leg(a, p), W.leg(p, b)
        if l1 + l2 >= r.legs[pos] + r.legs[pos + 1] - 1e-6:
            continue
        legs = r.legs[:pos] + [l1, l2] + r.legs[pos + 2:]
        ok, end, energy, ends = forward(W, r.s, r.cands, legs)
        if ok:
            r.pts[pos] = p
            r.legs = legs
            r.end, r.energy, r.ends = end, energy, ends
