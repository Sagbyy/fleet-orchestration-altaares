"""Small moves on a solution: relocate, Or-opt and tour merge."""
from __future__ import annotations

import random

from .. import clock
from ..world import World
from .insertion import repair
from .params import Params
from .solution import Sol
from .tour import set_route, time_tour


def _move_block(world: World, sol: Sol, deadline: float, size: int) -> bool:
    """Relocate (size 1) / Or-opt (size 2): move a block of consecutive visits to the
    position, in any tour, that saves the most energy. First improvement."""
    routes = sol.routes
    empties = [k for k, r in enumerate(routes) if not r.cands]
    for k in range(len(routes)):
        rk = routes[k]
        for i in range(len(rk.cands) - size + 1):
            if clock.now() > deadline:
                return False
            block = rk.cands[i:i + size]
            bpts = rk.pts[i:i + size]
            rest_c = rk.cands[:i] + rk.cands[i + size:]
            rest_p = rk.pts[:i] + rk.pts[i + size:]
            res_k = time_tour(world, rk.s, rest_c, rest_p)
            if not res_k[0]:
                continue
            targets = [t for t in range(len(routes)) if routes[t].cands or t in empties[:3]]
            for t in targets:
                rt = routes[t]
                base_c = rest_c if t == k else rt.cands
                base_p = rest_p if t == k else rt.pts
                before = rk.energy + (0.0 if t == k else rt.energy)
                for pos in range(len(base_c) + 1):
                    if t == k and pos == i:
                        continue
                    nc = base_c[:pos] + block + base_c[pos:]
                    npts = base_p[:pos] + bpts + base_p[pos:]
                    if size == 1:   # re-centre the single observation point between its new neighbours
                        a = rt.s.home if pos == 0 else base_p[pos - 1]
                        b = rt.s.home if pos == len(base_c) else base_p[pos]
                        npts[pos] = world.obs_point(world.cands[block[0]], a, b)
                    res_t = time_tour(world, rt.s, nc, npts)
                    if not res_t[0]:
                        continue
                    after = res_t[2] + (0.0 if t == k else res_k[2])
                    if after < before - 1e-3:
                        if t != k:
                            set_route(sol, k, rest_c, rest_p, res_k)
                        set_route(sol, t, nc, npts, res_t)
                        return True
    return False


def _merge(world: World, sol: Sol, deadline: float) -> bool:
    """Tour merge: fly two tours as one on a single scout (saves a base round trip and frees a scout)."""
    routes = sol.routes
    busy = [k for k, r in enumerate(routes) if r.cands]
    empties = [k for k, r in enumerate(routes) if not r.cands]
    for x in range(len(busy)):
        for y in range(x + 1, len(busy)):
            if clock.now() > deadline:
                return False
            k1, k2 = busy[x], busy[y]
            r1, r2 = routes[k1], routes[k2]
            before = r1.energy + r2.energy
            for a, b in ((r1, r2), (r2, r1)):
                nc, npts = a.cands + b.cands, a.pts + b.pts
                for t in [k1, k2] + empties:
                    res = time_tour(world, routes[t].s, nc, npts)
                    if res[0] and res[2] < before - 1e-3:
                        for kk in (k1, k2):
                            if kk != t:
                                set_route(sol, kk, [], [], (True, 0.0, 0.0, [], [0.0]))
                        set_route(sol, t, nc, npts, res)
                        return True
    return False


def local_search(world: World, sol: Sol, params: "Params", rng: random.Random, deadline: float) -> Sol:
    """Polishes a solution with small moves, then re-inserts contacts into freed capacity.
    Returns the better of the input and the polished solution (full objective)."""
    new = sol.copy()
    improved = True
    while improved and clock.now() < deadline:
        improved = _move_block(world, new, deadline, 1) or _move_block(world, new, deadline, 2) \
            or _merge(world, new, deadline)
    pool = [c.idx for c in world.cands if c.idx not in new.where]
    repair(world, new, pool, rng, params, regret=True)
    new.evaluate()
    return new if new.obj > sol.obj + 1e-6 else sol
