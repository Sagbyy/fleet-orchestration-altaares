"""Hungarian reassignment of tours to scouts (batteries and speeds differ)."""
from __future__ import annotations

import math

from ..algorithms.hungarian import assign
from ..world import World
from .solution import Sol
from .tour import Route, forward

INF = math.inf


def reassign_tours(world: World, sol: Sol) -> bool:
    """Gives each tour to the scout that flies it cheapest (Hungarian on tours x scouts).

    Scouts differ in battery, speed and consumption; a long tour on a small-battery scout can
    block contacts that only a big-battery scout could reach. Returns True if anything moved.
    """
    tours = [r for r in sol.routes if r.cands]
    if len(tours) < 1 or len(tours) > len(sol.routes):
        return False
    lam = world.rules.lambda_e
    cost, cache = [], {}
    for r in tours:
        row = []
        mid = r.legs[1:-1]
        for s in world.scouts:
            if s is r.s:
                row.append(lam * r.energy)
                cache[(id(r), s.k)] = (r.legs, r.end, r.energy, r.ends)
                continue
            legs = [world.leg(s.home, r.pts[0])] + mid + [world.leg(r.pts[-1], s.home)]
            ok, end, energy, ends = forward(world, s, r.cands, legs)
            if ok:
                row.append(lam * energy)
                cache[(id(r), s.k)] = (legs, end, energy, ends)
            else:
                row.append(INF)
        cost.append(row)
    res = assign(cost)
    if any(c is None for c in res):
        return False
    if all(tours[i].s.k == res[i] for i in range(len(tours))):
        return False
    new_routes = [Route(s) for s in world.scouts]
    for i, r in enumerate(tours):
        k = res[i]
        nr = new_routes[k]
        nr.cands, nr.pts = list(r.cands), list(r.pts)
        nr.legs, nr.end, nr.energy, nr.ends = cache[(id(r), k)]
        nr.legs = list(nr.legs)
        for j in nr.cands:
            sol.where[j] = k
    sol.routes = new_routes
    return True
