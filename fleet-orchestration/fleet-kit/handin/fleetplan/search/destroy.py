"""Destroy operators of the ALNS. Add an operator by appending it to DESTROY."""
from __future__ import annotations

import math
from typing import TYPE_CHECKING, List

from ..config import SETTINGS

if TYPE_CHECKING:
    from .solution import Sol


def _routed(sol: Sol) -> List[int]:
    return list(sol.where.keys())


def d_random(world, sol, q, rng):
    js = _routed(sol)
    rng.shuffle(js)
    return js[:q]


def d_worst(world, sol, q, rng):
    lam = world.rules.lambda_e
    scored = []
    for k, r in enumerate(sol.routes):
        for pos, j in enumerate(r.cands):
            c = world.cands[j]
            if c.kind == "u":
                val = c.points
            else:
                val = c.value if j in sol.served else 0.0
            a = r.s.home if pos == 0 else r.pts[pos - 1]
            b = r.s.home if pos == len(r.cands) - 1 else r.pts[pos + 1]
            saved = r.legs[pos] + r.legs[pos + 1] - math.hypot(a[0] - b[0], a[1] - b[1])
            contrib = val - lam * r.s.e_per_m * saved - 0.05 * saved / r.s.v_model
            scored.append((contrib, j))
    scored.sort()
    out = []
    while scored and len(out) < q:
        i = int(len(scored) * (rng.random() ** 3))
        out.append(scored.pop(i)[1])
    return out


def d_related(world, sol, q, rng):
    js = _routed(sol)
    if not js:
        return []
    seed = rng.choice(js)
    s = world.cands[seed]
    rel = sorted(js, key=lambda j: math.hypot(world.cands[j].x - s.x, world.cands[j].y - s.y)
                 + 3.0 * abs(world.cands[j].lo - s.lo))
    return rel[:q]


def d_route(world, sol, q, rng):
    ks = [k for k, r in enumerate(sol.routes) if r.cands]
    rng.shuffle(ks)
    out = []
    for k in ks:
        out.extend(sol.routes[k].cands)
        if len(out) >= q:
            break
    return out


def d_sector(world, sol, q, rng):
    js = _routed(sol)
    if not js:
        return []
    c0 = world.cands[rng.choice(js)]
    return sorted(js, key=lambda j: math.hypot(world.cands[j].x - c0.x, world.cands[j].y - c0.y))[:q]


def d_weak_route(world, sol, q, rng):
    """Empties the tour with the lowest value per Wh, so its contacts get absorbed by other
    tours: consolidation frees a scout and saves a base round trip."""
    lam = world.rules.lambda_e
    busy = [r for r in sol.routes if r.cands]
    if len(busy) < 2:
        return []

    def worth(r):
        v = sum(world.cands[j].value if (world.cands[j].kind == "u" or j in sol.served) else 0.0 for j in r.cands)
        return (v - lam * r.energy) / max(1.0, r.energy)

    busy.sort(key=worth)
    pick = busy[0] if rng.random() < 0.7 else rng.choice(busy[:3])
    return list(pick.cands)


DESTROY = [d_random, d_worst, d_related, d_route, d_sector] + \
    ([d_weak_route] if SETTINGS.weak_route else [])
