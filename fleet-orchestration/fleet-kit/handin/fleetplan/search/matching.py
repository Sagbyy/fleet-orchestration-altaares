"""Effector matching: which hostiles get which effectors, and when."""
from __future__ import annotations

import math
from typing import Dict, List, Optional, Tuple

from ..config import GROUP_DT, SETTINGS
from ..world import World

INF = math.inf


def knapsack_pick(items, cap: Dict[str, int]) -> List[int]:
    """Best subset of hostiles under effector counts (exact DP over the scarce subtypes only).

    items: (contact idx, value, needs). When no subtype is scarce every item fits and all are
    returned; otherwise a 1D or 2D knapsack picks the most valuable feasible subset."""
    subs = sorted({s for _, _, n in items for s in n})
    scarce = [s for s in subs if sum(n.get(s, 0) for _, _, n in items) > cap.get(s, 0)]
    if not scarce:
        return [j for j, _, _ in items]
    dp = {tuple(0 for _ in scarce): (0.0, ())}
    for j, v, n in items:
        if v <= 0.0:
            continue
        add = tuple(n.get(s, 0) for s in scarce)
        if any(a > cap.get(s, 0) for a, s in zip(add, scarce)):
            continue
        new = dict(dp)
        for st, (val, ch) in dp.items():
            ns = tuple(a + b for a, b in zip(st, add))
            if any(x > cap.get(s, 0) for x, s in zip(ns, scarce)):
                continue
            nv = val + v
            cur = new.get(ns)
            if cur is None or cur[0] < nv:
                new[ns] = (nv, ch + (j,))
        dp = new
    return list(max(dp.values(), key=lambda x: x[0])[1])


def match(W: World, hostiles: List[Tuple[int, float]], avail: Optional[set] = None):
    """Effector matching.

    1. pick the best set of hostiles under the effector counts (knapsack when effectors are scarce);
    2. assign effectors earliest-deadline first, the slowest effector that still makes it;
    3. fill the remaining effectors greedily with whatever was left out."""
    r = W.rules
    C = W.cands
    used = set()
    served: Dict[int, Tuple[float, List[int]]] = {}
    total = 0.0
    window = {}
    for j, oe in hostiles:
        c = C[j]
        t_lo = max(oe + r.confirm_margin_s + 0.5, c.lo + 0.05)
        t_hi = c.hi - GROUP_DT * (c.n_eff - 1) - 0.5
        if t_lo <= t_hi:
            window[j] = (t_lo, t_hi)
    cap: Dict[str, int] = {}
    for e in W.effs:
        if avail is None or e.k in avail:
            cap[e.sub] = cap.get(e.sub, 0) + 1
    chosen = knapsack_pick([(j, C[j].value, C[j].needs) for j in window], cap) if SETTINGS.knapsack else []
    chosen_set = set(chosen)
    rest = sorted((j for j in window if j not in chosen_set), key=lambda j: -C[j].value)
    order = sorted(chosen, key=lambda j: window[j][1]) + rest

    for j in order:
        c = C[j]
        t_lo, t_hi = window[j]
        pick = []
        ok = True
        for sub, n in c.needs.items():
            pool = [e for e in W.effs if e.sub == sub and e.k not in used
                    and (avail is None or e.k in avail)
                    and e.earliest.get(j, INF) <= t_hi - 2.0]
            if len(pool) < n:
                ok = False
                break
            pool.sort(key=lambda e: -e.earliest[j])
            pick.extend(pool[:n])
        if not ok:
            continue
        T = max(t_lo, max(e.earliest[j] for e in pick) + 1.0)
        if T > t_hi:
            T = t_hi
        for e in pick:
            used.add(e.k)
        served[j] = (T, [e.k for e in pick])
        total += c.points - r.c_eff * len(pick) - r.lambda_e * sum(e.energy[j] for e in pick)
    return served, total
