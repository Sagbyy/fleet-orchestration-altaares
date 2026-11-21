"""Optional verify-board mode: re-observations 10-120 s after engagements."""
from __future__ import annotations

import math
from typing import Dict, List

from .. import clock
from ..flight.airspace import Airspace, Item
from ..flight.scouts import build_scout
from ..config import SCOUT_Z
from ..world import World


def verify_pass(W: World, air: Airspace, routes: Dict[str, List[dict]],
                groups: Dict[str, List[str]], deadline: float) -> int:
    """Adds scout re-observations 10-120 s after an engagement (optional --verify bonus).

    Only worth it on the verify board: the extra flight costs energy in the main score.
    Returns the number of verification visits added.
    """
    r = W.rules
    lam = r.lambda_e
    dwell = r.obs_dwell_s
    by_id = {c.id: c for c in W.cands}
    scouts = {s.id: s for s in W.scouts}
    last_T = {cid: max(routes[e][-1]["t"] for e in es) for cid, es in groups.items()}
    first_T = {cid: min(routes[e][-1]["t"] for e in es) for cid, es in groups.items()}
    added = 0
    for cid in sorted(last_T, key=lambda c: -by_id[c].points):
        if clock.now() > deadline:
            break
        c = by_id[cid]
        gain_pts = r.verify_bonus * c.points
        T = last_T[cid]
        lo = max(c.lo + 0.05, T + r.confirm_margin_s - dwell + 0.3)
        hi = min(c.hi - 0.05, T + r.verify_window_s - 0.3)
        if hi - lo < dwell:
            continue
        cand_scouts = sorted((aid for aid in routes if aid in scouts),
                             key=lambda aid: min(math.hypot(w["x"] - c.x, w["y"] - c.y) for w in routes[aid]))[:4]
        best = None
        for aid in cand_scouts:
            s = scouts[aid]
            obs = [w for w in routes[aid] if w.get("action") == "observe"]
            nodes = [by_id[w["target"]].idx for w in obs]
            pts = [(w["x"], w["y"]) for w in obs]
            wins = [None] * len(nodes)
            for k, (j, w) in enumerate(zip(nodes, obs)):   # keep existing verification visits in their window
                cj = W.cands[j]
                if cj.id in last_T and w["t"] + dwell > first_T[cj.id] - r.confirm_margin_s:
                    lt = last_T[cj.id]
                    wins[k] = (max(cj.lo + 0.05, lt + r.confirm_margin_s - dwell + 0.3),
                               min(cj.hi - 0.05, lt + r.verify_window_s - 0.3))
            old = air.items[aid]
            for pos in range(len(nodes) + 1):
                a = s.home if pos == 0 else pts[pos - 1]
                b = s.home if pos == len(nodes) else pts[pos]
                p = W.obs_point(c, a, b)
                n2 = nodes[:pos] + [c.idx] + nodes[pos:]
                p2 = pts[:pos] + [p] + pts[pos:]
                w2 = wins[:pos] + [(lo, hi)] + wins[pos:]
                cache: dict = {}
                for z in SCOUT_Z:
                    wps, ends = build_scout(W, s, n2, p2, z, 0.0, 0.0, cache, w2)
                    if wps is None:
                        continue
                    # every hostile this scout confirms must still be confirmed in time
                    # (`ends` holds the first observation end per contact)
                    confirmers = [j for j, w in zip(nodes, obs) if W.cands[j].id in first_T
                                  and w["t"] + dwell <= first_T[W.cands[j].id] - r.confirm_margin_s]
                    if any(ends[j] > first_T[W.cands[j].id] - r.confirm_margin_s - 0.2 for j in confirmers):
                        continue
                    it = Item(aid, wps, s.ac)
                    net = gain_pts - lam * (it.energy - old.energy)
                    if net <= 0 or (best is not None and net <= best[0]):
                        continue
                    air.items.pop(aid)
                    clash = air.conflicts(it)
                    air.items[aid] = old
                    if not clash:
                        best = (net, aid, it)
        if best is not None:
            _, aid, it = best
            air.items[aid] = it
            routes[aid] = it.wps
            added += 1
    return added
