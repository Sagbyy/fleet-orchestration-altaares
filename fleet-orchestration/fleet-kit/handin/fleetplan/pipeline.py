"""End-to-end pipeline: preprocess, ALNS, realise, validate and repair, compare with the baseline."""
from __future__ import annotations

import os
import re
import time
from typing import Dict, List, Optional, Tuple

from fleetkit.model import EFFECTOR, SCOUT, Scenario
from fleetkit.scorer import evaluate

from . import alns, realize
from .params import level_params
from .world import World


def _plan(sc: Scenario, routes: Dict[str, List[dict]]) -> dict:
    return {"seed": sc.seed, "routes": [{"aircraft": a, "waypoints": w} for a, w in sorted(routes.items())]}


_PAIR = re.compile(r"^(\S+) and (\S+) come within")


def repair_until_valid(sc: Scenario, routes: Dict[str, List[dict]], groups: Dict[str, List[str]],
                       max_rounds: int = 40):
    """Drops whatever the official validator complains about until the plan is valid."""
    types = {a.id: a.type for a in sc.aircraft}
    eff_target = {e: c for c, es in groups.items() for e in es}
    for _ in range(max_rounds):
        res = evaluate(sc, _plan(sc, routes))
        if res.valid:
            return routes, res
        drop_aircraft, drop_contacts = set(), set()
        for v in res.violations:
            ids = []
            if v.aircraft:
                ids.append(v.aircraft)
            m = _PAIR.match(v.message)
            if m:
                ids.extend(m.groups())
                effs = [i for i in ids if types.get(i) == EFFECTOR]
                ids = effs[:1] if effs else ids[:1]
            for i in ids:
                if types.get(i) == EFFECTOR and i in eff_target:
                    drop_contacts.add(eff_target[i])
                else:
                    drop_aircraft.add(i)
            if v.contact and v.contact in groups and not ids:
                drop_contacts.add(v.contact)
        if not drop_aircraft and not drop_contacts:
            # unknown cause: drop the last effector group, then scouts
            if groups:
                drop_contacts.add(sorted(groups)[-1])
            elif routes:
                drop_aircraft.add(sorted(routes)[-1])
            else:
                break
        for c in drop_contacts:
            for e in groups.pop(c, []):
                routes.pop(e, None)
                eff_target.pop(e, None)
        for a in drop_aircraft:
            routes.pop(a, None)
            if a in eff_target:
                c = eff_target.pop(a)
                for e in groups.pop(c, []):
                    routes.pop(e, None)
                    eff_target.pop(e, None)
        if any(types.get(a) == SCOUT for a in drop_aircraft):
            observed = {w.get("target") for a, wps in routes.items() if types[a] == SCOUT
                        for w in wps if w.get("action") == "observe"}
            for c in [c for c in groups if c not in observed]:
                for e in groups.pop(c):
                    routes.pop(e, None)
                    eff_target.pop(e, None)
    res = evaluate(sc, _plan(sc, routes))
    return routes, res


def prune_scouts(W: World, sol: alns.Sol, air: realize.Airspace, routes: Dict[str, List[dict]],
                 groups: Dict[str, List[str]], deadline: float) -> None:
    """Rebuilds scouts without the hostile observations nobody engages, when that stays valid."""
    engaged_T = {}
    for cid, es in groups.items():
        engaged_T[cid] = min(routes[e][-1]["t"] for e in es)
    by_id = {c.id: c for c in W.cands}
    confirm = W.rules.confirm_margin_s
    for r in sol.routes:
        aid = r.s.id
        if aid not in routes or time.monotonic() > deadline:
            continue
        obs = [w["target"] for w in routes[aid] if w.get("action") == "observe"]
        keep = [cid for cid in obs if by_id[cid].kind == "u" or cid in engaged_T]
        if len(keep) == len(obs):
            continue
        if not keep:
            # nothing useful: drop the scout entirely
            routes.pop(aid)
            air.drop(aid)
            continue
        idx = [by_id[c].idx for c in keep]
        pts = [r.pts[r.cands.index(j)] if j in r.cands else (by_id[W.cands[j].id].x, by_id[W.cands[j].id].y)
               for j in idx]
        old = air.items.pop(aid)
        cache: dict = {}
        done = False
        for land in realize.LAND_DELAYS:
            for delta in realize.SCOUT_DELTAS:
                for z in realize.SCOUT_Z:
                    wps, ends = realize.build_scout(W, r.s, idx, pts, z, delta, land, cache)
                    if wps is None:
                        continue
                    if any(ends[j] > engaged_T[W.cands[j].id] - confirm - 0.2
                           for j in idx if W.cands[j].id in engaged_T):
                        continue
                    it = realize.Item(aid, wps, r.s.ac)
                    if it.energy >= old.energy:
                        continue
                    if not air.conflicts(it):
                        air.commit(it)
                        routes[aid] = wps
                        done = True
                        break
                if done:
                    break
            if done:
                break
        if not done:
            air.commit(old)


def solve(sc: Scenario, t_start: float, budget: float, log: Optional[dict] = None) -> Tuple[Optional[dict], float]:
    """Runs the full pipeline. Returns (plan or None, score)."""
    if log is None:
        log = {}
    P, shares = level_params(sc.level)
    W = World(sc)
    log["candidates"] = len(W.cands)
    t_alns = t_start + budget * shares["alns"]
    best = alns.run(W, t_alns, P, seed=sc.seed, log=log)
    t_real = t_start + budget * shares["realise"]
    routes, groups, air = realize.realise(W, best, t_real)
    prune_scouts(W, best, air, routes, groups, t_real)
    routes, res = repair_until_valid(sc, routes, groups)
    log["served"] = len(res.served) if res.valid else 0
    if not res.valid:
        return None, 0.0
    return _plan(sc, routes), res.score
