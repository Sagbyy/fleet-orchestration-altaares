"""Validator-driven repair: drop whatever the official validator flags until the plan is valid."""
from __future__ import annotations

import re
from typing import Dict, List, Optional

from fleetkit.model import EFFECTOR, SCOUT, Scenario
from fleetkit.scorer import evaluate


def build_plan(sc: Scenario, routes: Dict[str, List[dict]]) -> dict:
    return {"seed": sc.seed, "routes": [{"aircraft": a, "waypoints": w} for a, w in sorted(routes.items())]}


_PAIR = re.compile(r"^(\S+) and (\S+) come within")


def repair_until_valid(sc: Scenario, routes: Dict[str, List[dict]], groups: Dict[str, List[str]],
                       max_rounds: int = 40, log: Optional[dict] = None):
    """Drops whatever the official validator complains about until the plan is valid."""
    types = {a.id: a.type for a in sc.aircraft}
    eff_target = {e: c for c, es in groups.items() for e in es}
    for _ in range(max_rounds):
        res = evaluate(sc, build_plan(sc, routes))
        if res.valid:
            return routes, res
        if log is not None:
            log.setdefault("repair_rules", []).extend(v.rule for v in res.violations)
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
    res = evaluate(sc, build_plan(sc, routes))
    return routes, res
