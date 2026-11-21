"""Removes scout observations that no engagement uses, when that stays valid."""

from __future__ import annotations

from typing import Dict, List

from .. import clock
from ..flight.airspace import Airspace
from ..flight.scouts import try_place
from ..search.solution import Sol
from ..world import World


def prune_scouts(
    world: World,
    sol: Sol,
    air: Airspace,
    routes: Dict[str, List[dict]],
    groups: Dict[str, List[str]],
    deadline: float,
) -> None:
    """Rebuilds scouts without the hostile observations nobody engages, when that stays valid.

    Works from the realised routes (a tour may fly on another scout than the ALNS planned).
    """
    engaged_T = {cid: min(routes[e][-1]["t"] for e in es) for cid, es in groups.items()}
    by_id = {c.id: c for c in world.cands}
    scouts = {s.id: s for s in world.scouts}
    confirm = world.rules.confirm_margin_s
    for aid in [a for a in routes if a in scouts]:
        if clock.now() > deadline:
            break
        obs = [w for w in routes[aid] if w.get("action") == "observe"]
        keep = [
            w for w in obs if by_id[w["target"]].kind == "u" or w["target"] in engaged_T
        ]
        if len(keep) == len(obs):
            continue
        if not keep:
            routes.pop(aid)
            air.drop(aid)
            continue
        idx = [by_id[w["target"]].idx for w in keep]
        pts = [(w["x"], w["y"]) for w in keep]
        old = air.items.pop(aid)

        def ok(it, ends, old=old):
            return it.energy < old.energy and all(
                ends[j] <= engaged_T[world.cands[j].id] - confirm - 0.2
                for j in idx
                if world.cands[j].id in engaged_T
            )

        placed = try_place(world, scouts[aid], idx, pts, air, deadline, {}, accept=ok)
        if placed:
            air.commit(placed[0])
            routes[aid] = placed[0].wps
        else:
            air.commit(old)
