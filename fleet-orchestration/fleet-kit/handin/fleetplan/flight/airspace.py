"""Committed routes and the separation (R8) conflict test against them."""
from __future__ import annotations

from typing import Dict, List

from fleetkit.energy import route_energy_wh

from ..geometry.separation import separation_item, separation_violation
from ..world import World
from .waypoints import to_kroute


class Item:
    """A committed route with what the separation test needs, plus a bounding box."""

    def __init__(self, aid: str, wps: List[dict], ac):
        self.aid = aid
        self.wps = wps
        route = to_kroute(aid, wps)
        self.energy = route_energy_wh(route.waypoints, ac)
        self.sep = separation_item(aid, route, ac)
        xs = [w["x"] for w in wps]
        ys = [w["y"] for w in wps]
        zs = [w["z"] for w in wps]
        self.box = (min(xs), max(xs), min(ys), max(ys), min(zs), max(zs))
        self.t0 = wps[0]["t"]
        self.t1 = wps[-1]["t"] + wps[-1].get("dwell_s", 0.0)


class Airspace:
    def __init__(self, W: World):
        self.W = W
        self.items: Dict[str, Item] = {}
        self.contacts = W.sc.contact_by_id()
        self.d = W.rules.d_sep_m

    def conflicts(self, it: Item, extra: List[Item] = ()) -> bool:
        if it.sep is None:
            return False
        d = self.d
        b = it.box
        for other in list(self.items.values()) + list(extra):
            if other.aid == it.aid or other.sep is None:
                continue
            if other.t1 <= it.t0 or other.t0 >= it.t1:
                continue
            o = other.box
            if o[0] > b[1] + d or o[1] < b[0] - d or o[2] > b[3] + d or o[3] < b[2] - d \
                    or o[4] > b[5] + d or o[5] < b[4] - d:
                continue
            if separation_violation(self.W.rules, self.contacts, it.sep, other.sep) is not None:
                return True
        return False

    def commit(self, it: Item) -> None:
        self.items[it.aid] = it

    def drop(self, aid: str) -> None:
        self.items.pop(aid, None)
