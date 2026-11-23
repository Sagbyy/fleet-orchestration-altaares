"""A solution: one tour per scout, plus the effector matching it implies."""
from __future__ import annotations

from typing import Dict, List, Tuple

from ..geometry.obstacles import XY
from ..world import World
from .matching import match
from .tour import Route, refresh


class Sol:
    def __init__(self, world: World):
        self.world = world
        self.routes = [Route(s) for s in world.scouts]
        self.where: Dict[int, int] = {}       # cand idx -> route index
        self.need: Dict[str, int] = {}        # effectors claimed by routed hostiles, per subtype
        self.obj = 0.0
        self.served: Dict[int, Tuple[float, List[int]]] = {}

    def copy(self) -> "Sol":
        o = Sol.__new__(Sol)
        o.world = self.world
        o.routes = [r.copy() for r in self.routes]
        o.where = dict(self.where)
        o.need = dict(self.need)
        o.obj = self.obj
        o.served = dict(self.served)
        return o

    # ---- mutation -------------------------------------------------------------------------
    def insert(self, k: int, pos: int, j: int, p: XY, l1: float, l2: float) -> None:
        r = self.routes[k]
        r.cands.insert(pos, j)
        r.pts.insert(pos, p)
        r.legs[pos:pos + 1] = [l1, l2]
        refresh(self.world, r)
        self.where[j] = k
        for sub, n in self.world.cands[j].needs.items():
            self.need[sub] = self.need.get(sub, 0) + n

    def remove(self, j: int) -> None:
        k = self.where.pop(j)
        r = self.routes[k]
        pos = r.cands.index(j)
        a = r.s.home if pos == 0 else r.pts[pos - 1]
        b = r.s.home if pos == len(r.cands) - 1 else r.pts[pos + 1]
        r.cands.pop(pos)
        r.pts.pop(pos)
        r.legs[pos:pos + 2] = [self.world.leg(a, b)]
        for sub, n in self.world.cands[j].needs.items():
            self.need[sub] -= n
        if not refresh(self.world, r):
            self._trim(k)

    def _trim(self, k: int) -> None:
        r = self.routes[k]
        while r.cands and not refresh(self.world, r):
            self.remove(r.cands[-1])
        if not r.cands:
            r.legs, r.end, r.energy, r.ends = [0.0], 0.0, 0.0, []

    # ---- evaluation -----------------------------------------------------------------------
    def evaluate(self) -> float:
        world = self.world
        lam = world.rules.lambda_e
        total = 0.0
        hostiles = []
        for r in self.routes:
            total -= lam * r.energy
            for j, e in zip(r.cands, r.ends):
                c = world.cands[j]
                if c.kind == "u":
                    total += c.points
                else:
                    hostiles.append((j, e))
        served, val = match(world, hostiles)
        self.served = served
        self.obj = total + val
        return self.obj
