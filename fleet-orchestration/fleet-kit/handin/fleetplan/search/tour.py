"""Timing and energy model of one scout tour."""
from __future__ import annotations

from typing import TYPE_CHECKING, List

from ..geometry.obstacles import XY
from ..world import ScoutInfo, World

if TYPE_CHECKING:
    from .solution import Sol


class Route:
    __slots__ = ("s", "cands", "pts", "legs", "end", "energy", "ends")

    def __init__(self, s: ScoutInfo):
        self.s = s
        self.cands: List[int] = []
        self.pts: List[XY] = []
        self.legs: List[float] = [0.0]
        self.end = 0.0
        self.energy = 0.0
        self.ends: List[float] = []

    def copy(self) -> "Route":
        r = Route.__new__(Route)
        r.s = self.s
        r.cands = list(self.cands)
        r.pts = list(self.pts)
        r.legs = list(self.legs)
        r.end, r.energy = self.end, self.energy
        r.ends = list(self.ends)
        return r


def _forward_at(world: World, s: ScoutInfo, cidx: List[int], legs: List[float], v: float, epm: float):
    n = len(cidx)
    cands = world.cands
    dwell = world.rules.obs_dwell_s
    c0 = cands[cidx[0]]
    arr = s.climb_t + legs[0] / v
    lo = c0.lo + 0.05
    a = arr if arr > lo else lo
    start = a - arr
    t = a + dwell
    if t > c0.latest_end:
        return False, 0.0, 0.0, None
    ends = [t]
    # a wait is flown as a slower leg (the realiser stretches the leg), which costs
    # p0*D + k*L^2/D instead of cruising then hovering
    slow_extra = 0.0
    p0, kv = s.p0, s.kv
    for i in range(1, n):
        c = cands[cidx[i]]
        L = legs[i]
        arr = t + L / v
        lo = c.lo + 0.05
        if arr < lo:
            D = lo - t
            slow_extra += (p0 * D + kv * L * L / D) / 3600.0 - epm * L
            arr = lo
        t = arr + dwell
        if t > c.latest_end:
            return False, 0.0, 0.0, None
        ends.append(t)
    end = t + legs[n] / v + s.land_t
    if end > world.horizon - 2.0:
        return False, 0.0, 0.0, None
    dist = 0.0
    for l in legs:
        dist += l
    energy = s.fixed_wh + epm * dist + slow_extra + s.hover * dwell * n
    return True, end - start, energy, ends


def forward(world: World, s: ScoutInfo, cidx: List[int], legs: List[float]):
    """Times a scout tour. Returns (feasible, end time, energy Wh, observation end times).

    Flown at full speed when the battery allows; otherwise at the best-range speed v*
    (minimum energy per metre), if the windows still hold at that speed.
    """
    if not cidx:
        return True, 0.0, 0.0, []
    ok, end, energy, ends = _forward_at(world, s, cidx, legs, s.v_model, s.e_per_m)
    if ok and energy <= s.usable:
        return True, end, energy, ends
    if s.v_eco >= s.v_model - 1e-9:
        return False, 0.0, 0.0, None
    ok, end, energy, ends = _forward_at(world, s, cidx, legs, s.v_eco, s.e_per_m_eco)
    if ok and energy <= s.usable:
        return True, end, energy, ends
    return False, 0.0, 0.0, None


def refresh(world: World, r: Route) -> bool:
    ok, end, energy, ends = forward(world, r.s, r.cands, r.legs)
    if ok:
        r.end, r.energy, r.ends = end, energy, ends
    return ok


def time_tour(world: World, sc: ScoutInfo, cands: List[int], pts: List[XY]):
    """Times a candidate tour on scout sc: (ok, end, energy, ends, legs)."""
    if not cands:
        return True, 0.0, 0.0, [], [0.0]
    nodes = [sc.home] + list(pts) + [sc.home]
    legs = [world.leg(nodes[i], nodes[i + 1]) for i in range(len(nodes) - 1)]
    ok, end, energy, ends = forward(world, sc, cands, legs)
    return ok, end, energy, ends, legs


def set_route(sol: Sol, k: int, cands, pts, res) -> None:
    ok, end, energy, ends, legs = res
    r = sol.routes[k]
    r.cands, r.pts, r.legs = list(cands), list(pts), list(legs)
    r.end, r.energy, r.ends = end, energy, list(ends)
    for j in r.cands:
        sol.where[j] = k
