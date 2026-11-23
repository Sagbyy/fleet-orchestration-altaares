"""Scout realisation and placement (urgency order, variants, swaps, surgical trim)."""
from __future__ import annotations

import math
from typing import Dict, List, Optional

from fleetkit.energy import best_range_speed, route_energy_wh

from .. import clock
from ..config import LAND_DELAYS, LAND_SLOW, MIN_TRIES, OBS_RHO_HIGH, SCOUT_DELTAS, SCOUT_Z, SCOUT_Z_MODEL, V_SAFE
from ..geometry.obstacles import free_point, path2d
from ..search.solution import Sol
from ..world import ScoutInfo, World
from .airspace import Airspace, Item
from .waypoints import climb_leg, emit_leg, landing_leg, leg_min_time, to_kroute, waypoint


def build_scout(world: World, s: ScoutInfo, cidx: List[int], pts, z: float, delta: float,
                land_delay: float, paths_cache: dict, wins=None, land_slow: float = 1.0):
    """Returns (waypoints, first obs end per contact) or (None, index of the first failing contact).

    `wins` optionally gives a (start not before, end not after) window per node, used for
    verification visits; by default each node uses its contact's window.
    """
    if wins is None:
        wins = [None] * len(cidx)
    wins = [w if w is not None else (world.cands[j].lo + 0.05, world.cands[j].latest_end) for j, w in zip(cidx, wins)]
    ac = s.ac
    circles = world.circles_at(z)
    cands = world.cands
    if z > SCOUT_Z_MODEL + 1e-9:   # higher layer: observation points must be closer to the contact
        moved = []
        for j, p in zip(cidx, pts):
            c = cands[j]
            dx, dy = p[0] - c.x, p[1] - c.y
            d = math.hypot(dx, dy)
            if d > OBS_RHO_HIGH:
                p = (c.x + dx / d * OBS_RHO_HIGH, c.y + dy / d * OBS_RHO_HIGH)
                if not free_point(p[0], p[1], circles, 3.0):
                    return None, len(moved)
            moved.append(p)
        pts = moved
    v = V_SAFE * ac.v_h_mps
    dwell = world.rules.obs_dwell_s
    nodes = [s.home] + list(pts) + [s.home]
    legs3 = []
    for i in range(len(nodes) - 1):
        key = (nodes[i], nodes[i + 1], z)
        res = paths_cache.get(key)
        if res is None:
            res = path2d(nodes[i], nodes[i + 1], circles)
            paths_cache[key] = res if res is not None else False
        if not res:
            return None, min(i, len(cidx) - 1)
        seq = res[1]
        if i == 0:
            p3 = climb_leg(world, s.home, seq, z, ac, v)
        elif i == len(nodes) - 2:
            p3 = landing_leg(world, nodes[i], seq, z, ac, v)
        else:
            p3 = [(q[0], q[1], z) for q in seq]
        legs3.append(p3)
    n = len(cidx)
    starts = [(s.home[0], s.home[1], 0.0)] + [legs3[i][-1] for i in range(n)]
    fast = [leg_min_time(starts[i], legs3[i], ac, v) for i in range(n + 1)]
    v_pref = min(v, max(best_range_speed(ac.power), 0.6 * v))
    slow = [leg_min_time(starts[i], legs3[i], ac, v_pref) for i in range(n + 1)]
    # latest arrival at each observation (backward pass at full speed), so that flying slower
    # (closer to the best-range speed) never breaks a later window or the horizon
    latest = [0.0] * n
    nxt = world.horizon - 0.5 - land_delay - fast[n]
    for i in range(n - 1, -1, -1):
        latest[i] = min(wins[i][1], nxt) - dwell
        nxt = latest[i] - fast[i]
    # take off early enough to fly the first leg at the best-range speed when the window allows
    t0 = max(0.0, max(slow[0], wins[0][0]) - slow[0] + delta)
    wps = [waypoint(t0, s.home[0], s.home[1], 0.0)]
    t = t0
    ends: Dict[int, float] = {}
    for i, j in enumerate(cidx):
        c = cands[j]
        a = max(t + fast[i], wins[i][0])
        if i > 0:
            a = max(a, min(t + slow[i], latest[i]))
        if a + dwell > wins[i][1] + 1e-9 or a > latest[i] + 1e-9:
            return None, i
        emit_leg(wps, starts[i], legs3[i], t, a, ac)
        wps[-1].update(action="observe", target=c.id, dwell_s=dwell)
        t = a + dwell
        if j not in ends:
            ends[j] = t
    a = t + fast[n] + land_delay
    a = a + land_slow * max(0.0, min(t + slow[n] + land_delay, world.horizon - 0.5) - a)
    if a > world.horizon - 0.5 + 1e-9:
        return None, len(cidx) - 1
    emit_leg(wps, starts[n], legs3[n], t, a, ac)
    if route_energy_wh(to_kroute(s.id, wps).waypoints, ac) > ac.usable_wh - 0.05:
        return None, len(cidx) - 1
    return wps, ends


def try_place(world: World, s: ScoutInfo, cidx, pts, air: "Airspace", deadline: float, cache: dict,
               accept=None):
    """Searches a conflict-free variant of one tour on scout s. Returns (Item, ends) or None.

    `accept(item, ends)` can veto a variant (used when rebuilding a committed scout).
    Past the deadline it still tries a minimum number of variants: a tour must never be
    abandoned just because the machine is slow."""
    tries = 0
    for slow in LAND_SLOW:
        for z in SCOUT_Z:
            for land in LAND_DELAYS:
                feasible_any = False
                for delta in SCOUT_DELTAS:
                    tries += 1
                    if tries > MIN_TRIES and clock.now() > deadline:
                        return None
                    wps, info = build_scout(world, s, cidx, pts, z, delta, land, cache, land_slow=slow)
                    if wps is None:
                        continue
                    feasible_any = True
                    it = Item(s.id, wps, s.ac)
                    if accept is not None and not accept(it, info):
                        continue
                    if not air.conflicts(it):
                        return it, info
                if not feasible_any:
                    break      # later landings are infeasible too
    return None


def _launch_slack(world: World, s: ScoutInfo, cidx, pts, cache) -> float:
    """How late the scout can take off (s): small = urgent, place it first."""
    for d in (2.0, 6.0, 15.0):
        wps, _ = build_scout(world, s, cidx, pts, SCOUT_Z[0], d, 0.0, cache)
        if wps is None:
            return d
    return 99.0


class ScoutPlacer:
    """Places every scout tour in the airspace, keeping as many observations as possible.

    For each tour, in order: its own scout with variants -> a free scout on another pad ->
    the single cheapest contact removal that makes it fit -> drop the cheapest contact and retry.
    Contacts removed on the way get a last chance alone on a still-free scout."""

    def __init__(self, world: World, air: Airspace, deadline: float, tours, free):
        self.world, self.air, self.deadline = world, air, deadline
        self.caches = {r.s.id: {} for r in tours}
        self.free: List[ScoutInfo] = free
        self.obs_end: Dict[int, float] = {}
        self.lost: List[int] = []

    def _cache(self, scout: ScoutInfo) -> dict:
        return self.caches.setdefault(scout.id, {})

    def _commit(self, placed, scout: ScoutInfo) -> None:
        self.air.commit(placed[0])
        self.obs_end.update(placed[1])
        if scout in self.free:
            self.free.remove(scout)

    def _late(self) -> bool:
        return clock.now() > self.deadline

    def _on_free_scout(self, home_scout: ScoutInfo, cidx, pts) -> bool:
        """The same tour on a free scout, farthest pad first."""
        candidates = [] if self._late() else sorted(
            self.free, key=lambda f: -math.hypot(f.home[0] - home_scout.home[0], f.home[1] - home_scout.home[1]))
        for fs in candidates:
            placed = try_place(self.world, fs, cidx, pts, self.air, self.deadline, self._cache(fs))
            if placed:
                self._commit(placed, fs)
                self.free.append(home_scout)   # its own aircraft is now free for someone else
                return True
        return False

    def _surgical_trim(self, scout: ScoutInfo, cidx, pts) -> bool:
        """Removes the single cheapest contact whose removal lets the tour fit."""
        order = [] if self._late() else sorted(range(len(cidx)), key=lambda i: self.world.cands[cidx[i]].value)
        for k in order:
            c2, p2 = cidx[:k] + cidx[k + 1:], pts[:k] + pts[k + 1:]
            if not c2:
                break
            placed = try_place(self.world, scout, c2, p2, self.air, self.deadline, self._cache(scout))
            if placed:
                self.lost.append(cidx[k])
                self._commit(placed, scout)
                return True
            if self._late():
                break
        return False

    def place_tour(self, r) -> None:
        cidx, pts = list(r.cands), list(r.pts)
        while cidx:
            placed = try_place(self.world, r.s, cidx, pts, self.air, self.deadline, self._cache(r.s))
            if placed:
                self._commit(placed, r.s)
                return
            if self._on_free_scout(r.s, cidx, pts) or self._surgical_trim(r.s, cidx, pts):
                return
            k = min(range(len(cidx)), key=lambda i: self.world.cands[cidx[i]].value)
            self.lost.append(cidx[k])
            cidx.pop(k)
            pts.pop(k)

    def solo_rescue(self) -> None:
        """Puts each lost contact alone on a still-free scout."""
        for j in sorted(self.lost, key=lambda jj: -self.world.cands[jj].value):
            if j in self.obs_end or not self.free or self._late():
                continue
            c = self.world.cands[j]
            for fs in list(self.free):
                p = self.world.obs_point(c, fs.home, fs.home)
                placed = try_place(self.world, fs, [j], [p], self.air, self.deadline, self._cache(fs))
                if placed:
                    self._commit(placed, fs)
                    break


def place_scouts(world: World, sol: Sol, air: Airspace, deadline: float, log: Optional[dict] = None):
    """Realises and deconflicts every scout. Returns obs end times per contact.

    Tours are placed most urgent first (least take-off slack, then least battery margin)."""
    tours = [r for r in sol.routes if r.cands]
    free = [s for s in world.scouts if not any(r.s is s for r in tours)]
    placer = ScoutPlacer(world, air, deadline, tours, free)
    order = sorted(tours, key=lambda r: (_launch_slack(world, r.s, r.cands, r.pts, placer.caches[r.s.id]),
                                         r.s.usable - r.energy))
    for r in order:
        placer.place_tour(r)
    placer.solo_rescue()
    if log is not None:
        log["scout_lost"] = [world.cands[j].id for j in placer.lost if j not in placer.obs_end]
    return placer.obs_end
