"""3D realisation: timed waypoints, altitude layers and separation (R8) deconfliction.

Routes are committed one at a time; each new route is checked against the committed ones with the
official pairwise separation test, trying variants (altitude layer, time shifts, approach angles)
until one is conflict-free.
"""
from __future__ import annotations

import math
import time
from typing import Dict, List, Optional, Tuple

from fleetkit.energy import best_range_speed, route_energy_wh
from fleetkit.geometry import INF, segment_enters_cylinder
from fleetkit.model import Route as KRoute
from fleetkit.model import Waypoint
from fleetkit.validator import separation_item, separation_violation

from .alns import Sol, match
from .geom import CHECK_MARGIN, free_point, path2d
from .world import EFF_Z, GROUP_DT, SCOUT_Z, V_SAFE, Cand, EffInfo, ScoutInfo, World, dive_geometry, \
    effector_estimate

SPREAD = (0.0, 35.0, -35.0, 70.0, -70.0, 105.0)


def _wp(t, x, y, z, **extra) -> dict:
    d = {"t": t, "x": x, "y": y, "z": z}
    d.update(extra)
    return d


def _to_route(aid: str, wps: List[dict]) -> KRoute:
    return KRoute(aid, [Waypoint(w["t"], w["x"], w["y"], w["z"], w.get("action"), w.get("target"),
                                 w.get("dwell_s", 0.0)) for w in wps])


class Item:
    """A committed route with what the separation test needs, plus a bounding box."""

    def __init__(self, aid: str, wps: List[dict], ac):
        self.aid = aid
        self.wps = wps
        route = _to_route(aid, wps)
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


# ---- scouts ----------------------------------------------------------------------------------

def build_scout(W: World, s: ScoutInfo, cidx: List[int], pts, z: float, delta: float,
                land_delay: float, paths_cache: dict, wins=None):
    """Returns (waypoints, first obs end per contact) or (None, index of the first failing contact).

    `wins` optionally gives a (start not before, end not after) window per node, used for
    verification visits; by default each node uses its contact's window.
    """
    if wins is None:
        wins = [None] * len(cidx)
    wins = [w if w is not None else (W.cands[j].lo + 0.05, W.cands[j].latest_end) for j, w in zip(cidx, wins)]
    ac = s.ac
    circles = W.circles_at(z)
    v = V_SAFE * ac.v_h_mps
    climb_t = z / (V_SAFE * ac.v_up_mps) + 0.1
    land_t = z / (V_SAFE * ac.v_down_mps) + 0.1
    dwell = W.rules.obs_dwell_s
    nodes = [s.home] + list(pts) + [s.home]
    paths = []
    for i in range(len(nodes) - 1):
        key = (nodes[i], nodes[i + 1], z)
        res = paths_cache.get(key)
        if res is None:
            res = path2d(nodes[i], nodes[i + 1], circles)
            paths_cache[key] = res if res is not None else False
        if not res:
            return None, min(i, len(cidx) - 1)
        paths.append(res)
    C = W.cands
    n = len(cidx)
    # latest arrival at each observation (backward pass at full speed), so that flying slower
    # (closer to the best-range speed) never breaks a later window or the horizon
    latest = [0.0] * n
    nxt = W.horizon - 0.5 - land_t - land_delay - paths[n][0] / v
    for i in range(n - 1, -1, -1):
        latest[i] = min(wins[i][1], nxt) - dwell
        nxt = latest[i] - paths[i][0] / v
    v_pref = min(v, max(best_range_speed(ac.power), 0.6 * v))
    arr0 = climb_t + paths[0][0] / v
    t0 = max(0.0, max(arr0, wins[0][0]) - arr0 + delta)
    wps = [_wp(t0, s.home[0], s.home[1], 0.0), _wp(t0 + climb_t, s.home[0], s.home[1], z)]
    t = t0 + climb_t
    cur = s.home
    ends: Dict[int, float] = {}
    for i, j in enumerate(cidx):
        c = C[j]
        L, seq = paths[i]
        a = max(t + L / v, wins[i][0])
        if i > 0:
            a = max(a, min(t + L / v_pref, latest[i]))
        if a + dwell > wins[i][1] + 1e-9 or a > latest[i] + 1e-9:
            return None, i
        _emit_leg(wps, cur, seq, t, a, z)
        wps[-1].update(action="observe", target=c.id, dwell_s=dwell)
        t = a + dwell
        if j not in ends:
            ends[j] = t
        cur = seq[-1]
    L, seq = paths[-1]
    a = t + L / v + land_delay
    a = max(a, min(t + L / v_pref + land_delay, W.horizon - 0.5 - land_t))
    _emit_leg(wps, cur, seq, t, a, z)
    t_end = a + land_t
    if t_end > W.horizon - 0.5:
        return None, len(cidx) - 1
    wps.append(_wp(t_end, s.home[0], s.home[1], 0.0))
    if route_energy_wh(_to_route(s.id, wps).waypoints, ac) > ac.usable_wh - 0.05:
        return None, len(cidx) - 1
    return wps, ends


def _emit_leg(wps: List[dict], start, seq, t_dep: float, t_arr: float, z: float) -> None:
    """Appends the waypoints of a polyline leg, with times proportional to length."""
    total = 0.0
    lens = []
    cur = start
    for q in seq:
        l = math.hypot(q[0] - cur[0], q[1] - cur[1])
        lens.append(l)
        total += l
        cur = q
    acc = 0.0
    for q, l in zip(seq, lens):
        acc += l
        t = t_arr if total <= 1e-9 else t_dep + (t_arr - t_dep) * acc / total
        wps.append(_wp(t, q[0], q[1], z))
    wps[-1]["t"] = t_arr


SCOUT_DELTAS = (0.0, 4.0, -4.0, 8.0, -8.0, 15.0, -15.0, 25.0, -25.0, 40.0)
LAND_DELAYS = (0.0, 6.0, 15.0, 30.0)


def place_scouts(W: World, sol: Sol, air: Airspace, deadline: float):
    """Realises and deconflicts every scout. Returns obs end times per contact."""
    obs_end: Dict[int, float] = {}
    plans = []
    for r in sol.routes:
        if r.cands:
            plans.append((r.ends[0] - 30.0, r))
    plans.sort(key=lambda x: x[0])
    for _, r in plans:
        cidx, pts = list(r.cands), list(r.pts)
        cache: dict = {}
        while cidx:
            placed = None
            fail_idx = None
            for land in LAND_DELAYS:
                for delta in SCOUT_DELTAS:
                    for z in SCOUT_Z:
                        wps, info = build_scout(W, r.s, cidx, pts, z, delta, land, cache)
                        if wps is None:
                            if delta == 0.0 and land == 0.0:
                                fail_idx = info if fail_idx is None else min(fail_idx, info)
                            continue
                        it = Item(r.s.id, wps, r.s.ac)
                        if not air.conflicts(it):
                            placed = (it, info)
                            break
                    if placed or time.monotonic() > deadline:
                        break
                if placed or time.monotonic() > deadline:
                    break
            if placed:
                air.commit(placed[0])
                obs_end.update(placed[1])
                break
            # drop one observation (the failing one if known, else the least valuable) and retry
            if fail_idx is not None and fail_idx < len(cidx):
                k = fail_idx
            else:
                k = min(range(len(cidx)), key=lambda i: W.cands[cidx[i]].value)
            cidx.pop(k)
            pts.pop(k)
    return obs_end


# ---- effectors -------------------------------------------------------------------------------

def build_effector(W: World, e: EffInfo, c: Cand, T: float, z: float, ang: float, cache: dict):
    ac = e.ac
    circles = W.circles_at(z)
    hx, hy = e.home
    ux, uy = hx - c.x, hy - c.y
    n = math.hypot(ux, uy) or 1.0
    ux, uy = ux / n, uy / n
    a = math.radians(ang)
    ux, uy = ux * math.cos(a) - uy * math.sin(a), ux * math.sin(a) + uy * math.cos(a)
    dd, dive_t = dive_geometry(ac, z)
    D = (c.x + ux * dd, c.y + uy * dd)
    if not free_point(D[0], D[1], circles, 3.0):
        return None
    for cc in W.protected + W.zones:
        if segment_enters_cylinder((D[0], D[1], z), (c.x, c.y, 0.0), (cc.cx, cc.cy), cc.r + CHECK_MARGIN, cc.top):
            return None
    key = (e.home, D, z)
    res = cache.get(key)
    if res is None:
        res = path2d(e.home, D, circles)
        cache[key] = res if res is not None else False
    if not res:
        return None
    Lc, seq = res
    climb_t = z / (V_SAFE * ac.v_up_mps) + 0.1
    vmax = V_SAFE * ac.v_h_mps
    vc = min(best_range_speed(ac.power), vmax)
    t_cruise = Lc / vc
    t0 = T - dive_t - t_cruise - climb_t
    if t0 < 0.0:
        t_cruise = T - dive_t - climb_t
        if t_cruise < Lc / vmax - 1e-9:
            return None
        t0 = 0.0
    wps = [_wp(t0, hx, hy, 0.0), _wp(t0 + climb_t, hx, hy, z)]
    _emit_leg(wps, e.home, seq, t0 + climb_t, T - dive_t, z)
    wps.append(_wp(T, c.x, c.y, 0.0, action="engage", target=c.id))
    if route_energy_wh(_to_route(e.id, wps).waypoints, ac) > ac.usable_wh - 0.05:
        return None
    return wps


DTS = (0.0, 3.0, 6.0, 10.0, 15.0, 22.0, 30.0, 40.0, 55.0, 75.0, 100.0, 130.0, 170.0)
ANGLES = (0.0, 25.0, -25.0, 50.0, -50.0, 80.0, -80.0)


def place_group(W: World, air: Airspace, c: Cand, T_base: float, members: List[EffInfo],
                deadline: float, cache: dict) -> Optional[List[Item]]:
    t_hi = c.hi - 0.3
    n = len(members)
    for dT in DTS:
        T = T_base + dT
        if T + GROUP_DT * (n - 1) > t_hi:
            break
        for ang0 in ANGLES:
            if time.monotonic() > deadline:
                return None
            items: List[Item] = []
            ok = True
            for i, e in enumerate(members):
                # every cruise layer, cheapest energy first; keep the first conflict-free one
                options = []
                for z in EFF_Z:
                    wps = build_effector(W, e, c, T + GROUP_DT * i, z, ang0 + SPREAD[i % len(SPREAD)], cache)
                    if wps is not None:
                        options.append(Item(e.id, wps, e.ac))
                options.sort(key=lambda it: it.energy)
                chosen = next((it for it in options if not air.conflicts(it, items)), None)
                if chosen is None:
                    ok = False
                    break
                items.append(chosen)
            if ok:
                return items
    return None


def place_effectors(W: World, air: Airspace, obs_end: Dict[int, float], deadline: float):
    """Matches effectors to observed hostiles and realises each engagement group."""
    groups: Dict[str, List[str]] = {}
    free = {e.k for e in W.effs}
    cache: dict = {}
    hostiles = [(j, t) for j, t in obs_end.items() if W.cands[j].kind == "h"]
    done = set()
    for _ in range(3):
        todo = [(j, t) for j, t in hostiles if j not in done]
        if not todo or not free:
            break
        served, _ = match(W, todo, free)
        if not served:
            break
        progress = False
        for j in sorted(served, key=lambda jj: -W.cands[jj].value):
            T, ks = served[j]
            if any(k not in free for k in ks):
                continue
            members = [W.effs[k] for k in ks]
            items = place_group(W, air, W.cands[j], T, members, deadline, cache)
            done.add(j)
            if items is None:
                continue
            for it in items:
                air.commit(it)
            for k in ks:
                free.discard(k)
            groups[W.cands[j].id] = [it.aid for it in items]
            progress = True
        if not progress:
            break
    return groups


def realise(W: World, sol: Sol, deadline: float):
    """Returns (plan routes {aircraft id: waypoints}, groups {contact id: effector ids})."""
    air = Airspace(W)
    obs_end = place_scouts(W, sol, air, deadline)
    groups = place_effectors(W, air, obs_end, deadline)
    # scouts that only observed hostiles nobody engages are kept: they are valid and cheap to fix later
    return {aid: it.wps for aid, it in air.items.items()}, groups, air
