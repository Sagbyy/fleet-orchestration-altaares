"""3D realisation: timed waypoints, altitude layers and separation (R8) deconfliction.

Routes are committed one at a time; each new route is checked against the committed ones with the
official pairwise separation test, trying variants (altitude layer, time shifts, approach angles)
until one is conflict-free.
"""
from __future__ import annotations

import math
import os
import time
from typing import Dict, List, Optional, Tuple

from fleetkit.energy import best_range_speed, route_energy_wh
from fleetkit.geometry import INF, segment_enters_cylinder
from fleetkit.model import Route as KRoute
from fleetkit.model import Waypoint
from .sepcheck import separation_item, separation_violation

from .alns import Sol, match
from . import clock
from .hungarian import INF, assign
from .geom import CHECK_MARGIN, free_point, path2d
from .world import EFF_Z, GROUP_DT, OBS_RHO_HIGH, SCOUT_Z, SCOUT_Z_MODEL, V_SAFE, Cand, EffInfo, ScoutInfo, World, dive_geometry, \
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

def _seg_min(a, b, ac, vh: float) -> float:
    """Minimum duration of a straight 3D segment for this aircraft (horizontal speed vh)."""
    t = math.hypot(b[0] - a[0], b[1] - a[1]) / vh
    dz = b[2] - a[2]
    if dz > 0:
        t = max(t, dz / (V_SAFE * ac.v_up_mps))
    elif dz < 0:
        t = max(t, -dz / (V_SAFE * ac.v_down_mps))
    return t


def _leg_min(start, pts3, ac, vh: float) -> float:
    total, cur = 0.0, start
    for q in pts3:
        total += _seg_min(cur, q, ac, vh)
        cur = q
    return total


def _emit3(wps: List[dict], start, pts3, t_dep: float, t_arr: float, ac) -> None:
    """Appends a 3D polyline leg flown from t_dep to t_arr; each segment gets its minimum time
    (at the speed limits) scaled up uniformly, so no segment ever exceeds a limit."""
    mins, cur = [], start
    for q in pts3:
        mins.append(_seg_min(cur, q, ac, V_SAFE * ac.v_h_mps))
        cur = q
    m = sum(mins)
    d = t_arr - t_dep
    t = t_dep
    for q, mk in zip(pts3, mins):
        t += (mk * d / m) if m > 1e-12 else d / len(pts3)
        wps.append(_wp(t, q[0], q[1], q[2]))
    wps[-1]["t"] = t_arr


def _clear3(a, b, W: World) -> bool:
    """True if a (possibly sloped) segment stays out of every zone and protected area."""
    for cc in W.protected + W.zones:
        if segment_enters_cylinder(a, b, (cc.cx, cc.cy), cc.r + CHECK_MARGIN, cc.top):
            return False
    return True


def _point_along(start, seq, dist):
    """Point at arc length `dist` along the polyline start -> seq (2D); None past the end."""
    cur = start
    for q in seq:
        l = math.hypot(q[0] - cur[0], q[1] - cur[1])
        if dist < l - 1e-6:
            f = dist / l
            return (cur[0] + f * (q[0] - cur[0]), cur[1] + f * (q[1] - cur[1]))
        dist -= l
        cur = q
    return None


def _climb_leg(W: World, home, seq, z: float, ac, vh: float):
    """3D points of a leg that takes off from home (z = 0): diagonal climb when clear, else vertical."""
    dist_c = vh * z / (V_SAFE * ac.v_up_mps)
    p = _point_along(home, seq[:1], dist_c)   # only on the first segment, to keep the order
    pts = ([(p[0], p[1], z)] if p is not None else []) + [(q[0], q[1], z) for q in seq]
    if _clear3((home[0], home[1], 0.0), pts[0], W):
        return pts
    return [(home[0], home[1], z)] + [(q[0], q[1], z) for q in seq]


def _landing_leg(W: World, start, seq, z: float, ac, vh: float):
    """3D points of a leg that ends landed at seq[-1]: diagonal descent when clear, else vertical."""
    home = seq[-1]
    rev = list(reversed([start] + list(seq[:-1])))
    dist_d = vh * z / (V_SAFE * ac.v_down_mps)
    q = _point_along(home, rev[:1], dist_d)   # only on the last segment, to keep the order
    body = [(p[0], p[1], z) for p in seq[:-1]]
    if q is not None:
        body = body + [(q[0], q[1], z)]
    last_from = body[-1] if body else (start[0], start[1], z)
    if _clear3(last_from, (home[0], home[1], 0.0), W):
        return body + [(home[0], home[1], 0.0)]
    return [(p[0], p[1], z) for p in seq[:-1]] + [(home[0], home[1], z), (home[0], home[1], 0.0)]


def build_scout(W: World, s: ScoutInfo, cidx: List[int], pts, z: float, delta: float,
                land_delay: float, paths_cache: dict, wins=None, land_slow: float = 1.0):
    """Returns (waypoints, first obs end per contact) or (None, index of the first failing contact).

    `wins` optionally gives a (start not before, end not after) window per node, used for
    verification visits; by default each node uses its contact's window.
    """
    if wins is None:
        wins = [None] * len(cidx)
    wins = [w if w is not None else (W.cands[j].lo + 0.05, W.cands[j].latest_end) for j, w in zip(cidx, wins)]
    ac = s.ac
    circles = W.circles_at(z)
    C = W.cands
    if z > SCOUT_Z_MODEL + 1e-9:   # higher layer: observation points must be closer to the contact
        moved = []
        for j, p in zip(cidx, pts):
            c = C[j]
            dx, dy = p[0] - c.x, p[1] - c.y
            d = math.hypot(dx, dy)
            if d > OBS_RHO_HIGH:
                p = (c.x + dx / d * OBS_RHO_HIGH, c.y + dy / d * OBS_RHO_HIGH)
                if not free_point(p[0], p[1], circles, 3.0):
                    return None, len(moved)
            moved.append(p)
        pts = moved
    v = V_SAFE * ac.v_h_mps
    dwell = W.rules.obs_dwell_s
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
            p3 = _climb_leg(W, s.home, seq, z, ac, v)
        elif i == len(nodes) - 2:
            p3 = _landing_leg(W, nodes[i], seq, z, ac, v)
        else:
            p3 = [(q[0], q[1], z) for q in seq]
        legs3.append(p3)
    n = len(cidx)
    starts = [(s.home[0], s.home[1], 0.0)] + [legs3[i][-1] for i in range(n)]
    fast = [_leg_min(starts[i], legs3[i], ac, v) for i in range(n + 1)]
    v_pref = min(v, max(best_range_speed(ac.power), 0.6 * v))
    slow = [_leg_min(starts[i], legs3[i], ac, v_pref) for i in range(n + 1)]
    # latest arrival at each observation (backward pass at full speed), so that flying slower
    # (closer to the best-range speed) never breaks a later window or the horizon
    latest = [0.0] * n
    nxt = W.horizon - 0.5 - land_delay - fast[n]
    for i in range(n - 1, -1, -1):
        latest[i] = min(wins[i][1], nxt) - dwell
        nxt = latest[i] - fast[i]
    # take off early enough to fly the first leg at the best-range speed when the window allows
    t0 = max(0.0, max(slow[0], wins[0][0]) - slow[0] + delta)
    wps = [_wp(t0, s.home[0], s.home[1], 0.0)]
    t = t0
    ends: Dict[int, float] = {}
    for i, j in enumerate(cidx):
        c = C[j]
        a = max(t + fast[i], wins[i][0])
        if i > 0:
            a = max(a, min(t + slow[i], latest[i]))
        if a + dwell > wins[i][1] + 1e-9 or a > latest[i] + 1e-9:
            return None, i
        _emit3(wps, starts[i], legs3[i], t, a, ac)
        wps[-1].update(action="observe", target=c.id, dwell_s=dwell)
        t = a + dwell
        if j not in ends:
            ends[j] = t
    a = t + fast[n] + land_delay
    a = a + land_slow * max(0.0, min(t + slow[n] + land_delay, W.horizon - 0.5) - a)
    if a > W.horizon - 0.5 + 1e-9:
        return None, len(cidx) - 1
    _emit3(wps, starts[n], legs3[n], t, a, ac)
    if route_energy_wh(_to_route(s.id, wps).waypoints, ac) > ac.usable_wh - 0.05:
        return None, len(cidx) - 1
    return wps, ends


SCOUT_DELTAS = (0.0, 1.0, 2.0, 3.0, 4.5, 6.0, 8.0, 10.0, 13.0, 17.0, 22.0, 30.0, 40.0,
                -1.5, -3.0, -6.0, -10.0, -15.0, -25.0)
LAND_DELAYS = (0.0, 1.5, 3.0, 4.5, 6.0, 8.0, 11.0, 15.0, 20.0, 30.0)
LAND_SLOW = (1.0, 0.0, 0.5)   # 1 = slow best-range return (lands late), 0 = full-speed return
MIN_TRIES = 60                # variants always tried per tour, whatever the clock says


def _try_place(W: World, s: ScoutInfo, cidx, pts, air: "Airspace", deadline: float, cache: dict,
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
                    wps, info = build_scout(W, s, cidx, pts, z, delta, land, cache, land_slow=slow)
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


def _launch_slack(W: World, s: ScoutInfo, cidx, pts, cache) -> float:
    """How late the scout can take off (s): small = urgent, place it first."""
    for d in (2.0, 6.0, 15.0):
        wps, _ = build_scout(W, s, cidx, pts, SCOUT_Z[0], d, 0.0, cache)
        if wps is None:
            return d
    return 99.0


def place_scouts(W: World, sol: Sol, air: Airspace, deadline: float, log: Optional[dict] = None):
    """Realises and deconflicts every scout. Returns obs end times per contact.

    Tours are placed most urgent first (least take-off slack, then least battery margin). A tour
    that cannot be placed on its scout is tried on a free scout (another pad, other battery);
    only then is one contact removed, the cheapest single removal that makes it fit.
    """
    obs_end: Dict[int, float] = {}
    tours = [r for r in sol.routes if r.cands]
    caches = {r.s.id: {} for r in tours}
    order = sorted(tours, key=lambda r: (_launch_slack(W, r.s, r.cands, r.pts, caches[r.s.id]),
                                         r.s.usable - r.energy))
    free = [s for s in W.scouts if not any(r.s is s for r in tours)]
    lost: List[int] = []

    def commit(placed, s_used):
        air.commit(placed[0])
        obs_end.update(placed[1])
        if s_used in free:
            free.remove(s_used)

    for r in order:
        cidx, pts = list(r.cands), list(r.pts)
        while cidx:
            placed = _try_place(W, r.s, cidx, pts, air, deadline, caches[r.s.id])
            if placed:
                commit(placed, r.s)
                break
            # same tour on a free scout, farthest pad first
            hit = None
            for fs in ([] if clock.now() > deadline else sorted(free, key=lambda f: -math.hypot(f.home[0] - r.s.home[0], f.home[1] - r.s.home[1]))):
                placed = _try_place(W, fs, cidx, pts, air, deadline, caches.setdefault(fs.id, {}))
                if placed:
                    hit = fs
                    break
            if hit is not None:
                commit(placed, hit)
                free.append(r.s)   # its own aircraft is now free for someone else
                break
            # surgical trim: remove the single cheapest contact whose removal lets the tour fit
            done = False
            for k in ([] if clock.now() > deadline else
                      sorted(range(len(cidx)), key=lambda i: W.cands[cidx[i]].value)):
                c2, p2 = cidx[:k] + cidx[k + 1:], pts[:k] + pts[k + 1:]
                if not c2:
                    break
                placed = _try_place(W, r.s, c2, p2, air, deadline, caches[r.s.id])
                if placed:
                    lost.append(cidx[k])
                    commit(placed, r.s)
                    done = True
                    break
                if clock.now() > deadline:
                    break
            if done:
                break
            k = min(range(len(cidx)), key=lambda i: W.cands[cidx[i]].value)
            lost.append(cidx[k])
            cidx.pop(k)
            pts.pop(k)
    # solo rescue: put each lost contact alone on a still-free scout
    for j in sorted(lost, key=lambda jj: -W.cands[jj].value):
        if j in obs_end or not free or clock.now() > deadline:
            continue
        c = W.cands[j]
        for fs in list(free):
            p = W.obs_point(c, fs.home, fs.home)
            placed = _try_place(W, fs, [j], [p], air, deadline, caches.setdefault(fs.id, {}))
            if placed:
                commit(placed, fs)
                break
    if log is not None:
        log["scout_lost"] = [W.cands[j].id for j in lost if j not in obs_end]
    return obs_end


# ---- effectors -------------------------------------------------------------------------------

GROUND_RUN = os.environ.get("FLEET_GROUND_RUN", "0") == "1"


def _ground_effector(W: World, e: EffInfo, c: Cand, T: float):
    """Two-waypoint route [home, engage], both at z = 0 (R2 bounds only intermediate waypoints).

    Only when the straight line crosses no zone and no protected area. No climb energy at all."""
    ac = e.ac
    hx, hy = e.home
    if not _clear3((hx, hy, 0.0), (c.x, c.y, 0.0), W):
        return None
    d = math.hypot(c.x - hx, c.y - hy)
    vmax = V_SAFE * ac.v_h_mps
    vc = min(best_range_speed(ac.power), vmax)
    t0 = T - d / vc
    if t0 < 0.0:
        if T < d / vmax - 1e-9:
            return None
        t0 = 0.0
    wps = [_wp(t0, hx, hy, 0.0), _wp(T, c.x, c.y, 0.0, action="engage", target=c.id)]
    if route_energy_wh(_to_route(e.id, wps).waypoints, ac) > ac.usable_wh - 0.05:
        return None
    return wps


def build_effector(W: World, e: EffInfo, c: Cand, T: float, z: float, ang: float, cache: dict):
    if z == 0.0:
        return _ground_effector(W, e, c, T)
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
    vmax = V_SAFE * ac.v_h_mps
    vc = min(best_range_speed(ac.power), vmax)
    home3 = (hx, hy, 0.0)
    pts3 = _climb_leg(W, e.home, seq, z, ac, vmax) + [(c.x, c.y, 0.0)]
    t_fast = _leg_min(home3, pts3, ac, vmax)
    t_pref = _leg_min(home3, pts3, ac, vc)
    t0 = T - t_pref
    if t0 < 0.0:
        if T < t_fast - 1e-9:
            return None
        t0 = 0.0
    wps = [_wp(t0, hx, hy, 0.0)]
    _emit3(wps, home3, pts3, t0, T, ac)
    wps[-1].update(action="engage", target=c.id)
    if route_energy_wh(_to_route(e.id, wps).waypoints, ac) > ac.usable_wh - 0.05:
        return None
    return wps


DTS = (0.0, 3.0, 6.0, 10.0, 15.0, 22.0, 30.0, 40.0, 55.0, 75.0, 100.0, 130.0, 170.0)
ANGLES = (0.0, 25.0, -25.0, 50.0, -50.0, 80.0, -80.0)


def place_group(W: World, air: Airspace, c: Cand, T_base: float, members: List[EffInfo],
                deadline: float, cache: dict) -> Optional[List[Item]]:
    """Finds conflict-free routes for one engagement group.

    Variants are tried from the cheapest: low cruise layers first (climbing costs energy), and
    for each layer cap a few later arrivals and approach angles (waiting on the ground is free)
    before allowing a higher layer.
    """
    t_hi = c.hi - 0.3
    n = len(members)
    built: Dict[tuple, Optional[Item]] = {}

    def option(i, e, T, z, ang):
        key = (e.k, T, z, ang)
        if key not in built:
            wps = build_effector(W, e, c, T + GROUP_DT * i, z, ang + SPREAD[i % len(SPREAD)], cache)
            built[key] = Item(e.id, wps, e.ac) if wps is not None else None
        return built[key]

    last = len(EFF_Z) - 1
    tries = 0
    for cap in range(len(EFF_Z)):
        layers = EFF_Z[:cap + 1]
        dts = DTS if cap == last else DTS[:6]
        angs = ANGLES if cap == last else ANGLES[:3]
        for dT in dts:
            T = T_base + dT
            if T + GROUP_DT * (n - 1) > t_hi:
                break
            for ang0 in angs:
                tries += 1
                if tries > MIN_TRIES and clock.now() > deadline:
                    return None
                items: List[Item] = []
                ok = True
                for i, e in enumerate(members):
                    opts = [it for it in (option(i, e, T, z, ang0) for z in layers) if it is not None]
                    opts.sort(key=lambda it: it.energy)
                    chosen = next((it for it in opts if not air.conflicts(it, items)), None)
                    if chosen is None:
                        ok = False
                        break
                    items.append(chosen)
                if ok:
                    return items
    return None


def _fastest_members(W: World, j: int, free: set, t_hi: float):
    """Alternative group: the fastest free effectors of each needed subtype."""
    c = W.cands[j]
    out = []
    for sub, n in c.needs.items():
        pool = sorted((e for e in W.effs if e.sub == sub and e.k in free and j in e.earliest
                       and e.earliest[j] <= t_hi), key=lambda e: e.earliest[j])
        if len(pool) < n:
            return None
        out.extend(pool[:n])
    return out


def hungarian_assign(W: World, chosen: List[int], obs_end: Dict[int, float], free: set):
    """Energy-optimal effector assignment for a chosen set of hostiles (Hungarian, per subtype).

    A pair is allowed only if the effector arrives before the contact's latest engagement time.
    Contacts that cannot get their full set (all-or-nothing) are dropped, least valuable first,
    and the assignment is solved again. Returns {contact idx: (T, effector ks)}.
    """
    r = W.rules
    C = W.cands
    chosen = sorted(chosen, key=lambda j: -C[j].value)

    def window(j):
        c = C[j]
        return (max(obs_end[j] + r.confirm_margin_s + 0.5, c.lo + 0.05),
                c.hi - GROUP_DT * (c.n_eff - 1) - 0.5)

    while chosen:
        members: Dict[int, List[int]] = {j: [] for j in chosen}
        failed = None
        for sub in sorted({s for j in chosen for s in C[j].needs}):
            slots = [j for j in chosen for _ in range(C[j].needs.get(sub, 0))]
            cols = [e for e in W.effs if e.sub == sub and e.k in free]
            if len(slots) > len(cols):
                failed = min((j for j in chosen if C[j].needs.get(sub)), key=lambda j: C[j].value)
                break
            cost = []
            for j in slots:
                t_hi = window(j)[1] - 2.0
                cost.append([e.energy[j] if e.earliest.get(j, INF) <= t_hi else INF for e in cols])
            res = assign(cost)
            bad = [j for j, col in zip(slots, res) if col is None]
            if bad:
                failed = min(bad, key=lambda j: C[j].value)
                break
            for j, col in zip(slots, res):
                members[j].append(cols[col].k)
        if failed is None:
            out = {}
            for j, ks in members.items():
                t_lo, t_hi = window(j)
                T = min(t_hi, max(t_lo, max(W.effs[k].earliest[j] for k in ks) + 1.0))
                out[j] = (T, ks)
            return out
        chosen.remove(failed)
    return {}


def place_effectors(W: World, air: Airspace, obs_end: Dict[int, float], deadline: float):
    """Matches effectors to observed hostiles and realises each engagement group."""
    r = W.rules
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
        # same contacts, energy-optimal effectors (falls back to the greedy choice if it fails)
        try:
            opt = hungarian_assign(W, list(served), obs_end, free)
            if set(opt) == set(served):
                served = opt
        except Exception:
            pass
        progress = False
        for j in sorted(served, key=lambda jj: -W.cands[jj].value):
            T, ks = served[j]
            if any(k not in free for k in ks):
                continue
            c = W.cands[j]
            members = [W.effs[k] for k in ks]
            items = place_group(W, air, c, T, members, deadline, cache)
            if items is None:
                # retry with the fastest free effectors, which leave more room in the window
                t_lo = max(obs_end[j] + r.confirm_margin_s + 0.5, c.lo + 0.05)
                t_hi = c.hi - GROUP_DT * (c.n_eff - 1) - 0.5
                alt = _fastest_members(W, j, free, t_hi - 2.0)
                if alt is not None and {e.k for e in alt} != set(ks):
                    T2 = min(t_hi, max(t_lo, max(e.earliest[j] for e in alt) + 1.0))
                    items = place_group(W, air, c, T2, alt, deadline, cache)
                    if items is not None:
                        ks = [e.k for e in alt]
            if items is None:
                done.add(j)          # tried twice: give up on this contact
                continue
            done.add(j)
            for it in items:
                air.commit(it)
            for k in ks:
                free.discard(k)
            groups[c.id] = [it.aid for it in items]
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
