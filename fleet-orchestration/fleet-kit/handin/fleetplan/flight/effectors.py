"""Effector realisation: Hungarian assignment, group placement with variants and retries."""
from __future__ import annotations

import math
from typing import Dict, List, Optional

from fleetkit.energy import best_range_speed, route_energy_wh
from fleetkit.geometry import segment_enters_cylinder

from .. import clock
from ..algorithms.hungarian import INF, assign
from ..config import ANGLES, DTS, EFF_Z, GROUP_DT, MIN_TRIES, SPREAD, V_SAFE
from ..geometry.obstacles import CHECK_MARGIN, free_point, path2d
from ..search.matching import match
from ..world import Cand, EffInfo, World, dive_geometry
from .airspace import Airspace, Item
from .waypoints import climb_leg, emit_leg, leg_min_time, segment_clear, to_kroute, waypoint


def _ground_effector(W: World, e: EffInfo, c: Cand, T: float):
    """Two-waypoint route [home, engage], both at z = 0 (R2 bounds only intermediate waypoints).

    Only when the straight line crosses no zone and no protected area. No climb energy at all."""
    ac = e.ac
    hx, hy = e.home
    if not segment_clear((hx, hy, 0.0), (c.x, c.y, 0.0), W):
        return None
    d = math.hypot(c.x - hx, c.y - hy)
    vmax = V_SAFE * ac.v_h_mps
    vc = min(best_range_speed(ac.power), vmax)
    t0 = T - d / vc
    if t0 < 0.0:
        if T < d / vmax - 1e-9:
            return None
        t0 = 0.0
    wps = [waypoint(t0, hx, hy, 0.0), waypoint(T, c.x, c.y, 0.0, action="engage", target=c.id)]
    if route_energy_wh(to_kroute(e.id, wps).waypoints, ac) > ac.usable_wh - 0.05:
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
    pts3 = climb_leg(W, e.home, seq, z, ac, vmax) + [(c.x, c.y, 0.0)]
    t_fast = leg_min_time(home3, pts3, ac, vmax)
    t_pref = leg_min_time(home3, pts3, ac, vc)
    t0 = T - t_pref
    if t0 < 0.0:
        if T < t_fast - 1e-9:
            return None
        t0 = 0.0
    wps = [waypoint(t0, hx, hy, 0.0)]
    emit_leg(wps, home3, pts3, t0, T, ac)
    wps[-1].update(action="engage", target=c.id)
    if route_energy_wh(to_kroute(e.id, wps).waypoints, ac) > ac.usable_wh - 0.05:
        return None
    return wps


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
