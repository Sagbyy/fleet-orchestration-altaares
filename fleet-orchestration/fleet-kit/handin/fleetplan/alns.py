"""Abstract model and ALNS.

A solution is one ordered list of observations per scout (a team orienteering problem with time
windows). Hostile contacts only pay off if a full set of effectors can be matched to them after the
observation, so every evaluated solution runs the effector matching.
"""
from __future__ import annotations

import math
import os
import random
import time
from typing import Dict, List, Optional, Tuple

from .hungarian import assign
from .world import GROUP_DT, Cand, ScoutInfo, World, XY

INF = math.inf


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


def _forward_at(W: World, s: ScoutInfo, cidx: List[int], legs: List[float], v: float, epm: float):
    n = len(cidx)
    C = W.cands
    dwell = W.rules.obs_dwell_s
    c0 = C[cidx[0]]
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
        c = C[cidx[i]]
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
    if end > W.horizon - 2.0:
        return False, 0.0, 0.0, None
    dist = 0.0
    for l in legs:
        dist += l
    energy = s.fixed_wh + epm * dist + slow_extra + s.hover * dwell * n
    return True, end - start, energy, ends


def forward(W: World, s: ScoutInfo, cidx: List[int], legs: List[float]):
    """Times a scout tour. Returns (feasible, end time, energy Wh, observation end times).

    Flown at full speed when the battery allows; otherwise at the best-range speed v*
    (minimum energy per metre), if the windows still hold at that speed.
    """
    if not cidx:
        return True, 0.0, 0.0, []
    ok, end, energy, ends = _forward_at(W, s, cidx, legs, s.v_model, s.e_per_m)
    if ok and energy <= s.usable:
        return True, end, energy, ends
    if s.v_eco >= s.v_model - 1e-9:
        return False, 0.0, 0.0, None
    ok, end, energy, ends = _forward_at(W, s, cidx, legs, s.v_eco, s.e_per_m_eco)
    if ok and energy <= s.usable:
        return True, end, energy, ends
    return False, 0.0, 0.0, None


def refresh(W: World, r: Route) -> bool:
    ok, end, energy, ends = forward(W, r.s, r.cands, r.legs)
    if ok:
        r.end, r.energy, r.ends = end, energy, ends
    return ok


class Sol:
    def __init__(self, W: World):
        self.W = W
        self.routes = [Route(s) for s in W.scouts]
        self.where: Dict[int, int] = {}       # cand idx -> route index
        self.need: Dict[str, int] = {}        # effectors claimed by routed hostiles, per subtype
        self.obj = 0.0
        self.served: Dict[int, Tuple[float, List[int]]] = {}

    def copy(self) -> "Sol":
        o = Sol.__new__(Sol)
        o.W = self.W
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
        refresh(self.W, r)
        self.where[j] = k
        for sub, n in self.W.cands[j].needs.items():
            self.need[sub] = self.need.get(sub, 0) + n

    def remove(self, j: int) -> None:
        k = self.where.pop(j)
        r = self.routes[k]
        pos = r.cands.index(j)
        a = r.s.home if pos == 0 else r.pts[pos - 1]
        b = r.s.home if pos == len(r.cands) - 1 else r.pts[pos + 1]
        r.cands.pop(pos)
        r.pts.pop(pos)
        r.legs[pos:pos + 2] = [self.W.leg(a, b)]
        for sub, n in self.W.cands[j].needs.items():
            self.need[sub] -= n
        if not refresh(self.W, r):
            self._trim(k)

    def _trim(self, k: int) -> None:
        r = self.routes[k]
        while r.cands and not refresh(self.W, r):
            self.remove(r.cands[-1])
        if not r.cands:
            r.legs, r.end, r.energy, r.ends = [0.0], 0.0, 0.0, []

    # ---- evaluation -----------------------------------------------------------------------
    def evaluate(self) -> float:
        W = self.W
        lam = W.rules.lambda_e
        total = 0.0
        hostiles = []
        for r in self.routes:
            total -= lam * r.energy
            for j, e in zip(r.cands, r.ends):
                c = W.cands[j]
                if c.kind == "u":
                    total += c.points
                else:
                    hostiles.append((j, e))
        served, val = match(W, hostiles)
        self.served = served
        self.obj = total + val
        return self.obj


def knapsack_pick(items, cap: Dict[str, int]) -> List[int]:
    """Best subset of hostiles under effector counts (exact DP over the scarce subtypes only).

    items: (contact idx, value, needs). When no subtype is scarce every item fits and all are
    returned; otherwise a 1D or 2D knapsack picks the most valuable feasible subset."""
    subs = sorted({s for _, _, n in items for s in n})
    scarce = [s for s in subs if sum(n.get(s, 0) for _, _, n in items) > cap.get(s, 0)]
    if not scarce:
        return [j for j, _, _ in items]
    dp = {tuple(0 for _ in scarce): (0.0, ())}
    for j, v, n in items:
        if v <= 0.0:
            continue
        add = tuple(n.get(s, 0) for s in scarce)
        if any(a > cap.get(s, 0) for a, s in zip(add, scarce)):
            continue
        new = dict(dp)
        for st, (val, ch) in dp.items():
            ns = tuple(a + b for a, b in zip(st, add))
            if any(x > cap.get(s, 0) for x, s in zip(ns, scarce)):
                continue
            nv = val + v
            cur = new.get(ns)
            if cur is None or cur[0] < nv:
                new[ns] = (nv, ch + (j,))
        dp = new
    return list(max(dp.values(), key=lambda x: x[0])[1])


def match(W: World, hostiles: List[Tuple[int, float]], avail: Optional[set] = None):
    """Effector matching.

    1. pick the best set of hostiles under the effector counts (knapsack when effectors are scarce);
    2. assign effectors earliest-deadline first, the slowest effector that still makes it;
    3. fill the remaining effectors greedily with whatever was left out."""
    r = W.rules
    C = W.cands
    used = set()
    served: Dict[int, Tuple[float, List[int]]] = {}
    total = 0.0
    window = {}
    for j, oe in hostiles:
        c = C[j]
        t_lo = max(oe + r.confirm_margin_s + 0.5, c.lo + 0.05)
        t_hi = c.hi - GROUP_DT * (c.n_eff - 1) - 0.5
        if t_lo <= t_hi:
            window[j] = (t_lo, t_hi)
    cap: Dict[str, int] = {}
    for e in W.effs:
        if avail is None or e.k in avail:
            cap[e.sub] = cap.get(e.sub, 0) + 1
    chosen = knapsack_pick([(j, C[j].value, C[j].needs) for j in window], cap) if KNAPSACK else []
    chosen_set = set(chosen)
    rest = sorted((j for j in window if j not in chosen_set), key=lambda j: -C[j].value)
    order = sorted(chosen, key=lambda j: window[j][1]) + rest

    for j in order:
        c = C[j]
        t_lo, t_hi = window[j]
        pick = []
        ok = True
        for sub, n in c.needs.items():
            pool = [e for e in W.effs if e.sub == sub and e.k not in used
                    and (avail is None or e.k in avail)
                    and e.earliest.get(j, INF) <= t_hi - 2.0]
            if len(pool) < n:
                ok = False
                break
            pool.sort(key=lambda e: -e.earliest[j])
            pick.extend(pool[:n])
        if not ok:
            continue
        T = max(t_lo, max(e.earliest[j] for e in pick) + 1.0)
        if T > t_hi:
            T = t_hi
        for e in pick:
            used.add(e.k)
        served[j] = (T, [e.k for e in pick])
        total += c.points - r.c_eff * len(pick) - r.lambda_e * sum(e.energy[j] for e in pick)
    return served, total


KNAPSACK = os.environ.get("FLEET_KNAPSACK", "0") == "1"


# ---- insertion ------------------------------------------------------------------------------

class Params:
    def __init__(self, **kw):
        self.k_pos = kw.get("k_pos", 3)
        self.k_routes = kw.get("k_routes", 4)
        self.time_price = kw.get("time_price", 0.02)
        self.regret_w = kw.get("regret_w", 1.0)
        self.noise = kw.get("noise", 0.15)
        self.q_min = kw.get("q_min", 2)
        self.q_frac = kw.get("q_frac", 0.3)
        self.t0_frac = kw.get("t0_frac", 0.02)
        self.reaction = kw.get("reaction", 0.2)
        self.segment = kw.get("segment", 30)
        self.stall = kw.get("stall", 20000)
        self.reassign = kw.get("reassign", 1)
        self.local_search = kw.get("local_search", 1)
        self.elite = kw.get("elite", 5)          # distinct solutions handed to the 3D realisation
        self.elite_gap = kw.get("elite_gap", 0.03)   # relocate / Or-opt / merge on new bests   # Hungarian tour -> scout reassignment each iteration   # stop after this many iterations without a new best


def _gain(W: World, j: int, d_energy: float, d_end: float, P: Params) -> float:
    return W.cands[j].value - W.rules.lambda_e * d_energy - P.time_price * d_end


def eval_route(W: World, sol: Sol, k: int, j: int, P: Params):
    """Best insertion of contact j in route k: (gain, pos, point, l1, l2) or None."""
    r = sol.routes[k]
    c = W.cands[j]
    n = len(r.cands)
    home = r.s.home
    cx, cy = c.x, c.y
    est = []
    prev = home
    for pos in range(n + 1):
        nxt = home if pos == n else r.pts[pos]
        e = math.hypot(prev[0] - cx, prev[1] - cy) + math.hypot(cx - nxt[0], cy - nxt[1]) \
            - math.hypot(prev[0] - nxt[0], prev[1] - nxt[1])
        est.append((e, pos))
        prev = nxt
    est.sort()
    best = None
    for _, pos in est[:P.k_pos]:
        a = home if pos == 0 else r.pts[pos - 1]
        b = home if pos == n else r.pts[pos]
        p = W.obs_point(c, a, b)
        l1, l2 = W.leg(a, p), W.leg(p, b)
        legs = r.legs[:pos] + [l1, l2] + r.legs[pos + 1:]
        cidx = r.cands[:pos] + [j] + r.cands[pos:]
        ok, end, energy, _ = forward(W, r.s, cidx, legs)
        if not ok:
            continue
        g = _gain(W, j, energy - r.energy, end - r.end, P)
        if best is None or g > best[0]:
            best = (g, pos, p, l1, l2)
    return best


def _cheap(W: World, r: Route, c: Cand) -> float:
    n = len(r.cands)
    home = r.s.home
    prev = home
    best = INF
    for pos in range(n + 1):
        nxt = home if pos == n else r.pts[pos]
        e = math.hypot(prev[0] - c.x, prev[1] - c.y) + math.hypot(c.x - nxt[0], c.y - nxt[1]) \
            - math.hypot(prev[0] - nxt[0], prev[1] - nxt[1])
        if e < best:
            best = e
        prev = nxt
    return best


def _need_ok(W: World, sol: Sol, j: int) -> bool:
    for sub, n in W.cands[j].needs.items():
        if sol.need.get(sub, 0) + n > W.n_sub.get(sub, 0):
            return False
    return True


def repair(W: World, sol: Sol, pool: List[int], rng: random.Random, P: Params,
           regret: bool = True, noise: float = 0.0) -> None:
    """Regret (or greedy) insertion of the pool into the solution."""
    cache: Dict[int, Dict[int, tuple]] = {}
    pool = [j for j in pool if j not in sol.where]
    for j in pool:
        c = W.cands[j]
        busy = [k for k, r in enumerate(sol.routes) if r.cands]
        busy.sort(key=lambda k: _cheap(W, sol.routes[k], c))
        chosen = busy[:P.k_routes] + [k for k, r in enumerate(sol.routes) if not r.cands]
        cache[j] = {}
        for k in chosen:
            res = eval_route(W, sol, k, j, P)
            if res is not None:
                cache[j][k] = res
    while pool:
        best_j, best_score, best_ins = None, -INF, None
        for j in pool:
            if not _need_ok(W, sol, j):
                continue
            entries = sorted((v[0], k, v) for k, v in cache[j].items())
            if not entries:
                continue
            g1, k1, v1 = entries[-1]
            if g1 <= 0.0:
                continue
            if noise:
                g1n = g1 * (1.0 + noise * (2.0 * rng.random() - 1.0))
            else:
                g1n = g1
            if regret:
                g2 = entries[-2][0] if len(entries) > 1 else 0.0
                score = g1n + P.regret_w * (g1 - max(g2, 0.0))
            else:
                score = g1n
            if score > best_score:
                best_j, best_score, best_ins = j, score, (k1, v1)
        if best_j is None:
            break
        k, (g, pos, p, l1, l2) = best_ins
        sol.insert(k, pos, best_j, p, l1, l2)
        pool.remove(best_j)
        for j in pool:
            res = eval_route(W, sol, k, j, P)
            if res is None:
                cache[j].pop(k, None)
            else:
                cache[j][k] = res


def reoptimise_points(W: World, sol: Sol, k: int) -> None:
    """Moves each observation point to the best spot given its neighbours, when that helps."""
    r = sol.routes[k]
    n = len(r.cands)
    for pos in range(n):
        a = r.s.home if pos == 0 else r.pts[pos - 1]
        b = r.s.home if pos == n - 1 else r.pts[pos + 1]
        p = W.obs_point(W.cands[r.cands[pos]], a, b)
        if p == r.pts[pos]:
            continue
        l1, l2 = W.leg(a, p), W.leg(p, b)
        if l1 + l2 >= r.legs[pos] + r.legs[pos + 1] - 1e-6:
            continue
        legs = r.legs[:pos] + [l1, l2] + r.legs[pos + 2:]
        ok, end, energy, ends = forward(W, r.s, r.cands, legs)
        if ok:
            r.pts[pos] = p
            r.legs = legs
            r.end, r.energy, r.ends = end, energy, ends


def reassign_tours(W: World, sol: Sol) -> bool:
    """Gives each tour to the scout that flies it cheapest (Hungarian on tours x scouts).

    Scouts differ in battery, speed and consumption; a long tour on a small-battery scout can
    block contacts that only a big-battery scout could reach. Returns True if anything moved.
    """
    tours = [r for r in sol.routes if r.cands]
    if len(tours) < 1 or len(tours) > len(sol.routes):
        return False
    lam = W.rules.lambda_e
    cost, cache = [], {}
    for r in tours:
        row = []
        mid = r.legs[1:-1]
        for s in W.scouts:
            if s is r.s:
                row.append(lam * r.energy)
                cache[(id(r), s.k)] = (r.legs, r.end, r.energy, r.ends)
                continue
            legs = [W.leg(s.home, r.pts[0])] + mid + [W.leg(r.pts[-1], s.home)]
            ok, end, energy, ends = forward(W, s, r.cands, legs)
            if ok:
                row.append(lam * energy)
                cache[(id(r), s.k)] = (legs, end, energy, ends)
            else:
                row.append(INF)
        cost.append(row)
    res = assign(cost)
    if any(c is None for c in res):
        return False
    if all(tours[i].s.k == res[i] for i in range(len(tours))):
        return False
    new_routes = [Route(s) for s in W.scouts]
    for i, r in enumerate(tours):
        k = res[i]
        nr = new_routes[k]
        nr.cands, nr.pts = list(r.cands), list(r.pts)
        nr.legs, nr.end, nr.energy, nr.ends = cache[(id(r), k)]
        nr.legs = list(nr.legs)
        for j in nr.cands:
            sol.where[j] = k
    sol.routes = new_routes
    return True


# ---- local search (relocate, Or-opt, tour merge) ------------------------------------------

def _tour(W: World, sc: ScoutInfo, cands: List[int], pts: List[XY]):
    """Times a candidate tour on scout sc: (ok, end, energy, ends, legs)."""
    if not cands:
        return True, 0.0, 0.0, [], [0.0]
    nodes = [sc.home] + list(pts) + [sc.home]
    legs = [W.leg(nodes[i], nodes[i + 1]) for i in range(len(nodes) - 1)]
    ok, end, energy, ends = forward(W, sc, cands, legs)
    return ok, end, energy, ends, legs


def _set(sol: Sol, k: int, cands, pts, res) -> None:
    ok, end, energy, ends, legs = res
    r = sol.routes[k]
    r.cands, r.pts, r.legs = list(cands), list(pts), list(legs)
    r.end, r.energy, r.ends = end, energy, list(ends)
    for j in r.cands:
        sol.where[j] = k


def _move_block(W: World, sol: Sol, deadline: float, size: int) -> bool:
    """Relocate (size 1) / Or-opt (size 2): move a block of consecutive visits to the
    position, in any tour, that saves the most energy. First improvement."""
    routes = sol.routes
    empties = [k for k, r in enumerate(routes) if not r.cands]
    for k in range(len(routes)):
        rk = routes[k]
        for i in range(len(rk.cands) - size + 1):
            if time.monotonic() > deadline:
                return False
            block = rk.cands[i:i + size]
            bpts = rk.pts[i:i + size]
            rest_c = rk.cands[:i] + rk.cands[i + size:]
            rest_p = rk.pts[:i] + rk.pts[i + size:]
            res_k = _tour(W, rk.s, rest_c, rest_p)
            if not res_k[0]:
                continue
            targets = [t for t in range(len(routes)) if routes[t].cands or t in empties[:3]]
            for t in targets:
                rt = routes[t]
                base_c = rest_c if t == k else rt.cands
                base_p = rest_p if t == k else rt.pts
                before = rk.energy + (0.0 if t == k else rt.energy)
                for pos in range(len(base_c) + 1):
                    if t == k and pos == i:
                        continue
                    nc = base_c[:pos] + block + base_c[pos:]
                    npts = base_p[:pos] + bpts + base_p[pos:]
                    if size == 1:   # re-centre the single observation point between its new neighbours
                        a = rt.s.home if pos == 0 else base_p[pos - 1]
                        b = rt.s.home if pos == len(base_c) else base_p[pos]
                        npts[pos] = W.obs_point(W.cands[block[0]], a, b)
                    res_t = _tour(W, rt.s, nc, npts)
                    if not res_t[0]:
                        continue
                    after = res_t[2] + (0.0 if t == k else res_k[2])
                    if after < before - 1e-3:
                        if t != k:
                            _set(sol, k, rest_c, rest_p, res_k)
                        _set(sol, t, nc, npts, res_t)
                        return True
    return False


def _merge(W: World, sol: Sol, deadline: float) -> bool:
    """Tour merge: fly two tours as one on a single scout (saves a base round trip and frees a scout)."""
    routes = sol.routes
    busy = [k for k, r in enumerate(routes) if r.cands]
    empties = [k for k, r in enumerate(routes) if not r.cands]
    for x in range(len(busy)):
        for y in range(x + 1, len(busy)):
            if time.monotonic() > deadline:
                return False
            k1, k2 = busy[x], busy[y]
            r1, r2 = routes[k1], routes[k2]
            before = r1.energy + r2.energy
            for a, b in ((r1, r2), (r2, r1)):
                nc, npts = a.cands + b.cands, a.pts + b.pts
                for t in [k1, k2] + empties:
                    res = _tour(W, routes[t].s, nc, npts)
                    if res[0] and res[2] < before - 1e-3:
                        for kk in (k1, k2):
                            if kk != t:
                                _set(sol, kk, [], [], (True, 0.0, 0.0, [], [0.0]))
                        _set(sol, t, nc, npts, res)
                        return True
    return False


def local_search(W: World, sol: Sol, P: "Params", rng: random.Random, deadline: float) -> Sol:
    """Polishes a solution with small moves, then re-inserts contacts into freed capacity.
    Returns the better of the input and the polished solution (full objective)."""
    new = sol.copy()
    improved = True
    while improved and time.monotonic() < deadline:
        improved = _move_block(W, new, deadline, 1) or _move_block(W, new, deadline, 2) \
            or _merge(W, new, deadline)
    pool = [c.idx for c in W.cands if c.idx not in new.where]
    repair(W, new, pool, rng, P, regret=True)
    new.evaluate()
    return new if new.obj > sol.obj + 1e-6 else sol


# ---- destroy operators ----------------------------------------------------------------------

def _routed(sol: Sol) -> List[int]:
    return list(sol.where.keys())


def d_random(W, sol, q, rng):
    js = _routed(sol)
    rng.shuffle(js)
    return js[:q]


def d_worst(W, sol, q, rng):
    lam = W.rules.lambda_e
    scored = []
    for k, r in enumerate(sol.routes):
        for pos, j in enumerate(r.cands):
            c = W.cands[j]
            if c.kind == "u":
                val = c.points
            else:
                val = c.value if j in sol.served else 0.0
            a = r.s.home if pos == 0 else r.pts[pos - 1]
            b = r.s.home if pos == len(r.cands) - 1 else r.pts[pos + 1]
            saved = r.legs[pos] + r.legs[pos + 1] - math.hypot(a[0] - b[0], a[1] - b[1])
            contrib = val - lam * r.s.e_per_m * saved - 0.05 * saved / r.s.v_model
            scored.append((contrib, j))
    scored.sort()
    out = []
    while scored and len(out) < q:
        i = int(len(scored) * (rng.random() ** 3))
        out.append(scored.pop(i)[1])
    return out


def d_related(W, sol, q, rng):
    js = _routed(sol)
    if not js:
        return []
    seed = rng.choice(js)
    s = W.cands[seed]
    rel = sorted(js, key=lambda j: math.hypot(W.cands[j].x - s.x, W.cands[j].y - s.y)
                 + 3.0 * abs(W.cands[j].lo - s.lo))
    return rel[:q]


def d_route(W, sol, q, rng):
    ks = [k for k, r in enumerate(sol.routes) if r.cands]
    rng.shuffle(ks)
    out = []
    for k in ks:
        out.extend(sol.routes[k].cands)
        if len(out) >= q:
            break
    return out


def d_sector(W, sol, q, rng):
    js = _routed(sol)
    if not js:
        return []
    c0 = W.cands[rng.choice(js)]
    return sorted(js, key=lambda j: math.hypot(W.cands[j].x - c0.x, W.cands[j].y - c0.y))[:q]


def d_weak_route(W, sol, q, rng):
    """Empties the tour with the lowest value per Wh, so its contacts get absorbed by other
    tours: consolidation frees a scout and saves a base round trip."""
    lam = W.rules.lambda_e
    busy = [r for r in sol.routes if r.cands]
    if len(busy) < 2:
        return []

    def worth(r):
        v = sum(W.cands[j].value if (W.cands[j].kind == "u" or j in sol.served) else 0.0 for j in r.cands)
        return (v - lam * r.energy) / max(1.0, r.energy)

    busy.sort(key=worth)
    pick = busy[0] if rng.random() < 0.7 else rng.choice(busy[:3])
    return list(pick.cands)


DESTROY = [d_random, d_worst, d_related, d_route, d_sector] + \
    ([d_weak_route] if os.environ.get("FLEET_WEAK_ROUTE", "0") == "1" else [])
REPAIRS = [("regret", True, 0.0), ("regret_noise", True, 1.0), ("greedy_noise", False, 1.0)]


def _roulette(w: List[float], rng: random.Random) -> int:
    s = sum(w)
    x = rng.random() * s
    for i, v in enumerate(w):
        x -= v
        if x <= 0:
            return i
    return len(w) - 1


def initial(W: World, P: Params, rng: random.Random) -> Sol:
    sol = Sol(W)
    repair(W, sol, list(range(len(W.cands))), rng, P, regret=True)
    for k in range(len(sol.routes)):
        reoptimise_points(W, sol, k)
    sol.evaluate()
    return sol


def run(W: World, deadline: float, P: Params, seed: int = 0, log=None) -> Sol:
    rng = random.Random(seed)
    cur = initial(W, P, rng)
    best = cur.copy()
    if not W.cands:
        return best
    t_start = time.monotonic()
    span = max(1e-3, deadline - t_start)
    wd = [1.0] * len(DESTROY)
    wr = [1.0] * len(REPAIRS)
    sd = [0.0] * len(DESTROY)
    sr = [0.0] * len(REPAIRS)
    nd = [0] * len(DESTROY)
    nr = [0] * len(REPAIRS)
    t0 = max(1.0, P.t0_frac * abs(cur.obj))
    it = 0
    last_best = 0
    last_ls = 0.0
    elite: Dict[frozenset, "Sol"] = {}   # best distinct solutions (by set of routed contacts)

    def keep(sol_):
        key = frozenset(sol_.where)
        if key in elite and elite[key].obj >= sol_.obj:
            return
        if len(elite) >= P.elite and key not in elite:
            worst = min(elite, key=lambda kk: elite[kk].obj)
            if elite[worst].obj >= sol_.obj:
                return
            del elite[worst]
        elite[key] = sol_.copy()
    while time.monotonic() < deadline and it - last_best < P.stall:
        it += 1
        di, ri = _roulette(wd, rng), _roulette(wr, rng)
        new = cur.copy()
        n_routed = len(new.where)
        q = max(P.q_min, int(round(rng.uniform(0.1, P.q_frac) * n_routed)))
        removed = DESTROY[di](W, new, min(q, n_routed), rng)
        touched = set()
        for j in removed:
            if j in new.where:
                touched.add(new.where[j])
                new.remove(j)
        pool = [c.idx for c in W.cands if c.idx not in new.where]
        _, regret, noise = REPAIRS[ri]
        before = {k: len(r.cands) for k, r in enumerate(new.routes)}
        repair(W, new, pool, rng, P, regret=regret, noise=noise * P.noise)
        if P.reassign and reassign_tours(W, new):
            touched = set(range(len(new.routes)))
            pool = [c.idx for c in W.cands if c.idx not in new.where]
            repair(W, new, pool, rng, P, regret=regret, noise=noise * P.noise)
        for k, r in enumerate(new.routes):
            if k in touched or len(r.cands) != before[k]:
                reoptimise_points(W, new, k)
        new.evaluate()
        if new.obj > best.obj - P.elite_gap * abs(best.obj):
            keep(new)
        frac = (time.monotonic() - t_start) / span
        temp = t0 * max(0.0, 1.0 - frac) + 1e-9
        score = 0.0
        if new.obj > best.obj + 1e-6:
            if P.local_search and time.monotonic() - last_ls > 0.5:
                new = local_search(W, new, P, rng, min(deadline, time.monotonic() + 0.4))
                last_ls = time.monotonic()
            best = new.copy()
            cur = new
            score = 33.0
            last_best = it
        elif new.obj > cur.obj + 1e-6:
            cur = new
            score = 9.0
        elif rng.random() < math.exp(min(0.0, (new.obj - cur.obj) / temp)):
            cur = new
            score = 3.0
        sd[di] += score
        sr[ri] += score
        nd[di] += 1
        nr[ri] += 1
        if it % P.segment == 0:
            for i in range(len(wd)):
                if nd[i]:
                    wd[i] = (1 - P.reaction) * wd[i] + P.reaction * sd[i] / nd[i]
                wd[i] = max(wd[i], 0.05)
                sd[i], nd[i] = 0.0, 0
            for i in range(len(wr)):
                if nr[i]:
                    wr[i] = (1 - P.reaction) * wr[i] + P.reaction * sr[i] / nr[i]
                wr[i] = max(wr[i], 0.05)
                sr[i], nr[i] = 0.0, 0
    if P.local_search:   # final polish (bounded: the 3D realisation keeps its reserve)
        best = local_search(W, best, P, rng, time.monotonic() + 0.8)
    keep(best)
    best.elite = sorted((e for e in elite.values()), key=lambda e: -e.obj)
    if log is not None:
        log["iterations"] = it
        log["model_obj"] = best.obj
    return best
