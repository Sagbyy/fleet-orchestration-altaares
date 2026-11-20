"""Preprocessing: obstacles, candidate contacts, observation rings and per-aircraft estimates."""
from __future__ import annotations

import math
import os
from typing import Dict, List, Optional, Tuple

from fleetkit.energy import best_range_speed
from fleetkit.geometry import hdist
from fleetkit.model import CIVILIAN, EFFECTOR, FRIENDLY, HOSTILE, SCOUT, UNKNOWN, Aircraft, Scenario

from .geom import Circle, XY, free_point, model_leg

V_SAFE = 0.99          # realised speeds as a fraction of the limits
OBS_RHO = 146.5        # horizontal offset of an observation point at z = 30 m (3D range 149.5 < 150)
OBS_RHO_HIGH = 128.0   # max horizontal offset at z = 70 m (3D range 146 < 150)
SCOUT_Z = (30.0, 70.0)
SCOUT_Z_MODEL = 30.0
GROUND_RUN = os.environ.get("FLEET_GROUND_RUN", "1") == "1"   # effectors may fly [home, engage] at z = 0
EFF_Z = ((0.0,) if GROUND_RUN else ()) + (30.0, 40.0, 80.0, 110.0, 150.0)   # cheapest conflict-free layer wins
EFF_Z_MODEL = 40.0                   # layer for earliest-arrival estimates (most effectors fly low)
EFF_Z_ENERGY = 40.0                  # typical layer for energy estimates
GROUP_DT = 2.0         # arrival spacing between effectors of one engagement
RING_SAMPLES = 48


class ScoutInfo:
    def __init__(self, k: int, ac: Aircraft):
        self.k, self.ac, self.id = k, ac, ac.id
        self.home: XY = (ac.home[0], ac.home[1])
        p = ac.power
        self.v_real = V_SAFE * ac.v_h_mps
        self.v_model = 0.985 * ac.v_h_mps
        self.p0, self.kv = p.p0_w, p.k_v
        # climb-out and landing are flown diagonally on the first and last legs (R2 only bounds
        # the altitude of intermediate waypoints), so they cost almost no extra time
        self.climb_t = 1.0
        self.land_t = 1.0
        epm = lambda v: (p.p0_w + p.k_v * v * v) / v / 3600.0  # noqa: E731
        self.e_per_m = max(epm(self.v_model), epm(self.v_real))
        # best-range speed, used when the battery is what limits the tour
        self.v_eco = min(self.v_model, max(best_range_speed(p), 0.6 * self.v_model))
        self.e_per_m_eco = epm(self.v_eco) * 1.01
        self.hover = p.p0_w / 3600.0
        self.fixed_wh = p.e_climb_wh_per_m * SCOUT_Z_MODEL + self.hover * (self.climb_t + self.land_t)
        self.usable = ac.usable_wh - 0.2


class EffInfo:
    def __init__(self, k: int, ac: Aircraft):
        self.k, self.ac, self.id, self.sub = k, ac, ac.id, ac.subtype
        self.home: XY = (ac.home[0], ac.home[1])
        self.speed_rank = ac.v_h_mps
        self.earliest: Dict[int, float] = {}
        self.energy: Dict[int, float] = {}


class Cand:
    def __init__(self, idx: int, contact, kind: str):
        self.idx, self.c, self.id, self.kind = idx, contact, contact.id, kind
        self.x, self.y = contact.pos[0], contact.pos[1]
        self.lo, self.hi = contact.window
        self.points = contact.points
        self.needs = {k: v for k, v in contact.needs.items() if v > 0} if kind == "h" else {}
        self.n_eff = sum(self.needs.values())
        self.latest_end = 0.0
        self.value = 0.0
        self.ring: List[XY] = []
        self.near: List[Circle] = []     # obstacles that can contain one of its observation points
        self.neighbors: List[int] = []


def dive_geometry(ac: Aircraft, z: float) -> Tuple[float, float]:
    """Returns (horizontal dive length, dive duration) for a final dive from altitude z."""
    dd = min(350.0, max(60.0, z * ac.v_h_mps / ac.v_down_mps))
    t = max(dd / (V_SAFE * ac.v_h_mps), z / (V_SAFE * ac.v_down_mps)) + 0.05
    return dd, t


def effector_estimate(ac: Aircraft, dist: float, z_time: float = EFF_Z_MODEL,
                      z: float = EFF_Z_ENERGY) -> Tuple[float, float]:
    """Returns (earliest arrival time, energy Wh at cruise speed) for an effector flying `dist` metres.

    The arrival time assumes the highest layer (slowest climb); the energy assumes a low layer.
    """
    p = ac.power
    dd_t, dive_tt = dive_geometry(ac, z_time)
    earliest = z_time / (V_SAFE * ac.v_up_mps) + 0.2 + max(0.0, dist - dd_t) / (V_SAFE * ac.v_h_mps) + dive_tt
    climb_t = z / (V_SAFE * ac.v_up_mps) + 0.2
    dd, dive_t = dive_geometry(ac, z)
    cruise = max(0.0, dist - dd)
    vc = min(best_range_speed(p), V_SAFE * ac.v_h_mps)
    vd = dd / dive_t
    wh = (p.e_climb_wh_per_m * z + p.p0_w * climb_t / 3600.0
          + (p.p0_w + p.k_v * vc * vc) * cruise / vc / 3600.0
          + (p.p0_w + p.k_v * vd * vd) * dive_t / 3600.0)
    return earliest, wh


class World:
    """Everything the optimiser needs, computed once."""

    def __init__(self, sc: Scenario):
        self.sc = sc
        r = sc.rules
        self.rules = r
        self.horizon = sc.horizon_s
        self.protected = [Circle(c.pos[0], c.pos[1], r.protect_radius_m, math.inf, c.id)
                          for c in sc.contacts if c.cls in (FRIENDLY, CIVILIAN)]
        self.zones = [Circle(z.center[0], z.center[1], z.radius_m, z.top_m, z.id) for z in sc.zones]
        self.scout_model_circles = self.protected + self.zones
        self.scouts = [ScoutInfo(k, a) for k, a in enumerate(a for a in sc.aircraft if a.type == SCOUT)]
        self.effs = [EffInfo(k, a) for k, a in enumerate(a for a in sc.aircraft if a.type == EFFECTOR)]
        self.n_sub: Dict[str, int] = {}
        for e in self.effs:
            self.n_sub[e.sub] = self.n_sub.get(e.sub, 0) + 1
        homes = [a.home for a in sc.aircraft] or [(0.0, 0.0, 0.0)]
        self.base: XY = (sum(h[0] for h in homes) / len(homes), sum(h[1] for h in homes) / len(homes))
        self.cands: List[Cand] = []
        self._legs: Dict[tuple, float] = {}
        self._build_cands()

    def circles_at(self, z: float) -> List[Circle]:
        """Obstacles that matter for level flight at altitude z."""
        return self.protected + [c for c in self.zones if c.top > z]

    def _forbidden(self, c) -> bool:
        r = self.rules
        for f in self.sc.contacts:
            if f.cls in (FRIENDLY, CIVILIAN) and hdist(c.pos, f.pos) < r.collateral_radius_m + 1.0:
                return True
        return False

    def _build_cands(self) -> None:
        r = self.rules
        eff_circles = self.circles_at(EFF_Z_MODEL)
        for c in self.sc.contacts:
            if c.window is None:
                continue
            if c.cls == HOSTILE:
                if self._forbidden(c):
                    continue
                needs = {k: v for k, v in c.needs.items() if v > 0}
                if not needs or any(self.n_sub.get(k, 0) < v for k, v in needs.items()):
                    continue
                cand = Cand(len(self.cands), c, "h")
                cand.latest_end = cand.hi - r.confirm_margin_s - GROUP_DT * (cand.n_eff - 1) - 1.0
            elif c.cls == UNKNOWN:
                cand = Cand(len(self.cands), c, "u")
                cand.latest_end = cand.hi - 0.05
            else:
                continue
            if cand.latest_end - (cand.lo + 0.05) < r.obs_dwell_s:
                continue
            cand.near = [cc for cc in self.scout_model_circles
                         if math.hypot(cc.cx - cand.x, cc.cy - cand.y) < OBS_RHO + cc.wrap + 10.0]
            cand.ring = self._ring(cand)
            if not cand.ring:
                continue
            if cand.kind == "h":
                extra = model_leg(self.base[0], self.base[1], cand.x, cand.y, eff_circles) \
                    - math.hypot(cand.x - self.base[0], cand.y - self.base[1])
                tot_wh = 0.0
                for sub, n in cand.needs.items():
                    whs = []
                    for e in self.effs:
                        if e.sub != sub:
                            continue
                        dist = math.hypot(cand.x - e.home[0], cand.y - e.home[1]) + extra
                        t, wh = effector_estimate(e.ac, dist)
                        if wh <= e.ac.usable_wh - 0.5:
                            e.earliest[cand.idx] = t
                            e.energy[cand.idx] = wh
                            whs.append(wh)
                    if len(whs) < n:
                        tot_wh = None
                        break
                    whs.sort()
                    tot_wh += sum(whs[:n])
                if tot_wh is None:
                    for e in self.effs:
                        e.earliest.pop(cand.idx, None)
                        e.energy.pop(cand.idx, None)
                    continue
                cand.value = cand.points - r.c_eff * cand.n_eff - r.lambda_e * tot_wh
                if cand.value <= 0.5:
                    for e in self.effs:
                        e.earliest.pop(cand.idx, None)
                        e.energy.pop(cand.idx, None)
                    continue
            else:
                cand.value = cand.points
            self.cands.append(cand)
        # re-index (dropped candidates leave gaps) and keep effector tables consistent
        remap = {}
        for new, cand in enumerate(self.cands):
            remap[cand.idx] = new
            cand.idx = new
        for e in self.effs:
            e.earliest = {remap[k]: v for k, v in e.earliest.items() if k in remap}
            e.energy = {remap[k]: v for k, v in e.energy.items() if k in remap}
        for cand in self.cands:
            others = sorted((math.hypot(o.x - cand.x, o.y - cand.y), o.idx) for o in self.cands if o is not cand)
            cand.neighbors = [i for _, i in others]

    def _ring(self, cand: Cand) -> List[XY]:
        circles = self.scout_model_circles
        for rho in (OBS_RHO, 100.0, 75.0, 50.0, 25.0):
            pts = []
            for k in range(RING_SAMPLES):
                a = 2.0 * math.pi * k / RING_SAMPLES
                x, y = cand.x + rho * math.cos(a), cand.y + rho * math.sin(a)
                if free_point(x, y, circles, 3.0):
                    pts.append((x, y))
            if pts:
                return pts
        return []

    def obs_point(self, cand: Cand, a: XY, b: XY) -> XY:
        """Best observation point for `cand` when flying from a to b."""
        ax, ay = a
        bx, by = b
        dx, dy = bx - ax, by - ay
        L2 = dx * dx + dy * dy
        if L2 < 1e-9:
            t = 0.0
        else:
            t = max(0.0, min(1.0, ((cand.x - ax) * dx + (cand.y - ay) * dy) / L2))
        qx, qy = ax + t * dx, ay + t * dy
        vx, vy = qx - cand.x, qy - cand.y
        d = math.hypot(vx, vy)
        if d <= OBS_RHO - 1.0:
            px, py = qx, qy
        elif d < 1e-9:
            px, py = cand.x + OBS_RHO, cand.y
        else:
            px, py = cand.x + vx / d * OBS_RHO, cand.y + vy / d * OBS_RHO
        if free_point(px, py, cand.near, 3.0):
            return (px, py)
        best, bd = cand.ring[0], math.inf
        for (x, y) in cand.ring:
            dd = math.hypot(x - ax, y - ay) + math.hypot(bx - x, by - y)
            if dd < bd:
                best, bd = (x, y), dd
        return best

    def leg(self, a: XY, b: XY) -> float:
        key = (a, b)
        v = self._legs.get(key)
        if v is None:
            if len(self._legs) > 400000:
                self._legs.clear()
            v = model_leg(a[0], a[1], b[0], b[1], self.scout_model_circles)
            self._legs[key] = v
        return v
