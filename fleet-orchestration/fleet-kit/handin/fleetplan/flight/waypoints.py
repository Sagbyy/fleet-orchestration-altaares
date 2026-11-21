"""Waypoint construction: segment timing, diagonal climb-out and landing."""
from __future__ import annotations

import math
from typing import List

from fleetkit.geometry import segment_enters_cylinder
from fleetkit.model import Route as KRoute
from fleetkit.model import Waypoint

from ..config import V_SAFE
from ..geometry.obstacles import CHECK_MARGIN
from ..world import World


def waypoint(t, x, y, z, **extra) -> dict:
    d = {"t": t, "x": x, "y": y, "z": z}
    d.update(extra)
    return d


def to_kroute(aid: str, wps: List[dict]) -> KRoute:
    return KRoute(aid, [Waypoint(w["t"], w["x"], w["y"], w["z"], w.get("action"), w.get("target"),
                                 w.get("dwell_s", 0.0)) for w in wps])


def segment_min_time(a, b, ac, vh: float) -> float:
    """Minimum duration of a straight 3D segment for this aircraft (horizontal speed vh)."""
    t = math.hypot(b[0] - a[0], b[1] - a[1]) / vh
    dz = b[2] - a[2]
    if dz > 0:
        t = max(t, dz / (V_SAFE * ac.v_up_mps))
    elif dz < 0:
        t = max(t, -dz / (V_SAFE * ac.v_down_mps))
    return t


def leg_min_time(start, pts3, ac, vh: float) -> float:
    total, cur = 0.0, start
    for q in pts3:
        total += segment_min_time(cur, q, ac, vh)
        cur = q
    return total


def emit_leg(wps: List[dict], start, pts3, t_dep: float, t_arr: float, ac) -> None:
    """Appends a 3D polyline leg flown from t_dep to t_arr; each segment gets its minimum time
    (at the speed limits) scaled up uniformly, so no segment ever exceeds a limit."""
    mins, cur = [], start
    for q in pts3:
        mins.append(segment_min_time(cur, q, ac, V_SAFE * ac.v_h_mps))
        cur = q
    m = sum(mins)
    d = t_arr - t_dep
    t = t_dep
    for q, mk in zip(pts3, mins):
        t += (mk * d / m) if m > 1e-12 else d / len(pts3)
        wps.append(waypoint(t, q[0], q[1], q[2]))
    wps[-1]["t"] = t_arr


def segment_clear(a, b, W: World) -> bool:
    """True if a (possibly sloped) segment stays out of every zone and protected area."""
    for cc in W.protected + W.zones:
        if segment_enters_cylinder(a, b, (cc.cx, cc.cy), cc.r + CHECK_MARGIN, cc.top):
            return False
    return True


def point_along(start, seq, dist):
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


def climb_leg(W: World, home, seq, z: float, ac, vh: float):
    """3D points of a leg that takes off from home (z = 0): diagonal climb when clear, else vertical."""
    dist_c = vh * z / (V_SAFE * ac.v_up_mps)
    p = point_along(home, seq[:1], dist_c)   # only on the first segment, to keep the order
    pts = ([(p[0], p[1], z)] if p is not None else []) + [(q[0], q[1], z) for q in seq]
    if segment_clear((home[0], home[1], 0.0), pts[0], W):
        return pts
    return [(home[0], home[1], z)] + [(q[0], q[1], z) for q in seq]


def landing_leg(W: World, start, seq, z: float, ac, vh: float):
    """3D points of a leg that ends landed at seq[-1]: diagonal descent when clear, else vertical."""
    home = seq[-1]
    rev = list(reversed([start] + list(seq[:-1])))
    dist_d = vh * z / (V_SAFE * ac.v_down_mps)
    q = point_along(home, rev[:1], dist_d)   # only on the last segment, to keep the order
    body = [(p[0], p[1], z) for p in seq[:-1]]
    if q is not None:
        body = body + [(q[0], q[1], z)]
    last_from = body[-1] if body else (start[0], start[1], z)
    if segment_clear(last_from, (home[0], home[1], 0.0), W):
        return body + [(home[0], home[1], 0.0)]
    return [(p[0], p[1], z) for p in seq[:-1]] + [(home[0], home[1], z), (home[0], home[1], 0.0)]
