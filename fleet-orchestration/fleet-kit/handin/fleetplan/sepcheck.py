"""Private copy of the official separation test (R8), so the planner does not depend on
fleetkit internals that could change. The official validator still has the last word."""
from __future__ import annotations

import math
from typing import Optional, Sequence

EPS = 1e-6

def position_at(piece: Piece, t: float) -> Point:
    """Returns the position on a piece at time t."""
    t0, t1, p0, p1 = piece
    if t1 - t0 <= 1e-12:
        return p0
    f = (t - t0) / (t1 - t0)
    return (p0[0] + f * (p1[0] - p0[0]), p0[1] + f * (p1[1] - p0[1]), p0[2] + f * (p1[2] - p0[2]))


def overlapping_pieces(pieces_a: Sequence[Piece],
                       pieces_b: Sequence[Piece]) -> Iterator[Tuple[Piece, Piece, float, float]]:
    """Yields every pair of pieces that overlap in time, with the overlap interval."""
    i = j = 0
    while i < len(pieces_a) and j < len(pieces_b):
        a, b = pieces_a[i], pieces_b[j]
        lo, hi = max(a[0], b[0]), min(a[1], b[1])
        if hi - lo > 1e-12:
            yield a, b, lo, hi
        if a[1] < b[1]:
            i += 1
        else:
            j += 1


def _below_zero_interval(a: float, b: float, c: float, length: float,
                         closed: bool = False) -> Optional[Tuple[float, float]]:
    """Returns the part of [0, length] where a t^2 + b t + c is below zero (or at most zero)."""
    if a < 1e-12:
        inside = c <= 0.0 if closed else c < 0.0
        return (0.0, length) if inside else None
    disc = b * b - 4.0 * a * c
    if disc < 0.0 or (disc == 0.0 and not closed):
        return None
    root = math.sqrt(disc)
    lo = max((-b - root) / (2.0 * a), 0.0)
    hi = min((-b + root) / (2.0 * a), length)
    if hi - lo < (0.0 if closed else 1e-9):
        return None
    return (lo, hi)


def _relative_motion(a: Piece, b: Piece, lo: float):
    """Returns the relative position at `lo` and the relative velocity of b with respect to a."""
    pa, pb = position_at(a, lo), position_at(b, lo)
    va = _velocity(a)
    vb = _velocity(b)
    r0 = (pb[0] - pa[0], pb[1] - pa[1], pb[2] - pa[2])
    w = (vb[0] - va[0], vb[1] - va[1], vb[2] - va[2])
    return r0, w


def _velocity(piece: Piece) -> Point:
    t0, t1, p0, p1 = piece
    if t1 - t0 <= 1e-12:
        return (0.0, 0.0, 0.0)
    k = 1.0 / (t1 - t0)
    return ((p1[0] - p0[0]) * k, (p1[1] - p0[1]) * k, (p1[2] - p0[2]) * k)


def close_intervals(a: Piece, b: Piece, lo: float, hi: float,
                    d_min: float) -> Optional[Tuple[float, float]]:
    """Returns the sub-interval (absolute times) where two pieces are closer than `d_min`."""
    r0, w = _relative_motion(a, b, lo)
    qa = w[0] ** 2 + w[1] ** 2 + w[2] ** 2
    qb = 2.0 * (r0[0] * w[0] + r0[1] * w[1] + r0[2] * w[2])
    qc = r0[0] ** 2 + r0[1] ** 2 + r0[2] ** 2 - d_min * d_min
    found = _below_zero_interval(qa, qb, qc, hi - lo)
    if found is None:
        return None
    return (lo + found[0], lo + found[1])


def min_distance(a: Piece, b: Piece, lo: float, hi: float) -> float:
    """Returns the minimum 3D distance between two linear pieces over [lo, hi]."""
    r0, w = _relative_motion(a, b, lo)
    qa = w[0] ** 2 + w[1] ** 2 + w[2] ** 2
    qb = 2.0 * (r0[0] * w[0] + r0[1] * w[1] + r0[2] * w[2])
    length = hi - lo
    if qa < 1e-12:
        t = 0.0
    else:
        t = min(max(-qb / (2.0 * qa), 0.0), length)
    return math.sqrt(sum((r0[i] + t * w[i]) ** 2 for i in range(3)))


def within_radius_interval(piece: Piece, lo: float, hi: float, center: Sequence[float],
                           radius: float) -> Optional[Tuple[float, float]]:
    """Returns the sub-interval (absolute times) where a piece is within `radius` of a point."""
    p = position_at(piece, lo)
    v = _velocity(piece)
    f = (p[0] - center[0], p[1] - center[1], p[2] - center[2])
    qa = v[0] ** 2 + v[1] ** 2 + v[2] ** 2
    qb = 2.0 * (f[0] * v[0] + f[1] * v[1] + f[2] * v[2])
    qc = f[0] ** 2 + f[1] ** 2 + f[2] ** 2 - radius * radius
    found = _below_zero_interval(qa, qb, qc, hi - lo, closed=True)
    if found is None:
        return None
    return (lo + found[0], lo + found[1])


def route_pieces(route: Route) -> List[Piece]:
    """Returns the airborne motion of a route as linear pieces, from first departure to last arrival."""
    w = route.waypoints
    pieces: List[Piece] = []
    for k in range(len(w) - 1):
        pieces.append((w[k].depart, w[k + 1].t, w[k].pos, w[k + 1].pos))
        if w[k + 1].dwell_s > 0:
            pieces.append((w[k + 1].t, w[k + 1].depart, w[k + 1].pos, w[k + 1].pos))
    return pieces


def separation_item(aid: str, route: Route, aircraft: Aircraft):
    """Returns what the separation check needs from one route: pieces, time span, engage target."""
    pieces = route_pieces(route)
    if not pieces:
        return None
    last = route.waypoints[-1]
    target = last.target if (last.action == "engage" and aircraft.type == "effector") else None
    return (aid, pieces, pieces[0][0], pieces[-1][1], target)


def separation_violation(rules, contacts: Dict[str, Contact], item_a, item_b) -> Optional[Violation]:
    """Checks the separation rule (R8) for one pair of routes; returns the violation, if any."""
    d_sep = rules.d_sep_m - EPS
    aid_a, pa, s_a, e_a, tgt_a = item_a
    aid_b, pb, s_b, e_b, tgt_b = item_b
    if min(e_a, e_b) - max(s_a, s_b) <= 1e-12:
        return None
    exempt_center = None
    if tgt_a is not None and tgt_a == tgt_b and tgt_a in contacts:
        exempt_center = contacts[tgt_a].pos
    hit = None
    for a, b, lo, hi in overlapping_pieces(pa, pb):
        bad = close_intervals(a, b, lo, hi, d_sep)
        if bad is None:
            continue
        if exempt_center is not None:
            ea = within_radius_interval(a, lo, hi, exempt_center, rules.exempt_radius_m)
            eb = within_radius_interval(b, lo, hi, exempt_center, rules.exempt_radius_m)
            if ea is not None and eb is not None:
                e1, e2 = max(ea[0], eb[0]), min(ea[1], eb[1])
                if e1 <= e2:
                    parts = []
                    if e1 - bad[0] > 1e-6:
                        parts.append((bad[0], min(bad[1], e1)))
                    if bad[1] - e2 > 1e-6:
                        parts.append((max(bad[0], e2), bad[1]))
                    bad = parts[0] if parts else None
        if bad is not None and bad[1] - bad[0] > 1e-6:
            hit = (bad[0], min_distance(a, b, lo, hi))
            break
    if hit is None:
        return None
    return ("R8", f"{aid_a} and {aid_b} come within {hit[1]:.1f} m of each other (limit {rules.d_sep_m:g} m) around t = {hit[0]:.1f} s")
