"""2D obstacle geometry: blocking tests, wrap-around detours and obstacle-aware paths.

Obstacles are vertical circles (zones below their ceiling, protected areas around friendly and
civilian contacts). A leg that would cross one is wrapped around it along a circumscribed polygon,
so every produced segment stays strictly outside the true circle."""
from __future__ import annotations

import math
from typing import List, Optional, Sequence, Tuple


TWO_PI = 2.0 * math.pi


WRAP_MARGIN = 12.0   # detours follow a circle of radius r + WRAP_MARGIN


CHECK_MARGIN = 6.0   # a segment closer than r + CHECK_MARGIN to the centre is blocked


MAX_ARC_STEP = math.radians(30.0)


XY = Tuple[float, float]


class Circle:
    """A vertical obstacle cylinder, seen from above."""

    __slots__ = ("cx", "cy", "r", "top", "wrap", "chk", "chk2", "label")

    def __init__(self, cx: float, cy: float, r: float, top: float, label: str = ""):
        self.cx, self.cy, self.r, self.top, self.label = cx, cy, r, top, label
        self.wrap = r + WRAP_MARGIN
        self.chk = r + CHECK_MARGIN
        self.chk2 = self.chk * self.chk


def inside(px: float, py: float, c: Circle, pad: float = 0.0) -> bool:
    """Tells whether a point is within the wrap radius (plus pad) of an obstacle."""
    dx, dy = px - c.cx, py - c.cy
    rr = c.wrap + pad
    return dx * dx + dy * dy < rr * rr


def free_point(px: float, py: float, circles: Sequence[Circle], pad: float = 2.0) -> bool:
    """Tells whether a point is clear of every obstacle (with margin)."""
    for c in circles:
        if inside(px, py, c, pad):
            return False
    return True


def blocked(px: float, py: float, qx: float, qy: float, c: Circle) -> bool:
    """Tells whether the segment p-q passes closer than the check radius to the obstacle centre."""
    chk = c.chk
    if (px < c.cx - chk and qx < c.cx - chk) or (px > c.cx + chk and qx > c.cx + chk) \
            or (py < c.cy - chk and qy < c.cy - chk) or (py > c.cy + chk and qy > c.cy + chk):
        return False
    dx, dy = qx - px, qy - py
    fx, fy = px - c.cx, py - c.cy
    a = dx * dx + dy * dy
    if a < 1e-9:
        return fx * fx + fy * fy < c.chk2
    t = -(fx * dx + fy * dy) / a
    if t < 0.0:
        t = 0.0
    elif t > 1.0:
        t = 1.0
    ex, ey = fx + t * dx, fy + t * dy
    return ex * ex + ey * ey < c.chk2


def _wrap_options(px, py, qx, qy, c: Circle):
    """Returns the two ways around an obstacle: (length, sign, start angle, arc)."""
    R = c.wrap
    dpx, dpy = px - c.cx, py - c.cy
    dqx, dqy = qx - c.cx, qy - c.cy
    dp = math.hypot(dpx, dpy)
    dq = math.hypot(dqx, dqy)
    if dp <= R + 1e-6 or dq <= R + 1e-6:
        return None
    thp, thq = math.atan2(dpy, dpx), math.atan2(dqy, dqx)
    ap, aq = math.acos(R / dp), math.acos(R / dq)
    tp, tq = math.sqrt(dp * dp - R * R), math.sqrt(dq * dq - R * R)
    out = []
    for sgn in (1.0, -1.0):
        start = thp + sgn * ap
        end = thq - sgn * aq
        arc = ((end - start) * sgn) % TWO_PI
        out.append((tp + tq + R * arc, sgn, start, arc))
    return out


def model_leg(px: float, py: float, qx: float, qy: float, circles: Sequence[Circle]) -> float:
    """Approximate obstacle-aware length: straight length plus the detour of each blocking obstacle."""
    d = math.hypot(qx - px, qy - py)
    extra = 0.0
    if px < qx:
        xmin, xmax = px, qx
    else:
        xmin, xmax = qx, px
    if py < qy:
        ymin, ymax = py, qy
    else:
        ymin, ymax = qy, py
    for c in circles:
        chk = c.chk
        if c.cx + chk < xmin or c.cx - chk > xmax or c.cy + chk < ymin or c.cy - chk > ymax:
            continue
        if blocked(px, py, qx, qy, c):
            opts = _wrap_options(px, py, qx, qy, c)
            if opts is None:
                extra += 2.0 * c.wrap  # endpoint too close: pessimistic penalty
            else:
                extra += max(0.0, min(o[0] for o in opts) - d)
    return d + extra


def _arc_vertices(c: Circle, sgn: float, start: float, arc: float) -> List[XY]:
    n = max(1, int(math.ceil(arc / MAX_ARC_STEP - 1e-9)))
    step = arc / n
    rv = c.wrap / math.cos(step / 2.0)
    return [(c.cx + rv * math.cos(start + sgn * (k + 0.5) * step),
             c.cy + rv * math.sin(start + sgn * (k + 0.5) * step)) for k in range(n)]


class _Budget:
    __slots__ = ("left",)

    def __init__(self, n: int):
        self.left = n


def path2d(p: XY, q: XY, circles: Sequence[Circle], depth: int = 0,
           budget: Optional[_Budget] = None) -> Optional[Tuple[float, List[XY]]]:
    """Shortest-ish obstacle-free polyline from p to q.

    Returns (length, points after p up to and including q), or None if no path was found.
    """
    if budget is None:
        budget = _Budget(400)
    budget.left -= 1
    if budget.left < 0:
        return None
    px, py = p
    qx, qy = q
    hits = [c for c in circles if blocked(px, py, qx, qy, c)]
    if not hits:
        return math.hypot(qx - px, qy - py), [q]
    if depth >= 6:
        return None
    dx, dy = qx - px, qy - py
    a = dx * dx + dy * dy or 1e-9
    c = min(hits, key=lambda cc: ((cc.cx - px) * dx + (cc.cy - py) * dy) / a)
    opts = _wrap_options(px, py, qx, qy, c)
    if opts is None:
        return None
    best = None
    for _, sgn, start, arc in sorted(opts):
        verts = _arc_vertices(c, sgn, start, arc)
        if any(not free_point(vx, vy, [cc for cc in circles if cc is not c], 0.0) for vx, vy in verts):
            continue
        pts = [p] + verts + [q]
        total = 0.0
        out: List[XY] = []
        ok = True
        for i in range(len(pts) - 1):
            sub = path2d(pts[i], pts[i + 1], circles, depth + 1, budget)
            if sub is None:
                ok = False
                break
            total += sub[0]
            out.extend(sub[1])
            if best is not None and total >= best[0]:
                ok = False
                break
        if ok and (best is None or total < best[0]):
            best = (total, out)
    return best


def polyline_length(p: XY, pts: Sequence[XY]) -> float:
    total, cur = 0.0, p
    for q in pts:
        total += math.hypot(q[0] - cur[0], q[1] - cur[1])
        cur = q
    return total
