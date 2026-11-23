"""Hungarian algorithm (min-cost assignment) for rectangular matrices, pure Python.

Used for the final effector assignment: rows are engagement slots (one per effector a contact
needs), columns are effectors of that subtype, costs are energy (infeasible pairs = INF)."""
from __future__ import annotations

import math
from typing import List, Optional


INF = math.inf


BIG = 1e9  # stands for an infeasible pair inside the algorithm


def assign(cost: List[List[float]]) -> List[Optional[int]]:
    """Min-cost assignment of every row to a distinct column (rows <= columns).

    Returns the column of each row, or None for a row whose best column is infeasible (INF).
    O(n^2 m), the classic potentials formulation.
    """
    n = len(cost)
    if n == 0:
        return []
    m = len(cost[0])
    if n > m:
        raise ValueError("more rows than columns")
    a = [[BIG if c == INF else c for c in row] for row in cost]
    u = [0.0] * (n + 1)
    v = [0.0] * (m + 1)
    p = [0] * (m + 1)       # p[j] = row matched to column j (1-based), 0 = free
    way = [0] * (m + 1)
    for i in range(1, n + 1):
        p[0] = i
        j0 = 0
        minv = [INF] * (m + 1)
        used = [False] * (m + 1)
        while True:
            used[j0] = True
            i0 = p[j0]
            delta, j1 = INF, 0
            row = a[i0 - 1]
            ui0 = u[i0]
            for j in range(1, m + 1):
                if not used[j]:
                    cur = row[j - 1] - ui0 - v[j]
                    if cur < minv[j]:
                        minv[j] = cur
                        way[j] = j0
                    if minv[j] < delta:
                        delta, j1 = minv[j], j
            for j in range(m + 1):
                if used[j]:
                    u[p[j]] += delta
                    v[j] -= delta
                else:
                    minv[j] -= delta
            j0 = j1
            if p[j0] == 0:
                break
        while True:
            j1 = way[j0]
            p[j0] = p[j1]
            j0 = j1
            if j0 == 0:
                break
    out: List[Optional[int]] = [None] * n
    for j in range(1, m + 1):
        if p[j]:
            out[p[j] - 1] = j - 1
    return [c if c is not None and cost[r][c] != INF else None for r, c in enumerate(out)]
