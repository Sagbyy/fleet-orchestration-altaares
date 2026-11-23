"""Search parameters: the Params object and its per-level values."""
from __future__ import annotations




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



# Fractions of the time budget at which each phase must be done.
SHARES = {"alns": 0.85, "realise": 0.95}


LEVELS = {
    "L1": dict(k_pos=4, k_routes=4, time_price=0.02, regret_w=1.0, noise=0.2, q_frac=0.5, t0_frac=0.03),
    "L2": dict(k_pos=4, k_routes=4, time_price=0.02, regret_w=1.0, noise=0.2, q_frac=0.4, t0_frac=0.03),
    "L3": dict(k_pos=3, k_routes=4, time_price=0.02, regret_w=1.0, noise=0.15, q_frac=0.3, t0_frac=0.02),
    "L4": dict(k_pos=3, k_routes=4, time_price=0.02, regret_w=1.0, noise=0.15, q_frac=0.25, t0_frac=0.02),
}


def level_params(level: str):
    return Params(**LEVELS.get(level, LEVELS["L3"])), dict(SHARES)
