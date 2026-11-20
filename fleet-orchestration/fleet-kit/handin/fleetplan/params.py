"""Per-level parameters (tuned offline on the public generator; see bench.py)."""
from __future__ import annotations

from .alns import Params

# Fractions of the time budget at which each phase must be done.
SHARES = {"alns": 0.75, "realise": 0.92}

LEVELS = {
    "L1": dict(k_pos=4, k_routes=4, time_price=0.02, regret_w=1.0, noise=0.2, q_frac=0.5, t0_frac=0.03),
    "L2": dict(k_pos=4, k_routes=4, time_price=0.02, regret_w=1.0, noise=0.2, q_frac=0.4, t0_frac=0.03),
    "L3": dict(k_pos=3, k_routes=4, time_price=0.02, regret_w=1.0, noise=0.15, q_frac=0.3, t0_frac=0.02),
    "L4": dict(k_pos=3, k_routes=4, time_price=0.02, regret_w=1.0, noise=0.15, q_frac=0.25, t0_frac=0.02),
}


def level_params(level: str):
    return Params(**LEVELS.get(level, LEVELS["L3"])), dict(SHARES)
