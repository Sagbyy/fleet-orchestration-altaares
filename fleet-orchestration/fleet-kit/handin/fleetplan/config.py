"""Every tunable of the planner in one place.

- Settings: run-time switches read once from the environment (FLEET_*).
- Flight model: speed margin, observation geometry, altitude layers.
- Realisation search tables: the variants tried when deconflicting routes.

Search parameters per level (ALNS) live in search/params.py.
"""
from __future__ import annotations

import os
from dataclasses import dataclass


def _flag(name: str, default: str = "0") -> bool:
    return os.environ.get(name, default) == "1"


@dataclass(frozen=True)
class Settings:
    """Run-time switches. Defaults are the hand-in configuration."""

    budget_s: float          # FLEET_BUDGET: run-time budget of one scenario
    hard_limit_s: float      # FLEET_HARD_LIMIT: never run past this (the evaluation kills at 60 s)
    use_baseline: bool       # FLEET_NO_BASELINE=1 disables the greedy fallback (benchmarks)
    verify: bool             # FLEET_VERIFY=1 optimises for the optional verify board
    ground_run: bool         # FLEET_GROUND_RUN=1 allows [home, engage] at z = 0 (R2 loophole; off)
    knapsack: bool           # FLEET_KNAPSACK=1 exact hostile selection when effectors are scarce
    weak_route: bool         # FLEET_WEAK_ROUTE=1 adds the tour-consolidation destroy operator
    seed_offset: int         # FLEET_SEED: diversifies offline runs

    @classmethod
    def from_env(cls) -> "Settings":
        env = os.environ
        return cls(
            budget_s=float(env.get("FLEET_BUDGET", "45")),
            hard_limit_s=float(env.get("FLEET_HARD_LIMIT", "55")),
            use_baseline=env.get("FLEET_NO_BASELINE", "") != "1",
            verify=_flag("FLEET_VERIFY"),
            ground_run=_flag("FLEET_GROUND_RUN"),
            knapsack=_flag("FLEET_KNAPSACK"),
            weak_route=_flag("FLEET_WEAK_ROUTE"),
            seed_offset=int(env.get("FLEET_SEED", "0")),
        )


SETTINGS = Settings.from_env()

# ---- flight model ---------------------------------------------------------------------------
V_SAFE = 0.99          # realised speeds as a fraction of the limits
OBS_RHO = 146.5        # horizontal offset of an observation point at z = 30 m (3D range 149.5 < 150)
OBS_RHO_HIGH = 128.0   # max horizontal offset at z = 70 m (3D range 146 < 150)
SCOUT_Z = (30.0, 70.0)
SCOUT_Z_MODEL = 30.0
EFF_Z = ((0.0,) if SETTINGS.ground_run else ()) + (30.0, 40.0, 80.0, 110.0, 150.0)   # cheapest conflict-free layer wins
EFF_Z_MODEL = 40.0     # layer for earliest-arrival estimates (most effectors fly low)
EFF_Z_ENERGY = 40.0    # typical layer for energy estimates
GROUP_DT = 2.0         # arrival spacing between effectors of one engagement
RING_SAMPLES = 48      # candidate observation points sampled around each contact

# ---- realisation search tables --------------------------------------------------------------
SCOUT_DELTAS = (0.0, 1.0, 2.0, 3.0, 4.5, 6.0, 8.0, 10.0, 13.0, 17.0, 22.0, 30.0, 40.0,
                -1.5, -3.0, -6.0, -10.0, -15.0, -25.0)          # take-off shifts (s)
LAND_DELAYS = (0.0, 1.5, 3.0, 4.5, 6.0, 8.0, 11.0, 15.0, 20.0, 30.0)  # landing delays (s)
LAND_SLOW = (1.0, 0.0, 0.5)   # 1 = slow best-range return (lands late), 0 = full-speed return
MIN_TRIES = 60                # variants always tried per tour, whatever the clock says
DTS = (0.0, 3.0, 6.0, 10.0, 15.0, 22.0, 30.0, 40.0, 55.0, 75.0, 100.0, 130.0, 170.0)  # engagement delays (s)
ANGLES = (0.0, 25.0, -25.0, 50.0, -50.0, 80.0, -80.0)   # approach angle offsets (deg)
SPREAD = (0.0, 35.0, -35.0, 70.0, -70.0, 105.0)         # per-member angle spread inside a group (deg)
