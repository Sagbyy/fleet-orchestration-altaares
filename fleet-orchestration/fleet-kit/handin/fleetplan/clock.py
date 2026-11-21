"""Time source used by every deadline in the planner.

In normal runs this is the wall clock. With FLEET_DETERMINISTIC=1 time is virtual: it only
advances by a fixed step per ALNS iteration, so a run is reproducible bit for bit (used by the
regression tests that guard refactors).
"""
from __future__ import annotations

import os
import time

VIRTUAL_STEP_S = 0.025   # virtual seconds per ALNS iteration


class Clock:
    def __init__(self, virtual: bool):
        self.virtual = virtual
        self._t = 0.0

    def now(self) -> float:
        return self._t if self.virtual else time.monotonic()

    def tick(self) -> None:
        if self.virtual:
            self._t += VIRTUAL_STEP_S

    def reset(self) -> None:
        self._t = 0.0


CLOCK = Clock(os.environ.get("FLEET_DETERMINISTIC", "") == "1")


def now() -> float:
    return CLOCK.now()
