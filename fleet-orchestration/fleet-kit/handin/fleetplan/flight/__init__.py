"""3D realisation: timed waypoints, altitude layers and separation deconfliction."""
from __future__ import annotations

from ..search.solution import Sol
from ..world import World
from .airspace import Airspace
from .effectors import place_effectors
from .scouts import place_scouts


def realise(world: World, sol: Sol, deadline: float):
    """Returns (plan routes {aircraft id: waypoints}, groups {contact id: effector ids})."""
    air = Airspace(world)
    obs_end = place_scouts(world, sol, air, deadline)
    groups = place_effectors(world, air, obs_end, deadline)
    # scouts that only observed hostiles nobody engages are kept: they are valid and cheap to fix later
    return {aid: it.wps for aid, it in air.items.items()}, groups, air
