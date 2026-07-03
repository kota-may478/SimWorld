#!/usr/bin/env python3
"""Shared types for Dynamic NavMesh navigation."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Tuple

WorldXYZ = Tuple[float, float, float]


@dataclass(frozen=True)
class ActorBounds:
    actor_name: str
    cx: float
    cy: float
    cz: float
    half_x: float
    half_y: float
    half_z: float

    @property
    def obstacle_id(self) -> str:
        return f"nav_obs_{self.actor_name}"

    def center_xyz(self) -> WorldXYZ:
        return (self.cx, self.cy, self.cz)

    def half_extents_xyz(self) -> WorldXYZ:
        return (self.half_x, self.half_y, self.half_z)
