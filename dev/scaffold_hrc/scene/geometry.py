"""Stage-1 scaffold dimensions. Metres, 1F = ground."""

from __future__ import annotations

from dataclasses import dataclass

from scene.field import STORAGE_XY_M, scaffold_edge_x_m


@dataclass(frozen=True)
class ScaffoldGeom:
    deck_width_m: float = 2.4
    deck_length_m: float = 10.0
    lift_m: float = 1.8
    n_floors: int = 3
    stair_bay_m: float = 1.8
    corridor_m: float = 10.0
    cell_m: float = 0.4

    @property
    def total_length_m(self) -> float:
        return self.deck_length_m - scaffold_edge_x_m()

    def floor_z_m(self, floor: int) -> float:
        if floor < 1 or floor > self.n_floors:
            raise ValueError(f"floor must be in 1..{self.n_floors}, got {floor}")
        return (floor - 1) * self.lift_m

    def deck_xy_bounds(self) -> tuple[float, float, float, float]:
        return (0.0, self.deck_length_m, 0.0, self.deck_width_m)

    def stair_xy_bounds(self) -> tuple[float, float, float, float]:
        """Full Recast stair run (yard → deck), not only the 1.8 m post bay."""
        return (scaffold_edge_x_m(), 0.0, 0.0, self.deck_width_m)

    def storage_xy(self) -> tuple[float, float]:
        """Kei-truck yard; matches UE STAGING after the closer parking pass."""
        return STORAGE_XY_M


STAGE1_GEOM = ScaffoldGeom()
