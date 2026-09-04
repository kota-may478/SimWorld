"""Theta box shared by all front-discovery methods."""

from __future__ import annotations

from dataclasses import dataclass

from constraints.pareto import Theta

DMIN_LO = 0.35
DMIN_HI = 1.60
VMAX_LO = 0.20
VMAX_HI = 1.00
REF_THETA = Theta(dmin_m=DMIN_LO, vmax_mps=VMAX_HI)


@dataclass(frozen=True)
class ThetaBox:
    dmin_lo: float = DMIN_LO
    dmin_hi: float = DMIN_HI
    vmax_lo: float = VMAX_LO
    vmax_hi: float = VMAX_HI

    def clip(self, theta: Theta) -> Theta:
        return Theta(
            dmin_m=min(self.dmin_hi, max(self.dmin_lo, theta.dmin_m)),
            vmax_mps=min(self.vmax_hi, max(self.vmax_lo, theta.vmax_mps)),
        )

    def to_unit(self, theta: Theta) -> tuple[float, float]:
        u = (theta.dmin_m - self.dmin_lo) / (self.dmin_hi - self.dmin_lo)
        v = (theta.vmax_mps - self.vmax_lo) / (self.vmax_hi - self.vmax_lo)
        return (u, v)

    def from_unit(self, u: float, v: float) -> Theta:
        return self.clip(
            Theta(
                dmin_m=self.dmin_lo + u * (self.dmin_hi - self.dmin_lo),
                vmax_mps=self.vmax_lo + v * (self.vmax_hi - self.vmax_lo),
            )
        )
