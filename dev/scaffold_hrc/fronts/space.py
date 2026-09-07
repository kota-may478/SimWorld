"""Theta box shared by all front-discovery methods."""

from __future__ import annotations

from dataclasses import dataclass

from constraints.pareto import Theta

VMAX_LO = 0.20
VMAX_HI = 1.00
DMIN_LO = 0.35
DMIN_HI = 1.60
REF_THETA = Theta(vmax_mps=VMAX_HI, dmin_m=DMIN_LO)
HALLUCINATED_THETA = Theta(vmax_mps=3.0, dmin_m=8.0)


@dataclass(frozen=True)
class ThetaBox:
    vmax_lo: float = VMAX_LO
    vmax_hi: float = VMAX_HI
    dmin_lo: float = DMIN_LO
    dmin_hi: float = DMIN_HI

    def clip(self, theta: Theta) -> Theta:
        return Theta(
            vmax_mps=min(self.vmax_hi, max(self.vmax_lo, theta.vmax_mps)),
            dmin_m=min(self.dmin_hi, max(self.dmin_lo, theta.dmin_m)),
        )

    def to_unit(self, theta: Theta) -> tuple[float, float]:
        u = (theta.vmax_mps - self.vmax_lo) / (self.vmax_hi - self.vmax_lo)
        v = (theta.dmin_m - self.dmin_lo) / (self.dmin_hi - self.dmin_lo)
        return (u, v)

    def from_unit(self, u: float, v: float) -> Theta:
        return self.clip(
            Theta(
                vmax_mps=self.vmax_lo + u * (self.vmax_hi - self.vmax_lo),
                dmin_m=self.dmin_lo + v * (self.dmin_hi - self.dmin_lo),
            )
        )

    def aggressive(self) -> Theta:
        return Theta(vmax_mps=self.vmax_hi, dmin_m=self.dmin_lo)

    def conservative(self) -> Theta:
        return Theta(vmax_mps=self.vmax_lo, dmin_m=self.dmin_hi)

    def midpoint(self) -> Theta:
        return Theta(
            vmax_mps=0.5 * (self.vmax_lo + self.vmax_hi),
            dmin_m=0.5 * (self.dmin_lo + self.dmin_hi),
        )
