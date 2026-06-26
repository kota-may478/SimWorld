"""L2 cell-delta policy for replan triggers (reduces depth-churn replans)."""

from __future__ import annotations


def l2_cell_delta_warrants_replan(
    cells_added: int,
    cells_removed: int,
    *,
    threshold: int,
) -> bool:
    """Replan when peak or net cell delta exceeds threshold.

    Uses max(added, removed) and abs(added - removed) so small balanced
    flicker (+2/-2) does not replan while large clears (+0/-54) still can.
    """
    added = max(0, int(cells_added))
    removed = max(0, int(cells_removed))
    peak = max(added, removed)
    net = abs(added - removed)
    return max(peak, net) >= threshold
