"""Shared Nav2 stack data types."""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class PerceptionOutcome:
    """Result of one perception_server cycle (L2 + optional registry)."""

    detections: list
    cells_added: int = 0
    cells_removed: int = 0
    l2_applied: bool = False

    @property
    def l2_changed(self) -> bool:
        return self.cells_added > 0 or self.cells_removed > 0
