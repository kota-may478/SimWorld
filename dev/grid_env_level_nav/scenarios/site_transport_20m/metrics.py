#!/usr/bin/env python3
"""Mission metrics: success rate, total time, violation rates."""

from __future__ import annotations

import json
import math
from dataclasses import dataclass, field
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, List, Mapping, Optional, Sequence, Tuple

from level_coords import world_xy_to_local
from placement import SiteTransportRegistry
from zones import ForbiddenZone, point_in_forbidden_local

WorldXY = Tuple[float, float]
SPEED_LIMIT_KMH = 5.0
SPEED_LIMIT_CM_S = SPEED_LIMIT_KMH * 100_000.0 / 3600.0  # ≈138.89 cm/s


@dataclass
class NavTimingAccumulator:
    """Per-navigation-loop timing buckets (milliseconds)."""

    perceive_ms: float = 0.0
    move_ms: float = 0.0
    replan_ms: float = 0.0
    settle_ms: float = 0.0
    standoff_ms: float = 0.0
    standoff_events: int = 0
    label: str = ""

    def add(self, other: "NavTimingAccumulator") -> None:
        self.perceive_ms += other.perceive_ms
        self.move_ms += other.move_ms
        self.replan_ms += other.replan_ms
        self.settle_ms += other.settle_ms
        self.standoff_ms += other.standoff_ms
        self.standoff_events += other.standoff_events

    def total_ms(self) -> float:
        return self.perceive_ms + self.move_ms + self.replan_ms + self.settle_ms + self.standoff_ms

    def to_dict(self) -> Dict[str, Any]:
        return {
            "label": self.label,
            "perceive_ms": round(self.perceive_ms, 2),
            "move_ms": round(self.move_ms, 2),
            "replan_ms": round(self.replan_ms, 2),
            "settle_ms": round(self.settle_ms, 2),
            "standoff_ms": round(self.standoff_ms, 2),
            "standoff_events": self.standoff_events,
            "total_ms": round(self.total_ms(), 2),
        }


def build_timing_summary(
    *,
    legs: Sequence[NavTimingAccumulator],
    leg1_time_s: Optional[float] = None,
    leg2_time_s: Optional[float] = None,
    profile: Optional[str] = None,
) -> Dict[str, Any]:
    totals = NavTimingAccumulator(label="mission")
    per_leg: List[Dict[str, Any]] = []
    for leg in legs:
        totals.add(leg)
        row = leg.to_dict()
        if leg.label == "leg1" and leg1_time_s is not None:
            row["wall_time_s"] = round(leg1_time_s, 3)
        if leg.label == "leg2" and leg2_time_s is not None:
            row["wall_time_s"] = round(leg2_time_s, 3)
        per_leg.append(row)
    summary: Dict[str, Any] = {
        "profile": profile,
        "totals": totals.to_dict(),
        "per_leg": per_leg,
    }
    if leg1_time_s is not None:
        summary["leg1_time_s"] = round(leg1_time_s, 3)
    if leg2_time_s is not None:
        summary["leg2_time_s"] = round(leg2_time_s, 3)
    if leg1_time_s is not None and leg2_time_s is not None:
        summary["nav_wall_time_s"] = round(leg1_time_s + leg2_time_s, 3)
    return summary


def save_timing_json(
    timing: Mapping[str, Any],
    output_dir: Path,
    *,
    run_label: str | None = None,
    trial_index: int | None = None,
) -> Path:
    output_dir.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    if run_label is not None and trial_index is not None:
        path = output_dir / f"timing_{run_label}_{trial_index}.json"
    elif run_label is not None:
        path = output_dir / f"timing_{run_label}_{stamp}.json"
    else:
        path = output_dir / f"timing_{stamp}.json"
    path.write_text(json.dumps(dict(timing), indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    latest = output_dir / "latest_timing_json.json"
    latest.write_text(path.read_text(encoding="utf-8"), encoding="utf-8")
    return path


@dataclass
class MotionSample:
    t_s: float
    local_xy_cm: Tuple[float, float]
    speed_cm_s: float
    in_forbidden: bool
    overspeed: bool


@dataclass
class MissionRecorder:
    mission_t0: float
    forbidden_zones: Sequence[ForbiddenZone]
    samples: List[MotionSample] = field(default_factory=list)
    _last_t: Optional[float] = None
    _last_xy: Optional[WorldXY] = None

    def record_pose(self, pos_xy: WorldXY, *, now: Optional[float] = None) -> None:
        t = now if now is not None else self.mission_t0
        lx, ly = world_xy_to_local(pos_xy[0], pos_xy[1])
        speed = 0.0
        if self._last_t is not None and self._last_xy is not None:
            dt = max(1e-6, t - self._last_t)
            speed = math.hypot(pos_xy[0] - self._last_xy[0], pos_xy[1] - self._last_xy[1]) / dt
        in_forbidden = point_in_forbidden_local(lx, ly, self.forbidden_zones)
        overspeed = speed > SPEED_LIMIT_CM_S
        self.samples.append(
            MotionSample(
                t_s=t - self.mission_t0,
                local_xy_cm=(lx, ly),
                speed_cm_s=speed,
                in_forbidden=in_forbidden,
                overspeed=overspeed,
            )
        )
        self._last_t = t
        self._last_xy = pos_xy

    def finalize(
        self,
        *,
        success: bool,
        mission_end_t: float,
        layout_id: str,
        leg1_time_s: Optional[float] = None,
        leg2_time_s: Optional[float] = None,
        timing_summary: Optional[Dict[str, Any]] = None,
        profile: Optional[str] = None,
    ) -> Dict[str, Any]:
        total_time_s = max(0.0, mission_end_t - self.mission_t0)
        if len(self.samples) < 2:
            dt_total = total_time_s
        else:
            dt_total = sum(
                max(0.0, self.samples[i + 1].t_s - self.samples[i].t_s)
                for i in range(len(self.samples) - 1)
            )
            if dt_total <= 0.0:
                dt_total = total_time_s
        forbidden_time_s = 0.0
        overspeed_time_s = 0.0
        for i in range(len(self.samples) - 1):
            dt = max(0.0, self.samples[i + 1].t_s - self.samples[i].t_s)
            if self.samples[i].in_forbidden:
                forbidden_time_s += dt
            if self.samples[i].overspeed:
                overspeed_time_s += dt
        violation_forbidden = forbidden_time_s / dt_total if dt_total > 0 else 0.0
        violation_speed = overspeed_time_s / dt_total if dt_total > 0 else 0.0
        success_rate = 1.0 if success else 0.0
        trials = 1
        result: Dict[str, Any] = {
            "layout_id": layout_id,
            "success": success,
            "success_rate": success_rate,
            "success_trials": f"{int(success)}/{trials}",
            "total_time_s": round(total_time_s, 3),
            "rules": {
                "speed_limit_kmh": SPEED_LIMIT_KMH,
                "speed_limit_cm_s": round(SPEED_LIMIT_CM_S, 4),
                "forbidden_zones": [
                    {"zone_id": z.zone_id, "rect_local_cm": list(z.rect_local_cm), "note": z.note}
                    for z in self.forbidden_zones
                ],
            },
            "violations": {
                "forbidden_zone_time_s": round(forbidden_time_s, 3),
                "forbidden_zone_rate": round(violation_forbidden, 6),
                "overspeed_time_s": round(overspeed_time_s, 3),
                "overspeed_rate": round(violation_speed, 6),
                "tracked_motion_time_s": round(dt_total, 3),
                "sample_count": len(self.samples),
            },
            "trajectory_local_cm": [list(s.local_xy_cm) for s in self.samples],
        }
        if profile is not None:
            result["profile"] = profile
        if leg1_time_s is not None:
            result["leg1_time_s"] = round(leg1_time_s, 3)
        if leg2_time_s is not None:
            result["leg2_time_s"] = round(leg2_time_s, 3)
        if timing_summary is not None:
            result["timing_summary"] = timing_summary
        return result


def timing_breakdown_rows(metrics: Mapping[str, Any]) -> List[Tuple[str, str]]:
    """Build (label, value) rows for the metrics-summary timing table."""
    timing = metrics.get("timing_summary") or {}
    totals = timing.get("totals") or {}

    def _ms_to_s(ms: Optional[float]) -> Optional[float]:
        if ms is None:
            return None
        return float(ms) / 1000.0

    def _fmt_s(value: Optional[float]) -> str:
        if value is None:
            return "—"
        return f"{value:.2f} s"

    rows: List[Tuple[str, str]] = [
        ("Total mission time", _fmt_s(metrics.get("total_time_s"))),
    ]
    nav_wall = timing.get("nav_wall_time_s")
    if nav_wall is not None:
        rows.append(("Nav wall time (leg1+leg2)", _fmt_s(nav_wall)))
    bucket_specs = (
        ("Movement", _ms_to_s(totals.get("move_ms"))),
        ("Mapping / perception (SLAM/L2)", _ms_to_s(totals.get("perceive_ms"))),
        ("Replan", _ms_to_s(totals.get("replan_ms"))),
        ("Settle", _ms_to_s(totals.get("settle_ms"))),
        ("Standoff", _ms_to_s(totals.get("standoff_ms"))),
    )
    for label, seconds in bucket_specs:
        if seconds is not None and seconds > 0.0:
            rows.append((label, _fmt_s(seconds)))
    leg1 = timing.get("leg1_time_s", metrics.get("leg1_time_s"))
    leg2 = timing.get("leg2_time_s", metrics.get("leg2_time_s"))
    if leg1 is not None:
        rows.append(("Leg 1 wall time", _fmt_s(leg1)))
    if leg2 is not None:
        rows.append(("Leg 2 wall time", _fmt_s(leg2)))
    tracked = (metrics.get("violations") or {}).get("tracked_motion_time_s")
    if tracked is not None:
        rows.append(("Tracked motion time", _fmt_s(tracked)))
    return rows


def save_metrics_json(
    metrics: Dict[str, Any],
    output_dir: Path,
    *,
    run_label: str | None = None,
    trial_index: int | None = None,
) -> Path:
    output_dir.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    if run_label is not None and trial_index is not None:
        suffix = f"{run_label}_{trial_index}"
        path = output_dir / f"metricsSummary_{suffix}.json"
    else:
        path = output_dir / f"site_transport_metrics_{stamp}.json"
    path.write_text(json.dumps(metrics, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    latest = output_dir / "latest_metrics_json.json"
    latest.write_text(path.read_text(encoding="utf-8"), encoding="utf-8")
    return path
