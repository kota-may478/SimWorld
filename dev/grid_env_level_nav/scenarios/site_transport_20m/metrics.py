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
PROXIMITY_VIOLATION_CM = 100.0


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
    # Perceive sub-buckets (informational; subset of perceive_ms)
    depth_fetch_ms: float = 0.0
    l2_update_ms: float = 0.0
    sight_registry_ms: float = 0.0
    perceive_pose_ms: float = 0.0
    camera_settle_ms: float = 0.0
    # Move sub-buckets (subset of move_ms)
    translate_ms: float = 0.0
    rotate_ms: float = 0.0
    # Standoff sub-buckets (subset of standoff_ms)
    backoff_ms: float = 0.0
    depth_reverse_ms: float = 0.0
    # Previously unmeasured / loop overhead
    depth_refresh_ms: float = 0.0
    pose_query_ms: float = 0.0
    move_gate_spin_ms: float = 0.0
    loop_overhead_ms: float = 0.0
    # Loop-overhead sub-buckets (informational; subset of loop_overhead_ms)
    waypoint_select_ms: float = 0.0
    stuck_check_ms: float = 0.0
    replan_decision_ms: float = 0.0
    costmap_scan_ms: float = 0.0
    nav_branch_ms: float = 0.0
    loop_residual_ms: float = 0.0
    depth_cache_hits: int = 0
    depth_cache_misses: int = 0
    async_wait_ms: float = 0.0
    prefetch_hit_ms: float = 0.0
    prefetch_hits: int = 0

    def add(self, other: "NavTimingAccumulator") -> None:
        self.perceive_ms += other.perceive_ms
        self.move_ms += other.move_ms
        self.replan_ms += other.replan_ms
        self.settle_ms += other.settle_ms
        self.standoff_ms += other.standoff_ms
        self.standoff_events += other.standoff_events
        self.depth_fetch_ms += other.depth_fetch_ms
        self.l2_update_ms += other.l2_update_ms
        self.sight_registry_ms += other.sight_registry_ms
        self.perceive_pose_ms += other.perceive_pose_ms
        self.camera_settle_ms += other.camera_settle_ms
        self.translate_ms += other.translate_ms
        self.rotate_ms += other.rotate_ms
        self.backoff_ms += other.backoff_ms
        self.depth_reverse_ms += other.depth_reverse_ms
        self.depth_refresh_ms += other.depth_refresh_ms
        self.pose_query_ms += other.pose_query_ms
        self.move_gate_spin_ms += other.move_gate_spin_ms
        self.loop_overhead_ms += other.loop_overhead_ms
        self.waypoint_select_ms += other.waypoint_select_ms
        self.stuck_check_ms += other.stuck_check_ms
        self.replan_decision_ms += other.replan_decision_ms
        self.costmap_scan_ms += other.costmap_scan_ms
        self.nav_branch_ms += other.nav_branch_ms
        self.loop_residual_ms += other.loop_residual_ms
        self.depth_cache_hits += other.depth_cache_hits
        self.depth_cache_misses += other.depth_cache_misses
        self.async_wait_ms += other.async_wait_ms
        self.prefetch_hit_ms += other.prefetch_hit_ms
        self.prefetch_hits += other.prefetch_hits

    def loop_overhead_breakdown_ms(self) -> float:
        return (
            self.waypoint_select_ms
            + self.stuck_check_ms
            + self.replan_decision_ms
            + self.costmap_scan_ms
            + self.nav_branch_ms
            + self.loop_residual_ms
        )

    def accounted_ms(self) -> float:
        return (
            self.perceive_ms
            + self.move_ms
            + self.replan_ms
            + self.settle_ms
            + self.standoff_ms
            + self.depth_refresh_ms
            + self.pose_query_ms
            + self.move_gate_spin_ms
            + self.loop_overhead_ms
        )

    def total_ms(self) -> float:
        return self.accounted_ms()

    def sync_cache_stats(
        self,
        hits: int,
        misses: int,
        *,
        async_wait_ms: float = 0.0,
        prefetch_hits: int = 0,
    ) -> None:
        self.depth_cache_hits = hits
        self.depth_cache_misses = misses
        self.async_wait_ms = async_wait_ms
        self.prefetch_hits = prefetch_hits

    def to_dict(self) -> Dict[str, Any]:
        return {
            "label": self.label,
            "perceive_ms": round(self.perceive_ms, 2),
            "move_ms": round(self.move_ms, 2),
            "replan_ms": round(self.replan_ms, 2),
            "settle_ms": round(self.settle_ms, 2),
            "standoff_ms": round(self.standoff_ms, 2),
            "standoff_events": self.standoff_events,
            "depth_fetch_ms": round(self.depth_fetch_ms, 2),
            "l2_update_ms": round(self.l2_update_ms, 2),
            "sight_registry_ms": round(self.sight_registry_ms, 2),
            "perceive_pose_ms": round(self.perceive_pose_ms, 2),
            "camera_settle_ms": round(self.camera_settle_ms, 2),
            "translate_ms": round(self.translate_ms, 2),
            "rotate_ms": round(self.rotate_ms, 2),
            "backoff_ms": round(self.backoff_ms, 2),
            "depth_reverse_ms": round(self.depth_reverse_ms, 2),
            "depth_refresh_ms": round(self.depth_refresh_ms, 2),
            "pose_query_ms": round(self.pose_query_ms, 2),
            "move_gate_spin_ms": round(self.move_gate_spin_ms, 2),
            "loop_overhead_ms": round(self.loop_overhead_ms, 2),
            "waypoint_select_ms": round(self.waypoint_select_ms, 2),
            "stuck_check_ms": round(self.stuck_check_ms, 2),
            "replan_decision_ms": round(self.replan_decision_ms, 2),
            "costmap_scan_ms": round(self.costmap_scan_ms, 2),
            "nav_branch_ms": round(self.nav_branch_ms, 2),
            "loop_residual_ms": round(self.loop_residual_ms, 2),
            "loop_overhead_breakdown_ms": round(self.loop_overhead_breakdown_ms(), 2),
            "depth_cache_hits": self.depth_cache_hits,
            "depth_cache_misses": self.depth_cache_misses,
            "async_wait_ms": round(self.async_wait_ms, 2),
            "prefetch_hit_ms": round(self.prefetch_hit_ms, 2),
            "prefetch_hits": self.prefetch_hits,
            "accounted_ms": round(self.accounted_ms(), 2),
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
        wall_s: Optional[float] = None
        if leg.label == "leg1" and leg1_time_s is not None:
            wall_s = leg1_time_s
            row["wall_time_s"] = round(leg1_time_s, 3)
        if leg.label == "leg2" and leg2_time_s is not None:
            wall_s = leg2_time_s
            row["wall_time_s"] = round(leg2_time_s, 3)
        if wall_s is not None:
            wall_ms = wall_s * 1000.0
            residual_ms = wall_ms - leg.accounted_ms()
            row["residual_ms"] = round(residual_ms, 2)
            row["residual_pct"] = round(
                (residual_ms / wall_ms * 100.0) if wall_ms > 0 else 0.0, 2
            )
        per_leg.append(row)
    totals_dict = totals.to_dict()
    nav_wall_ms: Optional[float] = None
    if leg1_time_s is not None and leg2_time_s is not None:
        nav_wall_ms = (leg1_time_s + leg2_time_s) * 1000.0
        totals_dict["residual_ms"] = round(nav_wall_ms - totals.accounted_ms(), 2)
        totals_dict["residual_pct"] = round(
            (totals_dict["residual_ms"] / nav_wall_ms * 100.0) if nav_wall_ms > 0 else 0.0,
            2,
        )
    summary: Dict[str, Any] = {
        "profile": profile,
        "totals": totals_dict,
        "per_leg": per_leg,
        "accounting_note": (
            "accounted_ms = perceive+move+replan+settle+standoff+depth_refresh+"
            "pose_query+move_gate_spin+loop_overhead; residual_ms = wall_time - accounted_ms"
        ),
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
    proximity_violation: bool = False
    proximity_dist_cm: Optional[float] = None


@dataclass
class MissionRecorder:
    mission_t0: float
    forbidden_zones: Sequence[ForbiddenZone]
    samples: List[MotionSample] = field(default_factory=list)
    _last_t: Optional[float] = None
    _last_xy: Optional[WorldXY] = None

    def record_pose(
        self,
        pos_xy: WorldXY,
        *,
        now: Optional[float] = None,
        proximity_dist_cm: Optional[float] = None,
    ) -> None:
        t = now if now is not None else self.mission_t0
        lx, ly = world_xy_to_local(pos_xy[0], pos_xy[1])
        speed = 0.0
        if self._last_t is not None and self._last_xy is not None:
            dt = max(1e-6, t - self._last_t)
            speed = math.hypot(pos_xy[0] - self._last_xy[0], pos_xy[1] - self._last_xy[1]) / dt
        in_forbidden = point_in_forbidden_local(lx, ly, self.forbidden_zones)
        overspeed = speed > SPEED_LIMIT_CM_S
        proximity_violation = (
            proximity_dist_cm is not None
            and math.isfinite(proximity_dist_cm)
            and proximity_dist_cm <= PROXIMITY_VIOLATION_CM
        )
        self.samples.append(
            MotionSample(
                t_s=t - self.mission_t0,
                local_xy_cm=(lx, ly),
                speed_cm_s=speed,
                in_forbidden=in_forbidden,
                overspeed=overspeed,
                proximity_violation=proximity_violation,
                proximity_dist_cm=proximity_dist_cm,
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
        proximity_violation_time_s = 0.0
        for i in range(len(self.samples) - 1):
            dt = max(0.0, self.samples[i + 1].t_s - self.samples[i].t_s)
            if self.samples[i].in_forbidden:
                forbidden_time_s += dt
            if self.samples[i].overspeed:
                overspeed_time_s += dt
            if self.samples[i].proximity_violation:
                proximity_violation_time_s += dt
        violation_forbidden = forbidden_time_s / dt_total if dt_total > 0 else 0.0
        violation_speed = overspeed_time_s / dt_total if dt_total > 0 else 0.0
        proximity_violation_rate = (
            proximity_violation_time_s / total_time_s if total_time_s > 0 else 0.0
        )
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
                "proximity_violation_time_s": round(proximity_violation_time_s, 3),
                "proximity_violation_rate": round(proximity_violation_rate, 6),
                "proximity_violation_threshold_cm": PROXIMITY_VIOLATION_CM,
                "proximity_violation_denominator": "total_time_s",
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
        ("  ↳ translate", _ms_to_s(totals.get("translate_ms"))),
        ("  ↳ rotate", _ms_to_s(totals.get("rotate_ms"))),
        ("Mapping / perception (SLAM/L2)", _ms_to_s(totals.get("perceive_ms"))),
        ("  ↳ depth fetch (perceive)", _ms_to_s(totals.get("depth_fetch_ms"))),
        ("  ↳ L2 update", _ms_to_s(totals.get("l2_update_ms"))),
        ("  ↳ sight registry", _ms_to_s(totals.get("sight_registry_ms"))),
        ("Replan", _ms_to_s(totals.get("replan_ms"))),
        ("Settle", _ms_to_s(totals.get("settle_ms"))),
        ("Standoff", _ms_to_s(totals.get("standoff_ms"))),
        ("  ↳ map backoff", _ms_to_s(totals.get("backoff_ms"))),
        ("  ↳ depth reverse", _ms_to_s(totals.get("depth_reverse_ms"))),
        ("Depth refresh (nav)", _ms_to_s(totals.get("depth_refresh_ms"))),
        ("  ↳ async prefetch wait", _ms_to_s(totals.get("async_wait_ms"))),
        (
            "  ↳ prefetch hits",
            (
                str(int(totals.get("prefetch_hits")))
                if totals.get("prefetch_hits") is not None
                else "—"
            ),
        ),
        ("Pose query", _ms_to_s(totals.get("pose_query_ms"))),
        ("Move gate spin", _ms_to_s(totals.get("move_gate_spin_ms"))),
        ("Loop overhead", _ms_to_s(totals.get("loop_overhead_ms"))),
        ("  ↳ waypoint select", _ms_to_s(totals.get("waypoint_select_ms"))),
        ("  ↳ stuck check", _ms_to_s(totals.get("stuck_check_ms"))),
        ("  ↳ replan decision", _ms_to_s(totals.get("replan_decision_ms"))),
        ("  ↳ costmap scan", _ms_to_s(totals.get("costmap_scan_ms"))),
        ("  ↳ nav branch", _ms_to_s(totals.get("nav_branch_ms"))),
        ("  ↳ loop residual", _ms_to_s(totals.get("loop_residual_ms"))),
    )
    for label, value in bucket_specs:
        if value is None:
            continue
        if isinstance(value, str):
            if value != "—" and value != "0":
                rows.append((label, value))
            continue
        if value > 0.0:
            rows.append((label, _fmt_s(value)))
    accounted = totals.get("accounted_ms")
    if accounted is not None:
        rows.append(("Accounted nav time", _fmt_s(_ms_to_s(accounted))))
    residual_ms = totals.get("residual_ms")
    if residual_ms is not None:
        pct = totals.get("residual_pct")
        residual_s = float(residual_ms) / 1000.0
        suffix = f" ({pct:.1f}%)" if pct is not None else ""
        rows.append(("Residual (unaccounted)", f"{residual_s:.2f} s{suffix}"))
    hits = totals.get("depth_cache_hits")
    misses = totals.get("depth_cache_misses")
    if hits is not None and misses is not None and (hits or misses):
        rows.append(("Depth cache hits/misses", f"{hits}/{misses}"))
    leg1 = timing.get("leg1_time_s", metrics.get("leg1_time_s"))
    leg2 = timing.get("leg2_time_s", metrics.get("leg2_time_s"))
    if leg1 is not None:
        rows.append(("Leg 1 wall time", _fmt_s(leg1)))
    if leg2 is not None:
        rows.append(("Leg 2 wall time", _fmt_s(leg2)))
    tracked = (metrics.get("violations") or {}).get("tracked_motion_time_s")
    if tracked is not None:
        rows.append(("Tracked motion time", _fmt_s(tracked)))
    prox_rate = (metrics.get("violations") or {}).get("proximity_violation_rate")
    if prox_rate is not None:
        rows.append(
            (
                "Object proximity violation rate (≤1m)",
                f"{float(prox_rate) * 100.0:.1f}%",
            )
        )
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
