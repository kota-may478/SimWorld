#!/usr/bin/env python3
"""Verify handoff / delivery geometry without UE."""
from __future__ import annotations

import math
import sys
from pathlib import Path

HUMAN_APPROACH_STANDOFF_CM = 100.0


def xy_standoff_from_target(
    target_xy: tuple[float, float],
    from_xy: tuple[float, float],
    standoff_cm: float,
) -> tuple[float, float]:
    dx = from_xy[0] - target_xy[0]
    dy = from_xy[1] - target_xy[1]
    dist = math.hypot(dx, dy)
    if dist < 1e-3:
        return (target_xy[0] - standoff_cm, target_xy[1])
    scale = standoff_cm / dist
    return (target_xy[0] + dx * scale, target_xy[1] + dy * scale)


def get_human_approach_xy(
    human_xy: tuple[float, float],
    approach_from_xy: tuple[float, float],
    standoff_cm: float = HUMAN_APPROACH_STANDOFF_CM,
) -> tuple[float, float]:
    return xy_standoff_from_target(human_xy, approach_from_xy, standoff_cm)


def test_standoff_on_line() -> None:
    human = (1000.0, 0.0)
    robot = (3000.0, 0.0)
    delivery = get_human_approach_xy(human, robot, HUMAN_APPROACH_STANDOFF_CM)
    dist = math.hypot(delivery[0] - human[0], delivery[1] - human[1])
    assert abs(dist - HUMAN_APPROACH_STANDOFF_CM) < 0.01, dist
    assert delivery[0] == human[0] + HUMAN_APPROACH_STANDOFF_CM
    assert delivery[1] == human[1]
    assert delivery != robot


def test_standoff_not_at_robot_center() -> None:
    human = (1426.0, -1711.0)
    robot = (2500.0, -1200.0)
    delivery = xy_standoff_from_target(human, robot, 100.0)
    dist_h_d = math.hypot(delivery[0] - human[0], delivery[1] - human[1])
    dist_r_d = math.hypot(delivery[0] - robot[0], delivery[1] - robot[1])
    assert abs(dist_h_d - 100.0) < 0.01
    assert dist_r_d > 50.0, "delivery should not coincide with robot when robot is far"


def test_colinear() -> None:
    human = (0.0, 0.0)
    robot = (10.0, 10.0)
    delivery = xy_standoff_from_target(human, robot, 100.0)
    cross = (delivery[0] - human[0]) * (robot[1] - human[1]) - (
        delivery[1] - human[1]
    ) * (robot[0] - human[0])
    assert abs(cross) < 1e-6


def main() -> None:
    test_standoff_on_line()
    test_standoff_not_at_robot_center()
    test_colinear()
    print("OK handoff geometry checks")


if __name__ == "__main__":
    main()
