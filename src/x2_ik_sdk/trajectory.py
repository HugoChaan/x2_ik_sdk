from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable

import numpy as np


@dataclass(frozen=True)
class ArmWaypoint:
    arm_pos: list[float]
    duration: float


def interpolate_arm_pos(
    start: Iterable[float],
    goal: Iterable[float],
    *,
    duration: float = 2.0,
    rate_hz: float = 50.0,
    max_delta_per_step: float = 0.03,
) -> list[ArmWaypoint]:
    start_arr = np.asarray(list(start), dtype=float)
    goal_arr = np.asarray(list(goal), dtype=float)
    if start_arr.shape != (14,) or goal_arr.shape != (14,):
        raise ValueError("start and goal must be length-14 arm_pos arrays")

    delta = goal_arr - start_arr
    min_steps_by_time = max(2, int(round(duration * rate_hz)))
    min_steps_by_delta = max(2, int(np.ceil(float(np.max(np.abs(delta))) / max_delta_per_step)) + 1)
    steps = max(min_steps_by_time, min_steps_by_delta)
    dt = duration / max(1, steps - 1)

    waypoints = []
    for i in range(steps):
        s = i / max(1, steps - 1)
        smooth = s * s * (3.0 - 2.0 * s)
        q = start_arr + delta * smooth
        waypoints.append(ArmWaypoint(arm_pos=q.tolist(), duration=dt))
    return waypoints
