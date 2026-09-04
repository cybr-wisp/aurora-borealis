from __future__ import annotations

from collections.abc import Iterable
import numpy as np

from .target import TargetState


def _times(duration_sec: float, dt_sec: float) -> np.ndarray:
    if duration_sec <= 0.0 or dt_sec <= 0.0:
        raise ValueError("duration_sec and dt_sec must be positive")
    steps = int(round(duration_sec / dt_sec))
    return np.arange(steps + 1, dtype=float) * dt_sec


def constant_velocity(
    *, target_id: int, position0_m: Iterable[float], velocity_mps: Iterable[float],
    duration_sec: float, dt_sec: float,
) -> list[TargetState]:
    p0 = np.asarray(tuple(position0_m), dtype=float)
    v = np.asarray(tuple(velocity_mps), dtype=float)
    if p0.shape != (3,) or v.shape != (3,):
        raise ValueError("position0_m and velocity_mps must be length 3")
    return [TargetState(target_id, float(t), p0 + v * t, v.copy()) for t in _times(duration_sec, dt_sec)]


def coordinated_turn(
    *, target_id: int, position0_m: Iterable[float], speed_mps: float,
    heading0_rad: float, turn_rate_radps: float, vertical_speed_mps: float,
    duration_sec: float, dt_sec: float,
) -> list[TargetState]:
    p0 = np.asarray(tuple(position0_m), dtype=float)
    if p0.shape != (3,):
        raise ValueError("position0_m must be length 3")
    if abs(turn_rate_radps) < 1e-12:
        v = (speed_mps * np.cos(heading0_rad), speed_mps * np.sin(heading0_rad), vertical_speed_mps)
        return constant_velocity(target_id=target_id, position0_m=p0, velocity_mps=v, duration_sec=duration_sec, dt_sec=dt_sec)

    states: list[TargetState] = []
    w = float(turn_rate_radps)
    for t in _times(duration_sec, dt_sec):
        h = heading0_rad + w * t
        x = p0[0] + speed_mps / w * (np.sin(h) - np.sin(heading0_rad))
        y = p0[1] - speed_mps / w * (np.cos(h) - np.cos(heading0_rad))
        z = p0[2] + vertical_speed_mps * t
        v = np.array([speed_mps * np.cos(h), speed_mps * np.sin(h), vertical_speed_mps], dtype=float)
        states.append(TargetState(target_id, float(t), np.array([x, y, z]), v))
    return states


def acceleration_maneuver(
    *, target_id: int, position0_m: Iterable[float], velocity0_mps: Iterable[float],
    acceleration_mps2: Iterable[float], maneuver_start_sec: float, maneuver_end_sec: float,
    duration_sec: float, dt_sec: float,
) -> list[TargetState]:
    p = np.asarray(tuple(position0_m), dtype=float).copy()
    v = np.asarray(tuple(velocity0_mps), dtype=float).copy()
    a_cmd = np.asarray(tuple(acceleration_mps2), dtype=float)
    if p.shape != (3,) or v.shape != (3,) or a_cmd.shape != (3,):
        raise ValueError("vectors must be length 3")

    states: list[TargetState] = []
    times = _times(duration_sec, dt_sec)
    for i, t in enumerate(times):
        states.append(TargetState(target_id, float(t), p.copy(), v.copy()))
        if i == len(times) - 1:
            break
        a = a_cmd if maneuver_start_sec <= t < maneuver_end_sec else np.zeros(3)
        p = p + v * dt_sec + 0.5 * a * dt_sec**2
        v = v + a * dt_sec
    return states
