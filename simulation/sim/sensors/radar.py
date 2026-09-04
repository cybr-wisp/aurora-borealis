from __future__ import annotations

from dataclasses import dataclass
import math
import numpy as np

from ..target import TargetState


@dataclass(frozen=True)
class RadarConfig:
    sensor_id: int
    position_enu_m: tuple[float, float, float]
    range_sigma_m: float = 12.0
    azimuth_sigma_rad: float = 0.003
    elevation_sigma_rad: float = 0.003
    detection_probability: float = 0.98
    packet_loss_probability: float = 0.0
    false_alarm_rate_per_scan: float = 0.0
    range_bias_m: float = 0.0
    azimuth_bias_rad: float = 0.0
    elevation_bias_rad: float = 0.0


@dataclass(frozen=True)
class RadarObservation:
    sensor_id: int
    sequence_number: int
    target_id: int | None
    timestamp_sec: float
    range_m: float
    azimuth_rad: float
    elevation_rad: float
    range_sigma: float
    azimuth_sigma: float
    elevation_sigma: float
    is_clutter: bool = False


def cartesian_to_spherical(relative_enu_m: np.ndarray) -> tuple[float, float, float]:
    x, y, z = np.asarray(relative_enu_m, dtype=float)
    horizontal = math.hypot(x, y)
    r = math.sqrt(x*x + y*y + z*z)
    if r <= 1e-9:
        raise ValueError("target cannot coincide with radar")
    return r, math.atan2(y, x), math.atan2(z, horizontal)


def spherical_to_cartesian(range_m: float, azimuth_rad: float, elevation_rad: float) -> np.ndarray:
    ce = math.cos(elevation_rad)
    return np.array([
        range_m * ce * math.cos(azimuth_rad),
        range_m * ce * math.sin(azimuth_rad),
        range_m * math.sin(elevation_rad),
    ], dtype=float)


class RadarSensor:
    """Deterministic radar model. One RNG is owned by each sensor instance."""

    def __init__(self, config: RadarConfig, seed: int):
        self.config = config
        ss = np.random.SeedSequence([int(seed), int(config.sensor_id)])
        self._rng = np.random.default_rng(ss)
        self._sequence = 0
        self.generated_detections = 0
        self.packet_drops = 0
        self.missed_detections = 0

    def _next_sequence(self) -> int:
        value = self._sequence
        self._sequence += 1
        return value

    def observe(self, state: TargetState) -> list[RadarObservation]:
        cfg = self.config
        observations: list[RadarObservation] = []
        relative = state.position_enu_m - np.asarray(cfg.position_enu_m, dtype=float)
        true_r, true_az, true_el = cartesian_to_spherical(relative)

        if self._rng.random() <= cfg.detection_probability:
            self.generated_detections += 1
            if self._rng.random() < cfg.packet_loss_probability:
                self.packet_drops += 1
            else:
                observations.append(RadarObservation(
                    sensor_id=cfg.sensor_id,
                    sequence_number=self._next_sequence(),
                    target_id=state.target_id,
                    timestamp_sec=state.timestamp_sec,
                    range_m=true_r + cfg.range_bias_m + self._rng.normal(0.0, cfg.range_sigma_m),
                    azimuth_rad=true_az + cfg.azimuth_bias_rad + self._rng.normal(0.0, cfg.azimuth_sigma_rad),
                    elevation_rad=true_el + cfg.elevation_bias_rad + self._rng.normal(0.0, cfg.elevation_sigma_rad),
                    range_sigma=cfg.range_sigma_m,
                    azimuth_sigma=cfg.azimuth_sigma_rad,
                    elevation_sigma=cfg.elevation_sigma_rad,
                ))
        else:
            self.missed_detections += 1

        clutter_count = int(self._rng.poisson(cfg.false_alarm_rate_per_scan))
        for _ in range(clutter_count):
            observations.append(RadarObservation(
                sensor_id=cfg.sensor_id,
                sequence_number=self._next_sequence(),
                target_id=None,
                timestamp_sec=state.timestamp_sec,
                range_m=float(self._rng.uniform(200.0, max(500.0, true_r * 1.5))),
                azimuth_rad=float(self._rng.uniform(-math.pi, math.pi)),
                elevation_rad=float(self._rng.uniform(-0.25, 0.25)),
                range_sigma=cfg.range_sigma_m,
                azimuth_sigma=cfg.azimuth_sigma_rad,
                elevation_sigma=cfg.elevation_sigma_rad,
                is_clutter=True,
            ))
        return observations

