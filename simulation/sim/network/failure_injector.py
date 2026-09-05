from __future__ import annotations

from collections.abc import Iterable
from dataclasses import dataclass, replace
import random
from typing import TypeVar

from ..sensors.radar import RadarObservation


T = TypeVar("T")


@dataclass(frozen=True)
class FailureInjectorConfig:
    """Transport impairments applied after sensor measurement generation."""

    reorder_probability: float = 0.0

    def __post_init__(self) -> None:
        if not 0.0 <= self.reorder_probability <= 1.0:
            raise ValueError("reorder_probability must be in [0, 1]")


class FailureInjector:
    """Deterministic transport-layer packet reordering."""

    def __init__(self, config: FailureInjectorConfig, seed: int) -> None:
        self.config = config
        self._rng = random.Random(seed)

    def apply(self, observations: Iterable[T]) -> list[T]:
        items = list(observations)
        output: list[T] = []
        i = 0

        while i < len(items):
            can_swap = i + 1 < len(items)
            if (
                can_swap
                and self._rng.random() < self.config.reorder_probability
            ):
                output.append(items[i + 1])
                output.append(items[i])
                i += 2
            else:
                output.append(items[i])
                i += 1

        return output


@dataclass(frozen=True)
class SensorFaultWindow:
    sensor_id: int
    start_sec: float
    end_sec: float
    drop_all: bool = False
    packet_loss_probability: float = 0.0
    range_bias_m: float = 0.0
    azimuth_bias_rad: float = 0.0
    elevation_bias_rad: float = 0.0
    range_noise_sigma_m: float = 0.0
    azimuth_noise_sigma_rad: float = 0.0
    elevation_noise_sigma_rad: float = 0.0

    def __post_init__(self) -> None:
        if self.end_sec <= self.start_sec:
            raise ValueError("end_sec must be greater than start_sec")
        if not 0.0 <= self.packet_loss_probability <= 1.0:
            raise ValueError("packet_loss_probability must be in [0, 1]")
        if min(
            self.range_noise_sigma_m,
            self.azimuth_noise_sigma_rad,
            self.elevation_noise_sigma_rad,
        ) < 0.0:
            raise ValueError("fault noise sigmas must be non-negative")

    def active_for(self, observation: RadarObservation) -> bool:
        return (
            observation.sensor_id == self.sensor_id
            and self.start_sec
            <= observation.timestamp_sec
            < self.end_sec
        )


@dataclass
class SensorFaultStats:
    dropped: int = 0
    biased: int = 0
    noised: int = 0


class ScheduledSensorFaultInjector:
    """Repeatable outage, loss, bias and extra-noise injection."""

    def __init__(
        self,
        windows: Iterable[SensorFaultWindow],
        seed: int,
    ) -> None:
        self.windows = tuple(windows)
        self._rng = random.Random(seed)
        self.stats: dict[int, SensorFaultStats] = {}

    def _stats(self, sensor_id: int) -> SensorFaultStats:
        return self.stats.setdefault(sensor_id, SensorFaultStats())

    def apply(
        self,
        observations: Iterable[RadarObservation],
    ) -> list[RadarObservation]:
        output: list[RadarObservation] = []

        for observation in observations:
            active = [
                window
                for window in self.windows
                if window.active_for(observation)
            ]

            if not active:
                output.append(observation)
                continue

            stats = self._stats(observation.sensor_id)

            if any(window.drop_all for window in active):
                stats.dropped += 1
                continue

            survival = 1.0
            for window in active:
                survival *= 1.0 - window.packet_loss_probability
            loss_probability = 1.0 - survival

            if self._rng.random() < loss_probability:
                stats.dropped += 1
                continue

            range_bias = sum(w.range_bias_m for w in active)
            azimuth_bias = sum(w.azimuth_bias_rad for w in active)
            elevation_bias = sum(w.elevation_bias_rad for w in active)

            range_noise = sum(w.range_noise_sigma_m for w in active)
            azimuth_noise = sum(w.azimuth_noise_sigma_rad for w in active)
            elevation_noise = sum(w.elevation_noise_sigma_rad for w in active)

            if any(v != 0.0 for v in (range_bias, azimuth_bias, elevation_bias)):
                stats.biased += 1
            if any(v > 0.0 for v in (range_noise, azimuth_noise, elevation_noise)):
                stats.noised += 1

            output.append(
                replace(
                    observation,
                    range_m=(
                        observation.range_m
                        + range_bias
                        + self._rng.gauss(0.0, range_noise)
                    ),
                    azimuth_rad=(
                        observation.azimuth_rad
                        + azimuth_bias
                        + self._rng.gauss(0.0, azimuth_noise)
                    ),
                    elevation_rad=(
                        observation.elevation_rad
                        + elevation_bias
                        + self._rng.gauss(0.0, elevation_noise)
                    ),
                )
            )

        return output
