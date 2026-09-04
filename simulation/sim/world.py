from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable

from .network.failure_injector import FailureInjector
from .target import TargetState
from .sensors.radar import RadarObservation, RadarSensor


@dataclass(frozen=True)
class WorldResult:
    observations: tuple[RadarObservation, ...]
    truth: tuple[TargetState, ...]


class World:
    def __init__(
        self,
        sensors: Iterable[RadarSensor],
        failure_injector: FailureInjector | None = None,
    ):
        self.sensors = tuple(sensors)
        self.failure_injector = failure_injector

    def run(self, truth: Iterable[TargetState]) -> WorldResult:
        truth_states = tuple(truth)
        stream: list[RadarObservation] = []

        for state in truth_states:
            for sensor in self.sensors:
                stream.extend(sensor.observe(state))

        # Establish deterministic nominal delivery order first.
        stream.sort(
            key=lambda o: (
                o.timestamp_sec,
                o.sensor_id,
                o.sequence_number,
            )
        )

        # Apply transport impairments only after measurements have been generated.
        if self.failure_injector is not None:
            stream = self.failure_injector.apply(stream)

        return WorldResult(tuple(stream), truth_states)
