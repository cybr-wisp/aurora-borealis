from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable

from .target import TargetState
from .sensors.radar import RadarObservation, RadarSensor


@dataclass(frozen=True)
class WorldResult:
    observations: tuple[RadarObservation, ...]
    truth: tuple[TargetState, ...]


class World:
    def __init__(self, sensors: Iterable[RadarSensor]):
        self.sensors = tuple(sensors)

    def run(self, truth: Iterable[TargetState]) -> WorldResult:
        truth_states = tuple(truth)
        stream: list[RadarObservation] = []
        for state in truth_states:
            for sensor in self.sensors:
                stream.extend(sensor.observe(state))

        # Stable timestamp ordering is deterministic. Reordering is applied per sensor
        # by swapping adjacent packets using that sensor's seeded RNG in the sender layer.
        stream.sort(key=lambda o: (o.timestamp_sec, o.sensor_id, o.sequence_number))
        return WorldResult(tuple(stream), truth_states)
