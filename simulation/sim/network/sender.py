from __future__ import annotations

import socket
import time
from collections.abc import Iterable

from ..sensors.radar import RadarObservation

try:
    from . import observation_pb2
except ImportError as exc:  # generated at build time by protoc
    observation_pb2 = None
    _IMPORT_ERROR = exc
else:
    _IMPORT_ERROR = None


def send_observations(observations: Iterable[RadarObservation], host: str = "127.0.0.1", port: int = 46000, epoch_sec: float | None = None) -> int:
    if observation_pb2 is None:
        raise RuntimeError("Generate Python protobuf first: protoc -I engine/proto --python_out=simulation/sim/network engine/proto/observation.proto") from _IMPORT_ERROR
    epoch = time.time() if epoch_sec is None else epoch_sec
    sent = 0
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        for obs in observations:
            msg = observation_pb2.Observation(
                sensor_id=obs.sensor_id,
                sequence_number=obs.sequence_number,
                timestamp_sec=epoch + obs.timestamp_sec,
                range_m=obs.range_m,
                azimuth_rad=obs.azimuth_rad,
                elevation_rad=obs.elevation_rad,
                range_sigma=obs.range_sigma,
                azimuth_sigma=obs.azimuth_sigma,
                elevation_sigma=obs.elevation_sigma,
                sent_monotonic_ns=time.monotonic_ns(),
            )
            sock.sendto(msg.SerializeToString(), (host, port))
            sent += 1
    return sent
