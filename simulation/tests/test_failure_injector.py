from simulation.sim.network.failure_injector import FailureInjector, FailureInjectorConfig
from simulation.sim.sensors.radar import RadarObservation
from simulation.sim.world import World


def make_observation(sequence_number: int, timestamp_sec: float) -> RadarObservation:
    return RadarObservation(
        sensor_id=1,
        sequence_number=sequence_number,
        target_id=1,
        timestamp_sec=timestamp_sec,
        range_m=1000.0,
        azimuth_rad=0.0,
        elevation_rad=0.0,
        range_sigma=10.0,
        azimuth_sigma=0.003,
        elevation_sigma=0.003,
    )


class StubSensor:
    def __init__(self) -> None:
        self.sequence = 0

    def observe(self, state) -> list[RadarObservation]:
        observation = make_observation(
            sequence_number=self.sequence,
            timestamp_sec=float(self.sequence),
        )
        self.sequence += 1
        return [observation]


def test_world_applies_transport_reordering() -> None:
    injector = FailureInjector(
        FailureInjectorConfig(reorder_probability=1.0),
        seed=20260904,
    )

    world = World(
        sensors=[StubSensor()],
        failure_injector=injector,
    )

    result = world.run([object(), object(), object(), object()])

    sequence_numbers = [
        observation.sequence_number
        for observation in result.observations
    ]

    assert sequence_numbers == [1, 0, 3, 2]


def test_failure_injector_is_deterministic() -> None:
    packets = list(range(20))

    first = FailureInjector(
        FailureInjectorConfig(reorder_probability=0.5),
        seed=12345,
    ).apply(packets)

    second = FailureInjector(
        FailureInjectorConfig(reorder_probability=0.5),
        seed=12345,
    ).apply(packets)

    assert first == second


def test_zero_probability_preserves_order() -> None:
    packets = list(range(20))

    result = FailureInjector(
        FailureInjectorConfig(reorder_probability=0.0),
        seed=12345,
    ).apply(packets)

    assert result == packets
