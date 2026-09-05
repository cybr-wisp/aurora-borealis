from simulation.sim.network.failure_injector import (
    ScheduledSensorFaultInjector,
    SensorFaultWindow,
)
from simulation.sim.sensors.radar import RadarObservation


def obs(sensor_id: int, timestamp_sec: float, sequence: int) -> RadarObservation:
    return RadarObservation(
        sensor_id=sensor_id,
        sequence_number=sequence,
        target_id=1,
        timestamp_sec=timestamp_sec,
        range_m=1000.0,
        azimuth_rad=0.1,
        elevation_rad=0.02,
        range_sigma=10.0,
        azimuth_sigma=0.003,
        elevation_sigma=0.003,
    )


def test_outage_is_sensor_and_time_scoped() -> None:
    injector = ScheduledSensorFaultInjector(
        [
            SensorFaultWindow(
                sensor_id=11,
                start_sec=10.0,
                end_sec=20.0,
                drop_all=True,
            )
        ],
        seed=7,
    )

    result = injector.apply(
        [
            obs(11, 9.0, 1),
            obs(11, 12.0, 2),
            obs(22, 12.0, 3),
            obs(11, 21.0, 4),
        ]
    )

    assert [item.sequence_number for item in result] == [1, 3, 4]
    assert injector.stats[11].dropped == 1


def test_bias_is_exact_without_extra_noise() -> None:
    injector = ScheduledSensorFaultInjector(
        [
            SensorFaultWindow(
                sensor_id=11,
                start_sec=0.0,
                end_sec=10.0,
                range_bias_m=5.0,
                azimuth_bias_rad=0.01,
            )
        ],
        seed=9,
    )

    result = injector.apply([obs(11, 1.0, 1)])
    assert result[0].range_m == 1005.0
    assert result[0].azimuth_rad == 0.11


def test_same_seed_is_reproducible() -> None:
    windows = [
        SensorFaultWindow(
            sensor_id=11,
            start_sec=0.0,
            end_sec=100.0,
            packet_loss_probability=0.25,
            range_noise_sigma_m=4.0,
        )
    ]
    observations = [obs(11, float(i), i) for i in range(100)]

    first = ScheduledSensorFaultInjector(windows, seed=123)
    second = ScheduledSensorFaultInjector(windows, seed=123)

    assert first.apply(observations) == second.apply(observations)


def test_packet_loss_rate_is_reasonable() -> None:
    injector = ScheduledSensorFaultInjector(
        [
            SensorFaultWindow(
                sensor_id=11,
                start_sec=0.0,
                end_sec=1000.0,
                packet_loss_probability=0.4,
            )
        ],
        seed=1234,
    )

    result = injector.apply(
        [obs(11, float(i), i) for i in range(500)]
    )

    dropped = 500 - len(result)
    assert 150 <= dropped <= 250
