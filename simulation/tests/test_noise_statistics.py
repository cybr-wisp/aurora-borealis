import numpy as np
from scipy.stats import chisquare

from simulation.sim.target import TargetState
from simulation.sim.sensors.radar import RadarConfig, RadarSensor, cartesian_to_spherical


def test_noise_statistics_and_packet_loss_converge():
    cfg = RadarConfig(sensor_id=3, position_enu_m=(0, 0, 0), range_sigma_m=15.0, azimuth_sigma_rad=0.004, elevation_sigma_rad=0.003, detection_probability=1.0, packet_loss_probability=0.10)
    radar = RadarSensor(cfg, seed=42)
    state = TargetState(1, 0.0, np.array([1800.0, 600.0, 250.0]), np.zeros(3))
    true_r, true_az, true_el = cartesian_to_spherical(state.position_enu_m)

    errors = []
    for i in range(10000):
        s = TargetState(1, i * 0.1, state.position_enu_m, state.velocity_enu_mps)
        obs = radar.observe(s)
        if obs:
            o = obs[0]
            errors.append([o.range_m - true_r, o.azimuth_rad - true_az, o.elevation_rad - true_el])

    errors = np.asarray(errors)
    assert abs(radar.packet_drops / radar.generated_detections - 0.10) < 0.01
    assert abs(errors[:, 0].mean()) < 0.5
    assert abs(errors[:, 1].mean()) < 2e-4
    assert abs(errors[:, 2].mean()) < 2e-4
    std = errors.std(axis=0, ddof=1)
    assert abs(std[0] / 15.0 - 1.0) < 0.04
    assert abs(std[1] / 0.004 - 1.0) < 0.04
    assert abs(std[2] / 0.003 - 1.0) < 0.04

    # Variance-binned chi-square sanity test: normalized squared errors should
    # distribute across quartile bins approximately as chi-square(1).
    z2 = (errors[:, 0] / cfg.range_sigma_m) ** 2
    bins = np.array([0.0, 0.101531, 0.454936, 1.323304, np.inf])
    counts, _ = np.histogram(z2, bins=bins)
    _, p = chisquare(counts)
    assert p > 0.01
