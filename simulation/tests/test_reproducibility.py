from simulation.sim.trajectory import constant_velocity
from simulation.sim.sensors.radar import RadarConfig, RadarSensor
from simulation.sim.world import World


def _run(seed: int):
    truth = constant_velocity(target_id=1, position0_m=(1200, 400, 300), velocity_mps=(70, -12, 1), duration_sec=10.0, dt_sec=0.1)
    cfg = RadarConfig(sensor_id=11, position_enu_m=(0, 0, 0), detection_probability=0.93, packet_loss_probability=0.17, false_alarm_rate_per_scan=0.2)
    return World([RadarSensor(cfg, seed)]).run(truth).observations


def test_same_seed_is_bitwise_reproducible():
    assert _run(20260903) == _run(20260903)


def test_different_seed_changes_stream():
    assert _run(20260903) != _run(20260904)
