import numpy as np

from simulation.sim.trajectory import constant_velocity, coordinated_turn, acceleration_maneuver


def test_constant_velocity_matches_closed_form():
    states = constant_velocity(target_id=7, position0_m=(1, 2, 3), velocity_mps=(10, -4, 2), duration_sec=2.0, dt_sec=0.1)
    np.testing.assert_allclose(states[-1].position_enu_m, [21, -6, 7], atol=1e-12)
    np.testing.assert_allclose(states[-1].velocity_enu_mps, [10, -4, 2], atol=1e-12)


def test_coordinated_turn_preserves_horizontal_speed():
    states = coordinated_turn(target_id=1, position0_m=(1000, 0, 100), speed_mps=80, heading0_rad=0.2, turn_rate_radps=0.08, vertical_speed_mps=0, duration_sec=12, dt_sec=0.1)
    speeds = [np.linalg.norm(s.velocity_enu_mps[:2]) for s in states]
    np.testing.assert_allclose(speeds, 80.0, atol=1e-10)


def test_maneuver_changes_velocity_by_integrated_acceleration():
    states = acceleration_maneuver(target_id=1, position0_m=(0, 0, 0), velocity0_mps=(20, 0, 0), acceleration_mps2=(0, 5, 0), maneuver_start_sec=2, maneuver_end_sec=4, duration_sec=6, dt_sec=0.1)
    np.testing.assert_allclose(states[-1].velocity_enu_mps, [20, 10, 0], atol=1e-10)
