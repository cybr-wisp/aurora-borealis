from __future__ import annotations

import math

from simulation.live_operator import OperatorSimulation


def advance(sim: OperatorSimulation, steps: int = 30) -> dict:
    state = {}
    for _ in range(steps):
        state = sim.step()
    return state


def test_reset_starts_three_nominal_sensors_and_one_target():
    sim = OperatorSimulation()
    state = advance(sim, 30)

    assert len(state["sensors"]) == 3
    assert len(state["tracks"]) == 1
    assert all(sensor["health"] == "NOMINAL" for sensor in state["sensors"])


def test_kill_and_restore_are_sensor_scoped():
    sim = OperatorSimulation()
    advance(sim, 20)

    sim.command({"action": "kill_sensor", "sensorId": 2})
    state = advance(sim, 3)

    sensor_2 = next(sensor for sensor in state["sensors"] if sensor["id"] == 2)
    sensor_1 = next(sensor for sensor in state["sensors"] if sensor["id"] == 1)

    assert sensor_2["health"] == "OFFLINE"
    assert sensor_1["health"] != "OFFLINE"

    sim.command({"action": "restore_sensor", "sensorId": 2})
    state = advance(sim, 3)
    sensor_2 = next(sensor for sensor in state["sensors"] if sensor["id"] == 2)

    assert sensor_2["health"] != "OFFLINE"


def test_bias_and_noise_controls_toggle_per_sensor():
    sim = OperatorSimulation()

    sim.command({"action": "inject_bias", "sensorId": 1})
    sim.command({"action": "add_noise", "sensorId": 3})

    state = sim.step()

    sensor_1 = next(sensor for sensor in state["sensors"] if sensor["id"] == 1)
    sensor_3 = next(sensor for sensor in state["sensors"] if sensor["id"] == 3)

    assert sensor_1["biasActive"] is True
    assert sensor_3["noiseActive"] is True

    sim.command({"action": "inject_bias", "sensorId": 1})
    sim.command({"action": "add_noise", "sensorId": 3})

    state = sim.step()
    sensor_1 = next(sensor for sensor in state["sensors"] if sensor["id"] == 1)
    sensor_3 = next(sensor for sensor in state["sensors"] if sensor["id"] == 3)

    assert sensor_1["biasActive"] is False
    assert sensor_3["noiseActive"] is False


def test_add_target_creates_independent_simulated_track():
    sim = OperatorSimulation()
    advance(sim, 30)

    sim.command({"action": "add_target"})
    state = advance(sim, 30)

    assert len(state["tracks"]) == 2

    first, second = state["tracks"]
    assert first["id"] != second["id"]
    assert first["position"] != second["position"]
    assert first["velocity"] != second["velocity"]
    assert first["trail"] != second["trail"]


def test_filter_toggle_reinitializes_tracks_with_requested_filter():
    sim = OperatorSimulation()
    state = advance(sim, 30)
    assert all(track["filter"] == "EKF" for track in state["tracks"])

    sim.command({"action": "toggle_filter"})
    state = advance(sim, 30)

    assert all(track["filter"] == "UKF" for track in state["tracks"])


def test_raw_observations_are_measurement_derived_not_track_clones():
    sim = OperatorSimulation()
    state = advance(sim, 30)

    assert state["observations"]
    track = state["tracks"][0]

    distances = [
        math.hypot(
            observation["position"][0] - track["position"][0],
            observation["position"][1] - track["position"][1],
        )
        for observation in state["observations"]
        if observation["targetId"] == track["id"]
    ]

    assert any(distance > 0.01 for distance in distances)


def test_metrics_distinguish_live_update_from_benchmark_p99():
    sim = OperatorSimulation()
    state = advance(sim, 5)
    metrics = state["metrics"]

    assert metrics["liveUpdateMs"] >= 0.0
    assert metrics["benchAssociationP99Ms"] > 0.0
    assert metrics["activeTracks"] == len(state["tracks"])
