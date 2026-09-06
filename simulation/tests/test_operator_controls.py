from __future__ import annotations

import numpy as np

from simulation.live_operator import OperatorSimulation


def advance(sim: OperatorSimulation, steps: int) -> dict:
    state = {}
    for _ in range(steps):
        state = sim.step()
    return state


def sensor(state: dict, sensor_id: int) -> dict:
    return next(
        item
        for item in state["sensors"]
        if item["id"] == sensor_id
    )


def test_full_nominal_reset_state():
    sim = OperatorSimulation()
    state = advance(sim, 40)

    assert len(state["tracks"]) == 1
    assert state["tracks"][0]["label"] == "TRK-001"
    assert state["tracks"][0]["filter"] == "EKF"

    assert all(
        item["health"] == "NOMINAL"
        for item in state["sensors"]
    )

    assert all(
        item["biasActive"] is False
        and item["noiseActive"] is False
        for item in state["sensors"]
    )


def test_add_target_is_real_and_independent():
    sim = OperatorSimulation()
    advance(sim, 40)

    sim.command({"action": "add_target"})
    state = advance(sim, 40)

    assert len(state["tracks"]) == 2

    a, b = state["tracks"]

    assert a["label"] == "TRK-001"
    assert b["label"] == "TRK-002"

    assert a["position"] != b["position"]
    assert a["velocity"] != b["velocity"]
    assert a["trail"] != b["trail"]


def test_kill_and_restore_every_sensor():
    sim = OperatorSimulation()
    advance(sim, 40)

    for sensor_id in (1, 2, 3):
        sim.command({
            "action": "kill_sensor",
            "sensorId": sensor_id,
        })

        state = advance(sim, 3)

        assert sensor(state, sensor_id)["health"] == "OFFLINE"

        sim.command({
            "action": "restore_sensor",
            "sensorId": sensor_id,
        })

        state = advance(sim, 3)

        assert sensor(state, sensor_id)["health"] != "OFFLINE"


def test_bias_control_degrades_sensor():
    sim = OperatorSimulation()
    advance(sim, 40)

    sim.command({
        "action": "inject_bias",
        "sensorId": 1,
    })

    state = advance(sim, 140)
    radar = sensor(state, 1)

    assert radar["biasActive"] is True
    assert radar["estimatedR"] > 1.0

    print(
        "bias:",
        radar["health"],
        "R=",
        radar["estimatedR"],
    )

    sim.command({
        "action": "inject_bias",
        "sensorId": 1,
    })

    state = advance(sim, 300)

    assert sensor(state, 1)["biasActive"] is False


def test_noise_control_degrades_sensor():
    sim = OperatorSimulation()
    advance(sim, 40)

    sim.command({
        "action": "add_noise",
        "sensorId": 2,
    })

    state = advance(sim, 140)
    radar = sensor(state, 2)

    assert radar["noiseActive"] is True
    assert radar["estimatedR"] > 1.0

    print(
        "noise:",
        radar["health"],
        "R=",
        radar["estimatedR"],
    )

    sim.command({
        "action": "add_noise",
        "sensorId": 2,
    })

    state = advance(sim, 300)

    assert sensor(state, 2)["noiseActive"] is False


def test_trigger_maneuver_changes_track_motion():
    sim = OperatorSimulation()
    before = advance(sim, 50)

    v0 = np.asarray(
        before["tracks"][0]["velocity"],
        dtype=float,
    )

    sim.command({"action": "trigger_maneuver"})

    after = advance(sim, 30)

    v1 = np.asarray(
        after["tracks"][0]["velocity"],
        dtype=float,
    )

    assert np.linalg.norm(v1 - v0) > 1.0


def test_filter_toggle_changes_ekf_to_ukf():
    sim = OperatorSimulation()
    state = advance(sim, 40)

    assert state["tracks"][0]["filter"] == "EKF"

    sim.command({"action": "toggle_filter"})
    state = advance(sim, 40)

    assert all(
        track["filter"] == "UKF"
        for track in state["tracks"]
    )

    sim.command({"action": "toggle_filter"})
    state = advance(sim, 40)

    assert all(
        track["filter"] == "EKF"
        for track in state["tracks"]
    )


def test_reset_clears_every_operator_fault():
    sim = OperatorSimulation()
    advance(sim, 40)

    sim.command({"action": "add_target"})
    sim.command({"action": "kill_sensor", "sensorId": 1})
    sim.command({"action": "inject_bias", "sensorId": 2})
    sim.command({"action": "add_noise", "sensorId": 3})
    sim.command({"action": "toggle_filter"})

    advance(sim, 20)

    sim.command({"action": "reset"})
    state = advance(sim, 40)

    assert len(state["tracks"]) == 1
    assert state["tracks"][0]["label"] == "TRK-001"
    assert state["tracks"][0]["filter"] == "EKF"

    for radar in state["sensors"]:
        assert radar["health"] == "NOMINAL"
        assert radar["biasActive"] is False
        assert radar["noiseActive"] is False

def test_maneuver_targets_selected_track_only():
    sim = OperatorSimulation()

    advance(sim, 30)

    sim.command({"action": "add_target"})
    sim.command({"action": "add_target"})

    advance(sim, 10)

    before = {
        target_id: target.maneuver_until_sec
        for target_id, target in sim.targets.items()
    }

    sim.command({
        "action": "trigger_maneuver",
        "targetId": 2,
    })

    assert sim.targets[1].maneuver_until_sec == before[1]
    assert sim.targets[2].maneuver_until_sec > sim.t
    assert sim.targets[3].maneuver_until_sec == before[3]

    assert "TRK-002" in sim.message


def test_filter_toggle_preserves_track_history():
    sim = OperatorSimulation()

    advance(sim, 60)

    sim.command({"action": "add_target"})
    advance(sim, 30)

    before = {
        target_id: list(target.trail)
        for target_id, target in sim.targets.items()
    }

    assert all(len(trail) > 0 for trail in before.values())

    sim.command({"action": "toggle_filter"})

    for target_id, target in sim.targets.items():
        assert target.trail == before[target_id]

    state = advance(sim, 5)

    assert all(
        track["filter"] == "UKF"
        for track in state["tracks"]
    )

def test_operational_boundary_prevents_unbounded_drift():
    sim = OperatorSimulation()
    target = sim.targets[1]

    center = sim.operating_center_xy.copy()

    # Start just outside the soft boundary and moving
    # aggressively away from the radar field.
    target.position[:2] = (
        center
        + np.array(
            [
                sim.operating_soft_radius_m
                + 500.0,
                0.0,
            ]
        )
    )

    target.velocity[:2] = np.array(
        [110.0, 0.0]
    )

    maximum_radius = 0.0
    maximum_step = 0.0

    previous_position = target.position.copy()

    for _ in range(600):
        sim._truth_state(target)

        radius = float(
            np.linalg.norm(
                target.position[:2]
                - center
            )
        )

        step_distance = float(
            np.linalg.norm(
                target.position
                - previous_position
            )
        )

        maximum_radius = max(
            maximum_radius,
            radius,
        )

        maximum_step = max(
            maximum_step,
            step_distance,
        )

        previous_position = (
            target.position.copy()
        )

    assert (
        maximum_radius
        <= sim.operating_hard_radius_m
        + 1e-6
    )

    # Proves the normal soft-boundary path did not
    # teleport the target.
    assert maximum_step < 20.0


def test_added_targets_spawn_inside_safe_operational_radius():
    sim = OperatorSimulation()

    for _ in range(7):
        sim.command(
            {
                "action": "add_target",
            }
        )

    for target in sim.targets.values():
        radius = float(
            np.linalg.norm(
                target.position[:2]
                - sim.operating_center_xy
            )
        )

        assert (
            radius
            <= sim.spawn_safe_radius_m
            + 1e-6
        )
