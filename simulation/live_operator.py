from __future__ import annotations

import asyncio
from dataclasses import dataclass, field
import json
import math
from pathlib import Path
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

import numpy as np
import websockets

from experiments.tracking_benchmark import (
    EKF,
    UKF,
    H,
    h,
    obs_cov,
    first_measurement_state,
)
from simulation.sim.sensors.radar import (
    RadarConfig,
    RadarSensor,
    spherical_to_cartesian,
)
from simulation.sim.target import TargetState


DT = 0.1
HOST = "127.0.0.1"
PORT = 8765
SEED = 20260905
BENCH_ASSOCIATION_P99_MS = 3.34

# Live operator scenario geometry.
#
# Targets begin steering back toward the radar field outside
# the soft radius. The hard radius is only a safety guardrail.
OPERATIONAL_SOFT_RADIUS_M = 15000.0
OPERATIONAL_HARD_RADIUS_M = 20000.0
SPAWN_SAFE_RADIUS_M = 12000.0

BOUNDARY_MAX_ACCEL_MPS2 = 12.0
BOUNDARY_DESIRED_INWARD_MPS = 35.0
BOUNDARY_SPEED_GAIN = 0.25
BOUNDARY_DISTANCE_GAIN = 0.002


@dataclass
class RuntimeTarget:
    target_id: int
    position: np.ndarray
    velocity: np.ndarray
    filter: object | None = None
    trail: list[list[float]] = field(default_factory=list)
    maneuver_until_sec: float = 0.0


class OperatorSimulation:
    def __init__(self) -> None:
        self.reset()

    def reset(self) -> None:
        nominal = dict(
            range_sigma_m=10.0,
            azimuth_sigma_rad=0.0025,
            elevation_sigma_rad=0.0025,
            detection_probability=0.98,
            packet_loss_probability=0.02,
        )
        self.configs = [
            RadarConfig(
                sensor_id=1,
                position_enu_m=(0.0, 0.0, 0.0),
                **nominal,
            ),
            RadarConfig(
                sensor_id=2,
                position_enu_m=(2500.0, -800.0, 50.0),
                **nominal,
            ),
            RadarConfig(
                sensor_id=3,
                position_enu_m=(-1800.0, 1400.0, 80.0),
                **nominal,
            ),
        ]
        self.operating_center_xy = np.mean(
            np.asarray(
                [
                    cfg.position_enu_m[:2]
                    for cfg in self.configs
                ],
                dtype=float,
            ),
            axis=0,
        )

        self.operating_soft_radius_m = (
            OPERATIONAL_SOFT_RADIUS_M
        )
        self.operating_hard_radius_m = (
            OPERATIONAL_HARD_RADIUS_M
        )
        self.spawn_safe_radius_m = (
            SPAWN_SAFE_RADIUS_M
        )

        self.radars = [RadarSensor(cfg, SEED) for cfg in self.configs]

        self.enabled = {cfg.sensor_id: True for cfg in self.configs}
        self.bias_m = {cfg.sensor_id: 0.0 for cfg in self.configs}
        self.noise_scale = {cfg.sensor_id: 1.0 for cfg in self.configs}

        self.r_est = {cfg.sensor_id: 1.0 for cfg in self.configs}
        self.r_windows = {cfg.sensor_id: [] for cfg in self.configs}
        self.health = {cfg.sensor_id: "NOMINAL" for cfg in self.configs}
        self.degraded_streak = {cfg.sensor_id: 0 for cfg in self.configs}
        self.recovery_streak = {cfg.sensor_id: 0 for cfg in self.configs}

        self.filter_name = "EKF"
        self.targets: dict[int, RuntimeTarget] = {}
        self.next_target_id = 1
        self._create_target()

        self.t = 0.0
        self.nees_history: list[dict[str, float]] = []
        self.message = "nominal"

        self.received = {cfg.sensor_id: 0 for cfg in self.configs}
        self.generated = {cfg.sensor_id: 0 for cfg in self.configs}
        self.control_rng = np.random.default_rng(SEED + 811)

    def _create_target(self) -> RuntimeTarget:
        index = len(self.targets)
        target_id = self.next_target_id
        self.next_target_id += 1

        if not self.targets:
            position = np.array(
                [1200.0, 350.0, 300.0],
                dtype=float,
            )
            velocity = np.array(
                [75.0, 12.0, 0.5],
                dtype=float,
            )
        else:
            # Spawn new targets near the current operating volume
            # instead of back at the original scenario origin.
            active_positions = np.stack(
                [
                    target.position
                    for target in self.targets.values()
                ]
            )

            center = np.mean(active_positions, axis=0)

            angle = (
                (target_id - 2)
                * 2.399963229728653
            )

            radius = (
                1100.0
                + 350.0 * max(0, index - 1)
            )

            offset = np.array(
                [
                    radius * math.cos(angle),
                    radius * math.sin(angle),
                    35.0 * index,
                ],
                dtype=float,
            )

            position = center + offset

            primary = self.targets[min(self.targets)]

            velocity = (
                primary.velocity
                + np.array(
                    [
                        -5.0 - 2.0 * index,
                        8.0 + 3.0 * index,
                        0.1 * index,
                    ],
                    dtype=float,
                )
            )

        # Keep newly created targets inside the useful
        # radar operating volume even when existing targets
        # happen to be close to the boundary.
        spawn_offset = (
            position[:2]
            - self.operating_center_xy
        )

        spawn_radius = float(
            np.linalg.norm(spawn_offset)
        )

        if (
            spawn_radius
            > self.spawn_safe_radius_m
        ):
            position = position.copy()

            position[:2] = (
                self.operating_center_xy
                + (
                    spawn_offset
                    / spawn_radius
                )
                * self.spawn_safe_radius_m
            )

        target = RuntimeTarget(
            target_id=target_id,
            position=position,
            velocity=velocity,
        )

        self.targets[target_id] = target
        return target


    def command(self, payload: dict) -> None:
        action = payload.get("action")
        sid = int(payload.get("sensorId", 1))

        if action == "kill_sensor" and sid in self.enabled:
            self.enabled[sid] = False
            self.health[sid] = "OFFLINE"
            self.message = f"sensor {sid} offline"

        elif action == "restore_sensor" and sid in self.enabled:
            self.enabled[sid] = True
            self.degraded_streak[sid] = 0
            self.recovery_streak[sid] = 0

            self.health[sid] = (
                "DEGRADED"
                if self.r_est[sid] >= 2.0
                else "NOMINAL"
            )

            self.message = f"sensor {sid} restored"

        elif action == "inject_bias" and sid in self.bias_m:
            self.bias_m[sid] = (
                25.0
                if self.bias_m[sid] == 0.0
                else 0.0
            )

            self.message = (
                f"sensor {sid} range bias "
                f"{'on' if self.bias_m[sid] else 'off'}"
            )

        elif action == "add_noise" and sid in self.noise_scale:
            self.noise_scale[sid] = (
                4.0
                if self.noise_scale[sid] == 1.0
                else 1.0
            )

            self.message = (
                f"sensor {sid} extra noise "
                f"{'on' if self.noise_scale[sid] > 1.0 else 'off'}"
            )

        elif action == "trigger_maneuver":
            requested_target = payload.get("targetId")

            if requested_target is None:
                target_id = min(self.targets)
            else:
                target_id = int(requested_target)

            target = self.targets.get(target_id)

            if target is None:
                self.message = (
                    f"cannot maneuver unknown "
                    f"TRK-{target_id:03d}"
                )
            else:
                target.maneuver_until_sec = self.t + 5.0

                self.message = (
                    f"maneuver injected into "
                    f"TRK-{target.target_id:03d}"
                )

        elif action == "add_target":
            target = self._create_target()

            self.message = (
                f"added independent target "
                f"TRK-{target.target_id:03d}"
            )

        elif action == "toggle_filter":
            self.filter_name = (
                "UKF"
                if self.filter_name == "EKF"
                else "EKF"
            )

            for target in self.targets.values():
                target.filter = None

            self.nees_history.clear()
            self.message = f"filter={self.filter_name}"

        elif action == "reset":
            self.reset()


    def _truth_state(self, target: RuntimeTarget) -> TargetState:
        # Operator-requested maneuver remains independent
        # from the operational-volume steering.
        if self.t < target.maneuver_until_sec:
            direction = (
                -1.0
                if target.target_id % 2 == 0
                else 1.0
            )

            acceleration = np.array(
                [
                    -7.0,
                    direction * 10.0,
                    0.0,
                ],
                dtype=float,
            )

            target.velocity += (
                acceleration * DT
            )

        # ----------------------------------------------------
        # Soft operational boundary
        # ----------------------------------------------------
        #
        # Outside 15 km, progressively steer the target toward
        # the radar field. This is continuous acceleration, not
        # a teleport or position reset.
        horizontal_offset = (
            target.position[:2]
            - self.operating_center_xy
        )

        horizontal_radius = float(
            np.linalg.norm(
                horizontal_offset
            )
        )

        if (
            horizontal_radius
            > self.operating_soft_radius_m
        ):
            radial_hat = (
                horizontal_offset
                / horizontal_radius
            )

            radial_speed = float(
                np.dot(
                    target.velocity[:2],
                    radial_hat,
                )
            )

            excess_distance = (
                horizontal_radius
                - self.operating_soft_radius_m
            )

            speed_error = max(
                0.0,
                radial_speed
                + BOUNDARY_DESIRED_INWARD_MPS,
            )

            inward_accel = min(
                BOUNDARY_MAX_ACCEL_MPS2,
                (
                    BOUNDARY_SPEED_GAIN
                    * speed_error
                )
                + (
                    BOUNDARY_DISTANCE_GAIN
                    * excess_distance
                ),
            )

            target.velocity[:2] -= (
                radial_hat
                * inward_accel
                * DT
            )

        # Normal continuous integration.
        next_position = (
            target.position
            + target.velocity * DT
        )

        # ----------------------------------------------------
        # Hard safety guardrail
        # ----------------------------------------------------
        #
        # Normally the soft steering prevents reaching this.
        # If an extreme maneuver does cross 20 km, preserve the
        # tangential component and reflect only outward radial
        # velocity.
        next_offset = (
            next_position[:2]
            - self.operating_center_xy
        )

        next_radius = float(
            np.linalg.norm(next_offset)
        )

        if (
            next_radius
            > self.operating_hard_radius_m
        ):
            normal = (
                next_offset
                / next_radius
            )

            outward_speed = float(
                np.dot(
                    target.velocity[:2],
                    normal,
                )
            )

            if outward_speed > 0.0:
                target.velocity[:2] -= (
                    1.8
                    * outward_speed
                    * normal
                )

            next_position[:2] = (
                self.operating_center_xy
                + normal
                * self.operating_hard_radius_m
            )

        target.position = next_position

        return TargetState(
            target_id=target.target_id,
            timestamp_sec=self.t,
            position_enu_m=target.position.copy(),
            velocity_enu_mps=target.velocity.copy(),
        )


    def _predict_filters(self) -> None:
        if self.t <= 0.0:
            return
        for target in self.targets.values():
            if target.filter is not None:
                target.filter.predict(DT)

    def _adapt_sensor_noise(
        self,
        sensor_id: int,
        innovation: np.ndarray,
        predicted_measurement_cov: np.ndarray,
        configured_r: np.ndarray,
    ) -> None:
        window = self.r_windows[sensor_id]
        window.append(
            (
                np.square(innovation).copy(),
                np.diag(predicted_measurement_cov).copy(),
                np.diag(configured_r).copy(),
            )
        )
        del window[:-50]

        if len(window) < 15:
            return

        innovation_sq = np.mean(
            np.stack([sample[0] for sample in window]),
            axis=0,
        )
        predicted = np.mean(
            np.stack([sample[1] for sample in window]),
            axis=0,
        )
        configured = np.mean(
            np.stack([sample[2] for sample in window]),
            axis=0,
        )

        raw_variance = np.maximum(
            0.25 * configured,
            innovation_sq - predicted,
        )
        raw_variance = np.minimum(raw_variance, 25.0 * configured)

        ratio = float(np.max(raw_variance / configured))
        self.r_est[sensor_id] = (
            0.95 * self.r_est[sensor_id] + 0.05 * ratio
        )

    def _update_health(self, sensor_id: int) -> None:
        if not self.enabled[sensor_id]:
            self.health[sensor_id] = "OFFLINE"
            self.degraded_streak[sensor_id] = 0
            self.recovery_streak[sensor_id] = 0
            return

        ratio = self.r_est[sensor_id]

        if ratio >= 2.0:
            self.degraded_streak[sensor_id] += 1
            self.recovery_streak[sensor_id] = 0
        elif ratio <= 1.6:
            self.recovery_streak[sensor_id] += 1
            self.degraded_streak[sensor_id] = 0
        else:
            self.degraded_streak[sensor_id] = 0
            self.recovery_streak[sensor_id] = 0

        if self.degraded_streak[sensor_id] >= 5:
            self.health[sensor_id] = "DEGRADED"
        elif self.recovery_streak[sensor_id] >= 10:
            self.health[sensor_id] = "NOMINAL"

    def _filter_class(self):
        return EKF if self.filter_name == "EKF" else UKF

    def step(self) -> dict:
        started = time.perf_counter()

        truths = {
            target_id: self._truth_state(target)
            for target_id, target in self.targets.items()
        }
        self._predict_filters()

        observations: list[dict] = []

        for radar, cfg in zip(self.radars, self.configs):
            sid = cfg.sensor_id
            if not self.enabled[sid]:
                self._update_health(sid)
                continue

            sensor_position = np.asarray(cfg.position_enu_m, dtype=float)

            for target_id, truth in truths.items():
                self.generated[sid] += 1
                sensor_observations = radar.observe(truth)

                if not sensor_observations:
                    continue

                for observation in sensor_observations:
                    if observation.is_clutter:
                        continue

                    self.received[sid] += 1

                    z = np.array(
                        [
                            observation.range_m + self.bias_m[sid],
                            observation.azimuth_rad,
                            observation.elevation_rad,
                        ],
                        dtype=float,
                    )

                    if self.noise_scale[sid] > 1.0:
                        extra_scale = self.noise_scale[sid] - 1.0
                        z += np.array(
                            [
                                self.control_rng.normal(
                                    0.0,
                                    cfg.range_sigma_m * extra_scale,
                                ),
                                self.control_rng.normal(
                                    0.0,
                                    cfg.azimuth_sigma_rad * extra_scale,
                                ),
                                self.control_rng.normal(
                                    0.0,
                                    cfg.elevation_sigma_rad * extra_scale,
                                ),
                            ]
                        )

                    # Plot the actual measured point, not the filtered state.
                    raw_position = (
                        sensor_position
                        + spherical_to_cartesian(
                            float(z[0]),
                            float(z[1]),
                            float(z[2]),
                        )
                    )
                    observations.append(
                        {
                            "sensorId": sid,
                            "targetId": target_id,
                            "position": [
                                float(raw_position[0]),
                                float(raw_position[1]),
                            ],
                        }
                    )

                    target = self.targets[target_id]

                    if target.filter is None:
                        initial_observation = type(
                            "Observation",
                            (),
                            {
                                "range_m": float(z[0]),
                                "azimuth_rad": float(z[1]),
                                "elevation_rad": float(z[2]),
                            },
                        )()
                        x0 = first_measurement_state(
                            initial_observation,
                            sensor_position,
                        )
                        p0 = np.diag(
                            [400, 400, 400, 2500, 2500, 2500]
                        ).astype(float)
                        target.filter = self._filter_class()(x0, p0, 3.0)
                        continue

                    measurement_h = H(target.filter.x, sensor_position)
                    innovation = z - h(target.filter.x, sensor_position)
                    innovation[1:] = (
                        innovation[1:] + math.pi
                    ) % (2.0 * math.pi) - math.pi

                    predicted_measurement_cov = (
                        measurement_h
                        @ target.filter.p
                        @ measurement_h.T
                    )

                    configured_r = obs_cov(cfg)
                    effective_r = configured_r * self.r_est[sid]

                    target.filter.update(
                        z,
                        effective_r,
                        sensor_position,
                    )

                    self._adapt_sensor_noise(
                        sid,
                        innovation,
                        predicted_measurement_cov,
                        configured_r,
                    )

            self._update_health(sid)

        tracks: list[dict] = []

        for target_id, target in sorted(self.targets.items()):
            if target.filter is None:
                continue

            truth = truths[target_id]
            error = truth.vector() - target.filter.x

            try:
                nees = float(
                    error @ np.linalg.solve(target.filter.p, error)
                )
            except np.linalg.LinAlgError:
                nees = 0.0

            target.trail.append(
                [
                    float(target.filter.x[0]),
                    float(target.filter.x[1]),
                ]
            )
            del target.trail[:-300]

            p = target.filter.p
            tracks.append(
                {
                    "id": target_id,
                    "label": f"TRK-{target_id:03d}",
                    "position": [
                        float(target.filter.x[0]),
                        float(target.filter.x[1]),
                        float(target.filter.x[2]),
                    ],
                    "velocity": [
                        float(target.filter.x[3]),
                        float(target.filter.x[4]),
                        float(target.filter.x[5]),
                    ],
                    "covariance2d": [
                        float(p[0, 0]),
                        float(p[0, 1]),
                        float(p[1, 1]),
                    ],
                    "nees": nees,
                    "status": "CONFIRMED",
                    "filter": self.filter_name,
                    "trail": target.trail,
                }
            )

            if target_id == min(self.targets):
                self.nees_history.append(
                    {
                        "timeSec": self.t,
                        "value": nees,
                    }
                )
                del self.nees_history[:-180]

        sensors: list[dict] = []
        total_generated = 0
        total_received = 0

        for cfg in self.configs:
            sid = cfg.sensor_id
            total_generated += self.generated[sid]
            total_received += self.received[sid]

            packet_loss = (
                0.0
                if self.generated[sid] == 0
                else 100.0
                * (
                    1.0
                    - self.received[sid] / self.generated[sid]
                )
            )

            sensors.append(
                {
                    "id": sid,
                    "label": f"RADAR {chr(64 + sid)}",
                    "position": list(cfg.position_enu_m),
                    "health": self.health[sid],
                    "configuredR": 1.0,
                    "estimatedR": self.r_est[sid],
                    "packetsReceived": self.received[sid],
                    "packetLossPct": packet_loss,
                    "biasActive": self.bias_m[sid] != 0.0,
                    "noiseActive": self.noise_scale[sid] > 1.0,
                }
            )

        total_loss = (
            0.0
            if total_generated == 0
            else 100.0
            * (1.0 - total_received / total_generated)
        )

        live_update_ms = (
            time.perf_counter() - started
        ) * 1000.0

        state = {
            "timestampSec": self.t,
            "connected": True,
            "tracks": tracks,
            "sensors": sensors,
            "observations": observations,
            "neesHistory": self.nees_history,
            "metrics": {
                "observationsPerSecond": len(observations) / DT,
                "activeTracks": len(tracks),
                "liveUpdateMs": live_update_ms,
                "benchAssociationP99Ms": BENCH_ASSOCIATION_P99_MS,
                "packetLossPct": total_loss,
            },
            "message": self.message,
        }

        self.t += DT
        return state


simulation = OperatorSimulation()

clients: set[asyncio.Queue[str]] = set()
latest_payload: str | None = None


async def client_handler(websocket) -> None:
    queue: asyncio.Queue[str] = asyncio.Queue(maxsize=2)
    clients.add(queue)

    print(
        f"[ws] client connected; subscribers={len(clients)}",
        flush=True,
    )

    if latest_payload is not None:
        queue.put_nowait(latest_payload)

    async def sender() -> None:
        while True:
            payload = await queue.get()
            await websocket.send(payload)

    async def receiver() -> None:
        async for raw in websocket:
            simulation.command(json.loads(raw))

    sender_task = asyncio.create_task(sender())
    receiver_task = asyncio.create_task(receiver())

    try:
        done, pending = await asyncio.wait(
            {sender_task, receiver_task},
            return_when=asyncio.FIRST_COMPLETED,
        )

        for task in done:
            if task.cancelled():
                continue

            exc = task.exception()

            if exc is None:
                continue

            if isinstance(
                exc,
                websockets.exceptions.ConnectionClosedOK,
            ):
                continue

            raise exc

        for task in pending:
            task.cancel()

        await asyncio.gather(
            *pending,
            return_exceptions=True,
        )
    finally:
        sender_task.cancel()
        receiver_task.cancel()

        await asyncio.gather(
            sender_task,
            receiver_task,
            return_exceptions=True,
        )

        clients.discard(queue)

        print(
            f"[ws] client disconnected; subscribers={len(clients)}",
            flush=True,
        )


async def broadcast_loop() -> None:
    global latest_payload

    while True:
        started = time.perf_counter()

        state = simulation.step()
        latest_payload = json.dumps(state)

        for queue in list(clients):
            if queue.full():
                try:
                    queue.get_nowait()
                except asyncio.QueueEmpty:
                    pass

            try:
                queue.put_nowait(latest_payload)
            except asyncio.QueueFull:
                pass

        elapsed = time.perf_counter() - started
        await asyncio.sleep(max(0.0, DT - elapsed))


async def main() -> None:
    print(
        f"Aurora operator state stream: ws://{HOST}:{PORT}",
        flush=True,
    )

    async with websockets.serve(
        client_handler,
        HOST,
        PORT,
    ):
        await broadcast_loop()


if __name__ == "__main__":
    asyncio.run(main())


