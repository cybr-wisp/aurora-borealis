from __future__ import annotations

import csv
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

import numpy as np

from experiments.tracking_benchmark import (
    DT,
    EKF,
    build_observations,
    first_measurement_state,
    obs_cov,
)
from simulation.sim.sensors.radar import RadarConfig
from simulation.sim.trajectory import constant_velocity


RESULTS = ROOT / "experiments" / "results"


def configs():
    nominal = dict(
        range_sigma_m=10.0,
        azimuth_sigma_rad=0.0025,
        elevation_sigma_rad=0.0025,
    )
    return [
        RadarConfig(sensor_id=1, position_enu_m=(0, 0, 0), **nominal),
        RadarConfig(sensor_id=2, position_enu_m=(2500, -800, 50), **nominal),
        RadarConfig(sensor_id=3, position_enu_m=(-1800, 1400, 80), **nominal),
    ]


def run_outage(duration_sec: float) -> dict:
    truth = constant_velocity(
        target_id=1,
        position0_m=(1200, 350, 300),
        velocity_mps=(75, 12, 0.5),
        duration_sec=70,
        dt_sec=DT,
    )
    observations, _ = build_observations(truth, configs())

    first_t = min(observations)
    o0, c0 = observations[first_t][0]
    x0 = first_measurement_state(
        o0,
        np.asarray(c0.position_enu_m),
    )
    p0 = np.diag(
        [400, 400, 400, 2500, 2500, 2500]
    ).astype(float)

    filt = EKF(x0, p0, 0.02)

    outage_start = 20.0
    outage_end = outage_start + duration_sec
    previous_t = first_t
    baseline_trace = None
    max_trace = 0.0
    recovery_time = None
    stable = 0

    for state in truth:
        t = state.timestamp_sec
        if t < first_t or t not in observations:
            continue

        if t > first_t:
            filt.predict(t - previous_t)
        previous_t = t

        if 18.0 <= t < 20.0:
            baseline_trace = float(np.trace(filt.p[:3, :3]))

        for obs, cfg in observations[t]:
            if (
                cfg.sensor_id == 2
                and outage_start <= t < outage_end
            ):
                continue

            z = np.array(
                [obs.range_m, obs.azimuth_rad, obs.elevation_rad]
            )
            filt.update(
                z,
                obs_cov(cfg),
                np.asarray(cfg.position_enu_m),
            )

        trace = float(np.trace(filt.p[:3, :3]))
        if outage_start <= t <= outage_end:
            max_trace = max(max_trace, trace)

        if t >= outage_end and recovery_time is None:
            error = float(
                np.linalg.norm(state.vector()[:3] - filt.x[:3])
            )
            if error < 5.0:
                stable += 1
                if stable >= 10:
                    recovery_time = round(
                        t - outage_end - 0.9,
                        2,
                    )
            else:
                stable = 0

    baseline_trace = baseline_trace or max_trace
    return {
        "outage_sec": duration_sec,
        "max_position_cov_trace_m2": max_trace,
        "covariance_growth_ratio": (
            max_trace / baseline_trace
            if baseline_trace > 0.0
            else float("nan")
        ),
        "reacquisition_sec": (
            recovery_time
            if recovery_time is not None
            else ""
        ),
    }


def main() -> None:
    rows = [run_outage(v) for v in [5.0, 10.0, 30.0]]

    RESULTS.mkdir(parents=True, exist_ok=True)
    path = RESULTS / "sensor_outage.csv"
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)

    print(f"wrote {path}")


if __name__ == "__main__":
    main()
