from __future__ import annotations

import csv
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

import numpy as np

from experiments.tracking_benchmark import (
    DT,
    EKF,
    run_filter,
)
from simulation.sim.sensors.radar import RadarConfig
from simulation.sim.trajectory import constant_velocity


RESULTS = ROOT / "experiments" / "results"
RESULTS.mkdir(parents=True, exist_ok=True)

POSITIONS = [
    (0.0, 0.0, 0.0),
    (2500.0, -800.0, 50.0),
    (-1800.0, 1400.0, 80.0),
    (900.0, 2200.0, 40.0),
]


def truth():
    return constant_velocity(
        target_id=1,
        position0_m=(1200, 350, 300),
        velocity_mps=(75, 12, 0.5),
        duration_sec=60,
        dt_sec=DT,
    )


def configs(
    count: int,
    range_sigma: float = 10.0,
    packet_loss: float = 0.0,
):
    return [
        RadarConfig(
            sensor_id=i + 1,
            position_enu_m=POSITIONS[i],
            range_sigma_m=range_sigma,
            azimuth_sigma_rad=0.0025,
            elevation_sigma_rad=0.0025,
            detection_probability=0.98,
            packet_loss_probability=packet_loss,
        )
        for i in range(count)
    ]


def write_csv(name: str, rows: list[dict]) -> Path:
    path = RESULTS / name
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    return path


def noise_sweep() -> list[dict]:
    rows = []
    for sigma in [5.0, 10.0, 20.0, 30.0, 50.0]:
        result = run_filter(
            EKF,
            truth(),
            configs(3, range_sigma=sigma),
            q=0.02,
        )
        rows.append({
            "range_sigma_m": sigma,
            "rmse_m": result["rmse_m"],
            "raw_rmse_m": result["raw_rmse_m"],
            "nees_in_95_pct": result["nees_in_95_pct"],
        })
    write_csv("noise_sweep.csv", rows)
    return rows


def sensor_count_sweep() -> list[dict]:
    rows = []
    for count in [1, 2, 3, 4]:
        result = run_filter(
            EKF,
            truth(),
            configs(count),
            q=0.02,
        )
        rows.append({
            "sensor_count": count,
            "rmse_m": result["rmse_m"],
            "rmse_improvement_pct": result["rmse_improvement_pct"],
            "nees_in_95_pct": result["nees_in_95_pct"],
        })
    write_csv("sensor_count.csv", rows)
    return rows


def packet_loss_sweep() -> list[dict]:
    rows = []
    for loss in [0.0, 0.10, 0.20, 0.30, 0.40]:
        runs = []
        for seed in range(20260900, 20260910):
            runs.append(
                run_filter(
                    EKF,
                    truth(),
                    configs(3, packet_loss=loss),
                    q=0.02,
                    seed=seed,
                )
            )

        rmse = np.array([r["rmse_m"] for r in runs])
        consistency = np.array(
            [r["nees_in_95_pct"] for r in runs]
        )

        rows.append({
            "packet_loss_pct": 100.0 * loss,
            "rmse_mean_m": float(rmse.mean()),
            "rmse_p95_m": float(np.percentile(rmse, 95)),
            "nees_in_95_mean_pct": float(consistency.mean()),
            "runs_below_5m_pct": float(np.mean(rmse < 5.0) * 100.0),
        })

    write_csv("packet_loss.csv", rows)
    return rows


def main() -> None:
    summary = {
        "noise_sweep": noise_sweep(),
        "sensor_count": sensor_count_sweep(),
        "packet_loss": packet_loss_sweep(),
    }
    path = RESULTS / "sweeps_summary.json"
    path.write_text(json.dumps(summary, indent=2), encoding="utf-8")
    print(f"wrote {path}")


if __name__ == "__main__":
    main()
