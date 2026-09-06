from __future__ import annotations

import csv
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from experiments.tracking_benchmark import (
    DT,
    EKF,
    UKF,
    recovery_time,
    run_filter,
)
from simulation.sim.sensors.radar import RadarConfig
from simulation.sim.trajectory import (
    acceleration_maneuver,
    coordinated_turn,
)


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


def main() -> None:
    turn = coordinated_turn(
        target_id=1,
        position0_m=(1200, 350, 300),
        speed_mps=78,
        heading0_rad=0.15,
        turn_rate_radps=0.045,
        vertical_speed_mps=0.5,
        duration_sec=60,
        dt_sec=DT,
    )
    maneuver = acceleration_maneuver(
        target_id=1,
        position0_m=(1200, 350, 300),
        velocity0_mps=(75, 12, 0.5),
        acceleration_mps2=(-7, 10, 0),
        maneuver_start_sec=20,
        maneuver_end_sec=25,
        duration_sec=60,
        dt_sec=DT,
    )

    rows = []

    for label, cls in [("EKF", EKF), ("UKF", UKF)]:
        result = run_filter(cls, turn, configs(), q=12.0)
        rows.append({
            "scenario": "coordinated_turn",
            "filter": label,
            "adaptive_q": False,
            "rmse_m": result["rmse_m"],
            "nees_in_95_pct": result["nees_in_95_pct"],
            "recovery_sec": "",
        })

    for adaptive in [False, True]:
        result = run_filter(
            EKF,
            maneuver,
            configs(),
            q=2.5,
            adaptive=adaptive,
        )
        rows.append({
            "scenario": "abrupt_maneuver",
            "filter": "EKF",
            "adaptive_q": adaptive,
            "rmse_m": result["rmse_m"],
            "nees_in_95_pct": result["nees_in_95_pct"],
            "recovery_sec": recovery_time(
                result["position_errors_m"],
                25.0,
            ),
        })

    RESULTS.mkdir(parents=True, exist_ok=True)
    path = RESULTS / "maneuver_comparison.csv"
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)

    print(f"wrote {path}")


if __name__ == "__main__":
    main()
