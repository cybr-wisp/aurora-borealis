from __future__ import annotations

import json
import math
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

import numpy as np

from experiments.tracking_benchmark import (
    DT,
    NEES_BOUNDS,
    build_observations,
    first_measurement_state,
    obs_cov,
    h,
    H,
    residual,
)
from simulation.sim.sensors.radar import RadarConfig
from simulation.sim.trajectory import (
    coordinated_turn,
    acceleration_maneuver,
)


RESULTS = Path("experiments/results")
RESULTS.mkdir(parents=True, exist_ok=True)

# Development seeds only.
DEV_SEEDS = list(range(20263000, 20263020))

# White-noise jerk spectral density.
JERK_Q_GRID = [
    0.1,
    0.3,
    1.0,
    3.0,
    10.0,
    30.0,
    100.0,
]

# Initial acceleration-state variance.
P_ACC_GRID = [
    25.0,
    100.0,
    400.0,
]


nominal = dict(
    range_sigma_m=10.0,
    azimuth_sigma_rad=0.0025,
    elevation_sigma_rad=0.0025,
)

degraded_cfg = [
    RadarConfig(
        sensor_id=1,
        position_enu_m=(0, 0, 0),
        detection_probability=0.70,
        packet_loss_probability=0.10,
        **nominal,
    ),
    RadarConfig(
        sensor_id=2,
        position_enu_m=(2500, -800, 50),
        detection_probability=0.70,
        packet_loss_probability=0.10,
        **nominal,
    ),
    RadarConfig(
        sensor_id=3,
        position_enu_m=(-1800, 1400, 80),
        detection_probability=0.70,
        packet_loss_probability=0.10,
        **nominal,
    ),
]


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


def transition_ca(dt: float) -> np.ndarray:
    f = np.eye(9)

    # position <- velocity
    f[0:3, 3:6] = np.eye(3) * dt

    # position <- acceleration
    f[0:3, 6:9] = np.eye(3) * (0.5 * dt * dt)

    # velocity <- acceleration
    f[3:6, 6:9] = np.eye(3) * dt

    return f


def process_noise_ca(
    dt: float,
    jerk_q: float,
) -> np.ndarray:
    q = np.zeros((9, 9))

    block = jerk_q * np.array([
        [
            dt**5 / 20.0,
            dt**4 / 8.0,
            dt**3 / 6.0,
        ],
        [
            dt**4 / 8.0,
            dt**3 / 3.0,
            dt**2 / 2.0,
        ],
        [
            dt**3 / 6.0,
            dt**2 / 2.0,
            dt,
        ],
    ])

    for axis in range(3):
        idx = [axis, axis + 3, axis + 6]
        q[np.ix_(idx, idx)] = block

    return q


class CAEKF:
    def __init__(
        self,
        x: np.ndarray,
        p: np.ndarray,
        jerk_q: float,
    ):
        self.x = x.copy()
        self.p = p.copy()
        self.jerk_q = jerk_q

    def predict(self, dt: float):
        f = transition_ca(dt)

        self.x = f @ self.x

        self.p = (
            f @ self.p @ f.T
            + process_noise_ca(dt, self.jerk_q)
        )

        self.p = 0.5 * (self.p + self.p.T)

    def update(
        self,
        z: np.ndarray,
        r_cov: np.ndarray,
        sensor: np.ndarray,
    ) -> float:
        # Existing radar model is defined for the first six
        # position/velocity states. Extend its Jacobian with
        # zero acceleration columns.
        h6 = H(self.x[:6], sensor)

        hh = np.zeros((3, 9))
        hh[:, :6] = h6

        predicted = h(self.x[:6], sensor)
        innovation = residual(z, predicted)

        s = hh @ self.p @ hh.T + r_cov

        k = np.linalg.solve(
            s,
            hh @ self.p,
        ).T

        self.x = self.x + k @ innovation

        ident = np.eye(9)
        ikh = ident - k @ hh

        # Joseph covariance update.
        self.p = (
            ikh @ self.p @ ikh.T
            + k @ r_cov @ k.T
        )

        self.p = 0.5 * (self.p + self.p.T)

        return float(
            innovation
            @ np.linalg.solve(s, innovation)
        )


def run_ca(
    truth,
    configs,
    jerk_q: float,
    p_acc: float,
    seed: int,
):
    observations, _ = build_observations(
        truth,
        configs,
        seed=seed,
    )

    first_t = min(observations)

    # build_observations stores (observation, config) tuples.
    first_obs, first_cfg = observations[first_t][0]

    x6 = first_measurement_state(
        first_obs,
        np.asarray(first_cfg.position_enu_m),
    )

    x9 = np.r_[
        x6,
        [0.0, 0.0, 0.0],
    ]

    p9 = np.diag([
        400.0,
        400.0,
        400.0,
        2500.0,
        2500.0,
        2500.0,
        p_acc,
        p_acc,
        p_acc,
    ])

    filt = CAEKF(
        x9,
        p9,
        jerk_q,
    )

    sq_pos = []
    nees_vals = []
    nis_vals = []

    previous_t = first_t

    for state in truth:
        t = state.timestamp_sec

        if t < first_t or t not in observations:
            continue

        if t > first_t:
            filt.predict(t - previous_t)

        previous_t = t

        for obs, cfg in observations[t]:
            z = np.array([
                obs.range_m,
                obs.azimuth_rad,
                obs.elevation_rad,
            ])

            nis = filt.update(
                z,
                obs_cov(cfg),
                np.asarray(cfg.position_enu_m),
            )

            nis_vals.append(nis)

        if t < 5.0:
            continue

        truth6 = state.vector()

        error6 = truth6 - filt.x[:6]

        # Marginal 6D position/velocity covariance.
        p6 = filt.p[:6, :6]

        sq_pos.append(
            float(error6[:3] @ error6[:3])
        )

        nees_vals.append(
            float(
                error6
                @ np.linalg.solve(p6, error6)
            )
        )

    lo, hi = NEES_BOUNDS

    nees_array = np.asarray(nees_vals, dtype=float)

    return {
        "rmse_m": math.sqrt(
            float(np.mean(sq_pos))
        ),
        "nees_coverage_pct": (
            100.0
            * float(
                np.mean(
                    (nees_array >= lo)
                    & (nees_array <= hi)
                )
            )
        ),
        "nees_below_pct": (
            100.0
            * float(np.mean(nees_array < lo))
        ),
        "nees_above_pct": (
            100.0
            * float(np.mean(nees_array > hi))
        ),
        "nees_mean": float(
            np.mean(nees_array)
        ),
        "nis_mean": float(
            np.mean(nis_vals)
        ),
    }


def evaluate(
    truth,
    jerk_q: float,
    p_acc: float,
):
    return [
        run_ca(
            truth,
            degraded_cfg,
            jerk_q=jerk_q,
            p_acc=p_acc,
            seed=seed,
        )
        for seed in DEV_SEEDS
    ]


def summarize(runs):
    rmse = np.array(
        [r["rmse_m"] for r in runs],
        dtype=float,
    )

    coverage = np.array(
        [r["nees_coverage_pct"] for r in runs],
        dtype=float,
    )

    nees = np.array(
        [r["nees_mean"] for r in runs],
        dtype=float,
    )

    nis = np.array(
        [r["nis_mean"] for r in runs],
        dtype=float,
    )

    return {
        "runs": len(runs),
        "rmse_mean_m": float(rmse.mean()),
        "rmse_p95_m": float(
            np.percentile(rmse, 95)
        ),
        "rmse_max_m": float(rmse.max()),
        "runs_below_5m_pct": float(
            100.0 * np.mean(rmse < 5.0)
        ),
        "nees_coverage_pct": float(
            coverage.mean()
        ),
        "nees_mean": float(
            nees.mean()
        ),
        "nis_mean": float(
            nis.mean()
        ),
    }


def score(turn_s, man_s):
    total = 0.0

    for s in (turn_s, man_s):
        total += abs(
            s["nees_coverage_pct"] - 95.0
        )

        total += (
            0.5
            * abs(s["nees_mean"] - 6.0)
        )

        if s["rmse_p95_m"] >= 5.0:
            total += (
                150.0
                + 30.0
                * (s["rmse_p95_m"] - 5.0)
            )

        if s["rmse_max_m"] >= 5.0:
            total += (
                200.0
                + 40.0
                * (s["rmse_max_m"] - 5.0)
            )

        if s["runs_below_5m_pct"] < 100.0:
            total += 200.0

    return float(total)


def main():
    rows = []

    total = (
        len(JERK_Q_GRID)
        * len(P_ACC_GRID)
    )

    print("=== 9-STATE CONSTANT-ACCELERATION EKF DEV SEARCH ===")
    print("development seeds:", DEV_SEEDS)
    print("configs:", total)
    print("FINAL validation seeds 20264000-20264049 remain UNUSED")
    print()

    count = 0

    for jerk_q in JERK_Q_GRID:
        for p_acc in P_ACC_GRID:
            count += 1

            turn_s = summarize(
                evaluate(
                    turn,
                    jerk_q,
                    p_acc,
                )
            )

            man_s = summarize(
                evaluate(
                    maneuver,
                    jerk_q,
                    p_acc,
                )
            )

            s = score(
                turn_s,
                man_s,
            )

            row = {
                "jerk_q": jerk_q,
                "p_acc": p_acc,
                "score": s,
                "turn": turn_s,
                "maneuver": man_s,
            }

            rows.append(row)

            print(
                f"[{count:02d}/{total}] "
                f"jerkQ={jerk_q:>6.1f} "
                f"Pacc={p_acc:>6.1f} | "
                f"score={s:7.2f} | "
                f"TURN cov={turn_s['nees_coverage_pct']:6.2f}% "
                f"p95={turn_s['rmse_p95_m']:.3f} "
                f"max={turn_s['rmse_max_m']:.3f} | "
                f"MAN cov={man_s['nees_coverage_pct']:6.2f}% "
                f"p95={man_s['rmse_p95_m']:.3f} "
                f"max={man_s['rmse_max_m']:.3f}"
            )


    rows.sort(
        key=lambda row: row["score"]
    )

    output = {
        "protocol": {
            "model": "9-state constant-acceleration EKF",
            "development_seeds": DEV_SEEDS,
            "future_validation_seeds": (
                "20264000-20264049 — NOT USED"
            ),
            "jerk_q_grid": JERK_Q_GRID,
            "p_acc_grid": P_ACC_GRID,
            "detection_probability": 0.70,
            "packet_loss_probability": 0.10,
            "radars": 3,
            "nees": (
                "6D marginal position+velocity NEES"
            ),
            "nees_bounds": list(
                NEES_BOUNDS
            ),
        },
        "best": rows[0],
        "top_10": rows[:10],
    }


    out = RESULTS / "ca_ekf_dev_search.json"

    out.write_text(
        json.dumps(
            output,
            indent=2,
        ),
        encoding="utf-8",
    )


    print()
    print("=" * 80)
    print("TOP CONSTANT-ACCELERATION CONFIGS")
    print("=" * 80)

    for i, row in enumerate(
        rows[:10],
        1,
    ):
        print(
            f"\n#{i} "
            f"score={row['score']:.3f} "
            f"jerkQ={row['jerk_q']} "
            f"Pacc={row['p_acc']}"
        )

        print(
            " TURN: "
            f"coverage={row['turn']['nees_coverage_pct']:.2f}% "
            f"meanNEES={row['turn']['nees_mean']:.2f} "
            f"p95={row['turn']['rmse_p95_m']:.3f} "
            f"max={row['turn']['rmse_max_m']:.3f}"
        )

        print(
            " MAN:  "
            f"coverage={row['maneuver']['nees_coverage_pct']:.2f}% "
            f"meanNEES={row['maneuver']['nees_mean']:.2f} "
            f"p95={row['maneuver']['rmse_p95_m']:.3f} "
            f"max={row['maneuver']['rmse_max_m']:.3f}"
        )

    print()
    print("wrote", out)



if __name__ == "__main__":
    main()
