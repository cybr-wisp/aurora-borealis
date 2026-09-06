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

from experiments.ca_ekf_dev import CAEKF

from simulation.sim.sensors.radar import RadarConfig

from simulation.sim.trajectory import (
    coordinated_turn,
    acceleration_maneuver,
)


RESULTS = Path("experiments/results")
RESULTS.mkdir(parents=True, exist_ok=True)

# DEVELOPMENT ONLY.
DEV_SEEDS = list(range(20263000, 20263020))

SMOOTH_JERK_Q = 0.1
MANEUVER_JERK_Q = 10.0
P_ACC = 100.0

# Transition matrix:
#
# rows = current mode
# cols = next mode
#
# mode 0 = smooth
# mode 1 = maneuver
#
TRANSITION = np.array([
    [0.995, 0.005],
    [0.050, 0.950],
], dtype=float)

INITIAL_MODE_PROBS = np.array(
    [0.98, 0.02],
    dtype=float,
)


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


def measurement_jacobian_9(
    x: np.ndarray,
    sensor: np.ndarray,
) -> np.ndarray:
    hh = np.zeros((3, 9))
    hh[:, :6] = H(x[:6], sensor)
    return hh


def mix_models(
    models: list[CAEKF],
    mode_probs: np.ndarray,
) -> np.ndarray:
    # Predicted probability of each destination mode.
    predicted_probs = TRANSITION.T @ mode_probs

    mixed_x = []
    mixed_p = []

    for j in range(2):
        denom = predicted_probs[j]

        if denom <= 0.0:
            raise RuntimeError(
                "invalid IMM predicted mode probability"
            )

        weights = (
            mode_probs * TRANSITION[:, j] / denom
        )

        x_bar = sum(
            weights[i] * models[i].x
            for i in range(2)
        )

        p_bar = np.zeros((9, 9))

        for i in range(2):
            delta = models[i].x - x_bar

            p_bar += weights[i] * (
                models[i].p
                + np.outer(delta, delta)
            )

        mixed_x.append(x_bar)
        mixed_p.append(
            0.5 * (p_bar + p_bar.T)
        )

    for j in range(2):
        models[j].x = mixed_x[j]
        models[j].p = mixed_p[j]

    return predicted_probs


def update_model(
    model: CAEKF,
    z: np.ndarray,
    r_cov: np.ndarray,
    sensor: np.ndarray,
) -> tuple[float, float]:
    hh = measurement_jacobian_9(
        model.x,
        sensor,
    )

    predicted_z = h(
        model.x[:6],
        sensor,
    )

    innovation = residual(
        z,
        predicted_z,
    )

    s = (
        hh @ model.p @ hh.T
        + r_cov
    )

    nis = float(
        innovation
        @ np.linalg.solve(
            s,
            innovation,
        )
    )

    sign, logdet = np.linalg.slogdet(s)

    if sign <= 0:
        raise RuntimeError(
            "innovation covariance is not positive definite"
        )

    log_likelihood = -0.5 * (
        nis
        + logdet
        + 3.0 * math.log(2.0 * math.pi)
    )

    k = np.linalg.solve(
        s,
        hh @ model.p,
    ).T

    model.x = (
        model.x
        + k @ innovation
    )

    ident = np.eye(9)
    ikh = ident - k @ hh

    # Joseph form covariance update.
    model.p = (
        ikh @ model.p @ ikh.T
        + k @ r_cov @ k.T
    )

    model.p = (
        0.5
        * (model.p + model.p.T)
    )

    return nis, log_likelihood


def normalize_log_weights(
    log_weights: np.ndarray,
) -> np.ndarray:
    max_log = float(np.max(log_weights))

    weights = np.exp(
        log_weights - max_log
    )

    weights /= np.sum(weights)

    # Avoid exact mode extinction.
    weights = np.maximum(
        weights,
        1e-8,
    )

    weights /= np.sum(weights)

    return weights


def combine_models(
    models: list[CAEKF],
    mode_probs: np.ndarray,
    within_covariance_scale: float = 1.0,
) -> tuple[np.ndarray, np.ndarray]:
    x = sum(
        mode_probs[i] * models[i].x
        for i in range(2)
    )

    p = np.zeros((9, 9))

    for i in range(2):
        delta = models[i].x - x

        # Calibrate only each model's conditional covariance.
        #
        # The between-model disagreement term is deliberately
        # preserved because it represents genuine IMM model
        # uncertainty, especially during maneuver transitions.
        p += mode_probs[i] * (
            within_covariance_scale * models[i].p
            + np.outer(delta, delta)
        )

    p = 0.5 * (p + p.T)

    return x, p


def run_imm(
    truth,
    configs,
    seed: int,
    maneuver_window: tuple[float, float] | None = None,
    covariance_scale: float = 1.0,
):
    observations, _ = build_observations(
        truth,
        configs,
        seed=seed,
    )

    first_t = min(observations)

    first_obs, first_cfg = (
        observations[first_t][0]
    )

    x6 = first_measurement_state(
        first_obs,
        np.asarray(
            first_cfg.position_enu_m
        ),
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
        P_ACC,
        P_ACC,
        P_ACC,
    ]).astype(float)

    models = [
        CAEKF(
            x9,
            p9,
            SMOOTH_JERK_Q,
        ),
        CAEKF(
            x9,
            p9,
            MANEUVER_JERK_Q,
        ),
    ]

    mode_probs = (
        INITIAL_MODE_PROBS.copy()
    )

    previous_t = first_t

    sq_pos = []
    nees_vals = []
    nees_times = []
    nis_vals = []
    nis_times = []

    maneuver_probs = []
    maneuver_window_probs = []

    for state in truth:
        t = state.timestamp_sec

        if (
            t < first_t
            or t not in observations
        ):
            continue

        if t > first_t:
            # Interacting/mixing step.
            mode_probs = mix_models(
                models,
                mode_probs,
            )

            dt = t - previous_t

            for model in models:
                model.predict(dt)

        previous_t = t

        # Sequentially assimilate all radar
        # measurements at this timestamp.
        for obs, cfg in observations[t]:
            z = np.array([
                obs.range_m,
                obs.azimuth_rad,
                obs.elevation_rad,
            ])

            r_cov = obs_cov(cfg)

            sensor = np.asarray(
                cfg.position_enu_m
            )

            log_likelihoods = []
            measurement_nis = []

            for model in models:
                nis, log_likelihood = (
                    update_model(
                        model,
                        z,
                        r_cov,
                        sensor,
                    )
                )

                measurement_nis.append(nis)
                log_likelihoods.append(
                    log_likelihood
                )

            log_weights = (
                np.log(mode_probs)
                + np.asarray(
                    log_likelihoods,
                    dtype=float,
                )
            )

            mode_probs = (
                normalize_log_weights(
                    log_weights
                )
            )

            nis_vals.append(
                float(
                    np.sum(
                        mode_probs
                        * np.asarray(
                            measurement_nis
                        )
                    )
                )
            )

            nis_times.append(float(t))

        combined_x, combined_p = (
            combine_models(
                models,
                mode_probs,
                within_covariance_scale=covariance_scale,
            )
        )

        maneuver_probs.append(
            float(mode_probs[1])
        )

        if (
            maneuver_window is not None
            and maneuver_window[0]
            <= t
            <= maneuver_window[1]
        ):
            maneuver_window_probs.append(
                float(mode_probs[1])
            )

        if t < 5.0:
            continue

        truth6 = state.vector()

        error6 = (
            truth6
            - combined_x[:6]
        )

        # Calibration has already been applied only to the
        # within-model covariance during IMM combination.
        # The state estimate and the internal EKFs remain untouched.
        p6 = combined_p[:6, :6]

        sq_pos.append(
            float(
                error6[:3]
                @ error6[:3]
            )
        )

        nees_vals.append(
            float(
                error6
                @ np.linalg.solve(
                    p6,
                    error6,
                )
            )
        )

        nees_times.append(float(t))

    nees_array = np.asarray(
        nees_vals,
        dtype=float,
    )

    lo, hi = NEES_BOUNDS

    nees_times_array = np.asarray(
        nees_times,
        dtype=float,
    )

    def interval_stats(start, end):
        mask = (
            (nees_times_array >= start)
            & (nees_times_array < end)
        )

        values = nees_array[mask]

        if len(values) == 0:
            return None

        return {
            "samples": int(len(values)),
            "coverage_pct": float(
                100.0
                * np.mean(
                    (values >= lo)
                    & (values <= hi)
                )
            ),
            "below_pct": float(
                100.0
                * np.mean(values < lo)
            ),
            "above_pct": float(
                100.0
                * np.mean(values > hi)
            ),
            "mean_nees": float(
                np.mean(values)
            ),
        }

    nis_array = np.asarray(
        nis_vals,
        dtype=float,
    )

    nis_times_array = np.asarray(
        nis_times,
        dtype=float,
    )

    def nis_interval_stats(start, end):
        mask = (
            (nis_times_array >= start)
            & (nis_times_array < end)
        )

        values = nis_array[mask]

        if len(values) == 0:
            return None

        return {
            "samples": int(len(values)),
            "mean_nis": float(np.mean(values)),
            "p95_nis": float(
                np.percentile(values, 95)
            ),
            "above_7_815_pct": float(
                100.0
                * np.mean(values > 7.815)
            ),
        }

    result = {
        "rmse_m": math.sqrt(
            float(
                np.mean(sq_pos)
            )
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
            * float(
                np.mean(
                    nees_array < lo
                )
            )
        ),
        "nees_above_pct": (
            100.0
            * float(
                np.mean(
                    nees_array > hi
                )
            )
        ),
        "nees_mean": float(
            np.mean(nees_array)
        ),
        "nis_mean": float(
            np.mean(nis_vals)
        ),
        "maneuver_mode_prob_mean": float(
            np.mean(maneuver_probs)
        ),
        "maneuver_mode_prob_max": float(
            np.max(maneuver_probs)
        ),
        "nis_by_time": {
            "5_to_10_sec": nis_interval_stats(
                5.0,
                10.0,
            ),
            "10_to_20_sec": nis_interval_stats(
                10.0,
                20.0,
            ),
            "20_to_30_sec": nis_interval_stats(
                20.0,
                30.0,
            ),
            "30_to_60_sec": nis_interval_stats(
                30.0,
                60.1,
            ),
        },
        "nees_by_time": {
            "5_to_10_sec": interval_stats(
                5.0,
                10.0,
            ),
            "10_to_20_sec": interval_stats(
                10.0,
                20.0,
            ),
            "20_to_30_sec": interval_stats(
                20.0,
                30.0,
            ),
            "30_to_60_sec": interval_stats(
                30.0,
                60.1,
            ),
        },
    }

    if maneuver_window_probs:
        result[
            "maneuver_mode_prob_during_event"
        ] = float(
            np.mean(
                maneuver_window_probs
            )
        )

    return result


def evaluate(
    truth,
    maneuver_window=None,
):
    return [
        run_imm(
            truth,
            degraded_cfg,
            seed=seed,
            maneuver_window=maneuver_window,
        )
        for seed in DEV_SEEDS
    ]


def summarize(runs):
    def avg(key):
        return float(
            np.mean([
                r[key]
                for r in runs
            ])
        )

    rmse = np.asarray(
        [
            r["rmse_m"]
            for r in runs
        ],
        dtype=float,
    )

    output = {
        "runs": len(runs),
        "rmse_mean_m": float(
            np.mean(rmse)
        ),
        "rmse_p95_m": float(
            np.percentile(
                rmse,
                95,
            )
        ),
        "rmse_max_m": float(
            np.max(rmse)
        ),
        "runs_below_5m_pct": float(
            100.0
            * np.mean(rmse < 5.0)
        ),
        "nees_coverage_pct": avg(
            "nees_coverage_pct"
        ),
        "nees_below_pct": avg(
            "nees_below_pct"
        ),
        "nees_above_pct": avg(
            "nees_above_pct"
        ),
        "nees_mean": avg(
            "nees_mean"
        ),
        "nis_mean": avg(
            "nis_mean"
        ),
        "maneuver_mode_prob_mean": avg(
            "maneuver_mode_prob_mean"
        ),
        "maneuver_mode_prob_max": avg(
            "maneuver_mode_prob_max"
        ),
    }

    if (
        "maneuver_mode_prob_during_event"
        in runs[0]
    ):
        output[
            "maneuver_mode_prob_during_event"
        ] = avg(
            "maneuver_mode_prob_during_event"
        )

    return output


def main():
    print(
        "=== TWO-MODE IMM DEVELOPMENT TEST ==="
    )
    print(
        "smooth jerkQ:",
        SMOOTH_JERK_Q,
    )
    print(
        "maneuver jerkQ:",
        MANEUVER_JERK_Q,
    )
    print(
        "development seeds:",
        DEV_SEEDS,
    )
    print(
        "FINAL seeds 20264000-20264049 remain UNUSED"
    )
    print()

    turn_result = summarize(
        evaluate(turn)
    )

    maneuver_result = summarize(
        evaluate(
            maneuver,
            maneuver_window=(
                20.0,
                25.0,
            ),
        )
    )

    output = {
        "protocol": {
            "model": (
                "two-mode interacting "
                "multiple-model CA EKF"
            ),
            "development_seeds": (
                DEV_SEEDS
            ),
            "future_validation_seeds": (
                "20264000-20264049 — NOT USED"
            ),
            "smooth_jerk_q": (
                SMOOTH_JERK_Q
            ),
            "maneuver_jerk_q": (
                MANEUVER_JERK_Q
            ),
            "transition_matrix": (
                TRANSITION.tolist()
            ),
            "initial_mode_probs": (
                INITIAL_MODE_PROBS.tolist()
            ),
            "p_acc": P_ACC,
            "detection_probability": 0.70,
            "packet_loss_probability": 0.10,
            "radars": 3,
            "nees_bounds": list(
                NEES_BOUNDS
            ),
        },
        "coordinated_turn": (
            turn_result
        ),
        "abrupt_maneuver": (
            maneuver_result
        ),
    }

    out = (
        RESULTS
        / "imm_dev_baseline.json"
    )

    out.write_text(
        json.dumps(
            output,
            indent=2,
        ),
        encoding="utf-8",
    )

    print("COORDINATED TURN")
    print(
        json.dumps(
            turn_result,
            indent=2,
        )
    )

    print()
    print("ABRUPT MANEUVER")
    print(
        json.dumps(
            maneuver_result,
            indent=2,
        )
    )

    print()
    print("wrote", out)


if __name__ == "__main__":
    main()
