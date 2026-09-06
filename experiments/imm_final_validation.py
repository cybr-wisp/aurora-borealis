from __future__ import annotations

import json
import subprocess
from pathlib import Path

import numpy as np

import experiments.imm_dev as imm


RESULTS = Path("experiments/results")
RESULTS.mkdir(parents=True, exist_ok=True)

# ============================================================
# FROZEN FINAL PROTOCOL
# ============================================================

FINAL_SEEDS = list(range(20264000, 20264050))

SMOOTH_JERK_Q = 0.1
MANEUVER_JERK_Q = 100.0

P01 = 0.01
P10 = 0.10

INITIAL_MODE_PROBS = np.array(
    [0.98, 0.02],
    dtype=float,
)

P_ACC = 100.0

WITHIN_COVARIANCE_SCALE = 0.75


# Apply frozen parameters.
imm.SMOOTH_JERK_Q = SMOOTH_JERK_Q
imm.MANEUVER_JERK_Q = MANEUVER_JERK_Q
imm.P_ACC = P_ACC

imm.INITIAL_MODE_PROBS = (
    INITIAL_MODE_PROBS.copy()
)

imm.TRANSITION = np.array([
    [1.0 - P01, P01],
    [P10, 1.0 - P10],
], dtype=float)


def git_head() -> str:
    try:
        return subprocess.check_output(
            ["git", "rev-parse", "HEAD"],
            text=True,
        ).strip()
    except Exception:
        return "unknown"


def evaluate(
    truth,
    maneuver_window=None,
):
    runs = [
        imm.run_imm(
            truth,
            imm.degraded_cfg,
            seed=seed,
            maneuver_window=maneuver_window,
            covariance_scale=WITHIN_COVARIANCE_SCALE,
        )
        for seed in FINAL_SEEDS
    ]

    return runs, imm.summarize(runs)


def checks(summary):
    return {
        "p95_rmse_below_5m":
            summary["rmse_p95_m"] < 5.0,

        "max_rmse_below_5m":
            summary["rmse_max_m"] < 5.0,

        "all_runs_below_5m":
            summary["runs_below_5m_pct"] == 100.0,

        # Pre-registered aspirational consistency band.
        "nees_coverage_93_to_97_pct":
            93.0
            <= summary["nees_coverage_pct"]
            <= 97.0,
    }


print("=" * 80)
print("AURORA BOREALIS — FINAL HELD-OUT VALIDATION")
print("=" * 80)

print()
print("THIS EVALUATION USES PREVIOUSLY UNTOUCHED SEEDS.")
print("No parameter selection may follow from these results.")
print()

print("seeds:", FINAL_SEEDS[0], "through", FINAL_SEEDS[-1])
print("runs per scenario:", len(FINAL_SEEDS))
print("total scenario-runs:", len(FINAL_SEEDS) * 2)
print()

print("FROZEN CONFIG")
print(" smooth jerk Q:       ", SMOOTH_JERK_Q)
print(" maneuver jerk Q:     ", MANEUVER_JERK_Q)
print(" smooth -> maneuver:  ", P01)
print(" maneuver -> smooth:  ", P10)
print(" initial probabilities:", INITIAL_MODE_PROBS.tolist())
print(" acceleration P:      ", P_ACC)
print(" within-cov scale:    ", WITHIN_COVARIANCE_SCALE)
print()


turn_runs, turn = evaluate(
    imm.turn,
)

maneuver_runs, maneuver = evaluate(
    imm.maneuver,
    maneuver_window=(20.0, 25.0),
)


turn_checks = checks(turn)
maneuver_checks = checks(maneuver)


output = {
    "protocol": {
        "status": "FINAL HELD-OUT — DO NOT TUNE",
        "git_commit": git_head(),
        "seeds": FINAL_SEEDS,
        "runs_per_scenario": len(FINAL_SEEDS),
        "total_scenario_runs": len(FINAL_SEEDS) * 2,
        "conditions": {
            "radars": 3,
            "detection_probability": 0.70,
            "missed_detection_probability": 0.30,
            "packet_loss_probability": 0.10,
        },
        "estimator": {
            "type": "two-mode IMM with 9-state CA EKFs",
            "smooth_jerk_q": SMOOTH_JERK_Q,
            "maneuver_jerk_q": MANEUVER_JERK_Q,
            "p_smooth_to_maneuver": P01,
            "p_maneuver_to_smooth": P10,
            "initial_mode_probabilities":
                INITIAL_MODE_PROBS.tolist(),
            "initial_acceleration_variance": P_ACC,
            "within_model_covariance_scale":
                WITHIN_COVARIANCE_SCALE,
            "between_model_covariance_scaled": False,
        },
        "evaluation": {
            "nees_dimension": 6,
            "nees_bounds_95": list(
                imm.NEES_BOUNDS
            ),
            "consistency_target_pct": [
                93.0,
                97.0,
            ],
        },
    },

    "coordinated_turn": {
        "summary": turn,
        "checks": turn_checks,
        "per_seed": [
            {
                "seed": seed,
                **result,
            }
            for seed, result
            in zip(FINAL_SEEDS, turn_runs)
        ],
    },

    "abrupt_maneuver": {
        "summary": maneuver,
        "checks": maneuver_checks,
        "per_seed": [
            {
                "seed": seed,
                **result,
            }
            for seed, result
            in zip(FINAL_SEEDS, maneuver_runs)
        ],
    },
}


out = (
    RESULTS
    / "imm_final_heldout_validation.json"
)

out.write_text(
    json.dumps(
        output,
        indent=2,
    ),
    encoding="utf-8",
)


def print_result(
    name,
    summary,
    result_checks,
):
    print()
    print("=" * 80)
    print(name)
    print("=" * 80)

    print(
        f"runs:                    "
        f"{summary['runs']}"
    )

    print(
        f"RMSE mean:               "
        f"{summary['rmse_mean_m']:.4f} m"
    )

    print(
        f"RMSE p95:                "
        f"{summary['rmse_p95_m']:.4f} m"
    )

    print(
        f"RMSE max:                "
        f"{summary['rmse_max_m']:.4f} m"
    )

    print(
        f"runs below 5 m:          "
        f"{summary['runs_below_5m_pct']:.2f}%"
    )

    print(
        f"NEES coverage:           "
        f"{summary['nees_coverage_pct']:.2f}%"
    )

    print(
        f"NEES below lower bound:  "
        f"{summary['nees_below_pct']:.2f}%"
    )

    print(
        f"NEES above upper bound:  "
        f"{summary['nees_above_pct']:.2f}%"
    )

    print(
        f"mean 6D NEES:            "
        f"{summary['nees_mean']:.3f}"
    )

    print(
        f"mean NIS:                "
        f"{summary['nis_mean']:.3f}"
    )

    print()
    print("PRE-REGISTERED CHECKS")

    for key, passed in result_checks.items():
        print(
            f"  {'PASS' if passed else 'FAIL':4s} "
            f"{key}"
        )


print_result(
    "COORDINATED TURN",
    turn,
    turn_checks,
)

print_result(
    "ABRUPT MANEUVER",
    maneuver,
    maneuver_checks,
)

print()
print("=" * 80)
print("FINAL VALIDATION COMPLETE")
print("=" * 80)

print()
print("wrote:", out)
print()
print(
    "These seeds are now SPENT. "
    "Do not tune the estimator against this result."
)
