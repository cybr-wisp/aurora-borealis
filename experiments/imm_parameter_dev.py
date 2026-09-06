from __future__ import annotations

import json
from pathlib import Path

import numpy as np

import experiments.imm_dev as imm


RESULTS = Path("experiments/results")
RESULTS.mkdir(parents=True, exist_ok=True)

# Development only.
# FINAL 20264000-20264049 remain untouched.
P01_GRID = [
    0.001,
    0.003,
    0.005,
    0.010,
]

P10_GRID = [
    0.02,
    0.05,
    0.10,
    0.20,
]

MANEUVER_Q_GRID = [
    10.0,
    30.0,
    100.0,
]


def evaluate_scenario(
    truth,
    maneuver_window=None,
):
    runs = [
        imm.run_imm(
            truth,
            imm.degraded_cfg,
            seed=seed,
            maneuver_window=maneuver_window,
        )
        for seed in imm.DEV_SEEDS
    ]

    return imm.summarize(runs)


def score(turn, maneuver):
    total = 0.0

    for s in (turn, maneuver):
        # Main statistical target.
        total += abs(
            s["nees_coverage_pct"] - 95.0
        )

        # 6D NEES expectation is 6.
        total += 0.35 * abs(
            s["nees_mean"] - 6.0
        )

        # Preserve the hard tracking target.
        if s["rmse_p95_m"] >= 5.0:
            total += (
                200.0
                + 50.0
                * (s["rmse_p95_m"] - 5.0)
            )

        if s["rmse_max_m"] >= 5.0:
            total += (
                250.0
                + 75.0
                * (s["rmse_max_m"] - 5.0)
            )

        if s["runs_below_5m_pct"] < 100.0:
            total += (
                300.0
                + 2.0
                * (
                    100.0
                    - s["runs_below_5m_pct"]
                )
            )

    return float(total)


rows = []

total_configs = (
    len(P01_GRID)
    * len(P10_GRID)
    * len(MANEUVER_Q_GRID)
)

print("=== IMM DEVELOPMENT SWEEP ===")
print("configs:", total_configs)
print(
    "development seeds:",
    imm.DEV_SEEDS,
)
print(
    "FINAL seeds 20264000-20264049 remain UNUSED"
)
print()

count = 0

for maneuver_q in MANEUVER_Q_GRID:
    for p01 in P01_GRID:
        for p10 in P10_GRID:
            count += 1

            imm.MANEUVER_JERK_Q = maneuver_q

            imm.TRANSITION = np.array([
                [1.0 - p01, p01],
                [p10, 1.0 - p10],
            ], dtype=float)

            turn = evaluate_scenario(
                imm.turn
            )

            maneuver = evaluate_scenario(
                imm.maneuver,
                maneuver_window=(
                    20.0,
                    25.0,
                ),
            )

            s = score(
                turn,
                maneuver,
            )

            row = {
                "maneuver_jerk_q": maneuver_q,
                "p_smooth_to_maneuver": p01,
                "p_maneuver_to_smooth": p10,
                "score": s,
                "turn": turn,
                "maneuver": maneuver,
            }

            rows.append(row)

            print(
                f"[{count:02d}/{total_configs}] "
                f"MQ={maneuver_q:>5.1f} "
                f"p01={p01:.3f} "
                f"p10={p10:.2f} | "
                f"score={s:6.2f} | "
                f"TURN "
                f"cov={turn['nees_coverage_pct']:5.2f}% "
                f"p95={turn['rmse_p95_m']:.3f} "
                f"max={turn['rmse_max_m']:.3f} | "
                f"MAN "
                f"cov={maneuver['nees_coverage_pct']:5.2f}% "
                f"p95={maneuver['rmse_p95_m']:.3f} "
                f"max={maneuver['rmse_max_m']:.3f} "
                f"eventP="
                f"{maneuver['maneuver_mode_prob_during_event']:.3f}"
            )


rows.sort(
    key=lambda row: row["score"]
)

output = {
    "protocol": {
        "purpose": (
            "development-only IMM parameter selection"
        ),
        "development_seeds": imm.DEV_SEEDS,
        "future_validation_seeds": (
            "20264000-20264049 — NOT USED"
        ),
        "smooth_jerk_q": imm.SMOOTH_JERK_Q,
        "maneuver_q_grid": MANEUVER_Q_GRID,
        "p01_grid": P01_GRID,
        "p10_grid": P10_GRID,
        "initial_mode_probs": (
            imm.INITIAL_MODE_PROBS.tolist()
        ),
    },
    "best": rows[0],
    "top_10": rows[:10],
}

out = (
    RESULTS
    / "imm_parameter_dev_search.json"
)

out.write_text(
    json.dumps(
        output,
        indent=2,
    ),
    encoding="utf-8",
)


print()
print("=" * 84)
print("TOP 10 IMM CONFIGS")
print("=" * 84)

for i, row in enumerate(
    rows[:10],
    1,
):
    print()
    print(
        f"#{i} "
        f"score={row['score']:.3f} "
        f"MQ={row['maneuver_jerk_q']} "
        f"p01={row['p_smooth_to_maneuver']} "
        f"p10={row['p_maneuver_to_smooth']}"
    )

    t = row["turn"]
    m = row["maneuver"]

    print(
        " TURN: "
        f"cov={t['nees_coverage_pct']:.2f}% "
        f"below={t['nees_below_pct']:.2f}% "
        f"above={t['nees_above_pct']:.2f}% "
        f"mean={t['nees_mean']:.2f} "
        f"p95={t['rmse_p95_m']:.3f} "
        f"max={t['rmse_max_m']:.3f} "
        f"modeP={t['maneuver_mode_prob_mean']:.3f}"
    )

    print(
        " MAN:  "
        f"cov={m['nees_coverage_pct']:.2f}% "
        f"below={m['nees_below_pct']:.2f}% "
        f"above={m['nees_above_pct']:.2f}% "
        f"mean={m['nees_mean']:.2f} "
        f"p95={m['rmse_p95_m']:.3f} "
        f"max={m['rmse_max_m']:.3f} "
        f"eventP="
        f"{m['maneuver_mode_prob_during_event']:.3f}"
    )

print()
print("wrote", out)
