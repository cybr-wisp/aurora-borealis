from __future__ import annotations

import csv
from dataclasses import dataclass
from pathlib import Path

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
RESULTS = ROOT / "experiments" / "results"
FIGURES = ROOT / "docs" / "figures"

GATE_95_2D = 5.991464547107979
DT = 0.1
DURATION = 10.0
SPEED = 20.0


@dataclass
class CvTrack:
    identity: int
    x: np.ndarray
    p: np.ndarray


def matrices(dt: float, accel_spectral_density: float = 4.0):
    f = np.array(
        [
            [1.0, 0.0, dt, 0.0],
            [0.0, 1.0, 0.0, dt],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ],
        dtype=float,
    )

    q1 = np.array(
        [
            [dt**3 / 3.0, dt**2 / 2.0],
            [dt**2 / 2.0, dt],
        ],
        dtype=float,
    ) * accel_spectral_density

    q = np.zeros((4, 4), dtype=float)
    q[np.ix_([0, 2], [0, 2])] = q1
    q[np.ix_([1, 3], [1, 3])] = q1

    h = np.array(
        [
            [1.0, 0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0, 0.0],
        ],
        dtype=float,
    )

    return f, q, h


def truth_positions(t: float, separation_m: float):
    x_a = -100.0 + SPEED * t
    x_b = 100.0 - SPEED * t
    half = separation_m / 2.0
    return {
        0: np.array([x_a, -half], dtype=float),
        1: np.array([x_b, half], dtype=float),
    }


def two_by_two_assignment(costs: np.ndarray):
    candidates = []

    direct = costs[0, 0] + costs[1, 1]
    if np.isfinite(direct):
        candidates.append((direct, (0, 1)))

    crossed = costs[0, 1] + costs[1, 0]
    if np.isfinite(crossed):
        candidates.append((crossed, (1, 0)))

    if not candidates:
        return (-1, -1)

    return min(candidates, key=lambda item: item[0])[1]


def update(track: CvTrack, measurement: np.ndarray, r: np.ndarray, h: np.ndarray):
    innovation = measurement - h @ track.x
    s = h @ track.p @ h.T + r
    k = track.p @ h.T @ np.linalg.inv(s)

    track.x = track.x + k @ innovation

    identity = np.eye(4)
    ikh = identity - k @ h
    track.p = ikh @ track.p @ ikh.T + k @ r @ k.T
    track.p = 0.5 * (track.p + track.p.T)


def run_trial(separation_m: float, noise_sigma_m: float, seed: int) -> bool:
    rng = np.random.default_rng(seed)
    f, q, h = matrices(DT)
    r = np.eye(2) * noise_sigma_m**2

    t0_truth = truth_positions(0.0, separation_m)

    tracks = []
    for identity in (0, 1):
        measured = t0_truth[identity] + rng.normal(0.0, noise_sigma_m, 2)
        velocity = np.array(
            [SPEED if identity == 0 else -SPEED, 0.0],
            dtype=float,
        )
        state = np.array(
            [measured[0], measured[1], velocity[0], velocity[1]],
            dtype=float,
        )
        p = np.diag(
            [
                noise_sigma_m**2,
                noise_sigma_m**2,
                25.0,
                25.0,
            ]
        )
        tracks.append(CvTrack(identity, state, p))

    identity_failure = False

    steps = int(round(DURATION / DT))
    for step in range(1, steps + 1):
        t = step * DT
        truth = truth_positions(t, separation_m)

        measurements = [
            (identity, truth[identity] + rng.normal(0.0, noise_sigma_m, 2))
            for identity in (0, 1)
        ]
        rng.shuffle(measurements)

        for track in tracks:
            track.x = f @ track.x
            track.p = f @ track.p @ f.T + q

        costs = np.full((2, 2), np.inf, dtype=float)

        for row, track in enumerate(tracks):
            predicted = h @ track.x
            s = h @ track.p @ h.T + r
            s_inv = np.linalg.inv(s)

            for col, (_, measured) in enumerate(measurements):
                innovation = measured - predicted
                d2 = float(innovation.T @ s_inv @ innovation)
                if d2 <= GATE_95_2D:
                    costs[row, col] = d2

        assignment = two_by_two_assignment(costs)

        for row, col in enumerate(assignment):
            if col < 0 or not np.isfinite(costs[row, col]):
                continue

            target_id, measured = measurements[col]

            if target_id != tracks[row].identity:
                identity_failure = True

            update(tracks[row], measured, r, h)

    return identity_failure


def sweep():
    separations = [50.0, 30.0, 20.0, 10.0, 5.0]
    noise_levels = [2.0, 5.0, 10.0, 15.0]
    trials = 100

    rows = []

    for noise in noise_levels:
        for separation in separations:
            failures = 0

            for trial in range(trials):
                seed = (
                    20260905
                    + int(separation * 100)
                    + int(noise * 1000)
                    + trial
                )
                failures += int(
                    run_trial(separation, noise, seed)
                )

            rows.append(
                {
                    "separation_m": separation,
                    "noise_sigma_m": noise,
                    "trials": trials,
                    "runs_with_id_switch": failures,
                    "id_switch_run_pct": 100.0 * failures / trials,
                }
            )

    return rows


def write_csv(rows):
    RESULTS.mkdir(parents=True, exist_ok=True)
    path = RESULTS / "crossing_phase_diagram.csv"

    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(
            handle,
            fieldnames=list(rows[0].keys()),
        )
        writer.writeheader()
        writer.writerows(rows)

    return path


def svg_heatmap(rows):
    FIGURES.mkdir(parents=True, exist_ok=True)

    separations = sorted(
        {row["separation_m"] for row in rows},
        reverse=True,
    )
    noises = sorted({row["noise_sigma_m"] for row in rows})

    lookup = {
        (row["separation_m"], row["noise_sigma_m"]):
        row["id_switch_run_pct"]
        for row in rows
    }

    cell_w = 115
    cell_h = 72
    left = 120
    top = 80
    width = left + cell_w * len(separations) + 30
    height = top + cell_h * len(noises) + 80

    def shade(value):
        lightness = 96.0 - 58.0 * (value / 100.0)
        return f"hsl(0 0% {lightness:.1f}%)"

    parts = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">',
        '<rect width="100%" height="100%" fill="white"/>',
        '<text x="20" y="30" font-family="Arial, sans-serif" font-size="20" font-weight="700">Aurora Borealis — crossing-target association failure boundary</text>',
        '<text x="20" y="55" font-family="Arial, sans-serif" font-size="12">Cell value = percentage of deterministic trials with at least one identity switch</text>',
    ]

    for col, separation in enumerate(separations):
        x = left + col * cell_w + cell_w / 2
        parts.append(
            f'<text x="{x}" y="{top - 12}" text-anchor="middle" font-family="Arial, sans-serif" font-size="12">{separation:g} m</text>'
        )

    for row_idx, noise in enumerate(noises):
        y = top + row_idx * cell_h
        parts.append(
            f'<text x="{left - 12}" y="{y + cell_h / 2 + 4}" text-anchor="end" font-family="Arial, sans-serif" font-size="12">σ={noise:g} m</text>'
        )

        for col, separation in enumerate(separations):
            value = lookup[(separation, noise)]
            x = left + col * cell_w
            parts.append(
                f'<rect x="{x}" y="{y}" width="{cell_w}" height="{cell_h}" fill="{shade(value)}" stroke="#777"/>'
            )
            parts.append(
                f'<text x="{x + cell_w / 2}" y="{y + cell_h / 2 + 5}" text-anchor="middle" font-family="Arial, sans-serif" font-size="16" font-weight="700">{value:.0f}%</text>'
            )

    parts.append(
        f'<text x="{left + cell_w * len(separations) / 2}" y="{height - 20}" text-anchor="middle" font-family="Arial, sans-serif" font-size="12">minimum crossing separation</text>'
    )
    parts.append("</svg>")

    path = FIGURES / "crossing_phase_diagram.svg"
    path.write_text("\n".join(parts), encoding="utf-8")
    return path


if __name__ == "__main__":
    rows = sweep()
    csv_path = write_csv(rows)
    svg_path = svg_heatmap(rows)

    print(
        "separation_m,noise_sigma_m,trials,"
        "runs_with_id_switch,id_switch_run_pct"
    )
    for row in rows:
        print(
            f'{row["separation_m"]:.1f},'
            f'{row["noise_sigma_m"]:.1f},'
            f'{row["trials"]},'
            f'{row["runs_with_id_switch"]},'
            f'{row["id_switch_run_pct"]:.1f}'
        )

    print(f"wrote {csv_path}")
    print(f"wrote {svg_path}")
