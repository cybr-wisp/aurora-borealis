from __future__ import annotations

import csv
from dataclasses import dataclass
import math
from pathlib import Path
import random


ROOT = Path(__file__).resolve().parents[1]
RESULTS = ROOT / "experiments" / "results"
FIGURES = ROOT / "docs" / "figures"


@dataclass
class Estimator:
    window_size: int = 50
    min_samples: int = 15
    alpha: float = 0.05
    min_scale: float = 0.25
    max_scale: float = 25.0

    def __post_init__(self) -> None:
        self.samples: list[tuple[float, float]] = []
        self.current_r = 1.0

    def observe(
        self,
        innovation: float,
        predicted_variance: float,
        configured_r: float,
    ) -> None:
        self.samples.append(
            (innovation * innovation, predicted_variance)
        )
        self.samples = self.samples[-self.window_size :]

        if len(self.samples) < self.min_samples:
            return

        raw = (
            sum(item[0] for item in self.samples) / len(self.samples)
            - sum(item[1] for item in self.samples) / len(self.samples)
        )
        raw = max(
            configured_r * self.min_scale,
            min(raw, configured_r * self.max_scale),
        )
        self.current_r = (
            (1.0 - self.alpha) * self.current_r
            + self.alpha * raw
        )


def noise_scale_at(t: float) -> float:
    return 9.0 if 20.0 <= t < 40.0 else 1.0


def write_svg(rows: list[dict[str, float]]) -> None:
    width, height = 900, 360
    left, right, top, bottom = 55, 20, 25, 45
    x0, x1 = rows[0]["time_sec"], rows[-1]["time_sec"]
    y_max = 10.5

    def sx(x: float) -> float:
        return left + (x - x0) / (x1 - x0) * (width - left - right)

    def sy(y: float) -> float:
        return top + (y_max - y) / y_max * (height - top - bottom)

    actual = " ".join(
        f"{sx(r['time_sec']):.1f},{sy(r['actual_r_ratio']):.1f}"
        for r in rows
    )
    estimated = " ".join(
        f"{sx(r['time_sec']):.1f},{sy(r['estimated_r_ratio']):.1f}"
        for r in rows
    )

    svg = f"""<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}">
<rect width="100%" height="100%" fill="white"/>
<line x1="{left}" y1="{height-bottom}" x2="{width-right}" y2="{height-bottom}" stroke="black"/>
<line x1="{left}" y1="{top}" x2="{left}" y2="{height-bottom}" stroke="black"/>
<polyline points="{actual}" fill="none" stroke="black" stroke-width="2"/>
<polyline points="{estimated}" fill="none" stroke="#666" stroke-width="2.5"/>
<text x="{width/2}" y="{height-10}" text-anchor="middle" font-family="sans-serif" font-size="13">time (s)</text>
<text x="15" y="{height/2}" transform="rotate(-90 15 {height/2})" text-anchor="middle" font-family="sans-serif" font-size="13">R / configured R</text>
<text x="{left+10}" y="{top+18}" font-family="sans-serif" font-size="12">actual: black | adaptive estimate: gray</text>
</svg>
"""
    FIGURES.mkdir(parents=True, exist_ok=True)
    (FIGURES / "adaptive_noise_response.svg").write_text(
        svg,
        encoding="utf-8",
    )


def main() -> None:
    rng = random.Random(20260905)
    estimator = Estimator()
    configured_r = 1.0
    predicted_variance = 0.5
    rows: list[dict[str, float]] = []
    detection_time: float | None = None

    for step in range(600):
        t = step * 0.1
        actual_ratio = noise_scale_at(t)
        innovation = rng.gauss(
            0.0,
            math.sqrt(
                predicted_variance + configured_r * actual_ratio
            ),
        )

        estimator.observe(
            innovation,
            predicted_variance,
            configured_r,
        )

        estimated_ratio = estimator.current_r / configured_r

        if (
            detection_time is None
            and t >= 20.0
            and estimated_ratio >= 2.0
        ):
            detection_time = t

        rows.append(
            {
                "time_sec": t,
                "actual_r_ratio": actual_ratio,
                "estimated_r_ratio": estimated_ratio,
                "nis_configured": (
                    innovation * innovation
                    / (predicted_variance + configured_r)
                ),
                "nis_adapted": (
                    innovation * innovation
                    / (predicted_variance + estimator.current_r)
                ),
            }
        )

    RESULTS.mkdir(parents=True, exist_ok=True)
    csv_path = RESULTS / "adaptive_noise_response.csv"

    with csv_path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)

    write_svg(rows)

    degraded = [
        r for r in rows if 25.0 <= r["time_sec"] < 40.0
    ]
    recovered = [
        r for r in rows if 50.0 <= r["time_sec"] < 60.0
    ]

    configured_nis = sum(r["nis_configured"] for r in degraded) / len(degraded)
    adapted_nis = sum(r["nis_adapted"] for r in degraded) / len(degraded)
    recovered_ratio = sum(
        r["estimated_r_ratio"] for r in recovered
    ) / len(recovered)

    delay = (
        detection_time - 20.0
        if detection_time is not None
        else float("nan")
    )

    print(
        "adaptive_noise_eval "
        f"detection_delay_sec={delay:.2f} "
        f"degraded_mean_nis_configured={configured_nis:.3f} "
        f"degraded_mean_nis_adapted={adapted_nis:.3f} "
        f"recovered_mean_r_ratio={recovered_ratio:.3f}"
    )


if __name__ == "__main__":
    main()
