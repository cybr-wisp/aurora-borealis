from __future__ import annotations

import csv
from pathlib import Path

import matplotlib.pyplot as plt


ROOT = Path(__file__).resolve().parents[2]
RESULTS = ROOT / "experiments" / "results"
FIGURES = ROOT / "docs" / "figures"
FIGURES.mkdir(parents=True, exist_ok=True)


def read(name: str) -> list[dict[str, str]]:
    with (RESULTS / name).open(encoding="utf-8") as handle:
        return list(csv.DictReader(handle))


def save_line(
    rows: list[dict[str, str]],
    x_key: str,
    y_key: str,
    xlabel: str,
    ylabel: str,
    filename: str,
) -> None:
    x = [float(row[x_key]) for row in rows]
    y = [float(row[y_key]) for row in rows]

    plt.figure(figsize=(7.5, 4.5))
    plt.plot(x, y, marker="o")
    plt.xlabel(xlabel)
    plt.ylabel(ylabel)
    plt.grid(alpha=0.25)
    plt.tight_layout()
    plt.savefig(FIGURES / filename, dpi=170)
    plt.close()


def main() -> None:
    save_line(
        read("noise_sweep.csv"),
        "range_sigma_m",
        "rmse_m",
        "Range noise sigma (m)",
        "Tracking RMSE (m)",
        "noise_sweep.png",
    )
    save_line(
        read("sensor_count.csv"),
        "sensor_count",
        "rmse_m",
        "Sensor count",
        "Tracking RMSE (m)",
        "sensor_count.png",
    )
    save_line(
        read("packet_loss.csv"),
        "packet_loss_pct",
        "rmse_mean_m",
        "Packet loss (%)",
        "Mean tracking RMSE (m)",
        "packet_loss.png",
    )
    save_line(
        read("sensor_outage.csv"),
        "outage_sec",
        "covariance_growth_ratio",
        "Sensor outage (s)",
        "Covariance growth ratio",
        "sensor_outage.png",
    )
    save_line(
        read("association_benchmark.csv"),
        "tracks",
        "p99_us",
        "Track count",
        "Association p99 (us)",
        "association_scaling.png",
    )

    print(f"wrote figures to {FIGURES}")


if __name__ == "__main__":
    main()
