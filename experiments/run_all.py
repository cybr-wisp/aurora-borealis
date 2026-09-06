from __future__ import annotations

from pathlib import Path
import subprocess
import sys
import time


ROOT = Path(__file__).resolve().parents[1]

STEPS = [
    [sys.executable, "experiments/tracking_benchmark.py"],
    [sys.executable, "experiments/sweeps.py"],
    [sys.executable, "experiments/maneuver_comparison.py"],
    [sys.executable, "experiments/sensor_outage.py"],
    [sys.executable, "experiments/crossing_phase_diagram.py"],
    [sys.executable, "experiments/adaptive_noise_eval.py"],
    [sys.executable, "experiments/system_benchmarks.py"],
    [sys.executable, "experiments/plots/generate_figures.py"],
]


def main() -> None:
    started = time.perf_counter()

    for command in STEPS:
        print("+", " ".join(command))
        subprocess.run(command, cwd=ROOT, check=True)

    elapsed = time.perf_counter() - started
    print(f"complete elapsed_sec={elapsed:.3f}")


if __name__ == "__main__":
    main()
