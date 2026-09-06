from __future__ import annotations

import csv
from pathlib import Path
import re
import subprocess
import sys


ROOT = Path(__file__).resolve().parents[1]
RESULTS = ROOT / "experiments" / "results"


def exe(name: str) -> Path:
    suffix = ".exe" if sys.platform.startswith("win") else ""
    candidates = [
        ROOT / "build" / "Debug" / f"{name}{suffix}",
        ROOT / "build" / f"{name}{suffix}",
    ]
    for candidate in candidates:
        if candidate.exists():
            return candidate
    raise FileNotFoundError(
        f"could not find benchmark executable {name}"
    )


def capture(name: str) -> str:
    completed = subprocess.run(
        [str(exe(name))],
        cwd=ROOT,
        text=True,
        capture_output=True,
        check=True,
    )
    return completed.stdout.strip()


def parse_association(text: str) -> list[dict]:
    lines = [
        line.strip()
        for line in text.splitlines()
        if line.strip()
    ]
    rows = []
    for line in lines[1:]:
        tracks, iterations, p50, p95, p99 = line.split(",")
        rows.append({
            "tracks": int(tracks),
            "iterations": int(iterations),
            "p50_us": float(p50),
            "p95_us": float(p95),
            "p99_us": float(p99),
        })
    return rows


def parse_key_values(text: str) -> dict[str, float]:
    return {
        key: float(value)
        for key, value in re.findall(
            r"(\w+)=([-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?)",
            text,
        )
    }


def write(name: str, rows: list[dict]) -> None:
    path = RESULTS / name
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(
            handle,
            fieldnames=list(rows[0]),
        )
        writer.writeheader()
        writer.writerows(rows)
    print(f"wrote {path}")


def main() -> None:
    RESULTS.mkdir(parents=True, exist_ok=True)

    association = parse_association(
        capture("bench_association")
    )
    write("association_benchmark.csv", association)

    target_rows = [
        row
        for row in association
        if row["tracks"] in {1, 3, 5, 10}
    ]
    write("target_count.csv", target_rows)

    backpressure = parse_key_values(
        capture("bench_backpressure")
    )
    write("backpressure_benchmark.csv", [backpressure])

    ingestion = parse_key_values(
        capture("bench_ingestion")
    )
    write("ingestion_benchmark.csv", [ingestion])


if __name__ == "__main__":
    main()
