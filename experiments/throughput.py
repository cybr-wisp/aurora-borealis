from __future__ import annotations

from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from experiments.system_benchmarks import (
    capture,
    parse_key_values,
    write,
)


if __name__ == "__main__":
    write(
        "ingestion_benchmark.csv",
        [parse_key_values(capture("bench_ingestion"))],
    )
