from __future__ import annotations

from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from experiments.system_benchmarks import (
    capture,
    parse_association,
    write,
)


if __name__ == "__main__":
    rows = [
        row
        for row in parse_association(
            capture("bench_association")
        )
        if row["tracks"] in {1, 3, 5, 10}
    ]
    write("target_count.csv", rows)
