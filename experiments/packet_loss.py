from __future__ import annotations

from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from experiments.sweeps import packet_loss_sweep

if __name__ == "__main__":
    rows = packet_loss_sweep()
    print("packet-loss sweep rows=" + str(len(rows)))
