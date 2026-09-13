from __future__ import annotations

import asyncio
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from simulation.operator_simulation import OperatorSimulation
from simulation.operator_server import main


if __name__ == "__main__":
    asyncio.run(main())
