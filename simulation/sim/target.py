from __future__ import annotations

from dataclasses import dataclass
import numpy as np


@dataclass(frozen=True)
class TargetState:
    target_id: int
    timestamp_sec: float
    position_enu_m: np.ndarray
    velocity_enu_mps: np.ndarray

    def vector(self) -> np.ndarray:
        return np.concatenate((self.position_enu_m, self.velocity_enu_mps)).astype(float)
