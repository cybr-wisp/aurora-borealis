from __future__ import annotations

from collections.abc import Iterable
from dataclasses import dataclass
import random
from typing import TypeVar


T = TypeVar("T")


@dataclass(frozen=True)
class FailureInjectorConfig:
    """Transport impairments applied after sensor measurement generation."""

    reorder_probability: float = 0.0

    def __post_init__(self) -> None:
        if not 0.0 <= self.reorder_probability <= 1.0:
            raise ValueError("reorder_probability must be in [0, 1]")


class FailureInjector:
    """Deterministic transport-layer packet reordering."""

    def __init__(self, config: FailureInjectorConfig, seed: int) -> None:
        self.config = config
        self._rng = random.Random(seed)

    def apply(self, observations: Iterable[T]) -> list[T]:
        items = list(observations)
        output: list[T] = []

        i = 0
        while i < len(items):
            can_swap = i + 1 < len(items)

            if (
                can_swap
                and self._rng.random() < self.config.reorder_probability
            ):
                output.append(items[i + 1])
                output.append(items[i])
                i += 2
            else:
                output.append(items[i])
                i += 1

        return output
