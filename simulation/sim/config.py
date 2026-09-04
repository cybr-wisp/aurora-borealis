from __future__ import annotations

from pathlib import Path
import yaml


def load_yaml(path: str | Path) -> dict:
    with Path(path).open("r", encoding="utf-8") as handle:
        data = yaml.safe_load(handle)
    if not isinstance(data, dict):
        raise ValueError("scenario YAML must contain a mapping")
    return data
