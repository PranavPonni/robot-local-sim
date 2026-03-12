"""IO utilities for configuration and trajectory files."""
from __future__ import annotations

import json
from pathlib import Path
from typing import Any


def save_json(path: str, data: Any) -> None:
    with open(path, "w", encoding="utf-8") as f:
        json.dump(data, f, indent=2)


def load_json(path: str) -> Any:
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)


def safe_load_json(path: str) -> dict:
    p = Path(path)
    if not p.exists():
        raise FileNotFoundError(f"JSON file not found: {path}")
    content = load_json(path)
    if not isinstance(content, dict):
        raise ValueError("JSON top-level object must be a dictionary")
    return content
