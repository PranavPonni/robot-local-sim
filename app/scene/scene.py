"""Scene object and world model."""
from __future__ import annotations

import numpy as np
from dataclasses import dataclass
from typing import List


@dataclass
class SceneObject:
    name: str
    shape: str  # cube, cylinder, sphere
    pose: np.ndarray
    size: float | tuple[float, float, float]
    color: tuple[float, float, float, float]


class Scene:
    def __init__(self):
        self.objects: List[SceneObject] = []

    def add_cube(self, name: str, pose: np.ndarray, size: float = 0.05):
        self.objects.append(SceneObject(name, "cube", pose, size, (0.8, 0.2, 0.2, 1.0)))

    def add_cuboid(self, name: str, pose: np.ndarray, size_xyz: tuple[float, float, float]):
        self.objects.append(SceneObject(name, "cuboid", pose, size_xyz, (0.85, 0.45, 0.15, 1.0)))

    def add_cylinder(self, name: str, pose: np.ndarray, radius: float = 0.03, length: float = 0.1):
        self.objects.append(SceneObject(name, "cylinder", pose, radius, (0.2, 0.8, 0.2, 1.0)))

    def clear(self):
        self.objects.clear()
