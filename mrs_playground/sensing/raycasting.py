#!/usr/bin/python3

import numpy as np

from mrs_playground.common.sensing import SensingModel

class Raycasting(SensingModel):
    def __init__(self, sensing_radius: float,
                 sweep_angle: float = 360.0):
        super().__init__()
        self._sensing_radius = sensing_radius
        self._sweep_angle = sweep_angle
    
    def sense(self, state: np.ndarray,
              target: str) -> np.ndarray:
        pass