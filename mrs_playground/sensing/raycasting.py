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
        states: np.ndarray = self._all_states[target]
        state_in_range = np.empty((0, 6))
        for idx in range(states.shape[0]):
            xij = states[idx, :2] - state[:2]
            angle = np.arctan2(xij[1], xij[0])
            if abs(angle) > np.deg2rad(self._sweep_angle) / 2:
                # Skip if exceed sweep angle   
                continue
            d = np.linalg.norm(state[:2] - states[idx, :2])
            if d > 0.0 and d <= self._sensing_radius:
                state_in_range = np.vstack(
                    (state_in_range, states[idx, :]))
        return state_in_range