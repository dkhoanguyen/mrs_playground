# !/usr/bin/python3

import numpy as np
from scipy.linalg import expm
from mrs_playground.common.dynamic import DynamicModel


class KinematicSingleTrackCartesian(DynamicModel):
    """

    """

    def __init__(self, dt: float,
                 L: float):
        super().__init__()
        self._dt = dt
        self._L = L

    def __str__(self):
        return "kinematic_single_track"

    def step(self, x_t: np.ndarray, u_t: np.ndarray) -> np.ndarray:
        """Discrete-time update using Zero-Order Hold"""
        a, delta_dot = u_t
        x_t_1 = x_t.copy()
        x_t_1[0] = x_t[0] + x_t[3] * np.cos(x_t[2]) * self._dt
        x_t_1[1] = x_t[1] + x_t[3] * np.sin(x_t[2]) * self._dt
        x_t_1[2] = x_t[2] + (x_t[3] / self._L) * np.tan(x_t[4]) * self._dt
        x_t_1[3] = x_t[3] + a * self._dt
        x_t_1[4] = x_t[4] + delta_dot * self._dt
        return x_t_1

    def _f(self, x_t: np.ndarray, u_t: np.ndarray):
        """Kinematic bicycle model dynamics"""
        a, delta_dot = u_t
        x, y, theta, v, delta = x_t
        return np.array([v * np.cos(theta),
                         v * np.sin(theta),
                         (v / self._L) * np.tan(delta),
                         a,
                         delta_dot])


class KinematicSingleTrackFrenet(DynamicModel):
    """

    """
    pass
