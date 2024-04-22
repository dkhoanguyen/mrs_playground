# !/usr/bin/python3

import numpy as np
from scipy.linalg import expm
from mrs_playground.common.dynamic import DynamicModel
from mrs_playground.utils import utils


class KinematicDifferentialDrive(DynamicModel):
    """

    """

    def __init__(self, dt: float,
                 L: float):
        super().__init__()
        self._dt = dt
        self._L = L

    def __str__(self):
        return "kinematic_diff_drive"

    def step(self, x_t: np.ndarray, u_t: np.ndarray) -> np.ndarray:
        """Discrete-time update using Zero-Order Hold"""
        x_t_1 = x_t.copy()
        # Get current theta
        theta_t = utils.angle_between_with_direction(np.array([1, 0]), )
        return x_t_1
