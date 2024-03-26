# !/usr/bin/python3

import numpy as np
from scipy.linalg import expm
from mrs_playground.common.dynamic import Dynamic


class SingleIntegrator(Dynamic):
    """
    
    """
    def __init__(self, dt: float):
        # Original continous state transition matrix
        self._A = np.zeros((2,2))
        self._B = np.eye(2)

        # Discretise the above using Zero Order Hold method
        self._Phi = np.eye(2)
        self._Gamma = dt * np.eye(2)

    def step(self, x_t: np.ndarray, u_t: np.ndarray) -> np.ndarray:
        return self._Phi.dot(x_t) + self._Gamma.dot(u_t)

class DoubleIntegrator(Dynamic):
    """
    Discretised version of the classic double integrator model
    x_dot = v
    v_dot = u
    """
    def __init__(self, dt: float):
        # Original continous state transition matrix
        self._A = np.array([[0.0, 1.0], [0.0, 0.0]])
        self._B = np.array([0.0, 1.0]).transpose()

        # Discretise the above using Zero Order Hold method
        self._Phi = np.array([[1.0, dt], [0.0, 1.0]])
        self._Gamma = np.array([0.5 * dt**2, dt]).transpose()

    def step(self, x_t: np.ndarray, u_t: np.ndarray) -> np.ndarray:
        return self._Phi.dot(x_t) + self._Gamma.dot(u_t)
