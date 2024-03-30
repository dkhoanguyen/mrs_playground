# !/usr/bin/python3

import numpy as np
from scipy.linalg import expm
from mrs_playground.common.dynamic import DynamicModel


class SingleIntegrator(DynamicModel):
    """

    """

    def __init__(self, dt: float):
        super().__init__()
        # Original continous state transition matrix
        self._A = np.zeros((4, 4))
        self._B = np.eye(4)

        # Discretise the above using Zero Order Hold method
        self._Phi = np.array([[1.0, 0.0, 0.0, 0.0],
                              [0.0, 1.0, 0.0, 0.0],
                              [0.0, 0.0, 0.0, 0.0],
                              [0.0, 0.0, 0.0, 0.0]])
        self._Gamma = np.eye(4)
        np.fill_diagonal(self._Gamma, np.array([dt, dt, 1, 1]))

    def step(self, x_t: np.ndarray, u_t: np.ndarray) -> np.ndarray:
        u_t = np.hstack((u_t, u_t)).reshape((4, 1))
        return (self._Phi.dot(x_t.reshape(4, 1)) + self._Gamma.dot(u_t)).reshape(4)


class DoubleIntegrator(DynamicModel):
    """
    Discretised version of the classic double integrator model
    x_dot = v
    v_dot = u
    """

    def __init__(self, dt: float):
        super().__init__()
        # Original continous state transition matrix
        self._A = np.array([[0.0, 1.0], [0.0, 0.0]])
        self._B = np.array([0.0, 1.0]).transpose()

        # Discretise the above using Zero Order Hold method
        self._Phi = np.array([[1.0, 0.0, dt, 0.0],
                              [0.0, 1.0, 0.0, dt],
                              [0.0, 0.0, 1.0, 0.0],
                              [0.0, 0.0, 0.0, 1.0]])
        self._Gamma = np.eye(4)
        np.fill_diagonal(self._Gamma, np.array(
            [0.5 * dt**2, 0.5 * dt**2, dt, dt]))

    def step(self, x_t: np.ndarray, u_t: np.ndarray) -> np.ndarray:
        u_t = np.hstack((u_t, u_t)).reshape((4, 1))
        return (self._Phi.dot(x_t.reshape(4, 1)) + self._Gamma.dot(u_t)).reshape(4)
