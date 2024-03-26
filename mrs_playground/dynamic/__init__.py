# !/usr/bin/python3

import numpy as np
from mrs_playground.common.dynamic import Dynamic


class DoubleIntegrator(Dynamic):
    """
    Discretised version of the classic double integrator model
    """

    def __init__(self, dt:float):
        # Original continous state transition matrix
        self._A = np.array([0.0, 1.0], [0.0, 0.0])
        self._B = np.array([0.0,1.0]).transpose()

        # Discretised 
        self._Phi = 
        self._Gamma = None
