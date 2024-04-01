#!/usr/bin/python3
import pygame
import numpy as np

# QP solver
from qpsolvers import solve_qp

from mrs_playground.common.behavior import Behavior
from mrs_playground.utils import utils

from mr_herding.cbf.constraints import *

class CollisionAvoidance(Behavior):
    def __init__(self,
                 max_u: np.ndarray):
        super().__init__()
        self._max_u = max_u

        # Visulation
        self._pose = np.zeros(2)
        self._u = np.zeros(2)

    def update(self, state: np.ndarray,
               other_states: np.ndarray,
               animal_states: np.ndarray):
        pose = state[:2]
        velocity = state[2:4]

        # Nominal Controller
        # Chase the animal

        # For visualisation purpose
    