#!/usr/bin/python3

import pygame
import numpy as np

from mrs_playground.common.behavior import Behavior
from mrs_playground.utils.utils import *


class PIDController(Behavior):
    def __init__(self, target_state: np.ndarray):
        self._target_state = target_state
        self._dt = 0.1

        self._prev_error_pose = np.zeros((2,))

        self._Kp_a = 0.1
        self._Ki_a = 0.1
        self._Kd_a = 0.1

        self._Kp_v_delta = 0.1
        self._Ki_v_delta = 0.1
        self._Kd_v_delta = 0.1

    def update(self, *args, **kwargs):
        state = kwargs["state"]
        max_v = kwargs["max_v"]
        max_u = kwargs["max_u"]
        max_delta = kwargs["max_delta"]
        length = kwargs["length"]
        width = kwargs["width"]
        wheel_base = kwargs["wheel_base"]

        # Default - no steering and no accelerating
        u = np.array([0.0, 0.0])

        # Caculate position error
        err = self._target_state[:2] - state[:2]

        # Error becomes velocity
        v_next = err
        # Clamp v_next
        if np.linalg.norm(v_next) > max_v:
            v_next = unit_vector(v_next) * max_v

        # PID for acceleration to control v
        a = self._Kp_a * np.linalg.norm(v_next - state[3])
        if a > max_u:
            a = np.sign(a) * max_u
        u[0] = a
        return u

    def display(self, screen: pygame.Surface):
        return super().display(screen)
