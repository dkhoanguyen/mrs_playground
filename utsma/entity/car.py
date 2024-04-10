# !/usr/bin/python3

import pygame
import numpy as np

from mrs_playground.utils.utils import *
from mrs_playground.common.entity import Entity


class Car(Entity):
    def __init__(self,
                 state: np.ndarray,
                 max_v: float,
                 max_a: float,
                 max_delta: float):
        super().__init__(
            pose=state[:2],
            velocity=np.zeros(2),
            image_path='leader-boid.png'
        )

        self._state = state
        self._max_v = max_v
        self._max_u = max_a
        self._max_delta = max_delta

        self._behavior_state = None

    def __str__(self):
        return "car"
