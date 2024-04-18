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
                 max_delta: float,
                 length: float,
                 width: float,
                 wheelbase: float):
        heading = state[2]
        super().__init__(
            pose=state[:2],
            velocity=state[3]*np.array([np.cos(heading), np.sin(heading)]),
            image_path='car.png'
        )

        # # State should include
        # x: x position
        # y: y position
        # delta: steering angle
        # v: linear velocity
        # phi: vehicle heading
        self._state = state
        self._max_v = max_v
        self._max_u = max_a
        self._max_delta = max_delta
        self._length = length
        self._width = width
        self._wheel_base = wheelbase

        self._behavior_state = None

    def __str__(self):
        return "car"

    def update(self, *args, **kwargs):
        # Behavior tree should be here
        events = kwargs["events"]

        self._behavior_state = "pid"
        u_t = self._behaviors[str(self._behavior_state)].update(
            state=self._state,
            max_v=self._max_v,
            max_u=self._max_u,
            max_delta=self._max_delta,
            length=self._length,
            width=self._width,
            wheel_base=self._wheel_base
        )

        x_t = self._state
        x_t_1 = self._dynamic.step(x_t=x_t, u_t=u_t)
        self._state = x_t_1

        self._pose = self._state[:2]

    def display(self, screen: pygame.Surface, debug=False):
        heading = self._state[2]
        v = self._state[3]
        self._velocity = v * np.array([np.cos(heading), np.sin(heading)])
        self._move(self._velocity)
        return super().display(screen, debug)
