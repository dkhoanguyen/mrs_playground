# !/usr/bin/python3

import pygame
import numpy as np
from spatialmath.base import *

from mrs_playground.common.entity import Entity


class Robot(Entity):
    def __init__(self,
                 id: int,
                 pose: np.ndarray,
                 velocity: np.ndarray,
                 sensing_range: float,
                 comms_range: float,
                 min_v: float,
                 max_v: float,
                 max_a: float):
        super().__init__(
            pose=pose,
            velocity=velocity,
            image_path='leader-boid.png',
            min_v=min_v,
            max_v=max_v)

        self._max_v = max_v
        self._max_a = max_a

        self._sensing_range = sensing_range
        self._comms_range = comms_range

        # pygame annotation
        self._font = pygame.font.SysFont("comicsans", 16)
        self._text = None

    def __str__(self):
        return "robot"

    def update(self, *args, **kwargs):
        dt = kwargs["dt"]
        # Behavior tree should be here
        events = kwargs["events"]
        ids = kwargs["ids"]

        all_states = kwargs["entity_states"]
        all_animal_states = all_states["herd"]
        all_robot_states = all_states["robot"]

        # Check which robot is within vision
        robot_in_range = np.empty((0, 6))
        animal_in_range = self._get_state_within_sensing(self._sensing_range,
                                                         )

    def display(self, screen: pygame.Surface, debug=False):
        if self._behaviors[str(self._behavior_state)]:
            self._behaviors[str(self._behavior_state)].display(screen)

        if self._text:
            screen.blit(self._text, tuple(self.pose - np.array([20, 20])))

        return super().display(screen, debug)

    def _get_state_within_sensing(self, sensing_range: float,
                                  state: np.ndarray, states: np.ndarray):
        state_in_range = np.empty((0, 6))
        for idx in range(states.shape[0]):
            d = np.linalg.norm(state[:2] - states[idx, :2])
            if d > 0.0 and d <= sensing_range:
                state_in_range = np.vstack(
                    (state_in_range, states[idx, :]))
        return state_in_range
