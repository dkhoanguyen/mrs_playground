# !/usr/bin/python3

import pygame
import numpy as np

from mrs_playground.utils.utils import *
from mrs_playground.common.entity import Entity


class Robot(Entity):
    def __init__(self,
                 id: int,
                 pose: np.ndarray,
                 velocity: np.ndarray,
                 sensing_range: float,
                 comms_range: float,
                 max_v: float,
                 max_a: float):
        super().__init__(
            pose=pose,
            velocity=velocity,
            image_path='leader-boid.png')

        self._id = id
        self._max_v = max_v
        self._max_a = max_a

        self._sensing_range = sensing_range
        self._comms_range = comms_range

        # pygame annotation
        self._font = pygame.font.SysFont("comicsans", 16)
        self._text = None

        self._behavior_state = None

    def __str__(self):
        return "robot"

    def update(self, *args, **kwargs):
        # Behavior tree should be here
        events = kwargs["events"]

        # Check which robot is within vision
        robot_in_range = self._sensing.sense(state=self.state, target="robot")
        animal_in_range = self._sensing.sense(
            state=self.state, target="animal")
        u_t = np.zeros(2)
        
        # Calculate control
        self._behavior_state = "apf"
        u_t = self._behaviors[str(self._behavior_state)].update(
            state=self.state,
            other_states=robot_in_range,
            animal_states=animal_in_range)
        
        if np.linalg.norm(u_t) >= self._max_v:
            u_t = unit_vector(u_t) * self._max_v

        # State include position and velocity
        x_t = self.state[:4]
        x_t_1 = self._dynamic.step(x_t=x_t, u_t=u_t)

        # Update state
        # self._acceleration = u_t
        self._velocity = x_t_1[2:4]
        self._pose = x_t_1[0:2]

    def display(self, screen: pygame.Surface, debug=False):
        if self._behavior_state and self._behaviors[str(self._behavior_state)]:
            self._behaviors[str(self._behavior_state)].display(screen)

        if self._text:
            screen.blit(self._text, tuple(self._pose - np.array([20, 20])))

        # Update graphics accordingly
        self._move(self._velocity)
        self._text = self._font.render(str(self._id), 1, pygame.Color("white"))
        return super().display(screen, debug)
