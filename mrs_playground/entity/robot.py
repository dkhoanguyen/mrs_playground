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
        # Herding target
        self._target = np.array([1200, 600])

        # Behavior tree should be here
        events = kwargs["events"]
        all_comms = kwargs["comms"]

        self._comms.update({"id": self._id,
                            "state": self.state})

        # Check which robot is within vision
        robot_in_range = self._sensing.sense(state=self.state, target="robot")
        animal_in_range = self._sensing.sense(
            state=self.state, target="animal")
        u_t = np.zeros(2)

        # To faciliate for the fact that there is assuming a global planner to show the
        # robots where the animals are
        # Obtain raw animal states
        all_animal_states = self._sensing.get_raw_all_states(target="animal")

        # Obtain animal centroid
        animal_centroid: np.ndarray = np.sum(
            all_animal_states[:, :2], axis=0) / all_animal_states.shape[0]

        # Calculate control
        if len(self._behaviors) == 1:
            self._behavior_state = next(iter(self._behaviors))

        u_t = self._behaviors[str(self._behavior_state)].update(
            state=self.state,
            robot_states=robot_in_range,
            animal_states=animal_in_range,
            animal_centroid=animal_centroid,
            comms=self._comms,
            all_comms=all_comms,
            id=self._id
        )

        if str(self._dynamic) == "pm_doubleintegrator":
            if np.linalg.norm(u_t) >= self._max_a:
                u_t = unit_vector(u_t) * self._max_a

        if str(self._dynamic) == "pm_singleintegrator":
            if np.linalg.norm(u_t) >= self._max_v:
                u_t = unit_vector(u_t) * self._max_v

        # State include position and velocity
        x_t = self.state[:4]
        x_t_1 = self._dynamic.step(x_t=x_t, u_t=u_t)

        # Bound velocity if dynamic is double integrator
        if str(self._dynamic) == "pm_doubleintegrator":
            if np.linalg.norm(x_t_1[2:4]) > self._max_v:
                x_t_1[2:4] = unit_vector(x_t_1[2:4]) * self._max_v

        self._acceleration = u_t
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
