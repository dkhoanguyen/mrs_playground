# !/usr/bin/python3

from typing import Dict

import pygame
import numpy as np
from mrs_playground.params import params
from abc import ABC, abstractmethod

from mrs_playground.common.behavior import Behavior


class Entity(pygame.sprite.Sprite, ABC):
    def __init__(self,
                 pose: np.ndarray,
                 velocity: np.ndarray,
                 image_path: str):
        super(pygame.sprite.Sprite, self).__init__()
        if pose is None:
            pose = np.zeros(2)
        if velocity is None:
            velocity = np.zeros(2)

        # Pygame setup
        self._image_path = image_path
        self.base_image = pygame.image.load(
            f"{params.IMG_DIR}/{self._image_path}")
        self.rect = self.base_image.get_rect()
        self.image = self.base_image

        self._pose = pose
        self._velocity = velocity
        self._acceleration = np.zeros(2)

        angle = -np.rad2deg(np.angle(velocity[0] + 1j * velocity[1]))
        self._heading = np.deg2rad(angle)

        self._behaviors: Dict[str, Behavior] = {}

    @property
    def pose(self):
        return self._pose

    @pose.setter
    def pose(self, pose):
        self._pose = pose
        self.rect.center = tuple(pose)

    @property
    def velocity(self):
        return self._velocity

    @velocity.setter
    def velocity(self, velocity):
        self._velocity = velocity

    @property
    def acceleration(self, accel):
        self._acceleration = accel

    @property
    def state(self):
        return np.hstack((self._pose, self._velocity, self._acceleration))

    # Behaviors
    def add_behavior(self, behavior: dict):
        self._behaviors.update(behavior)

    @abstractmethod
    def update(self, *args, **kwargs):
        pass

    def _rotate_image(self, vector: np.ndarray):
        """Rotate base image using the velocity and assign to image."""
        angle = -np.rad2deg(np.angle(vector[0] + 1j * vector[1]))
        self._heading = np.deg2rad(angle)
        self.image = pygame.transform.rotate(self.base_image, angle)
        self.rect = self.image.get_rect(center=self.rect.center)

    def display(self, screen: pygame.Surface, debug: bool = False):
        screen.blit(self.image, self.rect)
