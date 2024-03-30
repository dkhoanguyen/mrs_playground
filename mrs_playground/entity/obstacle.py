# !/usr/bin/python3
import pygame
import numpy as np

from abc import abstractmethod
from mrs_playground.common.entity import Entity


class Obstacle(Entity):
    def __init__(self, display_func=None):
        self._display_func = display_func
        self._pose = np.zeros((1, 2))
        self._velocity = np.zeros((1, 2))
        self._acceleration = np.zeros((1, 2))

    def __str__(self):
        return "obstacle"

    def update(self, *args, **kwargs):
        pass

    def display(self, screen: pygame.Surface):
        if self._display_func is not None:
            self._display_func(screen)

    @abstractmethod
    def in_entity_radius(self, qi: np.ndarray, r: float):
        pass

    @abstractmethod
    def induce_beta_agent(self, qi: np.ndarray, pi: np.ndarray):
        pass


class Hyperplane(Obstacle):
    def __init__(self, ak: np.ndarray,
                 yk: np.ndarray,
                 boundary: np.ndarray):
        super().__init__(display_func=None)
        self._ak = np.array(ak).reshape((2, 1))
        self._yk = np.array(yk).reshape((2, 1))
        self._P = np.eye(2) - self._ak @ self._ak.transpose()
        self._boundary = np.array(boundary)

    def display(self, screen: pygame.Surface):
        pygame.draw.rect(screen, pygame.Color(
            'slate gray'), (self._boundary[0, 0], self._boundary[0, 1],
                            self._boundary[1, 0], self._boundary[1, 1]), 0)

    def in_entity_radius(self, qi: np.ndarray, r: float):
        # Project entity position onto the plane
        projected_q = self._P @ qi + (np.eye(2) - self._P) @ self._yk
        projected_q = np.array([projected_q[0,0],projected_q[1,1]])
        return np.linalg.norm(projected_q - qi) <= r

    def induce_beta_agent(self, qi: np.ndarray, pi: np.ndarray) -> np.ndarray:
        qi = qi.reshape((2, 1))
        pi = pi.reshape((2, 1))

        qik = self._P @ qi + (np.eye(2) - self._P) @ self._yk
        pik = self._P @ pi
        return np.hstack((qik.transpose(), pik.transpose())).reshape(4,)


class Sphere(Obstacle):
    def __init__(self, yk: np.ndarray,
                 Rk: float):
        self._yk = np.array(yk).reshape((2, 1))
        self._Rk = float(Rk)

    def display(self, screen: pygame.Surface):
        pygame.draw.circle(screen, pygame.Color(
            'slate gray'), center=self._yk.transpose()[0], radius=self._Rk)

    def in_entity_radius(self, qi: np.ndarray, r: float):
        # Project entity posit
        return np.linalg.norm(self._yk - qi.reshape((2, 1))) <= r + self._Rk

    def induce_beta_agent(self, qi: np.ndarray, pi: np.ndarray) -> np.ndarray:
        qi = qi.reshape((2, 1))
        pi = pi.reshape((2, 1))

        mu = self._Rk / np.linalg.norm(qi - self._yk)
        ak = (qi - self._yk)/np.linalg.norm(qi - self._yk)
        P = np.eye(2) - ak @ ak.transpose()

        qik = mu * qi + (1 - mu) * self._yk
        pik = mu * P @ pi
        return np.hstack((qik.transpose(), pik.transpose())).reshape(4,)


class LineSegment(Obstacle):
    def __init__(self, start: np.ndarray,
                 end: np.ndarray):
        self._start = start
        self._end = end
