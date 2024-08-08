#!/usr/bin/python3
import pygame
import numpy as np

from mrs_playground.common.behavior import Behavior
from mrs_playground.utils import utils


class DistributedOutmostPush(Behavior):
    def __init__(self,
                 max_u: float,
                 sensing_range: float,
                 collision_avoidance_d: float,
                 herding_target: np.ndarray):
        super().__init__()

        self._max_u = max_u
        self._sensing_range = 300.0
        self._collision_avoidance_d = collision_avoidance_d
        self._target = herding_target

        self._f_ove = np.zeros((2,))
        self._pose = np.zeros((2,))

    def update(self, *args, **kwargs):
        state: np.ndarray = kwargs["state"]
        other_states: np.ndarray = kwargs["robot_states"]
        animal_states: np.ndarray = kwargs["animal_states"]
        animal_centroid: np.ndarray = kwargs["animal_centroid"]
        comms: dict = kwargs["comms"]
        all_comms: list = kwargs["all_comms"]
        id: int = kwargs["id"]

        delta = 0.5
        R_e = 150

        # Check animal in sensing range
        if animal_states.shape[0] == 0:
            # Move towards centroid to find animals
            f_ove_robot = self._overlap_avoidance(
                state, other_states, 170.0)
            k_ove_robot = 100000
            f_ste = 100 * utils.unit_vector(animal_centroid[:2] - state[:2])

            u_i = k_ove_robot * f_ove_robot + f_ste
            return u_i

        y_out = self._find_outmost_evader(
            state, animal_states, self._target, self._sensing_range)

        f_ove_robot = self._overlap_avoidance(
            state, other_states, 170.0)
        f_ove_animal = self._overlap_avoidance(
            state, animal_states, 150.0)
        f_ste = self._steering_force(state, y_out, self._target, delta, R_e)

        k_ove_robot = 500000
        k_ove_animal = 1000000
        k_ste = 1

        self._f_ove = k_ove_robot * f_ove_robot
        self._pose = state[:2]

        u_i = k_ove_robot * f_ove_robot + k_ste * f_ste + k_ove_animal*f_ove_animal

        return u_i

    def display(self, screen: pygame.Surface):
        pygame.draw.line(
            screen, pygame.Color("blue"),
            tuple(self._pose), tuple(self._pose + (self._f_ove)))
        pygame.draw.circle(screen, pygame.Color("black"),
                           tuple(self._target), 50, 1)
        pygame.draw.circle(screen, pygame.Color("green"),
                           tuple(self._target), 50, 1)
        return super().display(screen)

    def _find_outmost_evader(self, state, other_states, c, R_h):
        x_i = state[:2]
        evaders_in_range = [
            y_j for y_j in other_states[:, :2] if np.linalg.norm(y_j - x_i) <= R_h]

        if not evaders_in_range:
            return []  # No other_states within range

        distances_to_goal = [np.linalg.norm(
            y_j - c) for y_j in evaders_in_range]
        outmost_index = np.argmax(distances_to_goal)
        y_out = evaders_in_range[outmost_index]
        return y_out

    # Herders dynamics
    def _overlap_avoidance(self, state, other_states, R_h):
        f_overlap = np.zeros((2,))
        xi = state[:2]
        for xj in other_states[:, :2]:
            distance = np.linalg.norm(xi - xj)
            if distance < R_h:
                f_overlap += (xi - xj) / (distance**3)
        return f_overlap

    def _steering_force(self, state, y_out, c, delta, R_e):
        x_i = state[:2]
        s_out = y_out
        s_i = s_out + delta * R_e * (s_out - c) / np.linalg.norm(s_out - c)
        f_ste = -(x_i - s_i)
        return f_ste
