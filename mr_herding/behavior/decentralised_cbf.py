#!/usr/bin/python3
import pygame
import numpy as np

# QP solver
from qpsolvers import solve_qp

from mrs_playground.common.behavior import Behavior
from mrs_playground.utils import utils

from mr_herding.cbf.constraints import *
from mr_herding.apf.potential_func import *


class DecentralisedCBF(Behavior):
    def __init__(self,
                 max_u: float):
        super().__init__()

        self._target_pos = np.array([500, 350])
        self._max_u = max_u

        self._pose = np.zeros(2)
        self._u = np.zeros(2)

        self._prev_u = np.zeros(2)

    def update(self, *args, **kwargs):
        state = kwargs["state"]
        other_states = kwargs["robot_states"]
        animal_states = kwargs["animal_states"]
        animal_centroid = kwargs["animal_centroid"]

        pose = state[:2]
        self._pose = state[:2]
        velocity = state[2:4]

        # If no animal, move toward centroid
        if animal_states.shape[0] == 0:
            u = 0.1 * (animal_centroid - pose)
            if np.linalg.norm(u) > self._max_u:
                u = self._max_u * utils.unit_vector(u)
            self._u = u
            return u

        xi = pose
        xj = animal_states[:, :2]

        # Nominal Controller
        u_nom = self._edge_following(
            xi=xi, xj=xj, vi=velocity, d=100.0, gain=5.0)

        # print(np.linalg.norm(u_nom))
        if np.linalg.norm(u_nom) > self._max_u:
            u_nom = self._max_u * utils.unit_vector(u_nom)

        vi = utils.unit_vector(u_nom) * self._max_u
        # vi = velocity
        vj = animal_states[:, 2:4]

        # u_nom = np.zeros(2)
        # CBF Constraints
        # Robot-animal formation
        A = np.empty((0, 4))
        b = np.empty((0, 1))
        A_r_a, b_r_a = self._robot_animal_formation(state=state,
                                                    animal_states=animal_states,
                                                    min_distance=100.0,
                                                    gamma_min=0.1,
                                                    max_distance=170.0,
                                                    gamma_max=1.0)

        A = np.vstack((A, A_r_a))
        b = np.vstack((b, b_r_a))

        # Robot-robot formation
        A_r_r, b_r_r = self._robot_robot_formation(state=state,
                                                   robot_states=other_states,
                                                   min_distance=170.0,
                                                   gamma_min=1.0,
                                                   max_distance=200.0,
                                                   gamma_max=1.0)

        A = np.vstack((A, A_r_r))
        b = np.vstack((b, b_r_r))

        P = np.identity(4)
        p_omega = 1000000.0
        omega_0 = 1.0
        P[2, 2] = p_omega
        P[3, 3] = 1.0

        q = -2 * np.array([u_nom[0], u_nom[1], omega_0 * p_omega, 0.0])
        UB = np.array([self._max_u, self._max_u, np.inf, np.inf])
        LB = np.array([-self._max_u, -self._max_u, -np.inf, -np.inf])

        u = solve_qp(P, q, G=A, h=b, lb=LB, ub=UB,
                     solver="cvxopt")  # osqp or cvxopt
        if u is None:
            u = np.zeros(2)
        else:
            u = u[:2]
        if np.linalg.norm(u) > self._max_u:
            u = self._max_u * utils.unit_vector(u)
        self._u = u_nom
        self._prev_u = u

        return u

    def display(self, screen: pygame.Surface):
        pygame.draw.line(
            screen, pygame.Color("yellow"),
            tuple(self._pose), tuple(self._pose + 5 * (self._u)))
        return super().display(screen)

    def _edge_following(self, xi: np.ndarray, xj: np.ndarray,
                        vi: np.ndarray,
                        d: float, gain: float):
        u = np.zeros(2).astype(np.float64)
        v_sum = np.zeros(2).astype(np.float64)
        for i in range(xj.shape[0]):
            xij = xi - xj[i, :]
            p = GradPotentionFunc.const_attract_fuc(xi=xi,
                                                    xj=xj,
                                                    d=d)
            # Obtain v
            v = gain * p * utils.unit_vector(xij)
            # v = -xij
            # P Controller to obtain control u
            v_sum += v
        if np.linalg.norm(v_sum) > 10.0:
            v_sum = utils.unit_vector(v_sum) * 10.0
        u = v_sum - vi
        return u

    def _robot_animal_formation(self, state: np.ndarray,
                                animal_states: np.ndarray,
                                min_distance: float,
                                gamma_min: float,
                                max_distance: float,
                                gamma_max: float,
                                relax_d_min: bool = False,
                                relax_d_max: bool = True):
        A = np.empty((0, 4))
        b = np.empty((0, 1))

        xi = state[:2]
        xj = animal_states[:, :2]
        vi = state[2:4]
        vj = animal_states[:, 2:4]

        A_dmin, b_dmin = MinDistance.build_constraint(
            xi=xi, xj=xj, vi=vi, vj=vj,
            ai=self._max_u, aj=self._max_u,
            d=min_distance, gamma=gamma_min)

        A = np.vstack((A, A_dmin))
        b = np.vstack((b, b_dmin))

        A_dmax, b_dmax = MaxDistance.build_constraint(
            xi=xi, xj=xj, vi=vi, vj=vj,
            ai=self._max_u, aj=self._max_u,
            d=max_distance, gamma=gamma_max)

        A = np.vstack((A, A_dmax))
        b = np.vstack((b, b_dmax))

        return A, b

    def _robot_robot_formation(self, state: np.ndarray,
                               robot_states: np.ndarray,
                               min_distance: float,
                               gamma_min: float,
                               max_distance: float,
                               gamma_max: float):
        A = np.empty((0, 4))
        b = np.empty((0, 1))

        xi = state[:2]
        xj = robot_states[:, :2]
        vi = state[2:4]
        vj = robot_states[:, 2:4]

        A_dmin, b_dmin = MinDistance.build_constraint(
            xi=xi, xj=xj, vi=vi, vj=vj,
            ai=self._max_u, aj=self._max_u,
            d=min_distance, gamma=gamma_min)

        A = np.vstack((A, A_dmin))
        b = np.vstack((b, b_dmin))

        A_dmax, b_dmax = MaxDistance.build_constraint(
            xi=xi, xj=xj, vi=vi, vj=vj,
            ai=self._max_u, aj=self._max_u,
            d=max_distance, gamma=gamma_max)

        A = np.vstack((A, A_dmax))
        b = np.vstack((b, b_dmax))

        return A, b


    def _robot_robot_collision_avoidance(self, state: np.ndarray,
                                         v_nom: np.ndarray,
                                         robot_states: np.ndarray,
                                         distance: float,
                                         relax: bool = False):
        ri = distance
        rj = np.ones(robot_states.shape[0]) * 0
        weight = np.ones(robot_states.shape[0]) * 0.5

        xi = state[:2]
        xj = robot_states[:, :2]
        vi = v_nom
        vj = robot_states[:, 2:4]

        A = np.empty((0, 4))
        b = np.empty((0, 1))

        planes = ORCA.construct_orca_planes(xi=xi, xj=xj, vi=vi, vj=vj,
                                            ri=ri, rj=rj,
                                            weight=weight,
                                            buffered_r=0.0,
                                            time_horizon=2.0)
        if len(planes) > 0:
            A_orca, b_ocra = ORCA.build_constraint(planes, vi,
                                                   1.0)
            A = np.vstack((A, A_orca,))
            b = np.vstack((b, b_ocra,))

        return A, b
