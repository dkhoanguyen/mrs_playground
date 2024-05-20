#!/usr/bin/python3
import pygame
import numpy as np
from collections import deque
import networkx as nx

# QP solver
from qpsolvers import solve_qp

from mrs_playground.common.behavior import Behavior
from mrs_playground.utils import utils

from mr_herding.cbf.constraints import *
from mr_herding.apf.potential_func import *


class DecentralisedCBF(Behavior):
    def __init__(self,
                 max_u: float,
                 max_num: int,
                 sensing_range: float,
                 comms_range: float,
                 min_robot_d: float,
                 max_robot_d: float,
                 min_animal_d: float,
                 max_animal_d: float,
                 collision_avoidance_d: float,
                 converge_std: float,
                 herding_target: np.ndarray,
                 coverage: float):
        super().__init__()

        self._max_u = max_u

        self._sensing_range = sensing_range
        self._comms_range = comms_range
        self._min_robot_d = min_robot_d
        self._max_robot_d = max_robot_d
        self._min_animal_d = min_animal_d
        self._max_animal_d = max_animal_d
        self._collision_avoidance_d = collision_avoidance_d
        self._converge_std = converge_std
        self._target = np.array(herding_target)
        self._coverage = coverage

        self._pose = np.zeros(2)
        self._u = np.zeros(2)
        self._prev_x = np.zeros(4)
        self._d_to_animal = deque(maxlen=100)
        self._converge_to_animal = False
        self._adj_vector = np.zeros((max_num))
        self._adj_matrix = np.zeros((max_num, max_num))
        self._adj_graph = nx.Graph()

        self._is_leader = False
        self._is_leaf = False

        self._set_theta = False

        # Debugging and plotting
        self._centroid_to_target = np.zeros(2)
        self._first_second = np.zeros(2)

        self._animal_heading = np.zeros(2)
        self._in_vision_animal_pos = np.zeros(2)

    def update(self, *args, **kwargs):
        state: np.ndarray = kwargs["state"]
        other_states: np.ndarray = kwargs["robot_states"]
        animal_states: np.ndarray = kwargs["animal_states"]
        animal_centroid: np.ndarray = kwargs["animal_centroid"]
        comms: dict = kwargs["comms"]
        all_comms: list = kwargs["all_comms"]
        id: int = kwargs["id"]

        pose = state[:2]
        self._pose = state[:2]
        velocity = state[2:4]

        A = np.empty((0, 4))
        b = np.empty((0, 1))

        comms.update({"converge_to_animal": self._converge_to_animal})

        # If no animal, move toward the initial herding pose
        if animal_states.shape[0] == 0:
            u_nom = 0.1 * (animal_centroid - pose)
            if np.linalg.norm(u_nom) > self._max_u:
                u_nom = self._max_u * utils.unit_vector(u_nom)

            # Formation control with

            A_orca, b_orca = self._robot_robot_collision_avoidance(state=state,
                                                                   v_nom=utils.unit_vector(
                                                                       u_nom)*self._max_u,
                                                                   robot_states=other_states,
                                                                   distance=self._collision_avoidance_d)
            A = np.vstack((A, A_orca))
            b = np.vstack((b, b_orca))

            u, x = self._solve_u(vi=velocity,
                                 u_nom=u_nom,
                                 prev_x=self._prev_x,
                                 A=A, b=b,
                                 lb=np.array(
                                     [-self._max_u, -self._max_u, -np.inf, -np.inf]),
                                 ub=np.array(
                                     [self._max_u, self._max_u, np.inf, np.inf]),
                                 p_omega=1000000.0,
                                 omega_0=1.0)
            self._prev_x = x
            self._u = u_nom
            return u

        xi = pose
        xj = animal_states[:, :2]

        # Find closest animal
        # Should change to closet cluster of animal
        all_d_to_animal = np.linalg.norm(xi - xj, axis=1)
        all_d_to_animal = np.sort(all_d_to_animal)
        self._d_to_animal.append(all_d_to_animal[0])
        if len(self._d_to_animal) >= self._d_to_animal.maxlen:
            if np.std(self._d_to_animal) < self._converge_std \
                    and np.mean(self._d_to_animal) <= self._max_animal_d + self._converge_std:
                self._converge_to_animal = True
            else:
                self._converge_to_animal = False

        # Nominal Controller
        u_nom = np.zeros(2)
        # Edge following
        # Need to rework this behavior but this is for future work
        u_edge_following = self._edge_following(
            xi=xi, xj=xj, vi=velocity, d=self._min_animal_d, gain=5.0)
        u_nom += u_edge_following

        # CBF Constraints
        # Robot-animal formation
        enforce_formation = False
        reach_consensus = np.empty((0, 1))
        for comm in all_comms:
            if "id" not in comm.keys():
                continue
            reach_consensus = np.vstack(
                (reach_consensus, int(comm["converge_to_animal"])))

        # If all reach consensus
        if np.all(reach_consensus == 1):
            enforce_formation = True

        if enforce_formation:
            # Estimate overall animal heading
            in_vision_animal_pos = np.average(
                animal_states[:, 0:2], axis=0)
            in_vision_animal_heading = np.average(
                animal_states[:, 2:4], axis=0)

            animal_in_vision_to_target = self._target - in_vision_animal_pos
            angle_to_target = angle_between_with_direction(
                animal_in_vision_to_target, in_vision_animal_heading)

            theta = angle_to_target
            if np.abs(theta) > np.pi/2:
                theta = np.sign(theta) * np.pi/2

            # If heading is incorrect, accelerate to adjust heading
            u_nom_flipped = np.array([[np.cos(theta), -np.sin(theta)],
                                      [np.sin(theta), np.cos(theta)]]).dot(u_nom.reshape(2, 1))
            # if self._is_leaf:
            u_nom = u_nom_flipped.reshape((2,))

            # Move forward target if angle is aligned
            if abs(angle_to_target) < 0.2:
                u_target = unit_vector(self._target - state[:2]) * np.linalg.norm(u_nom)
                u_nom = u_nom + u_target


        # Scale down u
        if np.linalg.norm(u_nom) > self._max_u:
            u_nom = self._max_u * utils.unit_vector(u_nom)

        A_r_a, b_r_a = self._robot_animal_formation(state=state,
                                                    animal_states=animal_states,
                                                    min_distance=self._min_animal_d,
                                                    gamma_min=1.0,
                                                    max_distance=self._max_animal_d,
                                                    gamma_max=1.0,
                                                    relax_d_min=False,
                                                    relax_d_max=True)
        A = np.vstack((A, A_r_a))
        b = np.vstack((b, b_r_a))  

        A_r_r, b_r_r = self._robot_robot_formation(state=state,
                                                   v_nom=utils.unit_vector(
                                                                       u_nom)*self._max_u,
                                                   robot_states=other_states,
                                                   min_distance=self._min_robot_d,
                                                   gamma_min=1.0,
                                                   max_distance=self._max_robot_d,
                                                   gamma_max=2.0,
                                                   relax_d_min=False,
                                                   relax_d_max=not enforce_formation)
        A = np.vstack((A, A_r_r))
        b = np.vstack((b, b_r_r))

        u, x = self._solve_u(vi=velocity,
                             u_nom=u_nom,
                             prev_x=self._prev_x,
                             A=A, b=b,
                             lb=np.array(
                                 [-self._max_u, -self._max_u, -np.inf, -np.inf]),
                             ub=np.array(
                                 [self._max_u, self._max_u, np.inf, np.inf]),
                             p_omega=100000000.0,
                             omega_0=1.0)
        self._u = u_nom
        self._prev_x = x
        return u

    def display(self, screen: pygame.Surface):
        pygame.draw.line(
            screen, pygame.Color("blue"),
            tuple(self._pose), tuple(self._pose + 5 * (self._u)))

        pygame.draw.line(
            screen, pygame.Color("white"),
            tuple(self._in_vision_animal_pos), tuple(self._in_vision_animal_pos + 10 * (self._animal_heading)))

        pygame.draw.circle(screen, pygame.Color("red"),
                           tuple(self._pose), self._max_robot_d, 1)
        pygame.draw.circle(screen, pygame.Color("black"),
                           tuple(self._target), 50, 1)
        pygame.draw.circle(screen, pygame.Color("dark green"),
                           tuple(self._pose), self._min_robot_d, 1)
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
            # P Controller to obtain control u
            v_sum += v
        u = self._max_u * (v_sum - vi)
        return u

    def _repulsion(self, xi: np.ndarray, xj: np.ndarray,
                   vi: np.ndarray,
                   d: float, gain: float):
        u = np.zeros(2).astype(np.float64)
        v_sum = np.zeros(2).astype(np.float64)
        for i in range(xj.shape[0]):
            xij = xi - xj[i, :]
            p = GradPotentionFunc.replusion_func(xi=xi,
                                                 xj=xj,
                                                 d=d)
            # Obtain v
            v = gain * p * utils.unit_vector(xij)
            # v = xij
            dot_product = unit_vector(xi - xj[i, :]).dot(unit_vector(xi))
            flip_direction = np.arccos(dot_product)
            theta = np.sign(flip_direction) * np.pi/2
            v_flipped = np.array([[np.cos(theta), -np.sin(theta)],
                                  [np.sin(theta), np.cos(theta)]]).dot(v.reshape(2, 1))
            v_sum += v_flipped.reshape((2,))
        u = (v_sum - vi)
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
            d=min_distance, gamma=gamma_min,
            relax=relax_d_min)

        A = np.vstack((A, A_dmin))
        b = np.vstack((b, b_dmin))

        A_dmax, b_dmax = MaxDistance.build_constraint(
            xi=xi, xj=xj, vi=vi, vj=vj,
            ai=self._max_u, aj=self._max_u,
            d=max_distance, gamma=gamma_max,
            relax=relax_d_max)

        A = np.vstack((A, A_dmax))
        b = np.vstack((b, b_dmax))

        return A, b

    def _robot_robot_formation(self, state: np.ndarray,
                               v_nom: np.ndarray,
                               robot_states: np.ndarray,
                               min_distance: float,
                               gamma_min: float,
                               max_distance: float,
                               gamma_max: float,
                               relax_d_min: bool = False,
                               relax_d_max: bool = True):
        A = np.empty((0, 4))
        b = np.empty((0, 1))

        xi = state[:2]
        xj = robot_states[:, :2]
        vi = state[2:4]
        vj = robot_states[:, 2:4]

        ri = min_distance
        rj = np.ones(robot_states.shape[0]) * 0
        weight = np.ones(robot_states.shape[0]) * 0.5

        xi = state[:2]
        xj = robot_states[:, :2]
        vi = vi
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
                                                   gamma_min,
                                                   1.2)
            A = np.vstack((A, A_orca,))
            b = np.vstack((b, b_ocra,))

        A_dmax, b_dmax = MaxDistance.build_constraint(
            xi=xi, xj=xj, vi=vi, vj=vj,
            ai=self._max_u, aj=self._max_u,
            d=max_distance, gamma=gamma_max,
            relax=relax_d_max)
        
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
                                                   1.2)
            A = np.vstack((A, A_orca,))
            b = np.vstack((b, b_ocra,))

        return A, b

    def _link_management(self):
        pass

    def _solve_u(self,
                 vi: np.ndarray,
                 u_nom: np.ndarray,
                 prev_x: np.ndarray,
                 A: np.ndarray,
                 b: np.ndarray,
                 lb: np.ndarray,
                 ub: np.ndarray,
                 p_omega: float,
                 omega_0: float):
        """
        Run the QP to find optimal u
        """

        H = np.identity(4) * 0.5
        # Control bound synthesis
        H[2, 2] = p_omega
        # Slack variable
        H[3, 3] = 1.0
        p = -2 * np.array([u_nom[0], u_nom[1], omega_0 * p_omega, 0.0])
        x = solve_qp(P=H, q=p, G=A, h=b, lb=lb, ub=ub,
                     solver="cvxopt")  # osqp or cvxopt

        u = np.zeros(2)
        if x is None:
            if np.linalg.norm(vi) > 0:
                # Slow to stop
                u = 2.0 * (-vi)
            else:
                u = np.zeros(2)
        else:
            u = x[:2]

        return u, x
