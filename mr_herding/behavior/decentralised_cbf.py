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
                 herding_target: np.ndarray):
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

        self._theta = 0
        self._set_theta = False

        self._centroid_to_target = np.zeros(2)
        self._first_second = np.zeros(2)

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
        comms.update({"adj_vector": self._adj_vector})
        comms.update({"leaf": self._is_leaf})
        comms.update({"leader": self._is_leader})
        comms.update({"animals_in_vision": animal_states})

        # Calculate robots centroid which can be approximated as the centroid
        # of the animals
        all_robots_states = np.empty((0, 2))
        for comm in all_comms:
            if "id" not in comm.keys():
                continue
            all_robots_states = np.vstack(
                (all_robots_states, comm["state"][:2]))
        centroid = np.average(all_robots_states, axis=0)

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

        floating_other_states = other_states

        # Find closest animal
        all_d_to_animal = np.linalg.norm(xi - xj, axis=1)
        all_d_to_animal = np.sort(all_d_to_animal)
        self._d_to_animal.append(all_d_to_animal[0])
        if len(self._d_to_animal) >= self._d_to_animal.maxlen:
            if np.std(self._d_to_animal) < self._converge_std \
                    and np.mean(self._d_to_animal) < self._max_animal_d + self._converge_std:
                self._converge_to_animal = True
            else:
                self._converge_to_animal = False

        # Nominal Controller
        u_nom = np.zeros(2)
        # Edge following
        u_edge_following = self._edge_following(
            xi=xi, xj=xj, vi=velocity, d=self._min_animal_d, gain=10.0)
        u_nom += u_edge_following

        if self._converge_to_animal:
            # Find neighboring robots that have converged to animal
            other_converged = np.empty((0, 6))
            other_not_converged = np.empty((0, 6))
            for comm in all_comms:
                if "id" not in comm.keys():
                    continue
                if comm["id"] != id \
                        and np.linalg.norm(pose - comm["state"][:2]) <= self._sensing_range:
                    if comm["converge_to_animal"]:
                        other_converged = np.vstack(
                            (other_converged, comm["state"]))
                    else:
                        other_not_converged = np.vstack(
                            (other_not_converged, comm["state"]))

            if other_converged.shape[0] > 0 and other_not_converged.shape[0] > 0:
                floating_other_states = np.empty((0,6))
                # Repulsion control for guiding robots towards the formation
                # u_repulse = self._repulsion(
                #     xi=xi, xj=other_not_converged[:,:2], vi=velocity,
                #     d=self._min_robot_d, gain=10.0)
                # u_nom += u_repulse
                floating_other_states = np.vstack((floating_other_states, other_not_converged))

                # 
        if np.linalg.norm(u_nom) > self._max_u:
            u_nom = self._max_u * utils.unit_vector(u_nom)

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

            # Construct adjacency matrix
            for comm in all_comms:
                adj_id = comm["id"]
                adj_state = comm["state"]

                if np.linalg.norm(pose - adj_state[:2]) < self._sensing_range \
                        and id != adj_id:
                    self._adj_vector[adj_id] = 1.0

            # If leaf node
            if np.sum(self._adj_vector) == 1:
                self._is_leaf = True

            # Reconstruct adj_matrix from all vector
            for comm in all_comms:
                adj_vector = comm["adj_vector"]
                comm_id = comm["id"]
                self._adj_matrix[comm_id, :] = adj_vector

            self._adj_graph = nx.from_numpy_array(self._adj_matrix)
            centrality = nx.betweenness_centrality(self._adj_graph)
            leader_node_id = max(centrality, key=centrality.get)
            if leader_node_id == id:
                self._is_leader = True

        if enforce_formation:
            leaf_node = np.empty((0, 2))
            leader_node = np.empty((0, 2))

            for comm in all_comms:
                if "leaf" in comm.keys() and comm["leaf"]:
                    leaf_node = np.vstack((leaf_node, comm["state"][:2]))
                if "leader" in comm.keys() and comm["leader"]:
                    leader_node = comm["state"][:2]

            self._centroid_to_target = unit_vector(self._target - centroid)
            if leaf_node.shape[0] == 2 and leader_node.shape[0] > 0:
                self._first_second = unit_vector(
                    leaf_node[0, :] - leaf_node[1, :])

                dot_product = self._centroid_to_target.dot(self._first_second)
                if not self._set_theta:
                    self._set_theta = True
                    self._theta = np.sign(dot_product) * np.pi/2
                target_to_leader = leader_node - self._target
                target_to_centroid = centroid - self._target
                d_to_leader = np.linalg.norm(target_to_leader)
                d_to_target = np.linalg.norm(target_to_centroid)
                d_to_leaf_1 = np.linalg.norm(leaf_node[0, :] - self._target)
                d_to_leaf_2 = np.linalg.norm(leaf_node[1, :] - self._target)

                # print(dot_product)
                # Unreliable condition, must double check further
                if d_to_leader > d_to_target \
                        and d_to_leader > d_to_leaf_1 \
                        and d_to_leader > d_to_leaf_2 \
                        and np.abs(d_to_leaf_1 - d_to_leaf_2) <= 10.0:
                    self._set_theta = False
            else:
                self._theta = 0.0

            # Get the 2 leaf node to decide what direction to steer
            u_nom_flipped = np.array([[np.cos(self._theta), -np.sin(self._theta)],
                                      [np.sin(self._theta), np.cos(self._theta)]]).dot(u_nom.reshape(2, 1))
            # Animal steering
            u_nom = u_nom + u_nom_flipped.reshape((2,))
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
                                                   robot_states=floating_other_states,
                                                   min_distance=self._min_robot_d,
                                                   gamma_min=1.0,
                                                   max_distance=self._max_robot_d,
                                                   gamma_max=1.0,
                                                   relax_d_min=False,
                                                   relax_d_max=not enforce_formation)
        A = np.vstack((A, A_r_r))
        b = np.vstack((b, b_r_r))

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
                             p_omega=10000000.0,
                             omega_0=1.0)

        self._u = u_nom
        self._prev_x = x
        return u

    def display(self, screen: pygame.Surface):
        pygame.draw.line(
            screen, pygame.Color("yellow"),
            tuple(self._pose), tuple(self._pose + 5 * (self._u)))

        pygame.draw.line(
            screen, pygame.Color("yellow"),
            tuple(np.array([500, 300])), tuple(np.array([500, 300]) + 50 * (self._centroid_to_target)))
        pygame.draw.line(
            screen, pygame.Color("white"),
            tuple(np.array([500, 300])), tuple(np.array([500, 300]) + 50 * (self._first_second)))

        pygame.draw.circle(screen, pygame.Color("white"),
                           tuple(self._target), 30, 1)
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
        # if np.linalg.norm(v_sum) > 10.0:
        #     v_sum = utils.unit_vector(v_sum) * 10.0
        u = 10 * (v_sum - vi)
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
        # if np.linalg.norm(v_sum) > 10.0:
        #     v_sum = utils.unit_vector(v_sum) * 10.0
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
                u = (-vi)
            else:
                u = np.zeros(2)
        else:
            u = x[:2]

        return u, x
