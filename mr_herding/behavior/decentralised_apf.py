# !/usr/bin/python3

import time
import math
import pygame
import numpy as np
from collections import deque

from mrs_playground.common.behavior import Behavior
from mrs_playground.entity.obstacle import Hyperplane
from mrs_playground.utils import utils

from mr_herding.apf.potential_func import *


class DecentralisedAPF(Behavior):
    def __init__(self,
                 potential_func: dict,
                 Cs: float = 30.0,
                 Cr: float = 2.0,
                 Cv: float = 1.2,
                 Co: float = 1.0,
                 distance_to_target: float = 125.0,
                 interagent_spacing: float = 150.0,
                 obstacle_range: float = 40.0,
                 sensing_range: float = 700.0,
                 herding_target: np.ndarray = np.array([350, 350])):
        super().__init__()

        self._potential_func = potential_func
        self._Cs = Cs
        self._Cr = Cr
        self._Cv = Cv
        self._Co = Co

        self._distance_to_target = distance_to_target
        self._interagent_spacing = interagent_spacing
        self._obstacle_range = obstacle_range
        self._sensing_range = sensing_range

        self._pose = np.zeros(2)
        self._force_ps = np.zeros(2)
        self._force_po = np.zeros(2)
        self._force_u = np.zeros(2)

        self._stopped = False

        # Plotting
        self._herd_density_to_plot = np.empty((0, 2))

        self._time_horizon = 200
        self._total_energy = deque(maxlen=self._time_horizon)
        self._stablised_energy = deque(maxlen=10)

        self._triggered = False
        self._voronoi = None

        self._plot_force = False
        self._plot_range = False

        self._current_time = time.time()
        self._wait_time = time.time() + 10

        self._change_state = False
        self._change_state_again = False
        self._change_state_once_again = False

        self._target = herding_target

    def transition(self, state: np.ndarray,
                   other_states: np.ndarray,
                   animal_states: np.ndarray,
                   consensus_states: dict):

        for idx in range(animal_states.shape[0]):
            if np.linalg.norm(state[:2] - animal_states[idx, :2]) <= self._sensing_range:
                return True
        return False

    def update(self, *args, **kwargs):
        state: np.ndarray = kwargs["state"]
        other_states: np.ndarray = kwargs["robot_states"]
        animal_states: np.ndarray = kwargs["animal_states"]
        animal_centroid: np.ndarray = kwargs["animal_centroid"]
        comms: dict = kwargs["comms"]
        all_comms: list = kwargs["all_comms"]
        # Control signal
        self._pose = state[:2]
        u = np.zeros(2)
        all_shepherd_states = np.vstack((state, other_states))

        di = state[:2]
        d_dot_i = state[2:4]

        d_to_target = self._distance_to_target
        spacing = self._interagent_spacing

        # Consensus check
        delta_adjacency_vector = self._get_delta_adjacency_vector(
            animal_states,
            state,
            r=self._distance_to_target + 10)

        neighbor_herd_idxs = delta_adjacency_vector

        ds_to_nearest_edge = np.Inf
        mean_norm_r_to_sj = np.Inf

        if sum(neighbor_herd_idxs) > 0:
            sj = animal_states[neighbor_herd_idxs, :2]
            r_to_sj = di - sj
            norm_r_to_sj = np.linalg.norm(r_to_sj, axis=1)
            ds_to_nearest_edge = norm_r_to_sj.min()

            mean_r_to_sj = np.sum(r_to_sj, axis=0) / sum(neighbor_herd_idxs)
            mean_norm_r_to_sj = np.linalg.norm(mean_r_to_sj)

        comms.update({
            "nearest_neighbor_ds": ds_to_nearest_edge,
            "mean_ds_to_edge": mean_norm_r_to_sj
        })

        alpha_adjacency_matrix = self._get_alpha_adjacency_matrix(
            all_shepherd_states,
            r=self._interagent_spacing + 15)
        neighbor_shepherd_idxs = alpha_adjacency_matrix[0]
        dr_to_nearest_edge = np.Inf
        mean_norm_r_to_rj = np.Inf

        if sum(neighbor_shepherd_idxs) > 0:
            rj = all_shepherd_states[neighbor_shepherd_idxs, :2]
            r_to_rj = di - rj
            norm_r_to_rj = np.linalg.norm(r_to_rj, axis=1)
            if norm_r_to_rj.shape[0] > 1:
                smallest_1, smallest_2 = np.partition(norm_r_to_rj, 1)[0:2]
                dr_to_nearest_edge = smallest_1 + smallest_2
            else:
                dr_to_nearest_edge = np.Inf

            mean_r_to_rj = np.sum(r_to_rj, axis=0) / \
                sum(neighbor_shepherd_idxs)
            mean_norm_r_to_rj = np.linalg.norm(mean_r_to_rj)

        comms.update({
            "nearest_neighbor_2dr": dr_to_nearest_edge,
            "mean_dr_to_edge": mean_norm_r_to_rj
        })

        total_valid = 0
        total_nearest_neighbor_ds = 0
        mean_nearest_neighbor_ds = 0
        s_var = np.Inf

        total_variance = 0
        for comm in all_comms:
            if "nearest_neighbor_ds" not in comm.keys():
                continue
            if comm["nearest_neighbor_ds"] == np.Inf:
                continue
            total_nearest_neighbor_ds += comm["nearest_neighbor_ds"]
            total_valid += 1
        if total_valid > 0:
            mean_nearest_neighbor_ds = total_nearest_neighbor_ds / total_valid
            s_var = (mean_nearest_neighbor_ds -
                     ds_to_nearest_edge)**2 / total_valid

        total_valid = 0
        total_nearest_neighbor_dr = 0
        mean_nearest_neighbor_dr = 0
        r_var = np.Inf
        for consensus_state in all_comms:
            if "nearest_neighbor_2dr" not in consensus_state.keys():
                continue
            if consensus_state["nearest_neighbor_2dr"] == np.Inf:
                continue
            total_nearest_neighbor_dr += consensus_state["nearest_neighbor_2dr"]
            total_valid += 1
        if total_valid > 0:
            mean_nearest_neighbor_dr = total_nearest_neighbor_dr / total_valid
            r_var = (mean_nearest_neighbor_dr -
                     dr_to_nearest_edge)**2 / total_valid

        comms.update(
            {"distribution_evenness": s_var + r_var})

        # Stabilise formation
        total_valid = 0
        for consensus_state in all_comms:
            if "distribution_evenness" not in consensus_state.keys():
                continue
            total_variance += consensus_state["distribution_evenness"]

        # Get nearest distance to target
        d_nearest_to_target = np.linalg.norm(self._pose - self._target)
        comms.update(
            {"d_nearest_to_target": d_nearest_to_target})

        tuning_ps = 1
        tuning_po = 1
        tuning_pv = 1
        move_to_target = False

        # if self._change_state_again:
        #     return -(self._pose - np.array([1500, 350])) * 0.1
        # if self._change_state_once_again:
        #     tuning_ps = 0.1
        #     tuning_po = 5
        #     tuning_pv = 0.5
        #     move_to_target = True
        #     herd_mean = np.sum(
        #         animal_states[:, :2], axis=0) / animal_states.shape[0]

        #     d_from_cetroid_to_target = np.linalg.norm(
        #         herd_mean - np.array([1000, 350]))

        #     # if d_from_cetroid_to_target <= 25:
        #     #     self._interagent_spacing = 200

        # if self._change_state and not self._change_state_again and not self._change_state_once_again:
        #     all_d_to_target = []
        #     for consensus_state in consensus_states:
        #         if "d_nearest_to_target" not in consensus_state.keys():
        #             continue
        #         all_d_to_target.append(consensus_state["d_nearest_to_target"])
        #     all_d_to_target = np.array(all_d_to_target)
        #     sorted_all_d_to_target = np.sort(all_d_to_target)
        #     nearest_3 = sorted_all_d_to_target[:3]

        #     if np.isin(d_nearest_to_target, nearest_3):
        #         self._change_state_again = True
        #         return -(self._pose - np.array([1500, 350])) * 0.1
        #     else:
        #         tuning_ps = 0.1
        #         tuning_po = 5
        #         tuning_pv = 0.5
        #         move_to_target = True
        #         self._change_state_once_again = True

        # Control
        delta_adjacency_vector = self._get_delta_adjacency_vector(
            animal_states,
            state,
            r=self._sensing_range)

        alpha_adjacency_matrix = self._get_alpha_adjacency_matrix(
            all_shepherd_states,
            r=self._interagent_spacing)

        neighbor_herd_idxs = delta_adjacency_vector
        ps = np.zeros(2)
        if sum(neighbor_herd_idxs) > 0:
            sj = animal_states[neighbor_herd_idxs, :2]
            s_dot_j = animal_states[neighbor_herd_idxs, 2:4]
            ps = self._potential_edge_following(qi=di,
                                                qj=sj,
                                                d=d_to_target,
                                                gain=self._Cs)

        po = np.zeros(2)
        neighbor_shepherd_idxs = alpha_adjacency_matrix[0]

        if sum(neighbor_shepherd_idxs) > 0:
            dj = all_shepherd_states[neighbor_shepherd_idxs, :2]
            po = self._collision_avoidance_term(
                gain=self._Cr,
                qi=di, qj=dj,
                d=spacing)
        self._force_po = po

        # Velocity consensus
        pv = np.zeros((2,))
        if sum(neighbor_herd_idxs) > 0:
            sj = animal_states[neighbor_herd_idxs, :2]
            s_dot_j = animal_states[neighbor_herd_idxs, 2:4]
            pv = self._velocity_consensus(pj=s_dot_j,
                                          gain=self._Cv)

        # Obstacle avoidance term
        obstacles = []
        if time.time() <= self._wait_time:
            hyperplane = self._verical_imaginary_hyperplane(
                animal_states, all_shepherd_states, 400)
            obstacles.append(hyperplane)
        beta_adj_vec = self._get_beta_adjacency_vector(state=state,
                                                       obstacles=obstacles,
                                                       r=self._sensing_range)
        p_avoid = np.zeros((2,))
        if sum(beta_adj_vec) > 0:
            p_avoid = self._obstacle_avoidance(qi=di, pi=d_dot_i,
                                               beta_adj_vec=beta_adj_vec,
                                               obstacle_list=obstacles,
                                               d=self._obstacle_range,
                                               gain=self._Co)
        herd_mean = np.sum(
            animal_states[:, :2], axis=0) / animal_states.shape[0]

        # d_from_cetroid_to_target = np.linalg.norm(
        #     herd_mean - np.array([1000, 350]))

        if 1/total_variance:
            move_to_target = True
            tuning_ps = 0.5
            tuning_po = 5
            tuning_pv = 0.5

        u = tuning_ps*ps + tuning_po*po + tuning_pv*pv + p_avoid

        if move_to_target:
            u = u + 0.075*(-(herd_mean - self._target))

        self._force_u = u
        return u

    def display(self, screen: pygame.Surface):
        if self._plot_force:
            pygame.draw.line(
                screen, pygame.Color("white"),
                tuple(self._pose), tuple(self._pose + 5 * (self._force_po)))
            pygame.draw.line(
                screen, pygame.Color("yellow"),
                tuple(self._pose), tuple(self._pose + 5 * (self._force_ps)))
        pygame.draw.line(
            screen, pygame.Color("blue"),
            tuple(self._pose), tuple(self._pose + 2 * (self._force_u)))

        pygame.draw.circle(screen, pygame.Color("black"),
                           tuple(self._target), 50, 1)
        if self._plot_range:
            pygame.draw.circle(screen, pygame.Color("white"),
                               tuple(self._pose), self._distance_to_target, 1)
            pygame.draw.circle(screen, pygame.Color("yellow"),
                               tuple(self._pose), self._interagent_spacing, 1)

        return super().display(screen)

    def _collision_avoidance_term(self, gain: float,
                                  qi: np.ndarray, qj: np.ndarray,
                                  d: float):

        u_sum = np.zeros(2).astype(np.float64)
        for i in range(qj.shape[0]):
            uij = qi - qj[i, :]
            func_input = self._potential_func['col_avoid']
            func_input.update({'xi': qi, 'xj': qj[i, :], 'd': d})
            p = PotentialFunc.const_attract_fuc(**func_input)
            u_sum += gain * p * utils.unit_vector(uij)
        return u_sum

    def _get_delta_adjacency_vector(self, animal_states: np.ndarray,
                                    shepherd_state: np.ndarray, r: float) -> np.ndarray:
        adj_vector = []
        for i in range(animal_states.shape[0]):
            adj_vector.append(np.linalg.norm(
                shepherd_state[:2] - animal_states[i, :2]) <= r)
        return np.array(adj_vector, dtype=np.bool8)

    def _get_alpha_adjacency_matrix(self, agent_states: np.ndarray,
                                    r: float) -> np.ndarray:
        adj_matrix = np.array(
            [np.linalg.norm(agent_states[i, :2]-agent_states[:, :2], axis=-1) <= r
             for i in range(agent_states.shape[0])])
        np.fill_diagonal(adj_matrix, False)
        return adj_matrix

    def _calc_group_objective_control(self, target: np.ndarray,
                                      c1: float, c2: float,
                                      qi: np.ndarray, pi: np.ndarray):
        def calc_group_objective_term(
                c1: float, c2: float,
                pos: np.ndarray, qi: np.ndarray, pi: np.ndarray):
            return -c1 * utils.MathUtils.sigma_1(qi - pos) - c2 * (pi)
        u_gamma = calc_group_objective_term(
            c1=c1,
            c2=c2,
            pos=target,
            qi=qi,
            pi=pi)
        return u_gamma

    def _potential_edge_following(self, qi: np.ndarray, qj: np.ndarray,
                                  d: float,
                                  gain: float):
        u_sum = np.zeros(2).astype(np.float64)
        for i in range(qj.shape[0]):
            uij = qi - qj[i, :]
            func_input = self._potential_func['edge_follow']
            func_input.update({'xi': qi, 'xj': qj[i, :], 'd': d})
            p = PotentialFunc.const_attract_fuc(**func_input)
            u_sum += gain * p * utils.unit_vector(uij)
        return u_sum

    def _velocity_consensus(self, pj: np.ndarray, gain: float):
        u_sum = np.zeros(2).astype(np.float64)
        for i in range(pj.shape[0]):
            u_sum += pj[i, :]
        return gain * u_sum / pj.shape[0]

    def _formation_stable(self):
        if len(self._total_energy) != self._time_horizon:
            return False
        y = np.array(self._total_energy)
        x = np.linspace(0, self._time_horizon,
                        self._time_horizon, endpoint=False)
        coeff, err, _, _, _ = np.polyfit(x, y, deg=1, full=True)
        if math.sqrt(err) >= 1200:
            return False
        poly = np.poly1d(coeff)
        polyder = np.polyder(poly)
        cond = np.abs(np.round(float(polyder.coef[0]), 1))
        return not bool(cond)

    def _get_beta_adjacency_vector(self, state: np.ndarray,
                                   obstacles: list, r: float) -> np.ndarray:
        adj_vec = []
        for obstacle in obstacles:
            adj_vec.append(obstacle.in_entity_radius(state[:2], r=r))
        return np.array(adj_vec)

    def _obstacle_avoidance(self, qi: np.ndarray,
                            pi: np.ndarray,
                            beta_adj_vec: np.ndarray,
                            obstacle_list: list,
                            d: float,
                            gain: float):
        obs_in_radius = np.where(beta_adj_vec)
        u_sum = np.zeros(2).astype(np.float64)
        for obs_idx in obs_in_radius[0]:
            beta_agent = obstacle_list[obs_idx].induce_beta_agent(
                qi, pi)
            qj = beta_agent[:2]
            qij = qi - qj
            func_input = self._potential_func['obs_avoid']
            func_input.update({'xi': qi, 'xj': qj, 'd': d})
            p = PotentialFunc.const_attract_fuc(**func_input)
            u_sum += gain * p * utils.unit_vector(qij)
        return u_sum

    def _density(self, si: np.ndarray, sj: np.ndarray, k: float, d: float):
        w_sum = np.zeros(2).astype(np.float64)
        for i in range(sj.shape[0]):
            sij = si - sj[i, :]
            w = np.abs(np.linalg.norm(sij) - d) * utils.unit_vector(sij)
            w_sum += w
        return w_sum

    def _calc_density(self, idx: int,
                      neighbors_idxs: np.ndarray,
                      animal_states: np.ndarray):
        qi = animal_states[idx, :2]
        density = np.zeros(2)
        if sum(neighbors_idxs) > 0:
            qj = animal_states[neighbors_idxs, :2]
            density = self._density(si=qi, sj=qj, k=0.375, d=0)
        return density

    def _herd_density(self, animal_states: np.ndarray,
                      shepherd_states: np.ndarray,
                      r_shepherd: float):
        herd_densities = np.zeros((animal_states.shape[0], 2))
        alpha_adjacency_matrix = self._get_alpha_adjacency_matrix(animal_states,
                                                                  r=40)
        for idx in range(animal_states.shape[0]):
            # Herd internal density
            neighbor_idxs = alpha_adjacency_matrix[idx]
            density = self._calc_density(
                idx=idx, neighbors_idxs=neighbor_idxs,
                animal_states=animal_states)
            herd_densities[idx] += density

            # Herd shepherd density
            delta_adj_vec = self._get_delta_adjacency_vector(
                shepherd_state=animal_states[idx, :2],
                animal_states=shepherd_states,
                r=self._sensing_range)

            qi = animal_states[idx, :2]
            qj = shepherd_states[delta_adj_vec, :2]
            density = self._density(si=qi, sj=qj, k=0.375, d=r_shepherd)
            herd_densities[idx] += density
        return herd_densities

    # Intitial State constraints
    def _verical_imaginary_hyperplane(self, animal_states: np.ndarray,
                                      shepherd_states: np.ndarray,
                                      r: float):
        herd_mean = np.sum(
            animal_states[:, :2], axis=0) / animal_states.shape[0]
        shepherd_mean = np.sum(
            shepherd_states[:, :2], axis=0) / shepherd_states.shape[0]
        hs = shepherd_mean - herd_mean
        d_hs = np.linalg.norm(hs)
        unit_hs = utils.unit_vector(hs)
        point = herd_mean + r * unit_hs
        hyperplane = Hyperplane(
            ak=unit_hs, yk=point, boundary=np.array([[0, 0], [0, 0]]))
        return hyperplane
