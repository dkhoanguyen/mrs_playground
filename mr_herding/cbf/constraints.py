# !/usr/bin/python3

from typing import List
import numpy as np

from mrs_playground.utils.utils import *

class Plane(object):
    def __init__(self):
        self.normal = np.zeros((2, 1))
        self.point = np.zeros((2, 1))


class ORCA():
    @staticmethod
    def construct_orca_plane(xi: np.ndarray, xj: np.ndarray,
                             vi: np.ndarray, vj: np.ndarray,
                             ri: float, rj: float,
                             weight: float = 1.0,
                             buffered_r: float = 0.0,
                             time_horizon: float = 1.0):
        '''
        Construct ORCA plane for each pair of i and j agents
        '''
        if np.linalg.norm(vj) <= 0.001:
            weight = 1.0
        plane = Plane()
        inv_time_horizon = 1.0/time_horizon
        x_ji = xj - xi
        v_ij = vi - vj
        x_ji_norm = np.linalg.norm(x_ji)
        r_ij = ri + rj + buffered_r

        # Check whether velocity lies within the VO
        # Find w and w norm
        w = v_ij - inv_time_horizon * x_ji
        w_norm = np.linalg.norm(w)

        # Find alpha, phi angle
        alpha = angle_between(w, -x_ji)
        phi = np.arccos((r_ij * inv_time_horizon) /
                        x_ji_norm * inv_time_horizon)

        if x_ji_norm <= r_ij:
            # print("Collision")
            inv_time_step = 1.0 / 0.1
            w = v_ij - inv_time_step * x_ji
            w_norm = np.linalg.norm(w)
            unit_w = w / w_norm

            plane.normal = unit_w.reshape((2, 1))

            # Find projected point
            u = (r_ij * inv_time_step - w_norm) * unit_w
            plane.point = (vi + weight * u).reshape((2, 1))

            return plane

        if alpha <= abs(phi):
            if w_norm <= (r_ij * inv_time_horizon):
                # print("Project on cut off circle (1)")
                # Find normal
                # Add offset to prevent stuck
                # if abs(alpha) <= 0.01:
                #     w = w - 0.1 * unit_vector(np.array([x_ji[1], x_ji[0]]))
                #     w_norm = np.linalg.norm(w)
                unit_w = w / w_norm
                plane.normal = unit_w.reshape((2, 1))

                # Find projected point
                u = (r_ij * inv_time_horizon - w_norm) * unit_w
                plane.point = (vi + weight * u).reshape((2, 1))

                return plane
            else:
                # print("v_ij is outside of VO cone (0)")
                return None
        else:
            # alpha > (pi - phi)
            if w_norm <= (r_ij * inv_time_horizon):
                # print("Project on cone (2)")
                # Find angle between v_ij and x_ji called beta
                beta = angle_between(v_ij, x_ji)
                # Find angle between v_ij and projected point p called gamma
                gamma = (np.pi/2 - phi) - beta
                norm_p = np.cos(gamma) * np.linalg.norm(v_ij)

                # Check to see if v_ij is closer to which leg
                angle_x_ji = angle_between(x_ji, np.array([1, 0]))
                angle_v_ij = angle_between(v_ij, np.array([1, 0]))
                lower_bound = angle_x_ji - phi
                upper_bound = angle_x_ji + phi

                p = np.zeros((1, 2))
                if np.abs(angle_v_ij - upper_bound) <= np.abs(angle_v_ij - lower_bound):
                    unit_p = simple_vector_rotate(unit_vector(v_ij), gamma)
                    p = norm_p * unit_p
                else:
                    unit_p = simple_vector_rotate(unit_vector(v_ij), -gamma)
                    p = norm_p * unit_p

                # Find normal n and point u
                n = p - v_ij
                plane.normal = unit_vector(n).reshape((2, 1))
                plane.point = (vi + weight * n).reshape((2, 1))
                return plane
            else:
                def check_vij_in_cone(v_ij: np.ndarray, x_ji: np.ndarray, phi: float):
                    angle_x_ji = angle_between(x_ji, np.array([1, 0]))
                    angle_v_ij = angle_between(v_ij, np.array([1, 0]))

                    lower_bound = angle_x_ji - phi
                    upper_bound = angle_x_ji + phi

                    return lower_bound <= angle_v_ij and angle_v_ij <= upper_bound
                if not check_vij_in_cone(v_ij=v_ij, x_ji=x_ji, phi=phi):
                    # print("v_ij is outside of VO cone (4)")
                    return None
                else:
                    # print("Project on cone (3)")
                    # Find angle between v_ij and x_ji called beta
                    beta = angle_between(v_ij, x_ji)
                    # Find angle between v_ij and projected point p called gamma
                    gamma = (np.pi/2 - phi) - beta
                    norm_p = np.cos(gamma) * np.linalg.norm(v_ij)

                    # Check to see if v_ij is closer to which leg
                    angle_x_ji = angle_between(x_ji, np.array([1, 0]))
                    angle_v_ij = angle_between(v_ij, np.array([1, 0]))
                    lower_bound = angle_x_ji - phi
                    upper_bound = angle_x_ji + phi

                    p = np.zeros((1, 2))
                    if np.abs(angle_v_ij - upper_bound) <= np.abs(angle_v_ij - lower_bound):
                        unit_p = simple_vector_rotate(unit_vector(v_ij), gamma)
                        p = norm_p * unit_p
                    else:
                        unit_p = simple_vector_rotate(
                            unit_vector(v_ij), -gamma)
                        p = norm_p * unit_p

                    # Find normal n and point u
                    n = p - v_ij
                    plane.normal = unit_vector(n).reshape((2, 1))
                    plane.point = (vi + weight * n).reshape((2, 1))
                    return plane

    @staticmethod
    def custom_orca_plane(xi: np.ndarray,
                          target_pose: np.ndarray,
                          animal_centroid: np.ndarray,
                          offset: float, is_left: bool):
        plane = Plane()
        x_lc = unit_vector(target_pose - animal_centroid)

        if is_left:
            # plane.normal = np.array([-x_lc[1], x_lc[0]])
            plane.normal = np.array([0,-1])
        else:
            # plane.normal = np.array([x_lc[1], -x_lc[0]])
            plane.normal = np.array([0,1])
        plane.point = animal_centroid + plane.normal * offset
        # print(plane.point)
        
        a, b = plane.normal
        x1, y1 = plane.point
        x2, y2 = xi
        
        v = (x2 - x1, y2 - y1)
        dot_product = a * v[0] + b * v[1]
        if dot_product < 0:
            return None

        return plane

    @staticmethod
    def construct_orca_planes(xi: np.ndarray, xj: np.ndarray,
                              vi: np.ndarray, vj: np.ndarray,
                              ri: float, rj: np.ndarray,
                              weight: np.ndarray,
                              buffered_r: float = 0.0,
                              time_horizon: float = 1.0):
        planes = []
        for i in range(xj.shape[0]):
            plane = ORCA.construct_orca_plane(xi=xi, xj=xj[i, :], vi=vi, vj=vj[i, :],
                                              ri=ri, rj=rj[i], weight=weight[i],
                                              buffered_r=buffered_r,
                                              time_horizon=time_horizon)
            if plane is not None:
                planes.append(plane)
        return planes

    @staticmethod
    def build_constraint(orca_planes: List[Plane],
                         vi: np.ndarray,
                         gamma: float = 1.0,
                         relax: bool = False):
        A = np.empty((0, 4))
        b = np.empty((0, 1))
        for plane in orca_planes:
            # Obtain normal and point of the orca plane
            w = plane.normal
            n = plane.point
            h = gamma * (w.transpose().dot(vi.reshape((2, 1))) -
                         w.transpose().dot(n))
            h = h.reshape(1)

            # Optimal decay to ensure feasibility
            row_A = np.append(-w.transpose(), -h)
            row_A = np.append(row_A, int(relax))
            A = np.vstack((A, row_A))

            b = np.vstack([b, 0])
        return A, b


class MinDistance:
    @staticmethod
    def build_constraint(xi: np.ndarray, xj: np.ndarray,
                         vi: np.ndarray, vj: np.ndarray,
                         ai: float, aj: float,
                         d: float, gamma: float,
                         relax: bool = False):
        A = np.empty((0, 4))
        # A = np.empty((0, 2))
        b = np.empty((0, 1))
        for idx in range(xj.shape[0]):
            xij = xi - xj[idx, :]
            vij = vi - vj[idx, :]

            xij_norm = np.linalg.norm(xij)
            vij_norm = np.linalg.norm(vij)

            h_min = np.sqrt(2 * (ai + aj) * (xij_norm - d)
                            ) + (xij / xij_norm).dot(vij.transpose())
            gamma_h_min = gamma * \
                (h_min**3) * xij_norm - (2 * vij.dot(xij.transpose()))/(xij_norm**2) + \
                vij_norm**2 + ((ai + aj) * vij.dot(xij.transpose()))/np.sqrt(
                    2 * (ai + aj) * (xij_norm - d))
            h_dot = xij
            row_A = np.append(-h_dot, -(ai/(ai + aj))*gamma_h_min)
            row_A = np.append(row_A, int(relax))
            A = np.vstack((A, row_A))
            b = np.vstack([b, 0.0])

        return A, b


class MaxDistance:
    @staticmethod
    def build_constraint(xi: np.ndarray, xj: np.ndarray,
                         vi: np.ndarray, vj: np.ndarray,
                         ai: float, aj: float,
                         d: float, gamma: float,
                         relax: bool = False):
        A = np.empty((0, 4))
        # A = np.empty((0, 2))
        b = np.empty((0, 1))
        for idx in range(xj.shape[0]):
            xij = xi - xj[idx, :]
            vij = vi - vj[idx, :]

            xij_norm = np.linalg.norm(xij)
            vij_norm = np.linalg.norm(vij)

            sqrt_x_d = np.sqrt(2 * (ai + aj) * (d - xij_norm))

            h_max = sqrt_x_d - (xij / xij_norm).dot(vij.transpose())

            gamma_h_max = gamma * (h_max**3) * xij_norm \
                + (vij.dot(xij.transpose())) ** 2/(xij_norm**2) \
                - vij_norm**2 \
                - ((ai + aj) * vij.dot(xij.transpose()))/sqrt_x_d
            h_dot = xij
            row_A = np.append(h_dot, -(ai/(ai + aj))*gamma_h_max)
            row_A = np.append(row_A, int(relax))
            A = np.vstack((A, row_A))
            b = np.vstack([b,  0.0])

        return A, b

class FormClosure():
    @staticmethod
    def build_constrain(xi: np.ndarray, xj: np.ndarray,
                         vi: np.ndarray, vj: np.ndarray,):
        pass

def unit_vector(v: np.ndarray):
    return v / np.linalg.norm(v)


def angle_between(v1: np.ndarray, v2: np.ndarray):
    u1 = unit_vector(v1)
    u2 = unit_vector(v2)
    return np.arccos(np.clip(np.dot(u1, u2), -1.0, 1.0))


def angle_between_with_direction(v1: np.ndarray, v2: np.ndarray):
    dot_product = np.dot(v1, v2)
    magnitude_v1 = np.linalg.norm(v1)
    magnitude_v2 = np.linalg.norm(v2)
    cos_theta = dot_product / (magnitude_v1 * magnitude_v2)
    angle_radians = np.arccos(np.clip(cos_theta, -1.0, 1.0))
    
    # Determine the sign of the angle using the cross product
    cross_product = np.cross(v1, v2)
    if cross_product < 0:
        angle_radians = -angle_radians
    
    return angle_radians

def simple_vector_rotate(v: np.ndarray, theta: float):
    cs = np.cos(theta)
    sn = np.sin(theta)
    x = v[0]
    y = v[1]
    x_new = x * cs - y * sn
    y_new = x * sn + y * cs
    return np.array([x_new, y_new])
