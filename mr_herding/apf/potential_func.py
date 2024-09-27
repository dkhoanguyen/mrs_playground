# !/usr/bin/python3

import numpy as np
from mrs_playground.utils import utils


class PotentialFunc(object):
    @staticmethod
    def const_attract_fuc(xi: np.ndarray, xj: np.ndarray,
                          d: float,
                          attract: bool = True,
                          repulse: bool = True,
                          c: float = 10,
                          m: float = 10):
        a = int(attract)
        r = int(repulse)

        xij = xi - xj
        xij_norm = np.linalg.norm(xij)

        n_xij_d = xij_norm - d
        # fx = a - r * np.exp(-n_xij_d/c)
        fx = a*((d - n_xij_d)/n_xij_d)*2
        gx = np.tanh(n_xij_d/m)

        # Potential function
        px = -fx
        return px

    @staticmethod
    def var_attract_func(xi: np.ndarray, xj: np.ndarray,
                         d: float,
                         attract: bool = True,
                         repulse: bool = True,
                         c: float = 10,
                         m: float = 10):
        a = 0.3
        r = int(repulse)

        xij = xi - xj
        xij_norm = np.linalg.norm(xij)

        n_xij_d = xij_norm - d
        fx = a*xij_norm - r * np.exp(-n_xij_d/c)
        gx = np.tanh(n_xij_d/m)

        # Potential function
        px = -fx*(gx**2)
        return px

    @staticmethod
    def density_func(xi: np.ndarray, xj: np.ndarray,
                     k: float):
        pass


class GradPotentionFunc(object):
    @staticmethod
    def const_attract_fuc(xi: np.ndarray, xj: np.ndarray,
                          vi: np.ndarray,
                          d: float):

        x = xi - xj
        xij_norm = np.linalg.norm(x)

        n_xij_d = xij_norm - d
        c = 2

        # Potential function
        # fx = 1/(1 + np.exp(-n_xij_d)) * np.tanh(n_xij_d)
        # exp_term = np.exp(-n_xij_d)

        fx = (1-np.exp(-n_xij_d/c))**3
        # dfx = 3 * ((1 - exp_term)**2) * exp_term
        # fx = ()

        px = -fx * utils.unit_vector(x)
        # px = dfx * utils.unit_vector(x)
        return px

    @staticmethod
    def replusion_func(xi: np.ndarray, xj: np.ndarray,
                       d: float):
        xij = xi - xj
        xij_norm = np.linalg.norm(xij)

        n_xij_d = xij_norm - d
        # fx = -np.exp(-n_xij_d)
        # gx = np.tanh(n_xij_d)

        # # Potential function
        # px = -fx*(gx**2)
        fx = 1/(1 + np.exp(-n_xij_d)) * np.tanh(n_xij_d)
        px = -fx
        return px
