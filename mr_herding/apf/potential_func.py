# !/usr/bin/python3

import numpy as np


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
        fx = a - r * np.exp(-n_xij_d/c)
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
                          d: float):

        xij = xi - xj
        xij_norm = np.linalg.norm(xij)

        n_xij_d = xij_norm - d

        # sech = 1/np.cosh(n_xij_d)
        tanh = np.tanh(n_xij_d)

        # Potential function
        # fx = ((n_xij_d * sech**2 * (np.abs(n_xij_d) + 1) + tanh)) / \
        #     (np.abs(n_xij_d) + 1)**2
        fx = 1/(1 + np.exp(-n_xij_d)) * tanh
        px = -fx
        return px
