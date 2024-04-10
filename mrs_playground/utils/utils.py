"""Utility functions and classes."""

import numpy as np
import math
from mrs_playground.params import params


def randrange(a, b):
    """Random number between a and b."""
    return a + np.random.random() * (b - a)


def px_to_grid(px_pos):
    """Convert pixel position to grid position."""
    return np.array([px_pos[0] / params.COL, px_pos[1] / params.ROW])


def grid_to_px(grid_pos):
    """Convert grid position to pixel position."""
    return np.array([grid_pos[0] * params.COL, grid_pos[1] * params.ROW])


def norm(vector):
    """Compute the norm of a vector."""
    return math.sqrt(vector[0]**2 + vector[1]**2)


def norm2(vector):
    """Compute the square norm of a vector."""
    return vector[0] * vector[0] + vector[1] * vector[1]


def dist2(a, b):
    """Return the square distance between two vectors.

    Parameters
    ----------
    a : np.array
    b : np.array
    """
    return norm2(a - b)


def dist(a, b):
    """Return the distance between two vectors.

    Parameters
    ----------
    a : np.array
    b : np.array
    """
    return norm(a - b)


def normalize(vector, pre_computed=None):
    """Return the normalized version of a vector.

    Parameters
    ----------
    vector : np.array
    pre_computed : float, optional
        The pre-computed norm for optimization. If not given, the norm
        will be computed.
    """
    n = pre_computed if pre_computed is not None else norm(vector)
    if n < 1e-13:
        return np.zeros(2)
    else:
        return np.array(vector) / n


def truncate(vector, max_length):
    """Truncate the length of a vector to a maximum value."""
    n = norm(vector)
    if n > max_length:
        return normalize(vector, pre_computed=n) * max_length
    else:
        return vector


def unit_vector(vector):
    return np.array(vector) / (1+np.linalg.norm(vector))


class MathUtils():

    EPSILON = 0.1
    H = 0.2
    A, B = 5, 5
    C = np.abs(A-B)/np.sqrt(4*A*B)  # phi

    R = 40
    D = 40

    # Math functions for flocking
    @staticmethod
    def sigma_1(z):
        return z / np.sqrt(1 + z**2)

    @staticmethod
    def sigma_norm(z, e=EPSILON):
        return (np.sqrt(1 + e * np.linalg.norm(z, axis=-1, keepdims=True)**2) - 1) / e

    @staticmethod
    def sigma_norm_grad(z, e=EPSILON):
        return z/np.sqrt(1 + e * np.linalg.norm(z, axis=-1, keepdims=True)**2)

    @staticmethod
    def bump_function(z, h=H):
        ph = np.zeros_like(z)
        ph[z <= 1] = (1 + np.cos(np.pi * (z[z <= 1] - h)/(1 - h)))/2
        ph[z < h] = 1
        ph[z < 0] = 0
        return ph

    @staticmethod
    def phi(z, a=A, b=B, c=C):
        return ((a + b) * MathUtils.sigma_1(z + c) + (a - b)) / 2

    @staticmethod
    def phi_alpha(z, r=R, d=D):
        r_alpha = MathUtils.sigma_norm([r])
        d_alpha = MathUtils.sigma_norm([d])
        return MathUtils.bump_function(z/r_alpha) * MathUtils.phi(z-d_alpha)

    @staticmethod
    def normalise(v, pre_computed=None):
        n = pre_computed if pre_computed is not None else math.sqrt(
            v[0]**2 + v[1]**2)
        if n < 1e-13:
            return np.zeros(2)
        else:
            return np.array(v) / n

    # Math for bearing formation control
    @staticmethod
    def orthogonal_projection_matrix(v: np.ndarray):
        d = v.size
        v = np.reshape(v, (d, 1))
        return np.eye(d) - (v @ v.transpose()) / (np.linalg.norm(v)**2)

    @staticmethod
    def g_ij(vi: np.ndarray, vj: np.ndarray):
        return (vj - vi) / (np.linalg.norm(vj - vi))

    @staticmethod
    def sigma(x):
        return (1/(1 + math.exp(-x)))
    
