import numpy as np
from scipy.io import savemat

class MathUtils:

    EPSILON = 0.1
    H = 0.2
    A, B = 5, 5
    C = np.abs(A-B)/np.sqrt(4*A*B)  # phi

    R = 40
    D = 40

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
        n = pre_computed if pre_computed is not None else np.linalg.norm(v)
        if n < 1e-13:
            return np.zeros(2)
        else:
            return np.array(v) / n

# Generate expected results
z = np.array([0.5, 1.5, 2.5])
v = np.array([3.0, 4.0])

results = {
    "sigma_1": MathUtils.sigma_1(z),
    "sigma_norm": MathUtils.sigma_norm(z),
    "sigma_norm_grad": MathUtils.sigma_norm_grad(z),
    "bump_function": MathUtils.bump_function(z),
    "phi": MathUtils.phi(z),
    "phi_alpha": MathUtils.phi_alpha(z),
    "normalise": MathUtils.normalise(v)
}

# Save results to a .mat file
savemat('expected_results.mat', results)
