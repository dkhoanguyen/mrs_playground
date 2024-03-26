# !/usr/bin/python3

from abc import ABC, abstractmethod
import numpy as np

class Dynamic(ABC):
    @abstractmethod
    def step(self, x_t: np.ndarray, u_t: np.ndarray) -> np.ndarray:
        """
        """