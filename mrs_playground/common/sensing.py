#!/usr/bin/python3

from abc import ABC, abstractmethod
import numpy as np


class SensingModel(ABC):

    def __init__(self):
        self._all_states = np.empty((0, 6))
    
    def update(self, all_states: dict):
        self._all_states = all_states

    @abstractmethod
    def sense(self, state: np.ndarray,
              target: str):
        """
        """
