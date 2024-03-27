# !/usr/bin/python3

import pygame
import numpy as np
from abc import ABC, abstractmethod


class Behavior(ABC):
    @abstractmethod
    def update(self, state: np.ndarray,
               other_states: np.ndarray,
               animal_states: np.ndarray):
        '''
        '''
    @abstractmethod
    def display(self, screen: pygame.Surface):
        '''
        '''
