# !/usr/bin/python3

import pygame
import numpy as np
from abc import ABC, abstractmethod


class Behavior(ABC):
    @abstractmethod
    def update(self, *args, **kwargs):
        '''
        '''
    @abstractmethod
    def display(self, screen: pygame.Surface):
        '''
        '''
