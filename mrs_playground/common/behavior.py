# !/usr/bin/python3

import pygame
import numpy as np
from abc import ABC, abstractmethod


class Behavior(ABC):
    @abstractmethod
    def update(self, state: np.ndarray,
               other_states: np.ndarray,
               herd_states: np.ndarray,
               consensus_states: dict,
               output_consensus_state: dict):
        '''
        '''
    @abstractmethod
    def display(self, screen: pygame.Surface):
        '''
        '''

    def _get_events(self, args):
        events = []
        for arg in args:
            if len(arg) == 0:
                continue
            for element in arg:
                if isinstance(element, pygame.event.Event):
                    events.append(element)

        return events
