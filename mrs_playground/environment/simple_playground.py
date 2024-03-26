#!/usr/bin/python3

import time
import pickle
import pygame
import numpy as np

from mrs_playground.params import params

from mrs_playground.common.entity import Entity
from mrs_playground.common.behavior import Behavior


class SimplePlayground(object):
    def __init__(self,
                 entity_names: list,
                 dt: float,
                 render: bool = True,
                 multi_threaded: bool = False,
                 save_to_file: bool = True,
                 save_path: str = "data/"):

        self._dt = dt
        self._multi_threaded = multi_threaded
        self._render = render
        self._save_to_file = save_to_file
        self._save_path = save_path

        # Pygame for visualisation
        if self._render:
            pygame.init()
            self._screen = pygame.display.set_mode(params.SCREEN_SIZE)
            self._rect = self._screen.get_rect()
            self._clock = pygame.time.Clock()
        self._running = True

        self._entities = {}
        self._behaviors = []

        self._entity_names = entity_names

        for entity_name in self._entity_names:
            self._entities[entity_name] = []

        self._running = True

    @property
    def ok(self):
        return self._running

    def add_entity(self, entity: Entity):
        self._entities[entity.__str__()].append(entity)

    def add_behaviour(self, behavior: Behavior):
        self._behaviors.append(behavior)

    def display(self):
        entity: Entity
        for entities in self._entities.values():
            for entity in entities:
                entity.display(self._screen)
        behavior: Behavior
        for behavior in self._behaviors:
            behavior.display(self._screen)

    def run_once(self):
        events = pygame.event.get()
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self._running = False

        behavior: Behavior
        for behavior in self._behaviors:
            behavior.update(events)

        # Grab all states
        all_states = {}
        for entity_type in self._entities.keys():
            all_states[entity_type] = np.empty((0, 6))
            entity: Entity
            for entity in self._entities[entity_type]:
                all_states[entity_type] = np.vstack(
                    (all_states[entity_type], entity.state))

        entity: Entity
        if not self._multi_threaded:
            for entity in self._entities["robot"]:
                entity.update(events=events,
                              entity_states=all_states,
                              dt=self._dt)

    def render(self):
        if self._render:
            self._screen.fill(params.SIMULATION_BACKGROUND)
            self.display()
            pygame.display.flip()
            self._clock.tick(params.FPS)
            pygame.display.set_caption("fps: " + str(self._clock.get_fps()))

    def quit(self):
        for event in pygame.event.get():
            if event.type == pygame.KEYDOWN and event.unicode.isalpha():
                letter = event.unicode.upper()
                if letter == 'Q':
                    self._running = False
                    return True
        return False
