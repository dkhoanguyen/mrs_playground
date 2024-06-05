#!/usr/bin/python3

from typing import Dict, List

import pygame
import numpy as np

from mrs_playground.params import params

from mrs_playground.common.entity import Entity
from mrs_playground.common.behavior import Behavior
from mrs_playground.common.sensing import SensingModel


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

        self._entities: Dict[str, List[Entity]] = {}
        self._behaviors: List[Behavior] = []
        self._sensing_model: List[SensingModel] = []

        self._entity_names: List[str] = entity_names

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

    def add_sensing_models(self, sensing_model: SensingModel):
        self._sensing_model.append(sensing_model)

    def display(self):
        entity: Entity
        for entities in self._entities.values():
            for entity in entities:
                entity.display(self._screen)
        behavior: Behavior
        for behavior in self._behaviors:
            behavior.display(self._screen)

    def step(self):
        events = pygame.event.get()
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self._running = False

        behavior: Behavior
        for behavior in self._behaviors:
            behavior.update(events)
            if behavior._is_at_target:
                self._running = False

        # Grab all states
        all_states = {}
        for entity_type in self._entities.keys():
            all_states[entity_type] = np.empty((0, 6))
            for entity in self._entities[entity_type]:
                all_states[entity_type] = np.vstack(
                    (all_states[entity_type], entity.state))

        # Delegate all states to sensors, ie sensors are "sensing" the environment
        for sensing_model in self._sensing_model:
            sensing_model.update(all_states=all_states)

        # All comms
        all_comms = []
        if 'robot' in self._entity_names:
            for entity in self._entities['robot']:
                all_comms.append(entity.comms)

        # Update all entities
        for entity_type in self._entities.keys():
            for entity in self._entities[entity_type]:
                entity.update(events=events,
                              comms=all_comms)

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
