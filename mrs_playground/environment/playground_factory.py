#!/usr/bin/python3

from typing import List
import numpy as np

from mrs_playground.common.entity import Entity
from mrs_playground.common.sensing import SensingModel
from mrs_playground.common.dynamic import DynamicModel
from mrs_playground.common.behavior import Behavior


class PlaygroundFactory(object):

    @staticmethod
    def spawn(entity, config):
        return entity(**config)

    @staticmethod
    # We might need to make this reuseable in the future
    def spawn_entities(config: dict, entity_type: Entity) -> List[Entity]:
        entities = []
        num = config.pop('num')
        x_interval = config.pop('x_interval')
        y_interval = config.pop('y_interval')
        x = np.random.randint(
            x_interval[0], x_interval[1], (num, 1)).astype('float')
        y = np.random.randint(
            y_interval[0], y_interval[1], (num, 1)).astype('float')
        initial_poses = np.hstack((x, y))

        for i in range(num):
            angle = np.pi * (2 * np.random.rand() - 1)
            vel = 0.0 * np.array([np.cos(angle), np.sin(angle)])

            config['pose'] = initial_poses[i, :]
            config['velocity'] = vel

            # For robots only
            if 'id' in config.keys():
                config['id'] = i

            entity = PlaygroundFactory.spawn(
                entity=entity_type, config=config)

            entities.append(entity)
        return entities

    @staticmethod
    def add_sensing(entities: List[Entity], config: dict, sensing_type: SensingModel):
        sensors = []
        for entity in entities:
            sensor = sensing_type(**config)
            entity.set_sensing_model(sensing=sensor)
            sensors.append(sensor)
        return sensors

    @staticmethod
    def add_dynamic(entities: List[Entity], config: dict, dynamic_type: DynamicModel):
        for entity in entities:
            dynamic = dynamic_type(**config)
            entity.set_dynamic_model(dynamic=dynamic)

    @staticmethod
    def add_behavior(entities: List[Entity], config: dict,
                     behavior_type: Behavior, behavior_name: str):
        for entity in entities:
            behavior = {behavior_name: behavior_type(**config)}
            entity.add_behavior(behavior)
