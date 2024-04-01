#!/usr/bin/python3

import os
import yaml

from mrs_playground.entity.animal import Animal
from mrs_playground.entity.robot import Robot

from mrs_playground.dynamic.point_mass_system import *
from mrs_playground.sensing.radius_sensing import RadiusSensing

from mrs_playground.behavior.mathematical_flock import MathematicalFlock

from mrs_playground.environment.simple_playground import SimplePlayground
from mrs_playground.environment.playground_factory import PlaygroundFactory

from mr_herding.behavior.decentralised_apf import DecentralisedAPF
from mr_herding.behavior.decentralised_cbf import DecentralisedCBF

PROJECT_DIR = os.path.dirname(os.path.abspath(__file__))
CONFIG_DIR = os.path.join(PROJECT_DIR, 'config')


def main():

    entity_config_file = os.path.join(CONFIG_DIR, 'entity.yaml')
    with open(entity_config_file, 'r') as file:
        entity_config = yaml.safe_load(file)

    env_config_file = os.path.join(CONFIG_DIR, 'playground.yaml')
    with open(env_config_file, 'r') as file:
        env_config = yaml.safe_load(file)

    behaviour_config_file = os.path.join(CONFIG_DIR, 'behavior.yaml')
    with open(behaviour_config_file, 'r') as file:
        behavior_config = yaml.safe_load(file)

    sensing_config_file = os.path.join(CONFIG_DIR, 'sensing_model.yaml')
    with open(sensing_config_file, 'r') as file:
        sensing_config = yaml.safe_load(file)

    dynamic_config_file = os.path.join(CONFIG_DIR, 'dynamic_model.yaml')
    with open(dynamic_config_file, 'r') as file:
        dynamic_config = yaml.safe_load(file)

    # Spawn animals
    animals = PlaygroundFactory.spawn_entities(entity_config['animal'], Animal)

    # Animal flocking behavior
    math_flock_config = behavior_config['math_flock']
    math_flock = MathematicalFlock(**math_flock_config['params'])
    for animal in animals:
        math_flock.add_animal(animal)

    # Spawn robots
    robots = PlaygroundFactory.spawn_entities(entity_config['robot'], Robot)

    # Add sensors and dynamics to robots
    sensors = PlaygroundFactory.add_sensing(entities=robots,
                                            config=sensing_config,
                                            sensing_type=RadiusSensing)
    PlaygroundFactory.add_dynamic(entities=robots,
                                  config=dynamic_config,
                                  dynamic_type=DoubleIntegrator)
    # Add behavior as well
    PlaygroundFactory.add_behavior(entities=robots,
                                   config=behavior_config['herding_cbf']['params'],
                                   behavior_type=DecentralisedCBF,
                                   behavior_name="cbf")
    
    # Add robots
    for robot in robots:
        math_flock.add_robot(robot)

    # Create environment
    env = SimplePlayground(**env_config)

    # Add entities to env
    for animal in animals:
        env.add_entity(animal)
    for robot in robots:
        env.add_entity(robot)

    env.add_behaviour(math_flock)

    # Add sensor
    for sensor in sensors:
        env.add_sensing_models(sensor)

    while env.ok:
        env.step()
        env.render()


if __name__ == '__main__':
    main()
