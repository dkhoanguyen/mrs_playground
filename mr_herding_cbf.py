#!/usr/bin/python3

import os
import yaml
import argparse

from mrs_playground.entity.animal import Animal
from mrs_playground.entity.robot import Robot

from mrs_playground.dynamic.point_mass_system import *
from mrs_playground.sensing.radius_sensing import RadiusSensing

from mrs_playground.behavior.mathematical_flock import MathematicalFlock
from mrs_playground.behavior.alternative_flocking import AlternativeFlock

from mrs_playground.environment.simple_playground import SimplePlayground
from mrs_playground.environment.playground_factory import PlaygroundFactory

from mr_herding.behavior.decentralised_cbf import DecentralisedCBF

PROJECT_DIR = os.path.dirname(os.path.abspath(__file__))
CONFIG_DIR = os.path.join(PROJECT_DIR, 'config')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('-g', '--gui',
                        help='Render gui', action='store_true')
    args = parser.parse_args()

    entity_config_file = os.path.join(CONFIG_DIR, 'entity_cbf.yaml')
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
    behavior_config['herding_cbf']['params'].update({"max_num": len(robots),
                                                     "sensing_range": entity_config['robot']["sensing_range"],
                                                     "comms_range": entity_config['robot']["comms_range"]})
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

    env.turn_on_render(args.gui)

    while env.ok:
        env.step()
        env.render()


def run_cbf(run_id: int = 0, gui: bool = False):
    entity_config_file = os.path.join(CONFIG_DIR, 'entity_cbf.yaml')
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
    behavior_config['herding_cbf']['params'].update({"max_num": len(robots),
                                                     "sensing_range": entity_config['robot']["sensing_range"],
                                                     "comms_range": entity_config['robot']["comms_range"]})
    cbf_config = behavior_config['herding_cbf']['params']
    PlaygroundFactory.add_behavior(entities=robots,
                                   config=cbf_config,
                                   behavior_type=DecentralisedCBF,
                                   behavior_name="cbf")

    # Add robots
    for robot in robots:
        math_flock.add_robot(robot)

    # Create environment
    env = SimplePlayground(**env_config)

    meta_data = {}
    meta_data.update(entity_config)
    meta_data.update(math_flock_config)
    meta_data.update(sensing_config)
    meta_data.update(dynamic_config)
    meta_data.update(cbf_config)

    env.add_meta_data(meta_data)

    # Add entities to env
    for animal in animals:
        env.add_entity(animal)
    for robot in robots:
        env.add_entity(robot)

    env.add_behaviour(math_flock)

    # Add sensor
    for sensor in sensors:
        env.add_sensing_models(sensor)

    env.turn_on_render(gui)

    while env.ok:
        env.step()
        env.render()
    env.save_data(data_name=f"cbf_{str(run_id)}",
                  path="data/aggregation_pref/success_rate/low/4/cbf/")


if __name__ == '__main__':
    main()
