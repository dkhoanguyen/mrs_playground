#!/usr/bin/python3

import numpy as np

from mrs_playground.entity.robot import Robot

from mrs_playground.environment.simple_playground import SimplePlayground


def main():
    robot = Robot(id=0,
                  pose=np.array([300, 300]),
                  velocity=np.array([10, 0]),
                  sensing_range=100.0,
                  comms_range=100.0,
                  max_v=10.0,
                  max_a=5.0)
    env = SimplePlayground(dt=0.05,entity_names=["robot"])
    env.add_entity(robot)

    while env.ok:
        env.run_once()
        env.render()
    
if __name__ == '__main__':
    main()
