#!/usr/bin/python3

import numpy as np

from utsma.entity.car import Car
from utsma.dynamic.kinematic_single_track import KinematicSingleTrackCartesian
from utsma.behavior.pid_controller import PIDController

from mrs_playground.environment.simple_playground import SimplePlayground


def main():
    x = 300
    y = 300
    theta = 0.0
    v = 0.0
    steering = 0.0
    state = np.array([x, y, theta, v, steering])
    max_v = 5.0
    max_a = 1.0
    max_steering = np.pi/2
    length = 50
    width = 30
    wheel_base = 50

    car = Car(state=state,
              max_v=max_v,
              max_a=max_a,
              max_delta=max_steering,
              length=length,
              width=width,
              wheelbase=wheel_base)
    # Kinematic Bicycle model as dynamic
    dynamic = KinematicSingleTrackCartesian(dt=0.1, L=wheel_base)
    car.set_dynamic_model(dynamic=dynamic)

    # Simple PID controller for now
    controller = PIDController(
        target_state=np.array([700, 300, 0.0, 0.0, 0.0]))
    car.add_behavior({"pid": controller})
    env = SimplePlayground(entity_names=["car"],
                           dt=0.1,
                           render=True,
                           multi_threaded=False,
                           save_to_file=False)
    env.add_entity(car)

    while env.ok:
        env.step()
        env.render()


if __name__ == '__main__':
    main()
