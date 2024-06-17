#!/usr/bin/python3
import pygame
import numpy as np

from mrs_playground.common.behavior import Behavior
from mrs_playground.utils import utils


class DistributedOutmostPush(Behavior):
    def __init__(self,
                 max_u: float,
                 sensing_range: float,
                 min_robot_d: float,
                 max_robot_d: float,
                 min_animal_d: float,
                 max_animal_d: float,
                 collision_avoidance_d: float,
                 converge_std: float,
                 herding_target: np.ndarray,
                 coverage: float):
        super().__init__()
