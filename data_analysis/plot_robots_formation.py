# !/usr/bin/python3

import os
import numpy as np
import pandas as pd
import pickle
import matplotlib.pyplot as plt

from data_preprocessing import *
from plotting_utils import *

color_list = ['cornflowerblue', 'green', 'darkorange']


def plot_animal_positions(data, window_size, ax, index_list, angle_list):
    plot_target(ax)
    all_robots_name = list(data['animals'].keys())
    all_x = []
    all_y = []
    for i, robot_name in enumerate(all_robots_name):
        robot_data = data['animals'][robot_name]
        x, y = data_preprocessing(robot_data, seconds_to_remove=10)
        indx = index_list[0]
        all_x.append(x[:index_list[0]+1])
        all_y.append(y[:index_list[0]+1])

        center = [x[indx], y[indx]]
        base = 0.14
        height = 0.2
        plot_triangle_with_centrer(
            center, height, base, angle_list[i], 'red', ax)

    for i in range(len(all_x)):
        ax.plot(all_x[i], all_y[i], '-', color='red', lw=1)


def plot_robots_formation(data, window_size, ax, index_list):
    all_robots_name = list(data['robot'].keys())
    # Extract trajectories of cf3, cf6, and cf7
    all_x = []
    all_y = []
    for i, robot_name in enumerate(all_robots_name):
        robot_data = data['robot'][robot_name]
        x, y = data_preprocessing(robot_data, seconds_to_remove=10)
        if len(index_list) > 1:
            all_x.append(x[:index_list[1]+1])
            all_y.append(y[:index_list[1]+1])
        else:
            all_x.append(x[:index_list[0]+1])
            all_y.append(y[:index_list[0]+1])

    for i in range(len(all_x)):
        if len(index_list) > 1:
            ax.plot(all_x[i][index_list[0]:index_list[1]+1], all_y[i]
                    [index_list[0]:index_list[1]+1], '--', color=color_list[i])
        else:
            ax.plot(all_x[i], all_y[i], '--', color=color_list[i], lw=1)

    actual_index = index_list[1] if len(index_list) > 1 else index_list[0]
    xy_points_1 = get_xy_points_at_intervals_given_index(
        all_x[0][:actual_index+1], all_y[0][:actual_index+1], [actual_index])
    xy_points_2 = get_xy_points_at_intervals_given_index(
        all_x[1][:actual_index+1], all_y[1][:actual_index+1],  [actual_index])
    xy_points_3 = get_xy_points_at_intervals_given_index(
        all_x[2][:actual_index+1], all_y[2][:actual_index+1],   [actual_index])

    ax.plot([xy_points_1[0][0], xy_points_3[0][0]],
            [xy_points_1[0][1], xy_points_3[0][1]], '-', color='black')
    ax.plot([xy_points_2[0][0], xy_points_3[0][0]],
            [xy_points_2[0][1], xy_points_3[0][1]], '-', color='black')

    for i, robot_name in enumerate(all_robots_name):
        x = all_x[i]
        y = all_y[i]
        xy_points = get_xy_points_at_intervals_given_index(
            x[:actual_index+1], y[:actual_index+1], [actual_index])
        x_points, y_points = zip(*xy_points)
        ax.plot(x_points, y_points, 'o',
                markersize=8, color=color_list[i])

    ax.set_xlim(-2, 3)
    ax.set_ylim(-2, 2)
    ax.set_xticks([-2, -1, 0, 1, 2, 3])
    ax.set_yticks([-2, -1, 0, 1, 2])
    ax.set_aspect('equal', adjustable='box')

    custom_lines = [plt.Line2D([0], [0], color='blue', lw=2), plt.Line2D([
        0], [0], color='blue', lw=2)]
    labels = [f'Custom Label {i * 3 + 1 + 1}', f'Custom Label {i * 3 + 1 + 1}']
    ax.legend(custom_lines, labels, loc='upper right')


path = "rosbag/extracted_data/"
name = "seventh_run"
full_path = os.path.join(path, name + ".pickle")
with open(full_path, 'rb') as file:
    data = pickle.load(file)

fig, ax = plt.subplots(2, 3, figsize=(15, 7))

#
plot_animal_positions(data=data, window_size=100,
                      ax=ax[0, 0], index_list=[3000], angle_list=[-87, -70, -110])
plot_robots_formation(data=data, window_size=100,
                      ax=ax[0, 0], index_list=[3000])

#
plot_animal_positions(data=data, window_size=100,
                      ax=ax[0, 1], index_list=[5000], angle_list=[-100, -114, -130])
plot_robots_formation(data=data, window_size=100,
                      ax=ax[0, 1], index_list=[5000])

#
plot_animal_positions(data=data, window_size=100,
                      ax=ax[0, 2], index_list=[7000], angle_list=[-175, -195, -180])
plot_robots_formation(data=data, window_size=100,
                      ax=ax[0, 2], index_list=[7000])

#
plot_animal_positions(data=data, window_size=100,
                      ax=ax[1, 0], index_list=[10000], angle_list=[90, 75, 30])
plot_robots_formation(data=data, window_size=100,
                      ax=ax[1, 0], index_list=[10000])

#
plot_animal_positions(data=data, window_size=100,
                      ax=ax[1, 1], index_list=[15000], angle_list=[70, 55, 68])
plot_robots_formation(data=data, window_size=100,
                      ax=ax[1, 1], index_list=[15000])

#
plot_animal_positions(data=data, window_size=100,
                      ax=ax[1, 2], index_list=[21300], angle_list=[25, 30, 35])
plot_robots_formation(data=data, window_size=100,
                      ax=ax[1, 2], index_list=[21300])

plt.tight_layout()
# plt.axis('equal')
plt.show()
