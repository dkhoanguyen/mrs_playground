# !/usr/bin/python3

import os
import numpy as np
import pandas as pd
import pickle
import matplotlib.pyplot as plt
import matplotlib
import matplotlib.patches as mpatches

from data_preprocessing import *
from plotting_utils import *

color_list = ['darkorange', 'hotpink', 'slateblue']
plt.rcParams['text.usetex'] = True
plt.rcParams['font.family'] = 'serif'
plt.rcParams['font.serif'] = 'Computer Modern'


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


def plot_robots_formation(data, window_size, ax, index_list, time, plot_legend=False):
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
            ax.plot(all_x[i], all_y[i], '--', color=color_list[i], lw=1.5)

    actual_index = index_list[1] if len(index_list) > 1 else index_list[0]
    xy_points_1 = get_xy_points_at_intervals_given_index(
        all_x[0][:actual_index+1], all_y[0][:actual_index+1], [actual_index])
    xy_points_2 = get_xy_points_at_intervals_given_index(
        all_x[1][:actual_index+1], all_y[1][:actual_index+1],  [actual_index])
    xy_points_3 = get_xy_points_at_intervals_given_index(
        all_x[2][:actual_index+1], all_y[2][:actual_index+1],   [actual_index])

    ax.plot([xy_points_1[0][0], xy_points_3[0][0]],
            [xy_points_1[0][1], xy_points_3[0][1]], '-', color='black', lw=1)
    ax.plot([xy_points_2[0][0], xy_points_3[0][0]],
            [xy_points_2[0][1], xy_points_3[0][1]], '-', color='black', lw=1)

    for i, robot_name in enumerate(all_robots_name):
        x = all_x[i]
        y = all_y[i]
        xy_points = get_xy_points_at_intervals_given_index(
            x[:actual_index+1], y[:actual_index+1], [actual_index])
        x_points, y_points = zip(*xy_points)
        ax.plot(x_points, y_points, 'o',
                markersize=8, color='forestgreen')

    ax.set_xlim(-2, 3)
    ax.set_ylim(-2, 2)
    ax.set_xticks([-2, -1, 0, 1, 2, 3])
    ax.set_yticks([-2, -1, 0, 1, 2])
    ax.set_aspect('equal', adjustable='box')
    ax.set_xlabel("x(m)", fontsize=14)
    ax.set_ylabel("y(m)", fontsize=14)

    ax.annotate(rf'$\textbf{{T={str(time)}s}}$', xy=(-1.35, 1.75),
                xytext=(0, 0), textcoords='offset points', ha='center', fontsize=16)

    if plot_legend:
        handles = [plt.Line2D([0], [0], marker='o', color='w',
                              markerfacecolor='forestgreen', markersize=11),
                   plt.Line2D([0], [0], marker='>', color='w',
                              markerfacecolor='red', markersize=11),
                   plt.Line2D([0], [0], color='black', lw=1)]

        labels = [r'Robot',
                  r'Animal', r'Virtual link']
        ax.legend(handles=handles,
                  labels=labels, loc='upper right', fontsize=12, handletextpad=0.5)
        ax.arrow(1.7, -1, 0.28, 0.85, head_width=0.1,
                 head_length=0.15, fc='k', ec='k', lw=0.75)
        ax.annotate(r'$\textbf{Goal area}$', xy=(1.7, -1.2),
                    xytext=(0, 0), textcoords='offset points', ha='center', fontsize=16)


path = "rosbag/extracted_data/"
name = "seventh_run"
full_path = os.path.join(path, name + ".pickle")
with open(full_path, 'rb') as file:
    data = pickle.load(file)

fig, ax = plt.subplots(2, 3, figsize=(20, 8.25))

#
plot_animal_positions(data=data, window_size=100,
                      ax=ax[0, 0], index_list=[3000], angle_list=[-87, -70, -110])
plot_robots_formation(data=data, window_size=100,
                      ax=ax[0, 0], index_list=[3000], time=3000/100, plot_legend=True)

#
plot_animal_positions(data=data, window_size=100,
                      ax=ax[0, 1], index_list=[5000], angle_list=[-100, -114, -130])
plot_robots_formation(data=data, window_size=100,
                      ax=ax[0, 1], index_list=[5000], time=5000/100)

#
plot_animal_positions(data=data, window_size=100,
                      ax=ax[0, 2], index_list=[7000], angle_list=[-175, -195, -180])
plot_robots_formation(data=data, window_size=100,
                      ax=ax[0, 2], index_list=[7000], time=7000/100)

#
plot_animal_positions(data=data, window_size=100,
                      ax=ax[1, 0], index_list=[10000], angle_list=[90, 75, 30])
plot_robots_formation(data=data, window_size=100,
                      ax=ax[1, 0], index_list=[10000], time=10000/100)

#
plot_animal_positions(data=data, window_size=100,
                      ax=ax[1, 1], index_list=[15000], angle_list=[70, 55, 68])
plot_robots_formation(data=data, window_size=100,
                      ax=ax[1, 1], index_list=[15000], time=15000/100)

#
plot_animal_positions(data=data, window_size=100,
                      ax=ax[1, 2], index_list=[21300], angle_list=[25, 30, 35])
plot_robots_formation(data=data, window_size=100,
                      ax=ax[1, 2], index_list=[21300], time=21300/100)

plt.tight_layout()
plt.show()
