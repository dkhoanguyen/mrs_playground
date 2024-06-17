# !/usr/bin/python3

import os
import numpy as np
import pandas as pd
import pickle
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection

path = "rosbag/extracted_data/"
name = "fifth_run"

full_path = os.path.join(path, name + ".pickle")
with open(full_path, 'rb') as file:
    data = pickle.load(file)

all_animals_states = data['animals']
all_robots_states = data['robot']


def plot_triangle(vertices, angle, color, ax):
    # Create a rotation matrix
    angle_rad = np.radians(angle)
    rotation_matrix = np.array([
        [np.cos(angle_rad), -np.sin(angle_rad)],
        [np.sin(angle_rad), np.cos(angle_rad)]
    ])

    # Rotate the vertices
    rotated_vertices = np.dot(vertices, rotation_matrix)

    # Translate vertices to the center
    translated_vertices = rotated_vertices

    triangle = plt.Polygon(vertices,
                           edgecolor=color, facecolor=color)

    # Plot the triangle
    ax.fill(*zip(*translated_vertices),
            edgecolor=color, facecolor=color)

    return triangle


def moving_average(data, window_size):
    return np.convolve(data, np.ones(window_size)/window_size, mode='valid')

# Function to get the XY points at specified intervals


def get_xy_points_at_intervals(x_data, y_data, interval, sampling_rate):
    points_per_interval = sampling_rate * interval
    return [(x_data[i], y_data[i]) for i in range(0, len(x_data), points_per_interval)]


def get_xy_points_at_intervals_given_index(x_data, y_data, interval, sampling_rate, index_list):
    extracted_points = []
    for i in range(len(index_list)):
        extracted_points.append((x_data[index_list[i]], y_data[index_list[i]]))
    return extracted_points


def plot_with_gradient(x, y, ax, label, color_map):
    points = np.array([x, y]).T.reshape(-1, 1, 2)
    segments = np.concatenate([points[:-1], points[1:]], axis=1)
    lc = LineCollection(segments, cmap=color_map, norm=plt.Normalize(0, 1))
    lc.set_array(np.linspace(0, 1, len(x)))
    lc.set_linewidth(2)
    ax.add_collection(lc)


window_size = 200

# Extract trajectories of cf3, cf6, and cf7
cf3_trajectory = data['robot']['cf3']
cf6_trajectory = data['robot']['cf6']
cf7_trajectory = data['robot']['cf7']

# Extract x and y coordinates for each robot
cf3_x = [point[0] for point in cf3_trajectory]
cf3_y = [point[1] for point in cf3_trajectory]

cf6_x = [point[0] for point in cf6_trajectory]
cf6_y = [point[1] for point in cf6_trajectory]

cf7_x = [point[0] for point in cf7_trajectory]
cf7_y = [point[1] for point in cf7_trajectory]

# Define the number of data points to remove based on the sampling rate
sampling_rate = 100  # Hz
seconds_to_remove = 20
interval = 30
points_to_remove = sampling_rate * seconds_to_remove
points_per_interval = sampling_rate * interval

# Remove the last points_to_remove data points from each trajectory
cf3_x_trimmed = cf3_x[:-points_to_remove]
cf3_y_trimmed = cf3_y[:-points_to_remove]

cf6_x_trimmed = cf6_x[:-points_to_remove]
cf6_y_trimmed = cf6_y[:-points_to_remove]

cf7_x_trimmed = cf7_x[:-points_to_remove]
cf7_y_trimmed = cf7_y[:-points_to_remove]

# Apply the moving average filter to the trimmed trajectories
cf3_x_smooth_trimmed = moving_average(cf3_x_trimmed, window_size)
cf3_y_smooth_trimmed = moving_average(cf3_y_trimmed, window_size)

cf6_x_smooth_trimmed = moving_average(cf6_x_trimmed, window_size)
cf6_y_smooth_trimmed = moving_average(cf6_y_trimmed, window_size)

cf7_x_smooth_trimmed = moving_average(cf7_x_trimmed, window_size)
cf7_y_smooth_trimmed = moving_average(cf7_y_trimmed, window_size)

index_list = [3000, 5000, 7000, 10000, 13000, 16500]

cf3_xy_points = get_xy_points_at_intervals_given_index(
    cf3_x_smooth_trimmed, cf3_y_smooth_trimmed, interval, sampling_rate, index_list)
cf6_xy_points = get_xy_points_at_intervals_given_index(
    cf6_x_smooth_trimmed, cf6_y_smooth_trimmed, interval, sampling_rate, index_list)
cf7_xy_points = get_xy_points_at_intervals_given_index(
    cf7_x_smooth_trimmed, cf7_y_smooth_trimmed, interval, sampling_rate, index_list)

cf3_x_points, cf3_y_points = zip(*cf3_xy_points)
cf6_x_points, cf6_y_points = zip(*cf6_xy_points)
cf7_x_points, cf7_y_points = zip(*cf7_xy_points)

# # Plot the smoothed and trimmed trajectories
fig, ax = plt.subplots()
fig.set_size_inches(10, 8)
ax.plot(cf3_x_smooth_trimmed, cf3_y_smooth_trimmed, '--',
        label='cf3 Smoothed Trajectory')
ax.plot(cf6_x_smooth_trimmed, cf6_y_smooth_trimmed, '--',
        label='cf6 Smoothed Trajectory')
ax.plot(cf7_x_smooth_trimmed, cf7_y_smooth_trimmed, '--',
        label='cf7 Smoothed Trajectory')

for i in range(len(cf3_x_points)):
    ax.plot([cf3_x_points[i], cf7_x_points[i]],
            [cf3_y_points[i], cf7_y_points[i]], '-', color='black')
    ax.plot([cf7_x_points[i], cf6_x_points[i]],
            [cf7_y_points[i], cf6_y_points[i]], '-', color='black')
    vertices = np.array([
        [cf3_x_points[i], cf3_y_points[i]],
        [cf7_x_points[i], cf7_y_points[i]],
        [cf6_x_points[i], cf6_y_points[i]]
    ])
    # ax.add_patch(plot_triangle(vertices, 0, 'lightskyblue', ax))

# Plot the points
ax.plot(cf3_x_points, cf3_y_points, 'o', label='cf3 points', markersize=10)
ax.plot(cf6_x_points, cf6_y_points, 'o', label='cf6 points', markersize=10)
ax.plot(cf7_x_points, cf7_y_points, 'o', label='cf7 points', markersize=10)

# plt.xlabel('X Coordinate')
# plt.ylabel('Y Coordinate')
# plt.title('Smoothed and Trimmed Trajectories of cf3, cf6, and cf7')
# # plt.legend()
ax.grid(True)
ax.axis('equal')
plt.show()
