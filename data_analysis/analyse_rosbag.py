# !/usr/bin/python3

import os
import numpy as np
import pandas as pd
import pickle
import matplotlib.pyplot as plt

path = "rosbag/extracted_data/"
name = "seventh_run"

full_path = os.path.join(path, name + ".pickle")
with open(full_path, 'rb') as file:
    data = pickle.load(file)

all_animals_states = data['animals']
all_robots_states = data['robot']


def moving_average(data, window_size):
    return np.convolve(data, np.ones(window_size)/window_size, mode='valid')

# Function to get the XY points at specified intervals


def get_xy_points_at_intervals(x_data, y_data, interval, sampling_rate):
    points_per_interval = sampling_rate * interval
    return [(x_data[i], y_data[i]) for i in range(0, len(x_data), points_per_interval)]


window_size = 100

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
interval = 20
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

cf3_xy_points = get_xy_points_at_intervals(
    cf3_x_smooth_trimmed, cf3_y_smooth_trimmed, interval, sampling_rate)
cf6_xy_points = get_xy_points_at_intervals(
    cf6_x_smooth_trimmed, cf6_y_smooth_trimmed, interval, sampling_rate)
cf7_xy_points = get_xy_points_at_intervals(
    cf7_x_smooth_trimmed, cf7_y_smooth_trimmed, interval, sampling_rate)

cf3_x_points, cf3_y_points = zip(*cf3_xy_points)
cf6_x_points, cf6_y_points = zip(*cf6_xy_points)
cf7_x_points, cf7_y_points = zip(*cf7_xy_points)

# # Plot the smoothed and trimmed trajectories
plt.figure(figsize=(10, 8))
plt.plot(cf3_x_smooth_trimmed, cf3_y_smooth_trimmed, '--',
         label='cf3 Smoothed Trajectory')
plt.plot(cf6_x_smooth_trimmed, cf6_y_smooth_trimmed, '--',
         label='cf6 Smoothed Trajectory')
plt.plot(cf7_x_smooth_trimmed, cf7_y_smooth_trimmed, '--',
         label='cf7 Smoothed Trajectory')
# Plot the points
plt.plot(cf3_x_points, cf3_y_points, 'o', label='cf3 points')
plt.plot(cf6_x_points, cf6_y_points, 'o', label='cf6 points')
plt.plot(cf7_x_points, cf7_y_points, 'o', label='cf7 points')

for i in range(len(cf3_x_points)):
    plt.plot([cf3_x_points[i], cf7_x_points[i]],
             [cf3_y_points[i], cf7_y_points[i]], '-', color='grey')
    plt.plot([cf7_x_points[i], cf6_x_points[i]],
             [cf7_y_points[i], cf6_y_points[i]], '-', color='grey')

plt.xlabel('X Coordinate')
plt.ylabel('Y Coordinate')
plt.title('Smoothed and Trimmed Trajectories of cf3, cf6, and cf7')
plt.legend()
plt.grid(True)
plt.axis('equal')
plt.show()
