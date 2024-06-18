# !/usr/bin/python3

import os
import numpy as np
import pandas as pd
import pickle
import matplotlib.pyplot as plt

from data_preprocessing import *

directory_path = "rosbag/extracted_data/"
all_data = []

target = np.array([2, 0])

plt.rcParams['text.usetex'] = True
plt.rcParams['font.family'] = 'serif'
plt.rcParams['font.serif'] = 'Computer Modern'

# Loop through all files in the directory
for filename in os.listdir(directory_path):
    if filename.endswith('.pkl') or filename.endswith('.pickle'):
        file_path = os.path.join(directory_path, filename)
        with open(file_path, 'rb') as file:
            data = pickle.load(file)
            all_data.append(data)

# Now, pickle_data contains the data from all pickle files in the directory
all_centroids_data = []
seconds_to_remove = [10, 190, 1, 1, 1, 1, 1]
for i, data in enumerate(all_data):
    all_x, all_y = extract_data(data, 'animals', seconds_to_remove[i])

    all_centroid = np.empty((0, 2))
    # For each point
    for k in range(len(all_x[0])):
        centroid = np.zeros((2,))
        for j in range(len(all_x)):
            try:
                x = all_x[j][k]
                y = all_y[j][k]
                centroid += np.array([x, y])
            except:
                pass
        centroid = centroid/len(all_x)
        all_centroid = np.vstack((all_centroid, centroid))
    all_centroids_data.append(all_centroid)


fig, ax = plt.subplots(figsize=(6.5, 5))
with open('rosbag/extracted_data/buffer/distance_data.pickle', 'rb') as file:
    distance_data = pickle.load(file)

for i, distance in enumerate(distance_data):
    x = np.linspace(0, distance.shape[0])
    ax.plot(distance, '-')
    ax.set_ylim(0.1, 3)
    ax.set_xlim(0, 22000)
    ax.set_xticks([0, 5000, 10000, 15000, 20000])
    ax.set_xticklabels(['0', '50', '100', '150', '200'])
    ax.tick_params(axis='both', labelsize=14)
    ax.set_xlabel("Timestamp(s)", fontsize=14)
    ax.set_ylabel("Distance to goal(m)", fontsize=14)
    ax.axhline(y=0.25, color='black', linestyle='-', lw=0.85)
    ax.text(x=500, y=0.3, s=r'"At goal" threshold $d=0.25m$',
            ha='left', fontsize=14)

plt.savefig('all_real_distance_to_goal.pdf', format='pdf')

# plt.show()
