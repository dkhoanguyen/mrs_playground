# !/usr/bin/python3

import os
import numpy as np
import pandas as pd
import pickle
import matplotlib.pyplot as plt
import matplotlib
import matplotlib.patches as mpatches
import math

# Parameters
box_width = 0.25
threshold = 5000
offset = 0.2


# LOW AGGREGATION PREF
# 4 robots
# CBF
directory_path = "data/aggregation_pref/success_rate/low/4/cbf/"
all_cbf_low_4_data = []
# Loop through all files in the directory
for filename in os.listdir(directory_path):
    if filename.endswith('.pkl') or filename.endswith('.pickle'):
        file_path = os.path.join(directory_path, filename)
        with open(file_path, 'rb') as file:
            data = pickle.load(file)
            all_cbf_low_4_data.append(data)

cbf_low_4_mean_dist_range = []
cbf_low_4_exec_time = []
for data in all_cbf_low_4_data:
    if len(data['mean_dist_range']) < threshold:
        cbf_low_4_mean_dist_range.append(
            np.mean(data['mean_dist_range']))
        cbf_low_4_exec_time.append(len(data['mean_dist_range']))
cbf_low_4_success_rate = len(cbf_low_4_exec_time) / len(all_cbf_low_4_data)

# Initialize lists to collect all x and y coordinates
all_d_cbf_low_4 = []
# Assuming all_cbf_low_4_data is defined and structured as shown in your example
for indx in range(0, len(all_cbf_low_4_data)):
    if (len(all_cbf_low_4_data[indx]['data']) > threshold):
        continue
    pre_state = np.zeros((4, 2))
    total_d = np.array([0, 0, 0, 0]).astype(np.float64)
    # For execution time
    for i, all_animal_data in enumerate(all_cbf_low_4_data[indx]['data'][:]):
        # For each robot in robots
        if i == 0:
            for num_r, animal_data in enumerate(all_animal_data['robot']):
                pre_state[num_r, :] = animal_data[:2]
            continue
        for num_r, animal_data in enumerate(all_animal_data['robot']):
            current_pos = animal_data[:2]
            d = np.linalg.norm(current_pos - pre_state[num_r, :])
            total_d[num_r] += d
            pre_state[num_r, :] = animal_data[:2]
    all_d_cbf_low_4 += total_d.tolist()

# OUTMOST
# 4 robots
directory_path = "data/aggregation_pref/success_rate/low/4/outmost/"
all_outmost_low_4_data = []
# Loop through all files in the directory
for filename in os.listdir(directory_path):
    if filename.endswith('.pkl') or filename.endswith('.pickle'):
        file_path = os.path.join(directory_path, filename)
        with open(file_path, 'rb') as file:
            data = pickle.load(file)
            all_outmost_low_4_data.append(data)

outmost_low_4_mean_dist_range = []
outmost_low_4_exec_time = []
for data in all_outmost_low_4_data:
    if len(data['mean_dist_range']) < threshold:
        outmost_low_4_exec_time.append(len(data['mean_dist_range']))
        outmost_low_4_mean_dist_range.append(
            np.mean(data['mean_dist_range']))
outmost_low_4_success_rate = len(
    outmost_low_4_exec_time) / len(all_outmost_low_4_data)

# Initialize lists to collect all x and y coordinates
all_d_outmost_low_4 = []
for indx in range(0, len(all_outmost_low_4_data)):
    if (len(all_outmost_low_4_data[indx]['data']) > threshold):
        continue
    pre_state = np.zeros((4, 2))
    total_d = np.array([0, 0, 0, 0]).astype(np.float64)
    # For execution time
    for i, all_animal_data in enumerate(all_outmost_low_4_data[indx]['data'][:]):
        # For each robot in robots
        if i == 0:
            for num_r, animal_data in enumerate(all_animal_data['robot']):
                pre_state[num_r, :] = animal_data[:2]
            continue
        for num_r, animal_data in enumerate(all_animal_data['robot']):
            current_pos = animal_data[:2]
            d = np.linalg.norm(current_pos - pre_state[num_r, :])
            total_d[num_r] += d
            pre_state[num_r, :] = animal_data[:2]
    all_d_outmost_low_4 += total_d.tolist()

# HIGH AGGREGATION PREF
# 4 robots
# CBF
directory_path = "data/aggregation_pref/success_rate/high/4/cbf/"
all_cbf_high_4_data = []
# Loop through all files in the directory
for filename in os.listdir(directory_path):
    if filename.endswith('.pkl') or filename.endswith('.pickle'):
        file_path = os.path.join(directory_path, filename)
        with open(file_path, 'rb') as file:
            data = pickle.load(file)
            all_cbf_high_4_data.append(data)

cbf_high_4_mean_dist_range = []
cbf_high_4_exec_time = []
for data in all_cbf_high_4_data:
    if len(data['mean_dist_range']) < threshold:
        cbf_high_4_mean_dist_range.append(np.mean(data['mean_dist_range']))
        cbf_high_4_exec_time.append(len(data['mean_dist_range']))
cbf_high_4_success_rate = len(cbf_high_4_exec_time) / len(all_cbf_high_4_data)

# Initialize lists to collect all x and y coordinates
all_d_cbf_high_4 = []
for indx in range(0, len(all_cbf_high_4_data)):
    if (len(all_cbf_high_4_data[indx]['data']) > threshold):
        continue
    pre_state = np.zeros((4, 2))
    total_d = np.array([0, 0, 0, 0]).astype(np.float64)
    # For execution time
    for i, all_animal_data in enumerate(all_cbf_high_4_data[indx]['data'][:]):
        # For each robot in robots
        if i == 0:
            for num_r, animal_data in enumerate(all_animal_data['robot']):
                pre_state[num_r, :] = animal_data[:2]
            continue
        for num_r, animal_data in enumerate(all_animal_data['robot']):
            current_pos = animal_data[:2]
            d = np.linalg.norm(current_pos - pre_state[num_r, :])
            total_d[num_r] += d
            pre_state[num_r, :] = animal_data[:2]
    all_d_cbf_high_4 += total_d.tolist()

# OUTMOST
# 4 robots
directory_path = "data/aggregation_pref/success_rate/high/4/outmost/"
all_outmost_high_4_data = []
# Loop through all files in the directory
for filename in os.listdir(directory_path):
    if filename.endswith('.pkl') or filename.endswith('.pickle'):
        file_path = os.path.join(directory_path, filename)
        with open(file_path, 'rb') as file:
            data = pickle.load(file)
            all_outmost_high_4_data.append(data)

outmost_high_4_mean_dist_range = []
outmost_high_4_exec_time = []
for data in all_outmost_high_4_data:
    outmost_high_4_mean_dist_range.append(
        np.mean(data['mean_dist_range']))
    if len(data['mean_dist_range']) < threshold:
        outmost_high_4_exec_time.append(len(data['mean_dist_range']))
outmost_high_4_success_rate = len(
    outmost_high_4_exec_time) / len(all_outmost_high_4_data)

# Initialize lists to collect all x and y coordinates
all_d_outmost_high_4 = []
for indx in range(0, len(all_outmost_high_4_data)):
    if (len(all_outmost_high_4_data[indx]['data']) > threshold):
        continue
    pre_state = np.zeros((4, 2))
    total_d = np.array([0, 0, 0, 0]).astype(np.float64)
    # For execution time
    for i, all_animal_data in enumerate(all_outmost_high_4_data[indx]['data'][:]):
        # For each robot in robots
        if i == 0:
            for num_r, animal_data in enumerate(all_animal_data['robot']):
                pre_state[num_r, :] = animal_data[:2]
            continue
        for num_r, animal_data in enumerate(all_animal_data['robot']):
            current_pos = animal_data[:2]
            d = np.linalg.norm(current_pos - pre_state[num_r, :])
            total_d[num_r] += d
            pre_state[num_r, :] = animal_data[:2]
    all_d_outmost_high_4 += total_d.tolist()


# group data
# Group data by algorithm and metric
data_low_dist = [cbf_low_4_mean_dist_range, cbf_high_4_mean_dist_range]
data_low_time = [cbf_low_4_exec_time, cbf_high_4_exec_time]
data_low_success = [cbf_low_4_success_rate, cbf_high_4_success_rate]
data_low_distance = [all_d_cbf_low_4, all_d_cbf_high_4]


data_high_dist = [outmost_low_4_mean_dist_range,
                  outmost_high_4_mean_dist_range]
data_high_time = [outmost_low_4_exec_time, outmost_high_4_exec_time]
data_high_success = [outmost_low_4_success_rate, outmost_high_4_success_rate]
data_high_distance = [all_d_outmost_low_4, all_d_outmost_high_4]
# print(data_high_distance)


# Create figure and axis
fig, ax1 = plt.subplots(figsize=(8, 7))
positions = np.array([1, 2, 3.5, 4.5, 6, 7])
# Boxplot for Algorithm 1
bp1_dist = ax1.boxplot(
    data_low_dist, positions=positions[0:2] - offset, patch_artist=True, widths=box_width)
for patch in bp1_dist['boxes']:
    patch.set(facecolor='lightskyblue', edgecolor='black')

# Boxplot for Algorithm 2 using twinx
ax2 = ax1.twinx()
bp2_dist = ax2.boxplot(
    data_high_dist, positions=positions[0:2] + offset, patch_artist=True, widths=box_width)
for patch in bp2_dist['boxes']:
    patch.set(facecolor='salmon', edgecolor='black')

ax3 = ax1.twinx()
bp1_time = ax3.boxplot(
    data_low_time, positions=positions[2:4] - offset, patch_artist=True, widths=box_width)
for patch in bp1_time['boxes']:
    patch.set(facecolor='lightgreen', edgecolor='black')

ax4 = ax1.twinx()
bp2_time = ax4.boxplot(
    data_high_time, positions=positions[2:4] + offset, patch_artist=True, widths=box_width)
for patch in bp2_time['boxes']:
    patch.set(facecolor='orange', edgecolor='black')

ax5 = ax1.twinx()
bp1_distance = ax5.boxplot(
    data_low_distance, positions=positions[4:] - offset, patch_artist=True, widths=box_width)
for patch in bp2_time['boxes']:
    patch.set(facecolor='lightskyblue', edgecolor='black')
ax6 = ax1.twinx()
bp2_distance = ax6.boxplot(
    data_high_distance, positions=positions[4:] + offset, patch_artist=True, widths=box_width)

# Set the same scale for both y-axes if necessary
ylim_min = min(ax1.get_ylim()[0], ax2.get_ylim()[0])
ylim_max = max(ax1.get_ylim()[1], ax2.get_ylim()[1])
ax1.set_ylim(ylim_min, ylim_max)
ax2.set_ylim(ylim_min, ylim_max)

ylim_min = min(ax3.get_ylim()[0], ax4.get_ylim()[0])
ylim_max = max(ax3.get_ylim()[1], ax4.get_ylim()[1])
ax3.set_ylim(ylim_min, ylim_max * 1.2)
ax4.set_ylim(ylim_min, ylim_max * 1.2)

ylim_min = min(ax5.get_ylim()[0], ax6.get_ylim()[0])
ylim_max = max(ax5.get_ylim()[1], ax6.get_ylim()[1])
ax5.set_ylim(ylim_min, ylim_max * 1.2)
ax6.set_ylim(ylim_min, ylim_max * 1.2)

ax2.set_yticklabels([])
ax4.set_yticklabels([])
# ax6.set_ylim(0, 1.1)


for element in ['whiskers', 'fliers', 'means', 'medians', 'caps']:
    plt.setp(bp1_dist[element], color='black')
    plt.setp(bp2_dist[element], color='black')
    plt.setp(bp1_time[element], color='black')
    plt.setp(bp2_time[element], color='black')
    plt.setp(bp1_distance[element], color='black')
    plt.setp(bp2_distance[element], color='black')


# X-axis labels
plt.xticks([1, 2, 3.5, 4.5], ['Low\naggregation', 'High\naggregation',
           'Low\naggregation', 'High\naggregation',])

# Custom legend
legend_patches = [
    mpatches.Patch(color='lightskyblue',
                   label='CBF (Ours) - Mean distribution range'),
    mpatches.Patch(
        color='salmon', label='Outmost push - Mean distribution range'),
    mpatches.Patch(color='lightgreen',
                   label='CBF (Ours) - Mean completion time'),
    mpatches.Patch(color='orange', label='Outmost push - Mean completion time')
]
plt.legend(handles=legend_patches, loc='upper right')

plt.title("Comparison of Mean Distribution Range and Execution Time")
plt.show()
