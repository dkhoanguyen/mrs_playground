# !/usr/bin/python3

import os
import numpy as np
import pandas as pd
import pickle
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import math


# # LOW AGGREGATION PREF
# # 4 robots
# # CBF
# directory_path = "data/aggregation_pref/low/4/cbf/"
# all_cbf_low_4_data = []
# # Loop through all files in the directory
# for filename in os.listdir(directory_path):
#     if filename.endswith('.pkl') or filename.endswith('.pickle'):
#         file_path = os.path.join(directory_path, filename)
#         with open(file_path, 'rb') as file:
#             data = pickle.load(file)
#             all_cbf_low_4_data.append(data)

# cbf_low_4_mean_dist_range = []
# cbf_low_4_exec_time = []
# for data in all_cbf_low_4_data:
#     cbf_low_4_mean_dist_range.append(np.mean(data['mean_dist_range']))
#     if len(data['mean_dist_range']) < 6000:
#         cbf_low_4_exec_time.append(len(data['mean_dist_range']))

# # OUTMOST
# # 4 robots
# directory_path = "data/aggregation_pref/low/4/outmost/"
# all_outmost_low_4_data = []
# # Loop through all files in the directory
# for filename in os.listdir(directory_path):
#     if filename.endswith('.pkl') or filename.endswith('.pickle'):
#         file_path = os.path.join(directory_path, filename)
#         with open(file_path, 'rb') as file:
#             data = pickle.load(file)
#             all_outmost_low_4_data.append(data)

# outmost_low_4_mean_dist_range = []
# outmost_low_4_exec_time = []
# for data in all_outmost_low_4_data:
#     outmost_low_4_mean_dist_range.append(np.mean(data['mean_dist_range']))
#     if len(data['mean_dist_range']) < 6000:
#         outmost_low_4_exec_time.append(len(data['mean_dist_range']))

# HIGH AGGREGATION PREF
# 4 robots
# CBF
directory_path = "data/aggregation_pref/success_rate/low/4/cbf/"
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
    if len(data['mean_dist_range']) < 6000:
        cbf_high_4_mean_dist_range.append(np.mean(data['mean_dist_range']))
        cbf_high_4_exec_time.append(len(data['mean_dist_range']))
cbf_high_4_success_rate = len(cbf_high_4_exec_time) / len(all_cbf_high_4_data)
# print(len(cbf_high_4_exec_time))
# print(len(all_cbf_high_4_data))

# OUTMOST
# 4 robots
directory_path = "data/aggregation_pref/success_rate/low/4/outmost/"
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
    if len(data['mean_dist_range']) < 6000:
        outmost_high_4_exec_time.append(len(data['mean_dist_range']))
outmost_high_4_success_rate = len(
    outmost_high_4_exec_time) / len(all_outmost_high_4_data)

# Ploting

# Create figure and axis objects
fig, ax1 = plt.subplots()
fig.set_size_inches(7, 6)
positions = np.array([1, 3])
offset = 0.3
bp = ax1.boxplot([outmost_high_4_exec_time, cbf_high_4_exec_time],
                 positions=positions - offset, vert=True, zorder=2, patch_artist=True)
for element in ['boxes', 'whiskers', 'fliers', 'means', 'medians', 'caps']:
    plt.setp(bp[element], color='black')
for patch in bp['boxes']:
    patch.set(facecolor='lightskyblue', edgecolor='black')

# ax2 = ax1.twinx()
# bp2 = ax1.boxplot([outmost_low_4_exec_time, outmost_high_4_exec_time],
#                   positions=positions + offset, vert=True, zorder=2, patch_artist=True)
# for element in ['boxes', 'whiskers', 'fliers', 'means', 'medians', 'caps']:
#     plt.setp(bp2[element], color='black')
# for patch in bp2['boxes']:
#     patch.set(facecolor='salmon', edgecolor='black')

plt.xticks([1, 3], ["Low pref", "High pref"])

plt.show()
