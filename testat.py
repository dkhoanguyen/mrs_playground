# !/usr/bin/python3

import os
import numpy as np
import pandas as pd
import pickle
import matplotlib.pyplot as plt
import matplotlib
import matplotlib.patches as mpatches
import math

# Example data
alg1_setup1_dist = np.random.normal(100, 10, 100)
alg1_setup1_time = np.random.normal(200, 20, 100)
alg1_setup2_dist = np.random.normal(110, 12, 100)
alg1_setup2_time = np.random.normal(210, 18, 100)

alg2_setup1_dist = np.random.normal(95, 15, 100)
alg2_setup1_time = np.random.normal(190, 25, 100)
alg2_setup2_dist = np.random.normal(105, 10, 100)
alg2_setup2_time = np.random.normal(205, 22, 100)

# Group data by algorithm and metric
data_alg1_dist = [alg1_setup1_dist, alg1_setup2_dist]
data_alg1_time = [alg1_setup1_time, alg1_setup2_time]

data_alg2_dist = [alg2_setup1_dist, alg2_setup2_dist]
data_alg2_time = [alg2_setup1_time, alg2_setup2_time]

# Create figure and axis
fig, ax1 = plt.subplots(figsize=(10, 7))
positions = np.array([1, 2, 4, 5])
offset = 0.2

# Boxplot for Algorithm 1
bp1_dist = ax1.boxplot(
    data_alg1_dist, positions=positions[:2] - offset, patch_artist=True)
for patch in bp1_dist['boxes']:
    patch.set(facecolor='lightskyblue', edgecolor='black')
bp1_time = ax1.boxplot(
    data_alg1_time, positions=positions[2:] - offset, patch_artist=True)
for patch in bp1_time['boxes']:
    patch.set(facecolor='lightgreen', edgecolor='black')

# Boxplot for Algorithm 2 using twinx
ax2 = ax1.twinx()
bp2_dist = ax2.boxplot(
    data_alg2_dist, positions=positions[:2] + offset, patch_artist=True)
for patch in bp2_dist['boxes']:
    patch.set(facecolor='salmon', edgecolor='black')
bp2_time = ax2.boxplot(
    data_alg2_time, positions=positions[2:] + offset, patch_artist=True)
for patch in bp2_time['boxes']:
    patch.set(facecolor='orange', edgecolor='black')

# Set the same scale for both y-axes if necessary
ylim_min = min(ax1.get_ylim()[0], ax2.get_ylim()[0])
ylim_max = max(ax1.get_ylim()[1], ax2.get_ylim()[1])
ax1.set_ylim(ylim_min, ylim_max)
ax2.set_ylim(ylim_min, ylim_max)

# X-axis labels
plt.xticks([1, 2, 4, 5], ['Setup 1 (Dist)', 'Setup 2 (Dist)',
           'Setup 1 (Time)', 'Setup 2 (Time)'])

# Custom legend
legend_patches = [
    mpatches.Patch(color='lightskyblue', label='Algorithm 1 - Dist'),
    mpatches.Patch(color='lightgreen', label='Algorithm 1 - Time'),
    mpatches.Patch(color='salmon', label='Algorithm 2 - Dist'),
    mpatches.Patch(color='orange', label='Algorithm 2 - Time')
]
plt.legend(handles=legend_patches, loc='upper right')

plt.title("Comparison of Mean Distribution Range and Execution Time")
plt.show()
