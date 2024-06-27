# !/usr/bin/python3

import os
import numpy as np
import pandas as pd
import pickle
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches

plt.rcParams['text.usetex'] = True
plt.rcParams['font.family'] = 'serif'
plt.rcParams['font.serif'] = 'Computer Modern'


directory_path = "data/benchmarks/apf/"
all_apf_data = []
# Loop through all files in the directory
for filename in os.listdir(directory_path):
    if filename.endswith('.pkl') or filename.endswith('.pickle'):
        file_path = os.path.join(directory_path, filename)
        with open(file_path, 'rb') as file:
            data = pickle.load(file)
            all_apf_data.append(data)

directory_path = "data/benchmarks/cbf/"
all_cbf_data = []
# Loop through all files in the directory
for filename in os.listdir(directory_path):
    if filename.endswith('.pkl') or filename.endswith('.pickle'):
        file_path = os.path.join(directory_path, filename)
        with open(file_path, 'rb') as file:
            data = pickle.load(file)
            all_cbf_data.append(data)

directory_path = "data/benchmarks/outmost/"
all_outmost_data = []
# Loop through all files in the directory
for filename in os.listdir(directory_path):
    if filename.endswith('.pkl') or filename.endswith('.pickle'):
        file_path = os.path.join(directory_path, filename)
        with open(file_path, 'rb') as file:
            data = pickle.load(file)
            all_outmost_data.append(data)

apf_execution_time = []

cbf_execution_time = []

outmost_execution_time = []

for data in all_apf_data:
    if len(data['data']) < 6000:
        apf_execution_time.append(len(data['data']))
for data in all_cbf_data:
    if len(data['data']) < 6000:
        cbf_execution_time.append(len(data['data']))
for data in all_outmost_data:
    if len(data['data']) < 6000:
        outmost_execution_time.append(len(data['data']))

# Print success rate
apf_success = len(apf_execution_time)/len(all_apf_data)
cbf_success = len(cbf_execution_time)/len(all_cbf_data)
outmost_success = len(outmost_execution_time)/len(all_outmost_data)

# Create figure and axis objects
fig, ax1 = plt.subplots()
fig.set_size_inches(7, 6)
positions = np.array([1, 3, 5])
offset = 0.3
bp = ax1.boxplot([apf_execution_time, cbf_execution_time,
                  outmost_execution_time], positions=positions - offset, vert=True, zorder=2, patch_artist=True)
for element in ['boxes', 'whiskers', 'fliers', 'means', 'medians', 'caps']:
    plt.setp(bp[element], color='black')
for patch in bp['boxes']:
    patch.set(facecolor='lightskyblue', edgecolor='black')
ax1.set_ylim(0, 7000)

ax2 = ax1.twinx()
brp = ax2.bar(positions+offset, [apf_success, cbf_success,
              outmost_success], edgecolor='black', facecolor='salmon', width=0.5, zorder=1)
ax2.set_ylim(0, 1.0)
for bar in brp:
    yval = bar.get_height()
    ax2.text(bar.get_x() + bar.get_width()/2, yval + 0.01,
             f'{yval:.2f}', ha='center', va='bottom', color='black')

box_legend = mpatches.Patch(color='lightskyblue', label='Execution time')
bar_legend = mpatches.Patch(color='salmon', label='Success rate')
plt.legend(handles=[box_legend, bar_legend], loc='upper right', ncol=2)
plt.xticks([1, 3, 5], ["Our ICRA", "Our TRO", "Other's TRO"])

plt.show()
