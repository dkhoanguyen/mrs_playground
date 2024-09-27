# !/usr/bin/python3

import os
import numpy as np
import seaborn as sns
import pickle
import matplotlib
import matplotlib.pyplot as plt
from matplotlib.patches import Circle


def cmap_map(function, cmap):
    """ Applies function (which should operate on vectors of shape 3: [r, g, b]), on colormap cmap.
    This routine will break any discontinuous points in a colormap.
    """
    cdict = cmap._segmentdata
    step_dict = {}
    # Firt get the list of points where the segments start or end
    for key in ('red', 'green', 'blue'):
        step_dict[key] = list(map(lambda x: x[0], cdict[key]))
    step_list = sum(step_dict.values(), [])
    step_list = np.array(list(set(step_list)))
    # Then compute the LUT, and apply the function to the LUT
    def reduced_cmap(step): return np.array(cmap(step)[0:3])
    old_LUT = np.array(list(map(reduced_cmap, step_list)))
    new_LUT = np.array(list(map(function, old_LUT)))
    # Now try to make a minimal segment definition of the new LUT
    cdict = {}
    for i, key in enumerate(['red', 'green', 'blue']):
        this_cdict = {}
        for j, step in enumerate(step_list):
            if step in step_dict[key]:
                this_cdict[step] = new_LUT[j, i]
            elif new_LUT[j, i] != old_LUT[j, i]:
                this_cdict[step] = new_LUT[j, i]
        colorvector = list(map(lambda x: x + (x[1], ), this_cdict.items()))
        colorvector.sort()
        cdict[key] = colorvector

    return matplotlib.colors.LinearSegmentedColormap('colormap', cdict, 1024)


normalization_method = 'none'
target = 'animal'
range_lim = 5000
light_jet = cmap_map(lambda x: x, matplotlib.cm.jet)

fig, ((ax1, ax2, ax3), (ax4, ax5, ax6)) = plt.subplots(
    2, 3, figsize=(15, 10.25))


def convert_to_heatmap_coords(real_x, real_y, x_min_real, x_max_real, y_min_real, y_max_real, x_min_heatmap, x_max_heatmap, y_min_heatmap, y_max_heatmap):
    heatmap_x = (real_x - x_min_real) / (x_max_real - x_min_real) * \
        (x_max_heatmap - x_min_heatmap) + x_min_heatmap
    heatmap_y = (real_y - y_min_real) / (y_max_real - y_min_real) * \
        (y_max_heatmap - y_min_heatmap) + y_min_heatmap
    return heatmap_x, heatmap_y


rate = 500
# Define specific axis limits for the heatmap
x_min_heatmap = 0
x_max_heatmap = rate
y_min_heatmap = 0
y_max_heatmap = rate

# Define specific axis limits for the real-world coordinates
x_min_real = -200
x_max_real = 1200
y_min_real = -200
y_max_real = 1200


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


# Initialize lists to collect all x and y coordinates
all_x = []
all_y = []

# Assuming all_cbf_low_4_data is defined and structured as shown in your example
for indx in range(0, len(all_cbf_low_4_data)):
    if (len(all_cbf_low_4_data[indx]['data']) > range_lim):
        continue
    for all_animal_data in all_cbf_low_4_data[indx]['data'][:]:
        for animal_data in all_animal_data[target]:
            all_x.append(animal_data[0])
            all_y.append(animal_data[1])

# Convert lists to numpy arrays
all_x = np.array(all_x)
all_y = np.array(all_y)
# Create a 2D histogram of the aggregated movement data
heatmap, xedges, yedges = np.histogram2d(
    all_x, all_y, bins=rate, range=[[x_min_real, x_max_real], [y_min_real, y_max_real]])

# Plot the heatmap
sns.heatmap(heatmap.T, square=True, cmap=light_jet,
            robust=True,
            cbar=False, xticklabels=False, yticklabels=False, ax=ax1)


# Select a subset of ticks to display, limiting to 10 or fewer ticks
max_ticks = 10
x_ticks_indices = np.linspace(
    0, len(xedges) - 1, min(len(xedges), max_ticks), dtype=int)
y_ticks_indices = np.linspace(
    0, len(yedges) - 1, min(len(yedges), max_ticks), dtype=int)

# # Set the x and y ticks based on the selected subset of bin edges
# ax1.set_xticks(x_ticks_indices, np.round(xedges[x_ticks_indices]))
# ax1.set_yticks(y_ticks_indices, np.round(yedges[y_ticks_indices]))
# # Function to convert real-world coordinates to heatmap coordinates

# Define the circle in real-world coordinates
# Replace with your desired center coordinates in real-world scale
goal_center_real = [420, 330]
# Replace with your desired radius in real-world scale
goal_radius_real = 75

# Convert the circle's real-world position to heatmap coordinates
goal_center_heatmap = convert_to_heatmap_coords(goal_center_real[0], goal_center_real[1],
                                                x_min_real, x_max_real, y_min_real, y_max_real,
                                                x_min_heatmap, x_max_heatmap, y_min_heatmap, y_max_heatmap)

# Convert the real-world radius to heatmap scale
cgoal_radius_heatmap = goal_radius_real * \
    (x_max_heatmap - x_min_heatmap) / (x_max_real - x_min_real)

# Scale the circle properly
circle = Circle(goal_center_heatmap, cgoal_radius_heatmap,
                color='black', fill=False, linewidth=2)

# Add the circle to the plot
ax1.add_patch(circle)


# Define the circle in real-world coordinates
# Replace with your desired center coordinates in real-world scale
start_center_real = [600, 450]
# Replace with your desired radius in real-world scale
start_radius_real = 75

# Convert the circle's real-world position to heatmap coordinates
start_center_heatmap = convert_to_heatmap_coords(start_center_real[0], start_center_real[1],
                                                 x_min_real, x_max_real, y_min_real, y_max_real,
                                                 x_min_heatmap, x_max_heatmap, y_min_heatmap, y_max_heatmap)

# Convert the real-world radius to heatmap scale
start_radius_heatmap = start_radius_real * \
    (x_max_heatmap - x_min_heatmap) / (x_max_real - x_min_real)

# Scale the circle properly
circle = Circle(start_center_heatmap, start_radius_heatmap,
                color='white', fill=False, linewidth=2)

# Add the circle to the plot
ax1.add_patch(circle)


####
####
# OUTMOST
directory_path = "data/aggregation_pref/success_rate/low/4/outmost/"
all_cbf_low_4_data = []
# Loop through all files in the directory
for filename in os.listdir(directory_path):
    if filename.endswith('.pkl') or filename.endswith('.pickle'):
        file_path = os.path.join(directory_path, filename)
        with open(file_path, 'rb') as file:
            data = pickle.load(file)
            all_cbf_low_4_data.append(data)


# Initialize lists to collect all x and y coordinates
all_x = []
all_y = []

# Assuming all_cbf_low_4_data is defined and structured as shown in your example
for indx in range(0, len(all_cbf_low_4_data)):
    if (len(all_cbf_low_4_data[indx]['data']) > range_lim):
        continue
    for all_animal_data in all_cbf_low_4_data[indx]['data'][:]:
        for animal_data in all_animal_data[target]:
            all_x.append(animal_data[0])
            all_y.append(animal_data[1])

# Convert lists to numpy arrays
all_x = np.array(all_x)
all_y = np.array(all_y)
# Create a 2D histogram of the aggregated movement data
heatmap, xedges, yedges = np.histogram2d(
    all_x, all_y, bins=rate, range=[[x_min_real, x_max_real], [y_min_real, y_max_real]])

# Plot the heatmap
sns.heatmap(heatmap.T, square=True, cmap=light_jet,
            robust=True,
            cbar=False, xticklabels=False, yticklabels=False, ax=ax4)
# Set axis labels as meters

# Select a subset of ticks to display, limiting to 10 or fewer ticks
max_ticks = 10
x_ticks_indices = np.linspace(
    0, len(xedges) - 1, min(len(xedges), max_ticks), dtype=int)
y_ticks_indices = np.linspace(
    0, len(yedges) - 1, min(len(yedges), max_ticks), dtype=int)

# # Set the x and y ticks based on the selected subset of bin edges
# ax4.set_xticks(x_ticks_indices, np.round(xedges[x_ticks_indices]))
# ax4.set_yticks(y_ticks_indices, np.round(yedges[y_ticks_indices]))
# # Function to convert real-world coordinates to heatmap coordinates


# Define the circle in real-world coordinates
# Replace with your desired center coordinates in real-world scale
goal_center_real = [420, 330]
# Replace with your desired radius in real-world scale
goal_radius_real = 75

# Convert the circle's real-world position to heatmap coordinates
goal_center_heatmap = convert_to_heatmap_coords(goal_center_real[0], goal_center_real[1],
                                                x_min_real, x_max_real, y_min_real, y_max_real,
                                                x_min_heatmap, x_max_heatmap, y_min_heatmap, y_max_heatmap)

# Convert the real-world radius to heatmap scale
cgoal_radius_heatmap = goal_radius_real * \
    (x_max_heatmap - x_min_heatmap) / (x_max_real - x_min_real)

# Scale the circle properly
circle = Circle(goal_center_heatmap, cgoal_radius_heatmap,
                color='black', fill=False, linewidth=2)

# Add the circle to the plot
ax4.add_patch(circle)


# Define the circle in real-world coordinates
# Replace with your desired center coordinates in real-world scale
start_center_real = [600, 450]
# Replace with your desired radius in real-world scale
start_radius_real = 75

# Convert the circle's real-world position to heatmap coordinates
start_center_heatmap = convert_to_heatmap_coords(start_center_real[0], start_center_real[1],
                                                 x_min_real, x_max_real, y_min_real, y_max_real,
                                                 x_min_heatmap, x_max_heatmap, y_min_heatmap, y_max_heatmap)

# Convert the real-world radius to heatmap scale
start_radius_heatmap = start_radius_real * \
    (x_max_heatmap - x_min_heatmap) / (x_max_real - x_min_real)

# Scale the circle properly
circle = Circle(start_center_heatmap, start_radius_heatmap,
                color='white', fill=False, linewidth=2)

# Add the circle to the plot
ax4.add_patch(circle)

################################################
################################################
################################################

################################################
################################################
################################################
################################################

################################################
################################################
################################################
################################################################################################
################################################


################################################

################################################
################################################
################################################

################################################
################################################
################################################
################################################

################################################
################################################
################################################
################################################################################################
################################################


################################################


# LOW AGGREGATION PREF
# 4 robots
# CBF
directory_path = "data/aggregation_pref/success_rate/low_cluster/4/cbf/"
all_cbf_low_4_data = []
# Loop through all files in the directory
for filename in os.listdir(directory_path):
    if filename.endswith('.pkl') or filename.endswith('.pickle'):
        file_path = os.path.join(directory_path, filename)
        with open(file_path, 'rb') as file:
            data = pickle.load(file)
            all_cbf_low_4_data.append(data)


# Initialize lists to collect all x and y coordinates
all_x = []
all_y = []

# Assuming all_cbf_low_4_data is defined and structured as shown in your example
for indx in range(0, len(all_cbf_low_4_data)):
    if (len(all_cbf_low_4_data[indx]['data']) > range_lim):
        continue
    for all_animal_data in all_cbf_low_4_data[indx]['data'][:]:
        for animal_data in all_animal_data[target]:
            all_x.append(animal_data[0])
            all_y.append(animal_data[1])

# Convert lists to numpy arrays
all_x = np.array(all_x)
all_y = np.array(all_y)
# Create a 2D histogram of the aggregated movement data
heatmap, xedges, yedges = np.histogram2d(
    all_x, all_y, bins=rate, range=[[x_min_real, x_max_real], [y_min_real, y_max_real]])

# Plot the heatmap
sns.heatmap(heatmap.T, square=True, cmap=light_jet,
            robust=True,
            cbar=False, xticklabels=False, yticklabels=False, ax=ax2)


# Select a subset of ticks to display, limiting to 10 or fewer ticks
max_ticks = 10
x_ticks_indices = np.linspace(
    0, len(xedges) - 1, min(len(xedges), max_ticks), dtype=int)
y_ticks_indices = np.linspace(
    0, len(yedges) - 1, min(len(yedges), max_ticks), dtype=int)

# # Set the x and y ticks based on the selected subset of bin edges
# ax2.set_xticks(x_ticks_indices, np.round(xedges[x_ticks_indices]))
# ax2.set_yticks(y_ticks_indices, np.round(yedges[y_ticks_indices]))
# # Function to convert real-world coordinates to heatmap coordinates

# Define the circle in real-world coordinates
# Replace with your desired center coordinates in real-world scale
goal_center_real = [420, 330]
# Replace with your desired radius in real-world scale
goal_radius_real = 75

# Convert the circle's real-world position to heatmap coordinates
goal_center_heatmap = convert_to_heatmap_coords(goal_center_real[0], goal_center_real[1],
                                                x_min_real, x_max_real, y_min_real, y_max_real,
                                                x_min_heatmap, x_max_heatmap, y_min_heatmap, y_max_heatmap)

# Convert the real-world radius to heatmap scale
cgoal_radius_heatmap = goal_radius_real * \
    (x_max_heatmap - x_min_heatmap) / (x_max_real - x_min_real)

# Scale the circle properly
circle = Circle(goal_center_heatmap, cgoal_radius_heatmap,
                color='black', fill=False, linewidth=2)

# Add the circle to the plot
ax2.add_patch(circle)


# Define the circle in real-world coordinates
# Replace with your desired center coordinates in real-world scale
start_center_real = [600, 450]
# Replace with your desired radius in real-world scale
start_radius_real = 75

# Convert the circle's real-world position to heatmap coordinates
start_center_heatmap = convert_to_heatmap_coords(start_center_real[0], start_center_real[1],
                                                 x_min_real, x_max_real, y_min_real, y_max_real,
                                                 x_min_heatmap, x_max_heatmap, y_min_heatmap, y_max_heatmap)

# Convert the real-world radius to heatmap scale
start_radius_heatmap = start_radius_real * \
    (x_max_heatmap - x_min_heatmap) / (x_max_real - x_min_real)

# Scale the circle properly
circle = Circle(start_center_heatmap, start_radius_heatmap,
                color='white', fill=False, linewidth=2)

# Add the circle to the plot
ax2.add_patch(circle)


####
####
# OUTMOST
directory_path = "data/aggregation_pref/success_rate/low_cluster/4/outmost/"
all_cbf_low_4_data = []
# Loop through all files in the directory
for filename in os.listdir(directory_path):
    if filename.endswith('.pkl') or filename.endswith('.pickle'):
        file_path = os.path.join(directory_path, filename)
        with open(file_path, 'rb') as file:
            data = pickle.load(file)
            all_cbf_low_4_data.append(data)


# Initialize lists to collect all x and y coordinates
all_x = []
all_y = []

# Assuming all_cbf_low_4_data is defined and structured as shown in your example
for indx in range(0, len(all_cbf_low_4_data)):
    if (len(all_cbf_low_4_data[indx]['data']) > range_lim):
        continue
    for all_animal_data in all_cbf_low_4_data[indx]['data'][:]:
        for animal_data in all_animal_data[target]:
            all_x.append(animal_data[0])
            all_y.append(animal_data[1])

# Convert lists to numpy arrays
all_x = np.array(all_x)
all_y = np.array(all_y)
# Create a 2D histogram of the aggregated movement data
heatmap, xedges, yedges = np.histogram2d(
    all_x, all_y, bins=rate, range=[[x_min_real, x_max_real], [y_min_real, y_max_real]])

# Plot the heatmap
sns.heatmap(heatmap.T, square=True, cmap=light_jet,
            robust=True,
            cbar=False, xticklabels=False, yticklabels=False, ax=ax5)
# Set axis labels as meters

# Select a subset of ticks to display, limiting to 10 or fewer ticks
max_ticks = 10
x_ticks_indices = np.linspace(
    0, len(xedges) - 1, min(len(xedges), max_ticks), dtype=int)
y_ticks_indices = np.linspace(
    0, len(yedges) - 1, min(len(yedges), max_ticks), dtype=int)

# # Set the x and y ticks based on the selected subset of bin edges
# ax2.set_xticks(x_ticks_indices, np.round(xedges[x_ticks_indices]))
# ax2.set_yticks(y_ticks_indices, np.round(yedges[y_ticks_indices]))
# # Function to convert real-world coordinates to heatmap coordinates


# Define the circle in real-world coordinates
# Replace with your desired center coordinates in real-world scale
goal_center_real = [420, 330]
# Replace with your desired radius in real-world scale
goal_radius_real = 75

# Convert the circle's real-world position to heatmap coordinates
goal_center_heatmap = convert_to_heatmap_coords(goal_center_real[0], goal_center_real[1],
                                                x_min_real, x_max_real, y_min_real, y_max_real,
                                                x_min_heatmap, x_max_heatmap, y_min_heatmap, y_max_heatmap)

# Convert the real-world radius to heatmap scale
cgoal_radius_heatmap = goal_radius_real * \
    (x_max_heatmap - x_min_heatmap) / (x_max_real - x_min_real)

# Scale the circle properly
circle = Circle(goal_center_heatmap, cgoal_radius_heatmap,
                color='black', fill=False, linewidth=2)

# Add the circle to the plot
ax5.add_patch(circle)


# Define the circle in real-world coordinates
# Replace with your desired center coordinates in real-world scale
start_center_real = [600, 450]
# Replace with your desired radius in real-world scale
start_radius_real = 75

# Convert the circle's real-world position to heatmap coordinates
start_center_heatmap = convert_to_heatmap_coords(start_center_real[0], start_center_real[1],
                                                 x_min_real, x_max_real, y_min_real, y_max_real,
                                                 x_min_heatmap, x_max_heatmap, y_min_heatmap, y_max_heatmap)

# Convert the real-world radius to heatmap scale
start_radius_heatmap = start_radius_real * \
    (x_max_heatmap - x_min_heatmap) / (x_max_real - x_min_real)

# Scale the circle properly
circle = Circle(start_center_heatmap, start_radius_heatmap,
                color='white', fill=False, linewidth=2)

# Add the circle to the plot
ax5.add_patch(circle)

################################################
################################################
################################################

################################################
################################################
################################################
################################################

################################################
################################################
################################################
################################################################################################
################################################


################################################

################################################
################################################
################################################

################################################
################################################
################################################
################################################

################################################
################################################
################################################
################################################################################################
################################################


################################################


# HIGH AGGREGATION PREF
# 4 robots
# CBF
directory_path = "data/aggregation_pref/success_rate/high/4/cbf/"
all_cbf_low_4_data = []
# Loop through all files in the directory
for filename in os.listdir(directory_path):
    if filename.endswith('.pkl') or filename.endswith('.pickle'):
        file_path = os.path.join(directory_path, filename)
        with open(file_path, 'rb') as file:
            data = pickle.load(file)
            all_cbf_low_4_data.append(data)


# Initialize lists to collect all x and y coordinates
all_x = []
all_y = []

# Assuming all_cbf_low_4_data is defined and structured as shown in your example
for indx in range(0, len(all_cbf_low_4_data)):
    if (len(all_cbf_low_4_data[indx]['data']) > range_lim):
        continue
    for all_animal_data in all_cbf_low_4_data[indx]['data'][:]:
        for animal_data in all_animal_data[target]:
            all_x.append(animal_data[0])
            all_y.append(animal_data[1])

# Convert lists to numpy arrays
all_x = np.array(all_x)
all_y = np.array(all_y)
# Create a 2D histogram of the aggregated movement data
heatmap, xedges, yedges = np.histogram2d(
    all_x, all_y, bins=rate, range=[[x_min_real, x_max_real], [y_min_real, y_max_real]])

# Plot the heatmap
sns.heatmap(heatmap.T, square=True, cmap=light_jet,
            robust=True,
            cbar=False, xticklabels=False, yticklabels=False, ax=ax3)


# Select a subset of ticks to display, limiting to 10 or fewer ticks
max_ticks = 10
x_ticks_indices = np.linspace(
    0, len(xedges) - 1, min(len(xedges), max_ticks), dtype=int)
y_ticks_indices = np.linspace(
    0, len(yedges) - 1, min(len(yedges), max_ticks), dtype=int)

# # Set the x and y ticks based on the selected subset of bin edges
# ax3.set_xticks(x_ticks_indices, np.round(xedges[x_ticks_indices]))
# ax3.set_yticks(y_ticks_indices, np.round(yedges[y_ticks_indices]))
# # Function to convert real-world coordinates to heatmap coordinates

# Define the circle in real-world coordinates
# Replace with your desired center coordinates in real-world scale
goal_center_real = [420, 330]
# Replace with your desired radius in real-world scale
goal_radius_real = 75

# Convert the circle's real-world position to heatmap coordinates
goal_center_heatmap = convert_to_heatmap_coords(goal_center_real[0], goal_center_real[1],
                                                x_min_real, x_max_real, y_min_real, y_max_real,
                                                x_min_heatmap, x_max_heatmap, y_min_heatmap, y_max_heatmap)

# Convert the real-world radius to heatmap scale
cgoal_radius_heatmap = goal_radius_real * \
    (x_max_heatmap - x_min_heatmap) / (x_max_real - x_min_real)

# Scale the circle properly
circle = Circle(goal_center_heatmap, cgoal_radius_heatmap,
                color='black', fill=False, linewidth=2)

# Add the circle to the plot
ax3.add_patch(circle)


# Define the circle in real-world coordinates
# Replace with your desired center coordinates in real-world scale
start_center_real = [600, 450]
# Replace with your desired radius in real-world scale
start_radius_real = 75

# Convert the circle's real-world position to heatmap coordinates
start_center_heatmap = convert_to_heatmap_coords(start_center_real[0], start_center_real[1],
                                                 x_min_real, x_max_real, y_min_real, y_max_real,
                                                 x_min_heatmap, x_max_heatmap, y_min_heatmap, y_max_heatmap)

# Convert the real-world radius to heatmap scale
start_radius_heatmap = start_radius_real * \
    (x_max_heatmap - x_min_heatmap) / (x_max_real - x_min_real)

# Scale the circle properly
circle = Circle(start_center_heatmap, start_radius_heatmap,
                color='white', fill=False, linewidth=2)

# Add the circle to the plot
ax3.add_patch(circle)


####
####
# OUTMOST
directory_path = "data/aggregation_pref/success_rate/high/4/outmost/"
all_cbf_low_4_data = []
# Loop through all files in the directory
for filename in os.listdir(directory_path):
    if filename.endswith('.pkl') or filename.endswith('.pickle'):
        file_path = os.path.join(directory_path, filename)
        with open(file_path, 'rb') as file:
            data = pickle.load(file)
            all_cbf_low_4_data.append(data)


# Initialize lists to collect all x and y coordinates
all_x = []
all_y = []

# Assuming all_cbf_low_4_data is defined and structured as shown in your example
for indx in range(0, len(all_cbf_low_4_data)):
    if (len(all_cbf_low_4_data[indx]['data']) > range_lim):
        continue
    for all_animal_data in all_cbf_low_4_data[indx]['data'][:]:
        for animal_data in all_animal_data[target]:
            all_x.append(animal_data[0])
            all_y.append(animal_data[1])

# Convert lists to numpy arrays
all_x = np.array(all_x)
all_y = np.array(all_y)
# Create a 2D histogram of the aggregated movement data
heatmap, xedges, yedges = np.histogram2d(
    all_x, all_y, bins=rate, range=[[x_min_real, x_max_real], [y_min_real, y_max_real]])

# Plot the heatmap
sns.heatmap(heatmap.T, square=True, cmap=light_jet,
            robust=True,
            cbar=False, xticklabels=False, yticklabels=False, ax=ax6)
# Set axis labels as meters

# Select a subset of ticks to display, limiting to 10 or fewer ticks
max_ticks = 10
x_ticks_indices = np.linspace(
    0, len(xedges) - 1, min(len(xedges), max_ticks), dtype=int)
y_ticks_indices = np.linspace(
    0, len(yedges) - 1, min(len(yedges), max_ticks), dtype=int)

# # Set the x and y ticks based on the selected subset of bin edges
# ax4.set_xticks(x_ticks_indices, np.round(xedges[x_ticks_indices]))
# ax4.set_yticks(y_ticks_indices, np.round(yedges[y_ticks_indices]))
# # Function to convert real-world coordinates to heatmap coordinates


# Define the circle in real-world coordinates
# Replace with your desired center coordinates in real-world scale
goal_center_real = [420, 330]
# Replace with your desired radius in real-world scale
goal_radius_real = 75

# Convert the circle's real-world position to heatmap coordinates
goal_center_heatmap = convert_to_heatmap_coords(goal_center_real[0], goal_center_real[1],
                                                x_min_real, x_max_real, y_min_real, y_max_real,
                                                x_min_heatmap, x_max_heatmap, y_min_heatmap, y_max_heatmap)

# Convert the real-world radius to heatmap scale
cgoal_radius_heatmap = goal_radius_real * \
    (x_max_heatmap - x_min_heatmap) / (x_max_real - x_min_real)

# Scale the circle properly
circle = Circle(goal_center_heatmap, cgoal_radius_heatmap,
                color='black', fill=False, linewidth=2)

# Add the circle to the plot
ax6.add_patch(circle)


# Define the circle in real-world coordinates
# Replace with your desired center coordinates in real-world scale
start_center_real = [600, 450]
# Replace with your desired radius in real-world scale
start_radius_real = 75

# Convert the circle's real-world position to heatmap coordinates
start_center_heatmap = convert_to_heatmap_coords(start_center_real[0], start_center_real[1],
                                                 x_min_real, x_max_real, y_min_real, y_max_real,
                                                 x_min_heatmap, x_max_heatmap, y_min_heatmap, y_max_heatmap)

# Convert the real-world radius to heatmap scale
start_radius_heatmap = start_radius_real * \
    (x_max_heatmap - x_min_heatmap) / (x_max_real - x_min_real)

# Scale the circle properly
circle = Circle(start_center_heatmap, start_radius_heatmap,
                color='white', fill=False, linewidth=2)

# Add the circle to the plot
ax6.add_patch(circle)

ax1.annotate('Low preference (Weak attraction) - Hybrid (Ours)', xy=(0.025, 0.025), xycoords='axes fraction', fontsize=12,
             ha='left', va='bottom', color='white')  # Add annotation at the left bottom
ax2.annotate('Low preference (Locally clustered) - Hybrid (Ours)', xy=(0.025, 0.025), xycoords='axes fraction', fontsize=12,
             ha='left', va='bottom', color='white')  # Add annotation at the left bottom
ax3.annotate('High preference - Hybrid (Ours)', xy=(0.025, 0.025), xycoords='axes fraction', fontsize=12,
             ha='left', va='bottom', color='white')  # Add annotation at the left bottom
ax4.annotate('Low preference (Weak attraction) - Outmost Push', xy=(0.025, 0.025), xycoords='axes fraction', fontsize=12,
             ha='left', va='bottom', color='white')  # Add annotation at the left bottom
ax5.annotate('Low preference (Locally clustered) - Outmost Push', xy=(0.025, 0.025), xycoords='axes fraction', fontsize=12,
             ha='left', va='bottom', color='white')  # Add annotation at the left bottom
ax6.annotate('High preference - Outmost Push', xy=(0.025, 0.025), xycoords='axes fraction', fontsize=12,
             ha='left', va='bottom', color='white')  # Add annotation at the left bottom

ax1.arrow(0, 0, 5, 5, head_width=1,
          head_length=0.15, fc='k', ec='k', lw=0.75)

handles = [plt.Line2D([0], [0], color='white',
                      markerfacecolor='white', markersize=11),
           plt.Line2D([0], [0], color='black',
                      markerfacecolor='black', markersize=11)]

labels = [r'Initial position',
          r'Goal']
ax1.legend(handles=handles,
           labels=labels, loc='upper left', fontsize=11, handletextpad=0.5)
# Adjust layout to have a small divider
plt.subplots_adjust(wspace=0.005, hspace=0.005)
plt.tight_layout(w_pad=0.005, h_pad=0.005)

plt.savefig('heatmap.pdf', format='pdf')
# plt.show()
