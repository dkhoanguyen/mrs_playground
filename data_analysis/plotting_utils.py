# !/usr/bin/python3

import os
import numpy as np
import pandas as pd
import pickle
import matplotlib.pyplot as plt


def plot_triangle_with_centrer(center, height, base, angle, color, ax):
    # Calculate the vertices of the triangle before rotation
    half_base = base / 2
    vertices = np.array([
        [-half_base, -height / 3],
        [half_base, -height / 3],
        [0, 2*height / 3]
    ])

    # Create a rotation matrix
    angle_rad = np.radians(angle)
    rotation_matrix = np.array([
        [np.cos(angle_rad), -np.sin(angle_rad)],
        [np.sin(angle_rad), np.cos(angle_rad)]
    ])

    # Rotate the vertices
    rotated_vertices = np.dot(vertices, rotation_matrix)

    # Translate vertices to the center
    translated_vertices = rotated_vertices + np.array(center)

    triangle = plt.Polygon(translated_vertices,
                           edgecolor=color, facecolor=color)

    # Plot the triangle
    ax.fill(*zip(*translated_vertices),
            edgecolor=color, facecolor=color)

    return triangle


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


def plot_target(ax):
    vertices = np.array([
        [1.5, 0.5],
        [2.5, 0.5],
        [2.5, -0.5],
        [1.5, -0.5],
        [1.5, 0.5],
    ])
    vertices1 = np.array([
        [1.55, 0.45],
        [2.45, 0.45],
        [2.45, -0.45],
        [1.55, -0.45],
        [1.55, 0.45],
    ])
    rec_1 = np.array([
        [1.25, 0.3],
        [2.75, 0.3],
        [2.75, -0.3],
        [1.25, -0.3],
        [1.25, 0.3],
    ])
    rec_2 = np.array([
        [1.75, 0.6],
        [2.25, 0.6],
        [2.25, -0.6],
        [1.75, -0.6],
        [1.75, 0.6],
    ])
    ax.fill(vertices[:, 0], vertices[:, 1], color='dimgray')
    ax.fill(vertices1[:, 0], vertices1[:, 1], color='white', edgecolor='white')
    ax.fill(rec_1[:, 0], rec_1[:, 1], color='white', edgecolor='white')
    ax.fill(rec_2[:, 0], rec_2[:, 1], color='white', edgecolor='white')


def get_xy_points_at_intervals(x_data, y_data, interval, sampling_rate):
    points_per_interval = sampling_rate * interval
    return [(x_data[i], y_data[i]) for i in range(0, len(x_data), points_per_interval)]


def get_xy_points_at_intervals_given_index(x_data, y_data, index_list):
    extracted_points = []
    for i in range(len(index_list)):
        extracted_points.append((x_data[index_list[i]], y_data[index_list[i]]))
    return extracted_points
