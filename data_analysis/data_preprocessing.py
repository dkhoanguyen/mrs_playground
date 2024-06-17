# !/usr/bin/python3

import numpy as np


def data_preprocessing(data, window_size=200, sampling_rate=100,
                       seconds_to_remove=20):
    def moving_average(data, window_size):
        return np.convolve(data, np.ones(window_size)/window_size, mode='valid')

    x = [point[0] for point in data]
    y = [point[1] for point in data]

    points_to_remove = sampling_rate * seconds_to_remove

    x_trimmed = x[:-points_to_remove]
    y_trimmed = y[:-points_to_remove]

    x_smooth_trimmed = moving_average(x_trimmed, window_size)
    y_smooth_trimmed = moving_average(y_trimmed, window_size)

    return x_smooth_trimmed, y_smooth_trimmed


def get_xy_points_at_intervals(x_data, y_data, interval, sampling_rate):
    points_per_interval = sampling_rate * interval
    return [(x_data[i], y_data[i]) for i in range(0, len(x_data), points_per_interval)]


def get_xy_points_at_intervals_given_index(x_data, y_data, interval, sampling_rate, index_list):
    extracted_points = []
    for i in range(len(index_list)):
        extracted_points.append((x_data[index_list[i]], y_data[index_list[i]]))
    return extracted_points
