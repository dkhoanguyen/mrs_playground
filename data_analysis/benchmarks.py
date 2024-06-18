# !/usr/bin/python3

import os
import numpy as np
import pandas as pd
import pickle
import matplotlib.pyplot as plt

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

apf_execution_time = []
cbf_execution_time = []

for data in all_apf_data:
    apf_execution_time.append(len(data['data']))
for data in all_cbf_data:
    cbf_execution_time.append(len(data['data']))

plt.boxplot([apf_execution_time, cbf_execution_time], vert=True)
plt.show()
