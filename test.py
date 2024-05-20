import numpy as np
import matplotlib.pyplot as plt

# Parameters
num_steps = 10000
step_size = 0.01

# Initialize position
x, y = 0.5, 0.5

# Lists to store positions
x_positions = [x]
y_positions = [y]

# Random walk
for _ in range(num_steps):
    x += step_size * (np.random.rand() - 0.5)
    y += step_size * (np.random.rand() - 0.5)
    
    # Reflect off boundaries
    if x < 0: x = -x
    if x > 1: x = 2 - x
    if y < 0: y = -y
    if y > 1: y = 2 - y
    
    x_positions.append(x)
    y_positions.append(y)

# Plot the trajectory
plt.figure(figsize=(6, 6))
plt.plot(x_positions, y_positions, lw=0.5)
plt.xlim(0, 1)
plt.ylim(0, 1)
plt.title('Random Walk in Unit Square')
plt.xlabel('x')
plt.ylabel('y')
plt.show()
