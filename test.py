import numpy as np
import matplotlib.pyplot as plt

# Define the circle parameters
radius = 1
theta = np.linspace(0, 2 * np.pi, 100)  # Full circle

# Define the circle
x_circle = radius * np.cos(theta)
y_circle = radius * np.sin(theta)

# Generate a random line within the bounds of the circle
# Ensure the line is within the circle's bounds
x_line = np.linspace(-radius, radius, 100)
y_line = np.random.uniform(-radius, radius, size=x_line.shape)

# Plot the circle and the line
plt.plot(x_circle, y_circle, label='Circle')
plt.plot(x_line, y_line, label='Random Line')

# Fill the area between the circle and the line
plt.fill_between(x_line, y_circle, y_line, where=(
    y_line <= y_circle), color='skyblue', alpha=0.5)

# Set the aspect ratio to be equal
plt.gca().set_aspect('equal', adjustable='box')

# Add labels and legend
plt.xlabel('x')
plt.ylabel('y')
plt.legend()

# Show the plot
plt.show()
