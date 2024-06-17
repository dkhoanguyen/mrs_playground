import matplotlib.pyplot as plt

# Define the corners of the square
corners_x = [0, 1, 1, 0, 0]
corners_y = [0, 0, 1, 1, 0]

# Create the figure and axis
fig, ax = plt.subplots()

# Plot the corners
ax.scatter(corners_x, corners_y, color='blue')

# Draw the lines with gaps in the middle of each edge
for i in range(len(corners_x) - 1):
    x_values = [corners_x[i],
                (corners_x[i] + corners_x[i+1]) / 2, corners_x[i+1]]
    y_values = [corners_y[i],
                (corners_y[i] + corners_y[i+1]) / 2, corners_y[i+1]]

    # Plot the first half of the edge
    ax.plot(x_values[:2], y_values[:2], color='blue')

    # Plot the second half of the edge
    ax.plot(x_values[1:], y_values[1:], color='blue')

# Set the limits and aspect ratio to make the plot square
ax.set_xlim(-0.5, 1.5)
ax.set_ylim(-0.5, 1.5)
ax.set_aspect('equal')

# Add grid and labels for better visualization
ax.grid(True)
ax.set_xlabel('X-axis')
ax.set_ylabel('Y-axis')
ax.set_title('Square with Corners and Gaps in the Middle of Edges')

# Show the plot
plt.show()
