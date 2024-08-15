import numpy as np
import matplotlib.pyplot as plt

# Example data: trajectories represented as heatmaps
# Assuming a grid of size 100x100 where trajectories are recorded
trajectory1 = np.zeros((100, 100))
trajectory2 = np.zeros((100, 100))

# Simulate some trajectory data (for example purposes)
# Robot 1 trajectory
trajectory1[20:80, 30:35] += 1
trajectory1[40:45, 30:70] += 1

# Robot 2 trajectory
trajectory2[60:90, 10:15] += 1
trajectory2[10:15, 50:90] += 1

# Create the figure and axis
fig, ax = plt.subplots()

# First heatmap (Robot 1) with the first colormap
# Adjust alpha for transparency
heatmap1 = ax.imshow(trajectory1, cmap='Blues', alpha=0.7)

# Second heatmap (Robot 2) with the second colormap
# Adjust alpha for transparency
heatmap2 = ax.imshow(trajectory2, cmap='Reds', alpha=0.7)

# Add colorbars
cbar1 = fig.colorbar(heatmap1, ax=ax, orientation='vertical',
                     fraction=0.046, pad=0.04)
cbar2 = fig.colorbar(heatmap2, ax=ax, orientation='vertical',
                     fraction=0.046, pad=0.04)

# Set labels for colorbars
cbar1.set_label('Robot 1 Trajectory')
cbar2.set_label('Robot 2 Trajectory')

# Set plot title and labels
ax.set_title('Overlayed Robot Trajectories')
ax.set_xlabel('X Coordinate')
ax.set_ylabel('Y Coordinate')

# Show plot
plt.show()
