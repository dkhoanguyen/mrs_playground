import numpy as np
import matplotlib.pyplot as plt

# Number of steps
num_steps = 20000

# Generate random walk
x = np.cumsum(np.random.randn(num_steps))
y = np.cumsum(np.random.randn(num_steps))

# Create a colormap from dark red to white
colors = np.linspace(0, 1, num_steps)
cmap = plt.get_cmap('Reds')

# Create the figure and axis
fig, ax = plt.subplots(figsize=(10, 6))

# Plot the random walk
for i in range(num_steps - 1):
    ax.plot(x[i:i+2], y[i:i+2], color=cmap(colors[i]), linewidth=2)

# Add a color bar for reference
sm = plt.cm.ScalarMappable(
    cmap=cmap, norm=plt.Normalize(vmin=0, vmax=num_steps))
sm.set_array([])
cbar = plt.colorbar(sm, ax=ax)
cbar.set_label('Step Index')

# Mark the starting and ending points
ax.scatter(x[0], y[0], color='white',
           edgecolor='black', zorder=5, label='Start')
ax.scatter(x[-1], y[-1], color='red', edgecolor='black', zorder=5, label='End')

ax.set_title("2D Random Squiggly Line with Gradient Effect")
ax.set_xlabel("X")
ax.set_ylabel("Y")
ax.legend()
plt.show()
