import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns

# Generate random movement data
np.random.seed(0)
x = np.cumsum(np.random.randn(1000))
y = np.cumsum(np.random.randn(1000))

# Create a 2D histogram of the movement data
heatmap, xedges, yedges = np.histogram2d(x, y, bins=50)

# Plot the heatmap
plt.figure(figsize=(10, 8))
sns.heatmap(heatmap.T, square=True, cmap='viridis', cbar=True)
plt.title("Heat Map of Robot Movement")
plt.xlabel("X-axis")
plt.ylabel("Y-axis")
plt.show()
