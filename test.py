import numpy as np
import matplotlib.pyplot as plt

# Define the bump function with V_max outside the sensing range and diminishing inside the range


def bump_function_vmax(x, V_max, r_s):
    if x <= r_s:
        return V_max * 0.5 * (1 - np.cos(np.pi * x / r_s))
    else:
        return V_max


# Vectorize the bump function for efficient application to arrays
bump_function_vec = np.vectorize(bump_function_vmax)

# Define parameters
V_max = 10.0  # Maximum value of the potential
r_s = 3.0     # Sensing range

# Create an array of input values for distance
x_values = np.linspace(0, 5, 500)

# Apply the bump function to each input value
y_values = bump_function_vec(x_values, V_max, r_s)

# Plot the bump function illustrating the V_max cutoff
plt.plot(x_values, y_values, label=r'$V_{max}$ Outside Sensing Range')
plt.title(r'Bump Function with $V_{max}$ Outside Sensing Range')
plt.xlabel('Distance (x)')
plt.ylabel('Potential Value (V(x))')
plt.axvline(x=r_s, color='red', linestyle='--', label=f'Sensing Range = {r_s}')
plt.axhline(y=V_max, color='green', linestyle='--',
            label=f'$V_{{max}} = {V_max}$')
plt.grid(True)
plt.legend()
plt.show()
