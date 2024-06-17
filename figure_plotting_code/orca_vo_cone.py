import matplotlib.pyplot as plt
import numpy as np


def unit_vector(v):
    return v / np.linalg.norm(v)


# Enable LaTeX for text rendering
plt.rcParams['text.usetex'] = True
plt.rcParams['font.family'] = 'serif'
plt.rcParams['font.serif'] = 'Computer Modern'
# Parameters for the plot
pA = np.array([0, 0])
pB = np.array([-2., 4.5])
v_A = np.array([1.2, 1.5])
truncated_vA = unit_vector(v_A) * (np.linalg.norm(v_A) - 0.2)
v_B = np.array([2.0, -0.7])
truncated_vB = unit_vector(v_B) * (np.linalg.norm(v_B) - 0.2)
vA_vB = v_A - v_B
truncated_vA_vB = unit_vector(vA_vB) * (np.linalg.norm(vA_vB) - 0.2)

rA = 0.8
rB = 0.8
tau = 2
side_length = 7

# Derived parameters
pB_pA = pB - pA
pB_pA_tau = pB_pA / tau
r_sum = rA + rB
# Plotting the vectors and shaded region
fig, ax = plt.subplots()

# # Shaded region (velocity obstacle)
cone_angle = np.arctan2(pB_pA[1], pB_pA[0])
cone_width_angle = np.arcsin(r_sum/np.linalg.norm(pB_pA))
lower_angle = cone_angle - cone_width_angle
upper_angle = cone_angle + cone_width_angle
left = np.array([side_length * np.cos(upper_angle),
                side_length * np.sin(upper_angle)])

lower_left_length = np.sqrt(np.linalg.norm(pB_pA_tau)**2 - (r_sum/tau)**2)
lower_left = np.array([lower_left_length * np.cos(upper_angle),
                       lower_left_length * np.sin(upper_angle)])
right = np.array([side_length * np.cos(lower_angle),
                 side_length * np.sin(lower_angle)])
lower_right = np.array([lower_left_length * np.cos(lower_angle),
                        lower_left_length * np.sin(lower_angle)])

# For drawing only

pB_pA_tau_offset = unit_vector(
    pB_pA_tau) * (np.linalg.norm(pB_pA_tau) + 0.005)


# Circles representing positions
circleA = plt.Circle(pB_pA_tau, r_sum / tau, edgecolor='black',
                     facecolor='lightyellow', linestyle='-', lw=1)


ax.add_patch(circleA)


vertices = np.array([
    [lower_right[0], lower_right[1]],
    [lower_left[0], lower_left[1]],
    [left[0], left[1]],
    [right[0], right[1]]])
# Plot the triangle
ax.fill(*zip(*vertices),
        edgecolor='none', facecolor='lightyellow')

circleC = plt.Circle(pB_pA, r_sum, edgecolor='black',
                     facecolor='lightyellow', linestyle='--', lw=0.5)
ax.add_patch(circleC)

# circleB = plt.Circle(pB_pA_tau, r_sum / tau, edgecolor='black',
#                      facecolor='lightyellow', lw=1, linestyle='--')
# ax.add_patch(circleB)

plt.plot([0, left[0]], [0, left[1]], 'k', lw=0.75, linestyle='--')
plt.plot([0, right[0]], [0, right[1]], 'k', lw=0.75, linestyle='--')

plt.arrow(lower_left[0], lower_left[1], left[0] - lower_left[0], left[1] - lower_left[1], head_width=0.1,
          head_length=0.2, fc='k', ec='k', lw=0.75)

plt.arrow(lower_right[0], lower_right[1], right[0] - lower_right[0], right[1] - lower_right[1], head_width=0.1,
          head_length=0.2, fc='k', ec='k', lw=0.75)

radius_angle = (np.pi/2 - cone_width_angle) + cone_angle
radius = np.array([np.cos(radius_angle + 0.3) * (r_sum - 0.2),
                  np.sin(radius_angle + 0.3) * (r_sum - 0.2)])

radius_tau = np.array([np.cos(radius_angle + 1.06) * (r_sum/tau - 0.2),
                       np.sin(radius_angle + 1.06) * (r_sum/tau - 0.2)])

plt.arrow(pB_pA[0], pB_pA[1], radius[0], radius[1], head_width=0.1,
          head_length=0.2, fc='k', ec='k', lw=0.75)
plt.arrow(pB_pA_tau[0], pB_pA_tau[1], radius_tau[0], radius_tau[1], head_width=0.1,
          head_length=0.2, fc='k', ec='k', lw=0.75)

# Annotations
ax.annotate(r'$\mathbf{r}_{ij}$', xy=(pB_pA[0] + radius[0], pB_pA[1] + radius[1]),
            xytext=(10, 15), textcoords='offset points', ha='center', fontsize=14)
ax.annotate(r'$\mathbf{r}_{ij}/\tau$', xy=(pB_pA_tau[0] + radius_tau[0], pB_pA_tau[1] +
            radius_tau[1]), xytext=(-20, -20), textcoords='offset points', ha='center', fontsize=14)

ax.set_aspect('equal', 'box')
ax.set_xlim(-6, 1)
ax.set_ylim(-1, 7)

# Adding arrows for axes
plt.arrow(0, 0, 0.5, 0, head_width=0.05,
          head_length=0.1, fc='k', ec='k', lw=0.5)
plt.arrow(0, 0, 0, 3.0, head_width=0.05,
          head_length=0.1, fc='k', ec='k', lw=0.5)
plt.arrow(0, 0, -3.0, 0, head_width=0.05,
          head_length=0.1, fc='k', ec='k', lw=0.5)
plt.arrow(0, 0, 0, -0.5, head_width=0.05,
          head_length=0.1, fc='k', ec='k', lw=0.5)

ax.plot(*pB_pA, 'ko', markersize=2, label='Agent i')
ax.plot(*pB_pA_tau, 'ko', markersize=2, label='Agent i')

plt.text(0.35, -0.3, r'$\mathbf{v_x}$', fontsize=14)
plt.text(0.1, 2.85, r'$\mathbf{v_y}$', fontsize=14)


# triangle = plt.Polygon(vertices,
#                        edgecolor='none', facecolor='lightyellow')

# Annotations
ax.annotate(r'$\Delta \mathbf{p}_{ij}$', xy=(pB_pA[0], pB_pA[1]),
            xytext=(10, 10), textcoords='offset points', ha='center', fontsize=14)

ax.annotate(r'$\Delta \mathbf{p}_{ij} / \tau$', xy=(pB_pA_tau[0], pB_pA_tau[1]),
            xytext=(0, 10), textcoords='offset points', ha='center', fontsize=14)

ax.annotate(r'$\mathbf{VO}_{i|j}^{\tau}$', xy=(-1.75, 2.25),
            xytext=(-15, 40), textcoords='offset points', ha='center', fontsize=14)


ax.axis('off')  # Disable the axis
ax.set_aspect('equal', adjustable='box')  # Maintain aspect ratio

plt.savefig('orca_cone_construct_plot.pdf', format='pdf')
